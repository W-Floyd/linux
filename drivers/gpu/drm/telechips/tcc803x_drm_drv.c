// SPDX-License-Identifier: GPL-2.0-only
/*
 * TCC803x DRM/KMS top-level driver
 *
 * Integration skeleton only: binds whatever VIOC CRTC/plane and HDMI
 * encoder/connector drivers are registered as components, registers a
 * minimal DRM device, and turns on generic fbdev/fbcon emulation so
 * kernel console output lands on the physical display. There is no
 * accessible UART on this board, so fbcon is the only debug console
 * available once CPU/GIC/timer bring-up has gotten this far.
 *
 * Structural reference: Telechips' own drivers/gpu/drm/telechips/
 * (github.com/topst-development/kernel-5.10, release/d3/1.3.0) and the
 * older in-tree vendor driver at drivers/gpu/drm/tcc/tcc_drm_drv.c in
 * the AVNT5W_TCC803XKK Android BSP, both of which use a component-match
 * master bound to per-block sub-drivers. Register-level details from
 * those trees are NOT reused here (different chip generation / output
 * type); only the probe/component/fbdev structure is adapted.
 *
 * What this file does NOT yet do (see bottom of file for the honest
 * status writeup): it does not itself implement a CRTC, plane, or
 * encoder. Those come from the VIOC and HDMI drivers; this is only the
 * glue that aggregates them into one struct drm_device.
 */

#include <linux/component.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mode_config.h>
#include <drm/drm_module.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#define TCC803X_DRM_NAME	"tcc803x-drm"

struct tcc803x_drm {
	struct drm_device base;
};

static const struct drm_mode_config_funcs tcc803x_mode_config_funcs = {
	.fb_create		= drm_gem_fb_create,
	.atomic_check		= drm_atomic_helper_check,
	.atomic_commit		= drm_atomic_helper_commit,
};

DEFINE_DRM_GEM_DMA_FOPS(tcc803x_drm_fops);

static const struct drm_driver tcc803x_drm_driver = {
	.driver_features	= DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.fops			= &tcc803x_drm_fops,
	DRM_GEM_DMA_DRIVER_OPS,
	DRM_FBDEV_DMA_DRIVER_OPS,
	.name			= "tcc803x-drm",
	.desc			= "Telechips TCC803x VIOC+HDMI DRM/KMS driver",
	.major			= 1,
	.minor			= 0,
};

/*
 * No of_device_id match table here: tcc803x_drm_add_components() below
 * builds the component_match set positionally, from the "ports" phandle
 * list on the display-subsystem node (drm_of_component_match_add() +
 * compare_of(), same as exynos/rockchip), not by compatible string. A
 * vioc_crtc0/hdmi_tx of_device_id table would never be consulted by that
 * path, so it isn't carried here.
 *
 * drivers/gpu/drm/telechips/tcc803x_drm_vioc.c implements the VIOC CRTC
 * side as a real component (telechips,tcc803x-vioc-crtc, component_add()
 * in its ->probe()) and is the only entry in "ports" (see tcc803x-avn.dts).
 *
 * drivers/gpu/drm/bridge/tcc803x-dw-hdmi.c implements the HDMI side as a
 * plain platform_driver around dw_hdmi_probe() - a DRM bridge, not a
 * component; it registers via drm_bridge_add(), not component_add(), and
 * is deliberately NOT listed in "ports". A phandle this match set can
 * never satisfy would leave component_bind_all() waiting forever. Instead
 * tcc803x_drm_vioc.c's ->bind() finds it via the vioc_crtc0/hdmi_tx
 * OF-graph endpoint pair in tcc803x.dtsi (devm_drm_of_get_bridge()) and
 * drm_bridge_attach()es it to its own CRTC-owned encoder directly - see
 * the status note at the bottom of this file.
 */

static int tcc803x_drm_bind(struct device *dev)
{
	struct tcc803x_drm *tcc;
	struct drm_device *drm;
	int ret;

	tcc = devm_drm_dev_alloc(dev, &tcc803x_drm_driver,
				  struct tcc803x_drm, base);
	if (IS_ERR(tcc))
		return PTR_ERR(tcc);
	drm = &tcc->base;

	dev_set_drvdata(dev, drm);

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	/*
	 * TODO(vioc): these limits should come from the VIOC display
	 * controller's real DISP timing generator constraints once that
	 * driver exists. Placeholder values only - not validated against
	 * hardware.
	 */
	drm->mode_config.min_width = 64;
	drm->mode_config.min_height = 64;
	drm->mode_config.max_width = 4096;
	drm->mode_config.max_height = 4096;
	drm->mode_config.funcs = &tcc803x_mode_config_funcs;

	/*
	 * component_bind_all() walks the match list built in
	 * tcc803x_drm_add_components() and calls each sub-driver's
	 * ->bind(), which is where the real
	 * drm_crtc_init/drm_universal_plane_init/drm_encoder_init/
	 * drm_connector_init calls are expected to happen (in the VIOC
	 * and HDMI drivers, not here).
	 */
	ret = component_bind_all(dev, drm);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	/*
	 * No drm_vblank_init(): the VIOC IRQ is not wired, so with vblank
	 * "supported" the atomic helpers wait for a vblank/flip_done that
	 * never comes. Without it they fake the event at commit time
	 * (drm_atomic_helper_fake_vblank()).
	 */

	drm_kms_helper_poll_init(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		goto err_unbind;

	/* fbcon: the actual point of deliverable 2. No UART on this
	 * board - this is the only way to see dmesg/panic output on
	 * real hardware once a CRTC+encoder can light up the panel/HDMI
	 * output. Registers the DMA-backed fbdev client; produces no
	 * visible output until a real CRTC/connector are bound (there
	 * are none yet - see status note at the bottom of this file).
	 */
	drm_client_setup(drm, NULL);

	return 0;

err_unbind:
	component_unbind_all(dev, drm);
	return ret;
}

static void tcc803x_drm_unbind(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);

	drm_dev_unregister(drm);
	drm_kms_helper_poll_fini(drm);
	component_unbind_all(dev, drm);
}

static const struct component_master_ops tcc803x_drm_component_ops = {
	.bind = tcc803x_drm_bind,
	.unbind = tcc803x_drm_unbind,
};

static int compare_of(struct device *dev, void *data)
{
	return dev->of_node == data;
}

/*
 * Walks the top-level display node's component phandles and adds each
 * one to the component match set. Mirrors exynos_drm_drv.c /
 * rockchip_drm_drv.c's approach rather than the vendor tree's manual
 * compatible-string scan (tcc_drm_match_add() in the Android BSP),
 * since phandle-based matching is what mainline review expects.
 *
 * Expects the SoC .dtsi (once VIOC/HDMI land) to gain a node like:
 *
 *   display: display-subsystem {
 *           compatible = "telechips,tcc803x-display-subsystem";
 *           ports = <&vioc_crtc0>, <&hdmi_encoder>;
 *   };
 *
 * This is NOT yet added to any .dts/.dtsi in this tree - there is
 * nothing for it to point at until the VIOC/HDMI DT nodes exist. See
 * the writeup at the end of this file.
 */
static int tcc803x_drm_add_components(struct device *dev,
				       struct component_match **match)
{
	struct device_node *np = dev->of_node;
	struct device_node *remote;
	int i;

	for (i = 0; ; i++) {
		remote = of_parse_phandle(np, "ports", i);
		if (!remote)
			break;

		drm_of_component_match_add(dev, match, compare_of, remote);
		of_node_put(remote);
	}

	return *match ? 0 : -ENODEV;
}

static int tcc803x_drm_platform_probe(struct platform_device *pdev)
{
	struct component_match *match = NULL;
	int ret;

	ret = tcc803x_drm_add_components(&pdev->dev, &match);
	if (ret)
		return ret;

	return component_master_add_with_match(&pdev->dev,
						 &tcc803x_drm_component_ops,
						 match);
}

static void tcc803x_drm_platform_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &tcc803x_drm_component_ops);
}

static const struct of_device_id tcc803x_drm_of_match[] = {
	{ .compatible = "telechips,tcc803x-display-subsystem" },
	{}
};
MODULE_DEVICE_TABLE(of, tcc803x_drm_of_match);

static struct platform_driver tcc803x_drm_platform_driver = {
	.probe = tcc803x_drm_platform_probe,
	.remove = tcc803x_drm_platform_remove,
	.driver = {
		.name = TCC803X_DRM_NAME,
		.of_match_table = tcc803x_drm_of_match,
	},
};

drm_module_platform_driver(tcc803x_drm_platform_driver);

MODULE_DESCRIPTION("Telechips TCC803x DRM/KMS driver");
MODULE_LICENSE("GPL");

/*
 * ===========================================================================
 * Honest status, updated now that the VIOC CRTC driver, the HDMI bridge
 * driver, and the display-subsystem DT node have all landed:
 *
 * DONE:
 *  - Top-level component-master skeleton: allocates the drm_device, wires
 *    atomic mode_config + GEM-DMA fops/ioctls, sets up component bind/unbind,
 *    and calls drm_fbdev_dma_setup() after drm_dev_register() so fbcon comes
 *    up automatically against whatever CRTC(s) the component drivers expose.
 *  - Kconfig/Makefile wiring (see Kconfig, Makefile in this directory) and
 *    drivers/gpu/drm/Kconfig source line.
 *  - tcc803x_drm_vioc.c implements the VIOC CRTC/plane as a real component
 *    (telechips,tcc803x-vioc-crtc, component_add()/component_del() in
 *    probe/remove) and will bind successfully against this file's match set,
 *    which now lists only &vioc_crtc0 (see tcc803x-avn.dts).
 *  - tcc803x-dw-hdmi.c implements the HDMI side as a DRM bridge
 *    (dw_hdmi_probe(), drm_bridge_add()) and is deliberately NOT a
 *    component match here: tcc803x_drm_vioc.c's ->bind() instead resolves
 *    it via the vioc_crtc0/hdmi_tx OF-graph "port"/"endpoint" pair in
 *    tcc803x.dtsi (devm_drm_of_get_bridge()) and drm_bridge_attach()s it to
 *    its own CRTC-owned encoder - this closes the hang this file used to
 *    describe here: component_bind_all() was waiting on a &hdmi_tx
 *    component match that a plain drm_bridge platform_driver could never
 *    satisfy, so tcc803x_drm_platform_probe() stayed deferred forever and
 *    nothing downstream (DISP0 timing programming, fbcon) ever ran.
 *
 * NOT DONE / BLOCKED:
 *  - mode_config min/max width/height are placeholders, not derived from
 *    real VIOC DISP timing-generator limits.
 *  - No IOMMU/VIOC-address-space wiring (tcc_drm_iommu.c in the vendor tree
 *    has an equivalent) - out of scope until GEM buffers need to be mapped
 *    into whatever MMU sits in front of the VIOC RDMA/WDMA blocks, if any
 *    exists on TCC803x's VIOC generation (not yet confirmed).
 *
 * Bottom line: the CRTC side is real and binds, and now pulls the HDMI
 * bridge in directly instead of waiting on an unsatisfiable component
 * match, so component_bind_all() can actually complete.
 * ===========================================================================
 */
