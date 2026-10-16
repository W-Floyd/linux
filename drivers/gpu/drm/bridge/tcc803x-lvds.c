// SPDX-License-Identifier: GPL-2.0-only
/*
 * Telechips TCC803x on-die LVDS PHY/serializer bridge glue.
 *
 * SCOPE, honestly: this enables the one thing vendor-confirmed and
 * live-confirmed working -- the PERI_LVDS_PHY clock gate (see
 * Documentation/devicetree/bindings/display/telechips/
 * telechips,tcc803x-lvds.yaml for the live register read) -- and maps
 * the PHY's 4 register regions so a later pass has somewhere to write
 * real init code once its bit layout is known. It does NOT program
 * those registers: the vendor panel driver
 * (drivers/video/fbdev/tcc-fb/lcd_TM123XDHP90.c) only calls
 * clk_prepare_enable() on this clock, nothing lower-level, and no
 * other source in the extracted AVNT5W_TCC803XKK_kernel GPL package
 * touches this PHY's own registers. That init sequence most likely
 * lives in this board's bootloader package, not extracted here.
 *
 * Practical consequence: attaching this bridge almost certainly does
 * NOT produce a working display. What it does do is get the DRM
 * pipeline's bridge-attach/mode-set machinery actually exercising the
 * LVDS path for the first time, with a clear dev_info() marking
 * exactly how far it got -- so a kexec test with earlycon/USB-console
 * access narrows the problem (does probe succeed? does atomic_enable
 * run? does it hang before or after the clock comes up?) instead of
 * guessing blind.
 *
 * No drm_panel chaining: telechips,tcc803x-lvds-panel.yaml's DT node
 * documents the real, live-captured panel dimensions/timing, but no
 * drm_panel driver registers against that compatible string yet (same
 * "structural, no driver" gap telechips,tcc803x-mrgb-panel.yaml
 * already documents for its own panel node) -- chaining to it via
 * devm_drm_panel_bridge_add() would just make this driver probe-defer
 * forever against a driver that will never exist until someone writes
 * one. This bridge is instead terminal: it reports itself as type
 * DRM_MODE_CONNECTOR_LVDS and lets tcc803x_drm_vioc.c's existing
 * drm_bridge_connector_init() build a connector generically from the
 * bridge chain, the same mechanism already used for hdmi_tx.
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>

struct tcc803x_lvds {
	struct device *dev;
	struct drm_bridge bridge;
	struct clk *phy_clk;
	void __iomem *regs[4];
	bool clk_enabled;
};

static const char * const tcc803x_lvds_reg_names[4] = {
	"lvds0", "lvds1", "lvds-cfg0", "lvds-cfg1",
};

static inline struct tcc803x_lvds *to_tcc803x_lvds(struct drm_bridge *bridge)
{
	return container_of(bridge, struct tcc803x_lvds, bridge);
}

static int tcc803x_lvds_attach(struct drm_bridge *bridge,
				struct drm_encoder *encoder,
				enum drm_bridge_attach_flags flags)
{
	/* Terminal bridge -- nothing further in the chain, see file header. */
	return 0;
}

static enum drm_mode_status
tcc803x_lvds_mode_valid(struct drm_bridge *bridge,
			 const struct drm_display_info *info,
			 const struct drm_display_mode *mode)
{
	/*
	 * No PHY-side pixel-clock table exists to validate against (see
	 * file header) -- accept whatever the CRTC/panel-timing side
	 * already constrains, same as the gap tcc803x_drm_vioc.c's own
	 * atomic_enable() comment tracks for the pixel-clock divider.
	 */
	return MODE_OK;
}

static void tcc803x_lvds_atomic_enable(struct drm_bridge *bridge,
					struct drm_atomic_commit *state)
{
	struct tcc803x_lvds *lvds = to_tcc803x_lvds(bridge);
	int ret;

	ret = clk_prepare_enable(lvds->phy_clk);
	if (ret) {
		dev_err(lvds->dev,
			"PERI_LVDS_PHY clk_prepare_enable failed: %d\n", ret);
		return;
	}
	lvds->clk_enabled = true;

	/*
	 * This is the real, honest boundary of what this driver does.
	 * The PHY's own register regions (lvds->regs[0..3], mapped at
	 * probe) are deliberately left unwritten: no source in the
	 * extracted vendor package confirms their bit layout. A
	 * developer reading this on a console log now knows the clock
	 * path ran and PHY-level bring-up is the next real gap, not an
	 * oversight silently skipped.
	 */
	dev_info(lvds->dev,
		 "PERI_LVDS_PHY clock enabled; PHY register-level init (regs @ %p/%p/%p/%p) is NOT implemented -- vendor source has no lower-level sequence than clk_prepare_enable(), see driver header\n",
		 lvds->regs[0], lvds->regs[1], lvds->regs[2], lvds->regs[3]);
}

static void tcc803x_lvds_atomic_disable(struct drm_bridge *bridge,
					 struct drm_atomic_commit *state)
{
	struct tcc803x_lvds *lvds = to_tcc803x_lvds(bridge);

	if (lvds->clk_enabled) {
		clk_disable_unprepare(lvds->phy_clk);
		lvds->clk_enabled = false;
	}
}

static const struct drm_bridge_funcs tcc803x_lvds_bridge_funcs = {
	.attach			= tcc803x_lvds_attach,
	.mode_valid		= tcc803x_lvds_mode_valid,
	.atomic_enable		= tcc803x_lvds_atomic_enable,
	.atomic_disable		= tcc803x_lvds_atomic_disable,
	.atomic_duplicate_state	= drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_bridge_destroy_state,
	.atomic_create_state	= drm_atomic_helper_bridge_create_state,
};

static int tcc803x_lvds_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tcc803x_lvds *lvds;
	unsigned int i;

	lvds = devm_drm_bridge_alloc(dev, struct tcc803x_lvds, bridge,
				      &tcc803x_lvds_bridge_funcs);
	if (IS_ERR(lvds))
		return PTR_ERR(lvds);

	lvds->dev = dev;

	lvds->phy_clk = devm_clk_get(dev, "lvds-phy");
	if (IS_ERR(lvds->phy_clk))
		return dev_err_probe(dev, PTR_ERR(lvds->phy_clk),
				      "Unable to get \"lvds-phy\" clock\n");

	for (i = 0; i < ARRAY_SIZE(lvds->regs); i++) {
		lvds->regs[i] = devm_platform_ioremap_resource_byname(pdev,
							tcc803x_lvds_reg_names[i]);
		if (IS_ERR(lvds->regs[i]))
			return dev_err_probe(dev, PTR_ERR(lvds->regs[i]),
					      "Unable to map \"%s\" region\n",
					      tcc803x_lvds_reg_names[i]);
	}

	lvds->bridge.of_node = dev->of_node;
	lvds->bridge.type = DRM_MODE_CONNECTOR_LVDS;
	drm_bridge_add(&lvds->bridge);

	platform_set_drvdata(pdev, lvds);

	dev_info(dev, "probed: clock path real and vendor-confirmed, PHY register init NOT implemented (see driver header)\n");

	return 0;
}

static void tcc803x_lvds_remove(struct platform_device *pdev)
{
	struct tcc803x_lvds *lvds = platform_get_drvdata(pdev);

	drm_bridge_remove(&lvds->bridge);
}

static const struct of_device_id tcc803x_lvds_of_table[] = {
	{ .compatible = "telechips,tcc803x-lvds" },
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, tcc803x_lvds_of_table);

static struct platform_driver tcc803x_lvds_platform_driver = {
	.probe	= tcc803x_lvds_probe,
	.remove	= tcc803x_lvds_remove,
	.driver	= {
		.name		= "tcc803x-lvds",
		.of_match_table	= tcc803x_lvds_of_table,
	},
};
module_platform_driver(tcc803x_lvds_platform_driver);

MODULE_AUTHOR("William Floyd <git@notmy.space>");
MODULE_DESCRIPTION("Telechips TCC803x on-die LVDS PHY bridge glue");
MODULE_LICENSE("GPL");
