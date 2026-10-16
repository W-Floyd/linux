// SPDX-License-Identifier: GPL-2.0+
/*
 * Telechips TCC803x DesignWare HDMI TX glue.
 *
 * The TCC803x HDMI TX is a Synopsys DesignWare HDMI TX controller
 * (confirmed from vendor driver source: Synopsys copyright headers and
 * "dwc_hdmi_*"-prefixed functions throughout drivers/char/hdmi_v2_0/ in
 * Telechips' AVNT5W_TCC803XKK_kernel GPLv2 vendor tree). Its companion PHY,
 * unlike most dw-hdmi platforms, is not reached over the controller's
 * internal I2C PHY master -- it's a dedicated MMIO register window written
 * byte-at-a-time (see drivers/char/hdmi_v2_0/hdmi_api_lib/src/phy/
 * phy_803x.c in that tree, hdmi_phy_write()/hdmi_phy_read()). That is
 * reproduced here as dw-hdmi's "vendor PHY" mechanism (struct
 * dw_hdmi_phy_ops), the same approach drivers/gpu/drm/meson/meson_dw_hdmi.c
 * uses for Amlogic's non-I2C PHY.
 *
 * SCOPE: this is deliberately minimal -- PHY init and TMDS output enable
 * for a single, vendor-confirmed pixel clock (148.5 MHz / 1920x1080p60,
 * 8 bits/component), matching this board's fixed_video_id_code = 16 and
 * the dw-hdmi core's own mode negotiation, link training, EDID handling,
 * and AVI infoframe generation above this glue. No HDCP, no audio, no CEC.
 * The vendor's phy_803x.c carries ~100 further per-clock PHY register
 * tables (25.175 MHz through 594 MHz, 8/10/12 bpc each) that a fuller port
 * would need; only the one entry this board actually drives is reproduced
 * below, each byte cited against that table rather than invented.
 *
 * Clock inputs are NOT wired up: the vendor DT lists 8 CKC outputs for
 * this node (PERI_HDMI_CORE/AUDIO/PCLK/CEC/HDCP14/HDCP22, DDIBUS_HDMI,
 * ISOIP_DDB_HDMI), none of which exist yet as clock IDs in
 * include/dt-bindings/clock/telechips,tcc803x-clks.h or in
 * drivers/clk/telechips/clk-tcc803x-ckc.c. This driver assumes clocks are
 * already enabled by firmware/bootloader (true on this board today, since
 * LK brings up the display path before Linux boots) and does not call
 * clk_prepare_enable() on anything -- a real dependency on the CKC driver
 * gaining these IDs, not something to fake here.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <drm/bridge/dw_hdmi.h>
#include <drm/drm_modes.h>

/*
 * Byte-addressed PHY config register offsets, vendor
 * phy_803x.c:88-92 (tcc_phy_config_addr[]). Not contiguous: 0x22 and
 * 0x2E are skipped in the vendor table, reproduced faithfully here.
 */
#define TCC803X_HDMI_PHY_NUM_REGS	45

static const u8 tcc803x_hdmi_phy_reg_addr[TCC803X_HDMI_PHY_NUM_REGS] = {
	0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a,
	0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14,
	0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e,
	0x1f, 0x20, 0x21, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29,
	0x2a, 0x2b, 0x2c, 0x2d, 0x2f,
};

struct tcc803x_hdmi_phy_cfg {
	unsigned long mpixelclock;
	u8 reg[TCC803X_HDMI_PHY_NUM_REGS];
};

/*
 * Single vendor-confirmed entry: phy_803x.c:816-823 (148500000 Hz,
 * .pixel = 8, i.e. 8 bits/component -- this board's only configured mode,
 * 1920x1080p60 / fixed_video_id_code 16). The 10/12 bpc siblings at the
 * same clock (phy_803x.c:824-841) are not reproduced: nothing in this
 * board's DT requests deep color.
 */
static const struct tcc803x_hdmi_phy_cfg tcc803x_hdmi_phy_cfgs[] = {
	{
		.mpixelclock = 148500000,
		.reg = {
			0xd1, 0x7c, 0x3d, 0xb0, 0x98, 0x88, 0x42, 0x4f, 0x30,
			0x33, 0x65, 0x20, 0xa6, 0x24, 0x80, 0x6c, 0xf2, 0x67,
			0x00, 0x10, 0x83, 0x30, 0x3a, 0x74, 0x8f, 0x00, 0x00,
			0x08, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0xe0, 0x83, 0x0f, 0x3e, 0xf8, 0x00,
		},
	},
};

struct tcc803x_hdmi {
	struct dw_hdmi_plat_data plat_data;
	struct dw_hdmi *dw_hdmi;
	void __iomem *phy_regs;
	u32 ref_clock_source;
	u32 phy_type;
};

/* Vendor phy_803x.c:1335-1341 (hdmi_phy_write): byte write, word-aligned offset. */
static void tcc803x_hdmi_phy_write(struct tcc803x_hdmi *hdmi, u32 offset, u8 data)
{
	iowrite32(data, hdmi->phy_regs + (offset * 4));
}

static const struct tcc803x_hdmi_phy_cfg *
tcc803x_hdmi_find_phy_cfg(unsigned long mpixelclock)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(tcc803x_hdmi_phy_cfgs); i++)
		if (tcc803x_hdmi_phy_cfgs[i].mpixelclock == mpixelclock)
			return &tcc803x_hdmi_phy_cfgs[i];

	return NULL;
}

static enum drm_mode_status
tcc803x_hdmi_mode_valid(struct dw_hdmi *dw_hdmi, void *data,
			 const struct drm_display_info *info,
			 const struct drm_display_mode *mode)
{
	/*
	 * Only the one PHY table entry this glue carries is usable. Reject
	 * everything else rather than silently applying the wrong PHY
	 * settings -- see the file header on why the table is this sparse.
	 */
	if (!tcc803x_hdmi_find_phy_cfg(mode->clock * 1000))
		return MODE_CLOCK_RANGE;

	return MODE_OK;
}

static int tcc803x_hdmi_phy_init(struct dw_hdmi *dw_hdmi, void *data,
				  const struct drm_display_info *display,
				  const struct drm_display_mode *mode)
{
	struct tcc803x_hdmi *hdmi = data;
	const struct tcc803x_hdmi_phy_cfg *cfg;
	unsigned int i;

	cfg = tcc803x_hdmi_find_phy_cfg(mode->clock * 1000);
	if (!cfg)
		return -EINVAL;

	for (i = 0; i < TCC803X_HDMI_PHY_NUM_REGS; i++)
		tcc803x_hdmi_phy_write(hdmi, tcc803x_hdmi_phy_reg_addr[i],
					cfg->reg[i]);

	return 0;
}

static void tcc803x_hdmi_phy_disable(struct dw_hdmi *dw_hdmi, void *data)
{
	/*
	 * Vendor tcc_hdmi_phy_standby() (phy_803x.c:1620 ff) parks the PHY
	 * through dw-hdmi core registers (PHY_CONF0 power-down bits), not
	 * this MMIO window -- nothing further to do here on disable.
	 */
}

static const struct dw_hdmi_phy_ops tcc803x_hdmi_phy_ops = {
	.init = tcc803x_hdmi_phy_init,
	.disable = tcc803x_hdmi_phy_disable,
};

static int tcc803x_dw_hdmi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tcc803x_hdmi *hdmi;
	struct dw_hdmi_plat_data *plat_data;

	hdmi = devm_kzalloc(dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;

	plat_data = &hdmi->plat_data;

	hdmi->phy_regs = devm_platform_ioremap_resource_byname(pdev, "hdmi_phy");
	if (IS_ERR(hdmi->phy_regs))
		return dev_err_probe(dev, PTR_ERR(hdmi->phy_regs),
				      "Unable to map hdmi_phy region\n");

	/*
	 * Vendor board properties (hdmi_tx.c:171-177), kept as raw selectors
	 * pending real clock/PHY-variant plumbing -- see
	 * telechips,tcc803x-hdmi.yaml.
	 */
	of_property_read_u32(dev->of_node, "telechips,ref-clock-source",
			      &hdmi->ref_clock_source);
	of_property_read_u32(dev->of_node, "telechips,phy-type", &hdmi->phy_type);
	if (hdmi->phy_type != 0)
		return dev_err_probe(dev, -EINVAL,
				      "Only phy-type 0 (PHY 8980) is supported by this table\n");

	plat_data->output_port = 1;
	plat_data->mode_valid = tcc803x_hdmi_mode_valid;
	plat_data->phy_ops = &tcc803x_hdmi_phy_ops;
	plat_data->phy_name = "tcc803x_dw_hdmi_phy";
	plat_data->phy_data = hdmi;
	plat_data->priv_data = hdmi;

	hdmi->dw_hdmi = dw_hdmi_probe(pdev, plat_data);
	if (IS_ERR(hdmi->dw_hdmi))
		return PTR_ERR(hdmi->dw_hdmi);

	platform_set_drvdata(pdev, hdmi);

	return 0;
}

static void tcc803x_dw_hdmi_remove(struct platform_device *pdev)
{
	struct tcc803x_hdmi *hdmi = platform_get_drvdata(pdev);

	dw_hdmi_remove(hdmi->dw_hdmi);
}

static const struct of_device_id tcc803x_dw_hdmi_of_table[] = {
	{ .compatible = "telechips,tcc803x-hdmi" },
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, tcc803x_dw_hdmi_of_table);

static struct platform_driver tcc803x_dw_hdmi_platform_driver = {
	.probe	= tcc803x_dw_hdmi_probe,
	.remove	= tcc803x_dw_hdmi_remove,
	.driver	= {
		.name		= "tcc803x-dw-hdmi",
		.of_match_table	= tcc803x_dw_hdmi_of_table,
	},
};
module_platform_driver(tcc803x_dw_hdmi_platform_driver);

MODULE_DESCRIPTION("Telechips TCC803x DesignWare HDMI TX glue driver");
MODULE_LICENSE("GPL");
