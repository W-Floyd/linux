// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2024 Otto Pflüger
// Based on command information from the vendor device tree

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/reset.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

/*
 * The ICNL9916 is a TDDI controller: the same part drives the panel over DSI
 * and reports touch over SPI (drivers/input/touchscreen/chipone_icnl9916.c).
 * Register layout is common to the family, but the values are tuned per panel
 * module, so each module needs its own command sequence and timings.
 */
struct icnl9916_panel_desc {
	const struct drm_display_mode *mode;
	unsigned long mode_flags;
	unsigned long hs_rate;
	unsigned long lp_rate;
	int (*on)(struct mipi_dsi_device *dsi);
	int (*off)(struct mipi_dsi_device *dsi);
	void (*reset)(struct reset_control *reset);
	/* Brightness is set with DCS rather than by a separate backlight. */
	bool dcs_backlight;
};

struct icnl9916_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct reset_control *reset;
	const struct icnl9916_panel_desc *desc;
};

static inline
struct icnl9916_panel *to_icnl9916_panel(struct drm_panel *panel)
{
	return container_of(panel, struct icnl9916_panel, panel);
}

static void icnl9916_panel_reset(struct reset_control *reset)
{
	reset_control_deassert(reset);
	usleep_range(10000, 11000);
	reset_control_assert(reset);
	usleep_range(10000, 11000);
	reset_control_deassert(reset);
	msleep(120);
}

/* Vendor reset for the Tianma module: 2 ms asserted, 10 ms to settle. */
static void icnl9916c_tm_panel_reset(struct reset_control *reset)
{
	reset_control_assert(reset);
	usleep_range(2000, 3000);
	reset_control_deassert(reset);
	usleep_range(10000, 11000);
}

static int icnl9916_panel_on(struct mipi_dsi_device *dsi)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x99, 0x16, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc1,
				     0x00, 0x20, 0x20, 0xb4, 0x04, 0x30, 0x30,
				     0x04, 0x40, 0x06, 0x22, 0x70, 0x33, 0x31,
				     0x07, 0x11, 0x84, 0x4c, 0x00, 0x93, 0x13,
				     0x63, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc2, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3,
				     0x06, 0x00, 0xff, 0x00, 0xff, 0x4d, 0x10,
				     0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc4,
				     0x04, 0x33, 0xb8, 0x40, 0x00, 0xbc, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0xf0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5,
				     0x03, 0x21, 0x96, 0xc8, 0x3e, 0x00, 0x04,
				     0x01, 0x14, 0x04, 0x0e, 0x18, 0xc6, 0x03,
				     0x64, 0xff, 0x01, 0x04, 0x18, 0x22, 0x45,
				     0x14, 0x38);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc6,
				     0x72, 0x24, 0x13, 0x2b, 0x2b, 0x28, 0x3f,
				     0x02, 0x16, 0x16, 0x00, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xca,
				     0x34, 0x50, 0x04, 0x19, 0x46, 0x94, 0x41,
				     0x8f, 0x44, 0x44, 0x36, 0x50, 0x54, 0x54,
				     0x39, 0x5a, 0x5a, 0x5a, 0x33, 0x00, 0x01,
				     0x01, 0x0e, 0x3f, 0xd2, 0x00, 0x05, 0x00,
				     0x00, 0x5a, 0x5a, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb2,
				     0x05, 0x04, 0x10, 0x10, 0x44, 0x44, 0x82,
				     0x88, 0x44, 0x86, 0x84, 0x86, 0x84, 0x86,
				     0x84, 0x86, 0x84, 0x86, 0x84, 0x86, 0x84,
				     0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb3,
				     0xf4, 0x01, 0x01, 0x11, 0x91, 0x86, 0x00,
				     0x00, 0x84, 0x00, 0x00, 0x65, 0x4a, 0x65,
				     0x4a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb4,
				     0x19, 0x0b, 0x06, 0x0b, 0x06, 0x26, 0x26,
				     0x88, 0xa2, 0x88, 0x44, 0x3b, 0x26, 0x00,
				     0x55, 0x3c, 0x02, 0x08, 0x20, 0x30, 0x00,
				     0x12, 0x20, 0x40, 0x11, 0x10, 0x20, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb5,
				     0x00, 0x00, 0x08, 0x04, 0x2e, 0x2f, 0x0c,
				     0x0e, 0x10, 0x12, 0x14, 0x16, 0x18, 0x1a,
				     0x28, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
				     0x02, 0xff, 0xff, 0xfc, 0x0c, 0x00, 0x00,
				     0x3c, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb6,
				     0x00, 0x00, 0x09, 0x05, 0x2e, 0x2f, 0x0d,
				     0x0f, 0x11, 0x13, 0x15, 0x17, 0x19, 0x1b,
				     0x29, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
				     0x02, 0xff, 0xff, 0xfc, 0x0c, 0x00, 0x00,
				     0x3c, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0,
				     0x40, 0x93, 0xff, 0xff, 0xff, 0x3f, 0xff,
				     0x00, 0xff, 0x00, 0xcc, 0xb1, 0x23, 0x45,
				     0x67, 0x89, 0xad, 0xff, 0xff, 0xf0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb7,
				     0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a,
				     0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a,
				     0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a,
				     0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbc,
				     0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0xff,
				     0xf0, 0x0b, 0x33, 0x5c, 0x5b, 0x43, 0x33,
				     0x00, 0x5a, 0x5a, 0x55, 0x55);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbd, 0xa1, 0x0a, 0x52, 0xa6);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbf,
				     0x0c, 0x19, 0x0c, 0x19, 0x00, 0x11, 0x04,
				     0x18, 0x50);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc7,
				     0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77,
				     0x77, 0x70, 0x77, 0x77, 0x77, 0x77, 0x77,
				     0x77, 0x77, 0x77, 0x70, 0x31, 0x00, 0x01,
				     0xff, 0xff, 0x00, 0x8a, 0x8a, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x80,
				     0xff, 0xfa, 0xf3, 0xec, 0xe6, 0xe0, 0xdb,
				     0xd7, 0xd3, 0xc5, 0xba, 0xb0, 0xa8, 0xa1,
				     0x9a, 0x8f, 0x85, 0x7b, 0x73, 0x72, 0x6a,
				     0x61, 0x58, 0x4e, 0x42, 0x32, 0x28, 0x1b,
				     0x17, 0x13, 0x0f, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x81,
				     0xff, 0xfa, 0xf3, 0xec, 0xe6, 0xe0, 0xdb,
				     0xd7, 0xd3, 0xc5, 0xba, 0xb0, 0xa8, 0xa1,
				     0x9a, 0x8f, 0x85, 0x7b, 0x73, 0x72, 0x6a,
				     0x61, 0x58, 0x4e, 0x42, 0x32, 0x28, 0x1b,
				     0x17, 0x13, 0x0f, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x82,
				     0xff, 0xfa, 0xf3, 0xec, 0xe6, 0xe0, 0xdb,
				     0xd7, 0xd3, 0xc5, 0xba, 0xb0, 0xa8, 0xa1,
				     0x9a, 0x8f, 0x85, 0x7b, 0x73, 0x72, 0x6a,
				     0x61, 0x58, 0x4e, 0x42, 0x32, 0x28, 0x1b,
				     0x17, 0x13, 0x0f, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x83,
				     0x01, 0x06, 0x02, 0x00, 0x00, 0x06, 0x02,
				     0x00, 0x00, 0x06, 0x02, 0x00, 0x00, 0x0a,
				     0x06, 0x02, 0x00, 0x0a, 0x06, 0x02, 0x00,
				     0x0a, 0x06, 0x02, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x84,
				     0x30, 0x3d, 0x13, 0x98, 0x32, 0x69, 0x01,
				     0xa7, 0x64, 0x30, 0x3d, 0x13, 0x98, 0x32,
				     0x69, 0x01, 0xa7, 0x64, 0x30, 0x3d, 0x13,
				     0x98, 0x32, 0x69, 0x01, 0xa7, 0x64);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x89, 0xfe, 0xfe, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x35, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x11, 0x00);
	mipi_dsi_msleep(&dsi_ctx, 120);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x29, 0x00);
	mipi_dsi_usleep_range(&dsi_ctx, 10000, 11000);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbd, 0xa1, 0x0a, 0x52, 0xae);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x6d, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x00, 0x00, 0x00);

	return dsi_ctx.accum_err;
}

static int icnl9916_panel_off(struct mipi_dsi_device *dsi)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };

	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x28);
	mipi_dsi_msleep(&dsi_ctx, 50);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x10);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

/*
 * Tianma 6.52" module, as fitted to the Motorola moto g play (2024).
 * Transcribed from qcom,mdss-dsi-on-command/off-command in the vendor
 * device tree (dsi-panel-mot-tm-icnl9916c-652-720x1600-vid.dtsi).
 */
static int icnl9916c_tm_panel_on(struct mipi_dsi_device *dsi)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };

	/* Unlock the vendor command pages. */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x99, 0x16, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc1,
				     0x11, 0x20, 0x2c, 0x2c, 0x04, 0x30, 0x30,
				     0x04, 0x40, 0x06, 0x22, 0x70, 0x35, 0x21,
				     0x07, 0x11, 0x84, 0x4c, 0x00, 0x93);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc8, 0x46, 0x00, 0x88, 0x98);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbd, 0xa1, 0x0a, 0x52, 0xa6);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x35, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x53, 0x2c, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x55, 0x03, 0x00);
	mipi_dsi_msleep(&dsi_ctx, 2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe0,
				     0x0c, 0x00, 0xb0, 0x0c, 0x00, 0x0a, 0x8c,
				     0x29, 0x04, 0x81, 0x1f, 0x00, 0x00, 0x00,
				     0x00, 0x24, 0x12, 0x08, 0x11, 0x48, 0x47,
				     0x00, 0x00);
	/* Gamma. */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe1,
				     0x0f, 0x1f, 0x2f, 0x3f, 0x4f, 0x5f, 0x6f,
				     0x7f, 0x90, 0xa0, 0xb0, 0xc0, 0xd0, 0xe0,
				     0xe8, 0xec, 0xed, 0xef, 0xf1, 0xf3, 0xf5,
				     0xf7, 0xf9, 0xfb, 0xff, 0xfc, 0xfd, 0xfe,
				     0xff, 0xff, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe2,
				     0x36, 0x24, 0x37, 0x4b, 0x38, 0x72, 0x39,
				     0x99, 0x3a, 0x1c, 0x3a, 0x9f, 0x3b, 0x22,
				     0x3b, 0xa5, 0x3c, 0x28, 0x3f, 0x50, 0x3d,
				     0xb1, 0x3e, 0x76, 0x3f, 0x3a, 0x3f, 0xff,
				     0x3f, 0xff, 0x3f, 0xff);

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 100);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6d, 0x02, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbd, 0xa1, 0x0a, 0x52, 0xae);
	/* Lock the vendor command pages again. */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x00, 0x00, 0x00);

	return dsi_ctx.accum_err;
}

static int icnl9916c_tm_panel_off(struct mipi_dsi_device *dsi)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6d, 0x25, 0x00);
	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 10);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 100);

	return dsi_ctx.accum_err;
}

static int icnl9916_panel_prepare(struct drm_panel *panel)
{
	struct icnl9916_panel *ctx = to_icnl9916_panel(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ctx->desc->reset(ctx->reset);

	ret = ctx->desc->on(ctx->dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		reset_control_assert(ctx->reset);
		return ret;
	}

	return 0;
}

static int icnl9916_panel_unprepare(struct drm_panel *panel)
{
	struct icnl9916_panel *ctx = to_icnl9916_panel(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = ctx->desc->off(ctx->dsi);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	reset_control_assert(ctx->reset);

	return 0;
}

static const struct drm_display_mode icnl9916_panel_mode = {
	.clock = 96000,
	.hdisplay = 720,
	.hsync_start = 720 + 70,
	.hsync_end = 720 + 70 + 4,
	.htotal = 720 + 70 + 4 + 80,
	.vdisplay = 1600,
	.vsync_start = 1600 + 180,
	.vsync_end = 1600 + 180 + 4,
	.vtotal = 1600 + 180 + 4 + 32,
	.width_mm = 68,
	.height_mm = 151,
	.type = DRM_MODE_TYPE_DRIVER,
};

/*
 * The Tianma module runs at a fixed 60 Hz with an unusually long vertical
 * front porch, which is what puts the pixel clock at 143 MHz rather than the
 * ~80 MHz this resolution would otherwise need: 820 * 2904 * 60.
 */
static const struct drm_display_mode icnl9916c_tm_panel_mode = {
	.clock = 142877,
	.hdisplay = 720,
	.hsync_start = 720 + 48,
	.hsync_end = 720 + 48 + 4,
	.htotal = 720 + 48 + 4 + 48,
	.vdisplay = 1600,
	.vsync_start = 1600 + 1268,
	.vsync_end = 1600 + 1268 + 4,
	.vtotal = 1600 + 1268 + 4 + 32,
	.width_mm = 70,
	.height_mm = 156,
	.type = DRM_MODE_TYPE_DRIVER,
};

static const struct icnl9916_panel_desc icnl9916_panel_desc = {
	.mode = &icnl9916_panel_mode,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
		      MIPI_DSI_MODE_VIDEO_HSE,
	.hs_rate = 691000000,
	.lp_rate = 20000000,
	.on = icnl9916_panel_on,
	.off = icnl9916_panel_off,
	.reset = icnl9916_panel_reset,
};

/*
 * Non-burst mode with sync events, and brightness over DCS -- the vendor
 * device tree asks for "non_burst_sync_event" and "bl_ctrl_dcs".  The command
 * sequences run in low-power mode ("dsi_lp_mode").
 */
static const struct icnl9916_panel_desc icnl9916c_tm_panel_desc = {
	.mode = &icnl9916c_tm_panel_mode,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM,
	.on = icnl9916c_tm_panel_on,
	.off = icnl9916c_tm_panel_off,
	.reset = icnl9916c_tm_panel_reset,
	.dcs_backlight = true,
};

static int icnl9916_panel_get_modes(struct drm_panel *panel,
				       struct drm_connector *connector)
{
	struct icnl9916_panel *ctx = to_icnl9916_panel(panel);

	return drm_connector_helper_get_modes_fixed(connector, ctx->desc->mode);
}

static int icnl9916_panel_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	ret = mipi_dsi_dcs_set_display_brightness(dsi, brightness);
	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return ret < 0 ? ret : 0;
}

static const struct backlight_ops icnl9916_panel_bl_ops = {
	.update_status = icnl9916_panel_bl_update_status,
};

static int icnl9916_panel_dcs_backlight(struct icnl9916_panel *ctx)
{
	struct device *dev = &ctx->dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 255,
		.max_brightness = 255,
	};

	ctx->panel.backlight = devm_backlight_device_register(dev,
							      dev_name(dev),
							      dev, ctx->dsi,
							      &icnl9916_panel_bl_ops,
							      &props);

	return PTR_ERR_OR_ZERO(ctx->panel.backlight);
}

static const struct drm_panel_funcs icnl9916_panel_panel_funcs = {
	.prepare = icnl9916_panel_prepare,
	.unprepare = icnl9916_panel_unprepare,
	.get_modes = icnl9916_panel_get_modes,
};

static int icnl9916_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct icnl9916_panel *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, __typeof(*ctx), panel,
				   &icnl9916_panel_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->reset = devm_reset_control_get_shared(dev, NULL);
	if (IS_ERR(ctx->reset))
		return dev_err_probe(dev, PTR_ERR(ctx->reset),
				     "Failed to get chip reset\n");

	ctx->desc = of_device_get_match_data(dev);
	if (!ctx->desc)
		return -ENODEV;

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = ctx->desc->mode_flags;
	dsi->hs_rate = ctx->desc->hs_rate;
	dsi->lp_rate = ctx->desc->lp_rate;

	ctx->panel.prepare_prev_first = true;

	if (ctx->desc->dcs_backlight)
		ret = icnl9916_panel_dcs_backlight(ctx);
	else
		ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void icnl9916_panel_remove(struct mipi_dsi_device *dsi)
{
	struct icnl9916_panel *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id icnl9916_panel_of_match[] = {
	{ .compatible = "chipone,icnl9916-720x1600-panel",
	  .data = &icnl9916_panel_desc },
	{ .compatible = "tianma,icnl9916c-720x1600",
	  .data = &icnl9916c_tm_panel_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, icnl9916_panel_of_match);

static struct mipi_dsi_driver icnl9916_panel_driver = {
	.probe = icnl9916_panel_probe,
	.remove = icnl9916_panel_remove,
	.driver = {
		.name = "panel-icnl9916-720x1600",
		.of_match_table = icnl9916_panel_of_match,
	},
};
module_mipi_dsi_driver(icnl9916_panel_driver);

MODULE_AUTHOR("Otto Pflüger <otto.pflueger@abscue.de>");
MODULE_DESCRIPTION("DRM driver for ICNL9916 720x1600 video mode DSI panel");
MODULE_LICENSE("GPL");
