// SPDX-License-Identifier: GPL-2.0-only
//
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//	Copyright (c) 2013, The Linux Foundation. All rights reserved.
// Copyright (c) 2024 Dzmitry Sankouski <dsankouski@gmail.com>

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_panel.h>

#include "s6e3ha8_dimming.h"
#include "s6e3ha8_candela_map.h"
#include "s6e3ha8_aid_elvss.h"

struct s6e3ha8 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data *supplies;

	/* Smart dimming: gamma is computed from calibration data read out of
	 * the panel, so it can only be set up once the panel is powered and
	 * out of sleep. Done on the first enable. */
	struct smartdim_conf *smartdim;
	bool dimming_ready;
};

/* Calibration data lives in MTP, register 0xc8. mtp_sorting() consumes 34
 * bytes of it. Reading needs the level 1 test key. */
#define S6E3HA8_MTP_REG		0xc8
#define S6E3HA8_MTP_LEN		34

/*
 * Getting the gamma sequence wrong on this panel does not merely look bad -- it
 * can leave the display wedged, and because this driver is normally built into
 * the kernel that persists across reboots. Brightness is therefore only armed
 * once calibration has been read back and sanity checked, and s6e3ha8.backlight=0
 * turns it off entirely if a unit still misbehaves.
 */
static bool s6e3ha8_backlight_enable = true;
module_param_named(backlight, s6e3ha8_backlight_enable, bool, 0444);
MODULE_PARM_DESC(backlight, "Enable brightness control (default on)");

static const struct regulator_bulk_data s6e3ha8_supplies[] = {
	{ .supply = "vdd3" },
	{ .supply = "vci" },
	{ .supply = "vddr" },
};

static inline
struct s6e3ha8 *to_s6e3ha8_amb577px01_wqhd(struct drm_panel *panel)
{
	return container_of(panel, struct s6e3ha8, panel);
}

#define s6e3ha8_test_key_on_lvl2(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf0, 0x5a, 0x5a)
#define s6e3ha8_test_key_off_lvl2(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf0, 0xa5, 0xa5)
#define s6e3ha8_test_key_on_lvl3(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0xfc, 0x5a, 0x5a)
#define s6e3ha8_test_key_off_lvl3(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0xfc, 0xa5, 0xa5)
#define s6e3ha8_test_key_on_lvl1(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0x9f, 0xa5, 0xa5)
#define s6e3ha8_test_key_off_lvl1(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0x9f, 0x5a, 0x5a)
#define s6e3ha8_afc_off(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0xe2, 0x00, 0x00)

static void s6e3ha8_amb577px01_wqhd_reset(struct s6e3ha8 *priv)
{
	gpiod_set_value_cansleep(priv->reset_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(priv->reset_gpio, 0);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(priv->reset_gpio, 1);
	usleep_range(5000, 6000);
}

static int s6e3ha8_amb577px01_wqhd_on(struct s6e3ha8 *priv)
{
	struct mipi_dsi_device *dsi = priv->dsi;
	struct mipi_dsi_multi_context ctx = { .dsi = dsi };

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	s6e3ha8_test_key_on_lvl1(&ctx);

	s6e3ha8_test_key_on_lvl2(&ctx);
	mipi_dsi_compression_mode_multi(&ctx, true);
	s6e3ha8_test_key_off_lvl2(&ctx);

	mipi_dsi_dcs_exit_sleep_mode_multi(&ctx);
	usleep_range(5000, 6000);

	s6e3ha8_test_key_on_lvl2(&ctx);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf2, 0x13);
	s6e3ha8_test_key_off_lvl2(&ctx);
	usleep_range(10000, 11000);

	s6e3ha8_test_key_on_lvl2(&ctx);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf2, 0x13);
	s6e3ha8_test_key_off_lvl2(&ctx);

	/* OMOK setting 1 (Initial setting) - Scaler Latch Setting Guide */
	s6e3ha8_test_key_on_lvl2(&ctx);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb0, 0x07);
	/* latch setting 1 : Scaler on/off & address setting & PPS setting -> Image update latch */
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf2, 0x3c, 0x10);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb0, 0x0b);
	/* latch setting 2 : Ratio change mode -> Image update latch */
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf2, 0x30);
	/* OMOK setting 2 - Seamless setting guide : WQHD */
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x2a, 0x00, 0x00, 0x05, 0x9f); /* CASET */
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x2b, 0x00, 0x00, 0x0b, 0x8f); /* PASET */
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xba, 0x01); /* scaler setup : scaler off */
	s6e3ha8_test_key_off_lvl2(&ctx);

	mipi_dsi_dcs_write_seq_multi(&ctx, 0x35, 0x00); /* TE Vsync ON */

	s6e3ha8_test_key_on_lvl2(&ctx);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xed, 0x4c); /* ERR_FG */
	s6e3ha8_test_key_off_lvl2(&ctx);

	s6e3ha8_test_key_on_lvl3(&ctx);
	/* FFC Setting 897.6Mbps */
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xc5, 0x0d, 0x10, 0xb4, 0x3e, 0x01);
	s6e3ha8_test_key_off_lvl3(&ctx);

	s6e3ha8_test_key_on_lvl2(&ctx);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb9,
				   0x00, 0xb0, 0x81, 0x09, 0x00, 0x00, 0x00,
				   0x11, 0x03); /* TSP HSYNC Setting */
	s6e3ha8_test_key_off_lvl2(&ctx);

	s6e3ha8_test_key_on_lvl2(&ctx);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb0, 0x03);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf6, 0x43);
	s6e3ha8_test_key_off_lvl2(&ctx);

	s6e3ha8_test_key_on_lvl2(&ctx);
	/* Brightness condition set */
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xca,
				   0x07, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80,
				   0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
				   0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
				   0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
				   0x80, 0x80, 0x80, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb1, 0x00, 0x0c); /* AID Set : 0% */
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb5,
				   0x19, 0xdc, 0x16, 0x01, 0x34, 0x67, 0x9a,
				   0xcd, 0x01, 0x22, 0x33, 0x44, 0x00, 0x00,
				   0x05, 0x55, 0xcc, 0x0c, 0x01, 0x11, 0x11,
				   0x10); /* MPS/ELVSS Setting */
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf4, 0xeb, 0x28); /* VINT */
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf7, 0x03); /* Gamma, LTPS(AID) update */
	s6e3ha8_test_key_off_lvl2(&ctx);

	s6e3ha8_test_key_off_lvl1(&ctx);

	return ctx.accum_err;
}


/* Platform brightness -> the candela value whose gamma we generate. */
static int s6e3ha8_brightness_to_candela(unsigned int brightness)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(s6e3ha8_candela_map); i++)
		if (brightness >= s6e3ha8_candela_map[i].from &&
		    brightness <= s6e3ha8_candela_map[i].till)
			return s6e3ha8_candela_map[i].candela;

	return s6e3ha8_candela_map[ARRAY_SIZE(s6e3ha8_candela_map) - 1].candela;
}

static const struct s6e3ha8_aid_elvss *s6e3ha8_aid_elvss_for(int candela)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(s6e3ha8_aid_elvss_table); i++)
		if (s6e3ha8_aid_elvss_table[i].candela == candela)
			return &s6e3ha8_aid_elvss_table[i];

	return NULL;
}

static int s6e3ha8_bl_update_status(struct backlight_device *bl)
{
	struct s6e3ha8 *priv = bl_get_data(bl);
	struct mipi_dsi_multi_context ctx = { .dsi = priv->dsi };
	const struct s6e3ha8_aid_elvss *set;
	u8 elvss[ARRAY_SIZE(s6e3ha8_elvss_payload)];
	u8 gamma[S6E3HA8_GAMMA_LEN + 1];
	u8 aid[3];
	int candela;

	if (!priv->dimming_ready)
		return -ENODEV;

	candela = s6e3ha8_brightness_to_candela(backlight_get_brightness(bl));
	set = s6e3ha8_aid_elvss_for(candela);
	if (!set) {
		dev_err(&priv->dsi->dev, "no AID/ELVSS for %d cd\n", candela);
		return -EINVAL;
	}

	/* generate_gamma() writes the 0xca payload; byte 0 is the command */
	gamma[0] = 0xca;
	priv->smartdim->generate_gamma(priv->smartdim, candela, &gamma[1]);

	aid[0] = 0xb1;
	aid[1] = set->aid[0];
	aid[2] = set->aid[1];

	memcpy(elvss, s6e3ha8_elvss_payload, sizeof(elvss));
	elvss[S6E3HA8_ELVSS_VAR_INDEX] = set->elvss;

	/*
	 * Gamma sets the transfer curve, but on this panel it does not set the
	 * luminance by itself: below the AID transition the brightness comes
	 * from the AID duty, and each gamma table is only correct at its
	 * matching AID/ELVSS operating point. All four have to go together,
	 * then be latched with 0xf7. Everything here needs the level 2 key.
	 */
	s6e3ha8_test_key_on_lvl2(&ctx);
	mipi_dsi_dcs_write_buffer_multi(&ctx, gamma, sizeof(gamma));
	mipi_dsi_dcs_write_buffer_multi(&ctx, aid, sizeof(aid));
	mipi_dsi_dcs_write_buffer_multi(&ctx, elvss, sizeof(elvss));
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf7, 0x03);
	s6e3ha8_test_key_off_lvl2(&ctx);

	return ctx.accum_err;
}
static const struct backlight_ops s6e3ha8_bl_ops = {
	.update_status = s6e3ha8_bl_update_status,
};

/*
 * Read the panel's own gamma calibration (MTP) and build the dimming tables
 * from it. Panel-specific, so it can only run once the panel is up; done once
 * on the first enable. Failure is not fatal -- the panel still displays, it
 * just has no brightness control.
 */
static void s6e3ha8_dimming_setup(struct s6e3ha8 *priv)
{
	struct mipi_dsi_device *dsi = priv->dsi;
	struct mipi_dsi_multi_context ctx = { .dsi = dsi };
	unsigned long mode_flags = dsi->mode_flags;
	u8 power_mode;
	int ret;

	if (priv->dimming_ready || !priv->smartdim)
		return;

	/*
	 * MTP is behind the 0xf0 key. Note the vendor driver calls that one
	 * "level 1" while the macros here number 0x9f as level 1, so this is
	 * deliberately lvl2 and not lvl1.
	 */
	s6e3ha8_test_key_on_lvl2(&ctx);
	if (ctx.accum_err)
		return;

	/* Reads have to be in low power mode. */
	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	/*
	 * The first read after panel init always comes back empty: the transfer
	 * reports success but the host's read-back registers are still clear.
	 * Spend that read on the power mode, which is worth checking anyway,
	 * and take the calibration with the second one.
	 *
	 * Note there is no set_maximum_return_packet_size() here on purpose --
	 * msm_dsi_host_cmd_rx() issues its own, chunking the read internally,
	 * and a second one from this side is redundant.
	 */
	ret = mipi_dsi_dcs_get_power_mode(dsi, &power_mode);
	if (ret < 0)
		dev_warn(&dsi->dev, "failed to read power mode: %d\n", ret);
	else
		dev_dbg(&dsi->dev, "panel power mode 0x%02x\n", power_mode);

	ret = mipi_dsi_dcs_read(dsi, S6E3HA8_MTP_REG, priv->smartdim->mtp_buffer,
				S6E3HA8_MTP_LEN);

	/*
	 * Restore rather than clear: the init sequence deliberately leaves LPM
	 * set so that later command traffic, brightness included, goes out in
	 * low power mode. Clearing it here sends those bursts in high speed
	 * mode interleaved with active video, which briefly scrambles the
	 * image on every brightness change.
	 */
	dsi->mode_flags = mode_flags;
	s6e3ha8_test_key_off_lvl2(&ctx);

	if (ret != S6E3HA8_MTP_LEN) {
		dev_warn(&dsi->dev, "MTP read returned %d, brightness disabled\n",
			 ret);
		return;
	}

	print_hex_dump(KERN_INFO, "s6e3ha8 mtp: ", DUMP_PREFIX_OFFSET, 16, 1,
		       priv->smartdim->mtp_buffer, S6E3HA8_MTP_LEN, false);

	/*
	 * A blank buffer means the read silently failed. Gamma generated from
	 * it would be garbage, and writing garbage gamma wedges this panel, so
	 * refuse rather than carry on.
	 */
	if (!memchr_inv(priv->smartdim->mtp_buffer, 0x00, S6E3HA8_MTP_LEN) ||
	    !memchr_inv(priv->smartdim->mtp_buffer, 0xff, S6E3HA8_MTP_LEN)) {
		dev_warn(&dsi->dev,
			 "MTP is blank, calibration unusable; brightness disabled\n");
		return;
	}

	priv->smartdim->init(priv->smartdim);
	priv->dimming_ready = true;
	dev_info(&dsi->dev, "smart dimming ready\n");

	if (s6e3ha8_backlight_enable && !priv->panel.backlight) {
		const struct backlight_properties props = {
			.type = BACKLIGHT_RAW,
			.brightness = 128,
			.max_brightness = 255,
		};

		priv->panel.backlight = devm_backlight_device_register(&dsi->dev,
				dev_name(&dsi->dev), &dsi->dev, priv,
				&s6e3ha8_bl_ops, &props);
		if (IS_ERR(priv->panel.backlight)) {
			dev_warn(&dsi->dev, "failed to register backlight: %pe\n",
				 priv->panel.backlight);
			priv->panel.backlight = NULL;
		}
	}
}

static int s6e3ha8_enable(struct drm_panel *panel)
{
	struct s6e3ha8 *priv = to_s6e3ha8_amb577px01_wqhd(panel);
	struct mipi_dsi_device *dsi = priv->dsi;
	struct mipi_dsi_multi_context ctx = { .dsi = dsi };

	s6e3ha8_test_key_on_lvl1(&ctx);
	mipi_dsi_dcs_set_display_on_multi(&ctx);
	s6e3ha8_test_key_off_lvl1(&ctx);

	return ctx.accum_err;
}

static int s6e3ha8_disable(struct drm_panel *panel)
{
	struct s6e3ha8 *priv = to_s6e3ha8_amb577px01_wqhd(panel);
	struct mipi_dsi_device *dsi = priv->dsi;
	struct mipi_dsi_multi_context ctx = { .dsi = dsi };

	s6e3ha8_test_key_on_lvl1(&ctx);
	mipi_dsi_dcs_set_display_off_multi(&ctx);
	s6e3ha8_test_key_off_lvl1(&ctx);
	mipi_dsi_msleep(&ctx, 20);

	s6e3ha8_test_key_on_lvl2(&ctx);
	s6e3ha8_afc_off(&ctx);
	s6e3ha8_test_key_off_lvl2(&ctx);

	/*
	 * Sleep in, not just display off. unprepare() releases the panel's
	 * supplies, but nothing guarantees they actually drop: on the S9 the
	 * bootloader framebuffer holds the same three rails, so the panel
	 * stays powered after the display is disabled. Display off alone
	 * leaves it driving a dim raster in that case; sleep in puts the
	 * driver IC down whether or not the rails go away.
	 */
	s6e3ha8_test_key_on_lvl1(&ctx);
	mipi_dsi_dcs_enter_sleep_mode_multi(&ctx);
	s6e3ha8_test_key_off_lvl1(&ctx);

	mipi_dsi_msleep(&ctx, 160);

	return ctx.accum_err;
}

static int s6e3ha8_amb577px01_wqhd_prepare(struct drm_panel *panel)
{
	struct s6e3ha8 *priv = to_s6e3ha8_amb577px01_wqhd(panel);
	struct mipi_dsi_device *dsi = priv->dsi;
	struct mipi_dsi_multi_context ctx = { .dsi = dsi };
	struct drm_dsc_picture_parameter_set pps;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(s6e3ha8_supplies), priv->supplies);
	if (ret < 0)
		return ret;
	mipi_dsi_msleep(&ctx, 120);
	s6e3ha8_amb577px01_wqhd_reset(priv);

	ret = s6e3ha8_amb577px01_wqhd_on(priv);
	if (ret < 0) {
		gpiod_set_value_cansleep(priv->reset_gpio, 1);
		goto err;
	}

	drm_dsc_pps_payload_pack(&pps, &priv->dsc);

	s6e3ha8_test_key_on_lvl1(&ctx);
	mipi_dsi_picture_parameter_set_multi(&ctx, &pps);
	s6e3ha8_test_key_off_lvl1(&ctx);

	mipi_dsi_msleep(&ctx, 28);

	/*
	 * Read calibration here rather than in enable(): the host cannot
	 * service DSI reads once the encoder has been enabled, and enable()
	 * runs after that.
	 */
	if (!ctx.accum_err)
		s6e3ha8_dimming_setup(priv);

	return ctx.accum_err;
err:
	regulator_bulk_disable(ARRAY_SIZE(s6e3ha8_supplies), priv->supplies);
	return ret;
}

static int s6e3ha8_amb577px01_wqhd_unprepare(struct drm_panel *panel)
{
	struct s6e3ha8 *priv = to_s6e3ha8_amb577px01_wqhd(panel);

	/*
	 * Hold the panel in reset before dropping the supplies. reset-gpios is
	 * described active high here and the reset pulse ends high, so zero is
	 * the asserted state.
	 */
	gpiod_set_value_cansleep(priv->reset_gpio, 0);

	return regulator_bulk_disable(ARRAY_SIZE(s6e3ha8_supplies), priv->supplies);
}

static const struct drm_display_mode s6e3ha8_amb577px01_wqhd_mode = {
	.clock = (1440 + 116 + 44 + 120) * (2960 + 120 + 80 + 124) * 60 / 1000,
	.hdisplay = 1440,
	.hsync_start = 1440 + 116,
	.hsync_end = 1440 + 116 + 44,
	.htotal = 1440 + 116 + 44 + 120,
	.vdisplay = 2960,
	.vsync_start = 2960 + 120,
	.vsync_end = 2960 + 120 + 80,
	.vtotal = 2960 + 120 + 80 + 124,
	.width_mm = 64,
	.height_mm = 132,
};

static int s6e3ha8_amb577px01_wqhd_get_modes(struct drm_panel *panel,
					     struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &s6e3ha8_amb577px01_wqhd_mode);
}

static const struct drm_panel_funcs s6e3ha8_amb577px01_wqhd_panel_funcs = {
	.prepare = s6e3ha8_amb577px01_wqhd_prepare,
	.unprepare = s6e3ha8_amb577px01_wqhd_unprepare,
	.get_modes = s6e3ha8_amb577px01_wqhd_get_modes,
	.enable = s6e3ha8_enable,
	.disable = s6e3ha8_disable,
};

static int s6e3ha8_amb577px01_wqhd_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct s6e3ha8 *priv;
	int ret;

	priv = devm_drm_panel_alloc(dev, struct s6e3ha8, panel,
				    &s6e3ha8_amb577px01_wqhd_panel_funcs,
				    DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(priv))
		return PTR_ERR(priv);

	ret = devm_regulator_bulk_get_const(dev, ARRAY_SIZE(s6e3ha8_supplies),
				      s6e3ha8_supplies,
				      &priv->supplies);
	if (ret < 0) {
		dev_err(dev, "failed to get regulators: %d\n", ret);
		return ret;
	}

	priv->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->reset_gpio),
				     "Failed to get reset-gpios\n");

	priv->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, priv);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	/*
	 * This is a command mode panel, driven off the TE pin -- the vendor
	 * describes it as "dsi_cmd_mode" with qcom,mdss-dsi-te-using-te-pin,
	 * and the init sequence turns TE on with DCS 0x35. The absence of
	 * MIPI_DSI_MODE_VIDEO below is therefore deliberate: adding it garbles
	 * the whole display. The MODE_VIDEO_NO_* flags only mean anything
	 * alongside MIPI_DSI_MODE_VIDEO, so they do nothing here, and the
	 * porch values in the mode above are not evidence otherwise -- the
	 * vendor carries the same ones for the timing engine.
	 */
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS |
		MIPI_DSI_MODE_VIDEO_NO_HFP | MIPI_DSI_MODE_VIDEO_NO_HBP |
		MIPI_DSI_MODE_VIDEO_NO_HSA | MIPI_DSI_MODE_NO_EOT_PACKET;

	priv->panel.prepare_prev_first = true;

	priv->smartdim = s6e3ha8_smartdim_get_conf();
	if (priv->smartdim) {
		priv->smartdim->lux_tab = s6e3ha8_lux_tab;
		priv->smartdim->lux_tabsize = ARRAY_SIZE(s6e3ha8_lux_tab);
		priv->smartdim->panel_revision = 'A';
	} else {
		dev_warn(dev, "no smart dimming, brightness unavailable\n");
	}

	drm_panel_add(&priv->panel);

	/* This panel only supports DSC; unconditionally enable it */
	dsi->dsc = &priv->dsc;

	priv->dsc.dsc_version_major = 1;
	priv->dsc.dsc_version_minor = 1;

	priv->dsc.slice_height = 40;
	priv->dsc.slice_width = 720;
	WARN_ON(1440 % priv->dsc.slice_width);
	priv->dsc.slice_count = 1440 / priv->dsc.slice_width;
	priv->dsc.bits_per_component = 8;
	priv->dsc.bits_per_pixel = 8 << 4; /* 4 fractional bits */
	priv->dsc.block_pred_enable = true;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
		drm_panel_remove(&priv->panel);
		return ret;
	}

	return 0;
}

static void s6e3ha8_amb577px01_wqhd_remove(struct mipi_dsi_device *dsi)
{
	struct s6e3ha8 *priv = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&priv->panel);
}

static const struct of_device_id s6e3ha8_amb577px01_wqhd_of_match[] = {
	{ .compatible = "samsung,s6e3ha8" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s6e3ha8_amb577px01_wqhd_of_match);

static struct mipi_dsi_driver s6e3ha8_amb577px01_wqhd_driver = {
	.probe = s6e3ha8_amb577px01_wqhd_probe,
	.remove = s6e3ha8_amb577px01_wqhd_remove,
	.driver = {
		.name = "panel-s6e3ha8",
		.of_match_table = s6e3ha8_amb577px01_wqhd_of_match,
	},
};
module_mipi_dsi_driver(s6e3ha8_amb577px01_wqhd_driver);

MODULE_AUTHOR("Dzmitry Sankouski <dsankouski@gmail.com>");
MODULE_DESCRIPTION("DRM driver for S6E3HA8 panel");
MODULE_LICENSE("GPL");
