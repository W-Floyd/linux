// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2026 William Floyd <git@notmy.space>
//
// Driver for the Samsung S5K5F1 monochrome IR image sensor.
//
// The S5K5F1 is the iris-recognition camera of the Galaxy S9 (SM-G9600). It
// has a 2400x2400 active array, sends 8-bit greyscale over MIPI CSI-2 D-PHY
// on two data lanes, and is paired with an IR illuminator rather than a
// flash. Its register map follows the same MIPI CCS layout as the other
// Samsung sensors, so the s5k3m5 driver is the closest relative.
//
// The register sequences, mode timings and power sequence were recovered
// from the Qualcomm CHI sensor module description shipped with the vendor
// camera stack (com.qti.sensormodule.2_s5k5f1sx.bin); the link frequency is
// derived from the pixel rate that blob records, not from a datasheet.

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define S5K5F1_LINK_FREQ_768MHZ		(768ULL * HZ_PER_MHZ)
#define S5K5F1_MCLK_FREQ_24MHZ		(24 * HZ_PER_MHZ)
#define S5K5F1_DATA_LANES		2
#define S5K5F1_BITS_PER_SAMPLE		8

#define S5K5F1_REG_CHIP_ID		CCI_REG16(0x0000)
#define S5K5F1_CHIP_ID			0x5061

/* Streaming is bit 8 of the 16-bit write, as on the other S5K sensors */
#define S5K5F1_REG_CTRL_MODE		CCI_REG16(0x0100)
#define S5K5F1_MODE_STREAMING		BIT(8)

#define S5K5F1_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5K5F1_EXPOSURE_MIN		8
#define S5K5F1_EXPOSURE_STEP		1
#define S5K5F1_EXPOSURE_MARGIN		4

/*
 * Analogue gain is a linear code with 0x20 as unity, i.e. the same 1/32
 * steps the rest of the S5K family uses. The vendor default of 0x2c is
 * 1.375x.
 */
#define S5K5F1_REG_AGAIN		CCI_REG16(0x0204)
#define S5K5F1_AGAIN_MIN		0x20
#define S5K5F1_AGAIN_MAX		0x200
#define S5K5F1_AGAIN_STEP		1
#define S5K5F1_AGAIN_DEFAULT		0x2c

#define S5K5F1_REG_VTS			CCI_REG16(0x0340)
#define S5K5F1_VTS_MAX			0xffff

#define S5K5F1_REG_TEST_PATTERN		CCI_REG16(0x0600)

/* Page pointer, software reset and the clock enables the vendor writes */
#define S5K5F1_REG_PAGE			CCI_REG16(0xfcfc)
#define S5K5F1_PAGE_DEFAULT		0x4000
#define S5K5F1_REG_SW_RESET		CCI_REG16(0x6010)
#define S5K5F1_SW_RESET			0x0001

/*
 * The vendor power sequence releases reset first and starts the clock
 * 4ms later, which is the opposite order from the rear camera on the same
 * board. Keep it: the sensor latches its I2C interface out of reset while
 * XCLR rises.
 */
#define S5K5F1_XCLR_TO_MCLK_US		4000
#define S5K5F1_MCLK_TO_I2C_US		10000
#define S5K5F1_DELAY_RANGE_US		1000

/* The sensor is monochrome, so there is no Bayer phase to get wrong */
#define S5K5F1_MBUS_CODE		MEDIA_BUS_FMT_Y8_1X8

#define to_s5k5f1(_sd)			container_of(_sd, struct s5k5f1, sd)

static const s64 s5k5f1_link_freq_menu[] = {
	S5K5F1_LINK_FREQ_768MHZ,
};

/*
 * Brings the sensor out of software reset and enables its internal clocks.
 * Run before the mode registers, exactly as the vendor blob's initSettings
 * descriptor does.
 */
static const struct cci_reg_sequence s5k5f1_init_regs[] = {
	{ S5K5F1_REG_PAGE, S5K5F1_PAGE_DEFAULT },
	{ S5K5F1_REG_SW_RESET, S5K5F1_SW_RESET },
	{ S5K5F1_REG_PAGE, S5K5F1_PAGE_DEFAULT },
	{ CCI_REG16(0x6214), 0x7971 },
	{ CCI_REG16(0x6218), 0x7150 },
};

/* 2400x2400 RAW8 at 30fps, the only mode the vendor blob describes */
static const struct cci_reg_sequence s5k5f1_2400x2400_30fps_mode[] = {
	{ S5K5F1_REG_EXPOSURE, 0x13b2 },
	{ S5K5F1_REG_AGAIN, 0x002c },
	{ CCI_REG16(0xf43e), 0x24ce },
	{ CCI_REG16(0xf440), 0x402f },
	{ CCI_REG16(0x354c), 0x0004 },
	{ CCI_REG16(0x3544), 0x110b },
	{ CCI_REG16(0x3540), 0x110b },
	{ CCI_REG16(0x3082), 0x0100 },
	{ CCI_REG16(0x3168), 0x00a0 },
	{ CCI_REG16(0x31a6), 0x0100 },
	{ CCI_REG16(0xf470), 0x0000 },
	{ CCI_REG16(0xf43a), 0x0010 },
	{ CCI_REG16(0x3572), 0x0012 },
	{ CCI_REG16(0xf420), 0x0013 },
	{ CCI_REG16(0xf422), 0x0000 },
	{ CCI_REG16(0xf424), 0x000b },
	{ CCI_REG16(0xf426), 0x000e },
	{ CCI_REG16(0xf488), 0x0008 },
	{ CCI_REG16(0x3534), 0x0708 },
	{ CCI_REG16(0x320e), 0x0000 },
	{ CCI_REG16(0x3304), 0x0094 },
	{ CCI_REG16(0xf4ba), 0x0008 },
	{ CCI_REG16(0x319e), 0x0001 },
	{ CCI_REG16(0x3078), 0x0340 },
	{ CCI_REG16(0x31c6), 0x0000 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x0e1e },
	{ CCI_REG16(0x6f12), 0x8110 },
	/* PLL multipliers: 0x95 for the pixel clock, 0xaa for the link */
	{ CCI_REG16(0x0306), 0x0095 },
	{ CCI_REG16(0x030e), 0x00aa },
	{ CCI_REG16(0x3560), 0x005a },
	{ CCI_REG16(0x300a), 0x0000 },
	{ CCI_REG16(0x0342), 0x0a38 },	/* line_length_pck = 2616 */
	{ S5K5F1_REG_VTS, 0x13c6 },	/* frame_length_lines = 5062 */
	{ CCI_REG16(0x0114), 0x0100 },	/* two data lanes */
	{ CCI_REG16(0xb134), 0x0180 },
	{ CCI_REG16(0x0344), 0x0008 },	/* x_addr_start */
	{ CCI_REG16(0x0346), 0x0008 },	/* y_addr_start */
	{ CCI_REG16(0x0348), 0x0967 },	/* x_addr_end */
	{ CCI_REG16(0x034a), 0x0967 },	/* y_addr_end */
	{ CCI_REG16(0x034c), 0x0960 },	/* x_output_size = 2400 */
	{ CCI_REG16(0x034e), 0x0960 },	/* y_output_size = 2400 */
	{ CCI_REG16(0x0112), 0x0a08 },	/* 10 bit sampled, 8 bit on the bus */
	{ CCI_REG16(0xf1a2), 0x0200 },
	{ CCI_REG16(0xf1a8), 0x09b0 },
	{ CCI_REG16(0xf1aa), 0x09b0 },
	{ CCI_REG16(0x30be), 0x0100 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x0f10 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x0f32 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x0f1e },
	{ CCI_REG16(0x6f12), 0x0483 },
	{ CCI_REG16(0x6f12), 0x071f },
};

struct s5k5f1_reg_list {
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct s5k5f1_mode {
	u32 width;			/* Frame width in pixels */
	u32 height;			/* Frame height in pixels */
	u32 hts;			/* Horizontal timing size */
	u32 vts;			/* Default vertical timing size */
	u32 exposure;			/* Default exposure value */

	const struct s5k5f1_reg_list reg_list;	/* Sensor register setting */
};

static const struct s5k5f1_mode s5k5f1_supported_modes[] = {
	{
		.width = 2400,
		.height = 2400,
		.hts = 2616,
		.vts = 5062,
		.exposure = 5042,
		.reg_list = {
			.regs = s5k5f1_2400x2400_30fps_mode,
			.num_regs = ARRAY_SIZE(s5k5f1_2400x2400_30fps_mode),
		},
	},
};

static const char * const s5k5f1_test_pattern_menu[] = {
	"Disabled",
	"Solid colour",
	"Colour bars",
	"Fade to grey colour bars",
	"PN9",
};

static const char * const s5k5f1_supply_names[] = {
	/* Enabled in this order by the vendor power sequence */
	"vio",		/* Interface (1.8V) supply */
	"vdig",		/* Digital core (1.05V) supply */
	"vana",		/* Analog (2.8V) supply */
};

#define S5K5F1_NUM_SUPPLIES	ARRAY_SIZE(s5k5f1_supply_names)

struct s5k5f1 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[S5K5F1_NUM_SUPPLIES];

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;

	const struct s5k5f1_mode *mode;
};

static int s5k5f1_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k5f1 *s5k5f1 = container_of(ctrl->handler, struct s5k5f1,
					     ctrl_handler);
	const struct s5k5f1_mode *mode = s5k5f1->mode;
	s64 exposure_max;
	int ret;

	/* Propagate change of current control to all related controls */
	if (ctrl->id == V4L2_CID_VBLANK) {
		/* Update max exposure while meeting expected vblanking */
		exposure_max = mode->height + ctrl->val - S5K5F1_EXPOSURE_MARGIN;
		__v4l2_ctrl_modify_range(s5k5f1->exposure,
					 s5k5f1->exposure->minimum,
					 exposure_max,
					 s5k5f1->exposure->step,
					 s5k5f1->exposure->default_value);
	}

	/* V4L2 controls are applied, when sensor is powered up for streaming */
	if (!pm_runtime_get_if_active(s5k5f1->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(s5k5f1->regmap, S5K5F1_REG_AGAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cci_write(s5k5f1->regmap, S5K5F1_REG_EXPOSURE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(s5k5f1->regmap, S5K5F1_REG_VTS,
				ctrl->val + mode->height, NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(s5k5f1->regmap, S5K5F1_REG_TEST_PATTERN,
				ctrl->val, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(s5k5f1->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5k5f1_ctrl_ops = {
	.s_ctrl = s5k5f1_set_ctrl,
};

static inline u64 s5k5f1_freq_to_pixel_rate(const u64 freq)
{
	return div_u64(freq * 2 * S5K5F1_DATA_LANES, S5K5F1_BITS_PER_SAMPLE);
}

static int s5k5f1_init_controls(struct s5k5f1 *s5k5f1)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &s5k5f1->ctrl_handler;
	const struct s5k5f1_mode *mode = s5k5f1->mode;
	s64 pixel_rate, hblank, vblank, exposure_max;
	struct v4l2_fwnode_device_properties props;
	int ret;

	v4l2_ctrl_handler_init(ctrl_hdlr, 8);

	s5k5f1->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr, &s5k5f1_ctrl_ops,
				       V4L2_CID_LINK_FREQ,
				       ARRAY_SIZE(s5k5f1_link_freq_menu) - 1,
				       0, s5k5f1_link_freq_menu);
	if (s5k5f1->link_freq)
		s5k5f1->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	pixel_rate = s5k5f1_freq_to_pixel_rate(s5k5f1_link_freq_menu[0]);
	s5k5f1->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &s5k5f1_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       0, pixel_rate, 1, pixel_rate);

	hblank = mode->hts - mode->width;
	s5k5f1->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k5f1_ctrl_ops,
					   V4L2_CID_HBLANK, hblank,
					   hblank, 1, hblank);
	if (s5k5f1->hblank)
		s5k5f1->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank = mode->vts - mode->height;
	s5k5f1->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k5f1_ctrl_ops,
					   V4L2_CID_VBLANK, vblank,
					   S5K5F1_VTS_MAX - mode->height, 1,
					   vblank);

	v4l2_ctrl_new_std(ctrl_hdlr, &s5k5f1_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  S5K5F1_AGAIN_MIN, S5K5F1_AGAIN_MAX,
			  S5K5F1_AGAIN_STEP, S5K5F1_AGAIN_DEFAULT);

	exposure_max = mode->vts - S5K5F1_EXPOSURE_MARGIN;
	s5k5f1->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &s5k5f1_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     S5K5F1_EXPOSURE_MIN,
					     exposure_max,
					     S5K5F1_EXPOSURE_STEP,
					     mode->exposure);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &s5k5f1_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5k5f1_test_pattern_menu) - 1,
				     0, 0, s5k5f1_test_pattern_menu);

	ret = v4l2_fwnode_device_parse(s5k5f1->dev, &props);
	if (ret)
		goto error_free_hdlr;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &s5k5f1_ctrl_ops,
					      &props);
	if (ret)
		goto error_free_hdlr;

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		goto error_free_hdlr;
	}

	s5k5f1->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error_free_hdlr:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static int s5k5f1_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct s5k5f1 *s5k5f1 = to_s5k5f1(sd);
	const struct s5k5f1_reg_list *reg_list = &s5k5f1->mode->reg_list;
	int ret;

	ret = pm_runtime_resume_and_get(s5k5f1->dev);
	if (ret)
		return ret;

	cci_multi_reg_write(s5k5f1->regmap, s5k5f1_init_regs,
			    ARRAY_SIZE(s5k5f1_init_regs), &ret);
	if (ret)
		goto error;

	/* Let the software reset above settle before programming a mode */
	usleep_range(5 * USEC_PER_MSEC, 6 * USEC_PER_MSEC);

	cci_multi_reg_write(s5k5f1->regmap, reg_list->regs,
			    reg_list->num_regs, &ret);
	if (ret)
		goto error;

	ret = __v4l2_ctrl_handler_setup(s5k5f1->sd.ctrl_handler);
	if (ret)
		goto error;

	ret = cci_write(s5k5f1->regmap, S5K5F1_REG_CTRL_MODE,
			S5K5F1_MODE_STREAMING, NULL);
	if (ret)
		goto error;

	return 0;

error:
	dev_err(s5k5f1->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put_autosuspend(s5k5f1->dev);

	return ret;
}

static int s5k5f1_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct s5k5f1 *s5k5f1 = to_s5k5f1(sd);
	int ret;

	ret = cci_write(s5k5f1->regmap, S5K5F1_REG_CTRL_MODE, 0, NULL);
	if (ret)
		dev_err(s5k5f1->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(s5k5f1->dev);

	return ret;
}

static void s5k5f1_update_pad_format(const struct s5k5f1_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = S5K5F1_MBUS_CODE;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int s5k5f1_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct s5k5f1 *s5k5f1 = to_s5k5f1(sd);
	s64 hblank, vblank, exposure_max;
	const struct s5k5f1_mode *mode;

	mode = v4l2_find_nearest_size(s5k5f1_supported_modes,
				      ARRAY_SIZE(s5k5f1_supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);

	s5k5f1_update_pad_format(mode, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY || s5k5f1->mode == mode)
		goto set_format;

	/* Update limits and set FPS and exposure to default values */
	hblank = mode->hts - mode->width;
	__v4l2_ctrl_modify_range(s5k5f1->hblank, hblank, hblank, 1, hblank);

	vblank = mode->vts - mode->height;
	__v4l2_ctrl_modify_range(s5k5f1->vblank, vblank,
				 S5K5F1_VTS_MAX - mode->height, 1, vblank);
	__v4l2_ctrl_s_ctrl(s5k5f1->vblank, vblank);

	exposure_max = mode->vts - S5K5F1_EXPOSURE_MARGIN;
	__v4l2_ctrl_modify_range(s5k5f1->exposure, S5K5F1_EXPOSURE_MIN,
				 exposure_max, S5K5F1_EXPOSURE_STEP,
				 mode->exposure);
	__v4l2_ctrl_s_ctrl(s5k5f1->exposure, mode->exposure);

	if (s5k5f1->sd.ctrl_handler->error)
		return s5k5f1->sd.ctrl_handler->error;

	s5k5f1->mode = mode;

set_format:
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int s5k5f1_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = S5K5F1_MBUS_CODE;

	return 0;
}

static int s5k5f1_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(s5k5f1_supported_modes))
		return -EINVAL;

	if (fse->code != S5K5F1_MBUS_CODE)
		return -EINVAL;

	fse->min_width = s5k5f1_supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = s5k5f1_supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int s5k5f1_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct s5k5f1 *s5k5f1 = to_s5k5f1(sd);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = s5k5f1->mode->width;
		sel->r.height = s5k5f1->mode->height;
		return 0;
	default:
		return -EINVAL;
	}
}

static int s5k5f1_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct s5k5f1 *s5k5f1 = to_s5k5f1(sd);
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.width = s5k5f1->mode->width,
			.height = s5k5f1->mode->height,
		},
	};

	return s5k5f1_set_pad_format(sd, state, &fmt);
}

static const struct v4l2_subdev_video_ops s5k5f1_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5k5f1_pad_ops = {
	.set_fmt = s5k5f1_set_pad_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.get_selection = s5k5f1_get_selection,
	.enum_mbus_code = s5k5f1_enum_mbus_code,
	.enum_frame_size = s5k5f1_enum_frame_size,
	.enable_streams = s5k5f1_enable_streams,
	.disable_streams = s5k5f1_disable_streams,
};

static const struct v4l2_subdev_ops s5k5f1_subdev_ops = {
	.video = &s5k5f1_video_ops,
	.pad = &s5k5f1_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k5f1_internal_ops = {
	.init_state = s5k5f1_init_state,
};

static const struct media_entity_operations s5k5f1_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int s5k5f1_identify_sensor(struct s5k5f1 *s5k5f1)
{
	u64 val;
	int ret;

	ret = cci_read(s5k5f1->regmap, S5K5F1_REG_CHIP_ID, &val, NULL);
	if (ret)
		return dev_err_probe(s5k5f1->dev, ret,
				     "failed to read chip id\n");

	if (val != S5K5F1_CHIP_ID)
		return dev_err_probe(s5k5f1->dev, -ENODEV,
				     "chip id mismatch: %x!=%llx\n",
				     S5K5F1_CHIP_ID, val);

	return 0;
}

static int s5k5f1_check_hwcfg(struct s5k5f1 *s5k5f1)
{
	struct fwnode_handle *fwnode = dev_fwnode(s5k5f1->dev), *ep;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	unsigned long freq_bitmap;
	int ret;

	if (!fwnode)
		return -ENODEV;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return dev_err_probe(s5k5f1->dev, -ENXIO,
				     "missing endpoint node\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(s5k5f1->dev, ret,
				     "parsing endpoint failed\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != S5K5F1_DATA_LANES) {
		ret = dev_err_probe(s5k5f1->dev, -EINVAL,
				    "number of CSI2 data lanes %u is not supported\n",
				    bus_cfg.bus.mipi_csi2.num_data_lanes);
		goto endpoint_free;
	}

	ret = v4l2_link_freq_to_bitmap(s5k5f1->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       s5k5f1_link_freq_menu,
				       ARRAY_SIZE(s5k5f1_link_freq_menu),
				       &freq_bitmap);

endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int s5k5f1_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k5f1 *s5k5f1 = to_s5k5f1(sd);
	int ret;

	ret = regulator_bulk_enable(S5K5F1_NUM_SUPPLIES, s5k5f1->supplies);
	if (ret) {
		dev_err(dev, "failed to enable regulators\n");
		return ret;
	}

	gpiod_set_value_cansleep(s5k5f1->reset_gpio, 0);
	usleep_range(S5K5F1_XCLR_TO_MCLK_US,
		     S5K5F1_XCLR_TO_MCLK_US + S5K5F1_DELAY_RANGE_US);

	ret = clk_prepare_enable(s5k5f1->mclk);
	if (ret) {
		dev_err(dev, "failed to enable clock\n");
		goto reset_assert;
	}

	usleep_range(S5K5F1_MCLK_TO_I2C_US,
		     S5K5F1_MCLK_TO_I2C_US + S5K5F1_DELAY_RANGE_US);

	return 0;

reset_assert:
	gpiod_set_value_cansleep(s5k5f1->reset_gpio, 1);
	regulator_bulk_disable(S5K5F1_NUM_SUPPLIES, s5k5f1->supplies);

	return ret;
}

static int s5k5f1_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k5f1 *s5k5f1 = to_s5k5f1(sd);

	gpiod_set_value_cansleep(s5k5f1->reset_gpio, 1);
	clk_disable_unprepare(s5k5f1->mclk);
	regulator_bulk_disable(S5K5F1_NUM_SUPPLIES, s5k5f1->supplies);

	return 0;
}

static int s5k5f1_probe(struct i2c_client *client)
{
	struct s5k5f1 *s5k5f1;
	unsigned long freq;
	unsigned int i;
	int ret;

	s5k5f1 = devm_kzalloc(&client->dev, sizeof(*s5k5f1), GFP_KERNEL);
	if (!s5k5f1)
		return -ENOMEM;

	s5k5f1->dev = &client->dev;
	v4l2_i2c_subdev_init(&s5k5f1->sd, client, &s5k5f1_subdev_ops);

	s5k5f1->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s5k5f1->regmap))
		return dev_err_probe(s5k5f1->dev, PTR_ERR(s5k5f1->regmap),
				     "failed to init CCI\n");

	s5k5f1->mclk = devm_v4l2_sensor_clk_get(s5k5f1->dev, NULL);
	if (IS_ERR(s5k5f1->mclk))
		return dev_err_probe(s5k5f1->dev, PTR_ERR(s5k5f1->mclk),
				     "failed to get MCLK clock\n");

	freq = clk_get_rate(s5k5f1->mclk);
	if (freq != S5K5F1_MCLK_FREQ_24MHZ)
		return dev_err_probe(s5k5f1->dev, -EINVAL,
				     "MCLK clock frequency %lu is not supported\n",
				     freq);

	ret = s5k5f1_check_hwcfg(s5k5f1);
	if (ret)
		return ret;

	s5k5f1->reset_gpio = devm_gpiod_get_optional(s5k5f1->dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(s5k5f1->reset_gpio))
		return dev_err_probe(s5k5f1->dev, PTR_ERR(s5k5f1->reset_gpio),
				     "cannot get reset GPIO\n");

	for (i = 0; i < S5K5F1_NUM_SUPPLIES; i++)
		s5k5f1->supplies[i].supply = s5k5f1_supply_names[i];

	ret = devm_regulator_bulk_get(s5k5f1->dev, S5K5F1_NUM_SUPPLIES,
				      s5k5f1->supplies);
	if (ret)
		return dev_err_probe(s5k5f1->dev, ret,
				     "failed to get supply regulators\n");

	/* The sensor must be powered on to read the CHIP_ID register */
	ret = s5k5f1_power_on(s5k5f1->dev);
	if (ret)
		return ret;

	ret = s5k5f1_identify_sensor(s5k5f1);
	if (ret)
		goto power_off;

	s5k5f1->mode = &s5k5f1_supported_modes[0];
	ret = s5k5f1_init_controls(s5k5f1);
	if (ret) {
		dev_err_probe(s5k5f1->dev, ret, "failed to init controls\n");
		goto power_off;
	}

	s5k5f1->sd.state_lock = s5k5f1->ctrl_handler.lock;
	s5k5f1->sd.internal_ops = &s5k5f1_internal_ops;
	s5k5f1->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s5k5f1->sd.entity.ops = &s5k5f1_subdev_entity_ops;
	s5k5f1->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s5k5f1->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&s5k5f1->sd.entity, 1, &s5k5f1->pad);
	if (ret) {
		dev_err_probe(s5k5f1->dev, ret,
			      "failed to init media entity pads\n");
		goto v4l2_ctrl_handler_free;
	}

	ret = v4l2_subdev_init_finalize(&s5k5f1->sd);
	if (ret < 0) {
		dev_err_probe(s5k5f1->dev, ret, "failed to init subdev\n");
		goto media_entity_cleanup;
	}

	pm_runtime_set_active(s5k5f1->dev);
	pm_runtime_enable(s5k5f1->dev);

	ret = v4l2_async_register_subdev_sensor(&s5k5f1->sd);
	if (ret < 0) {
		dev_err_probe(s5k5f1->dev, ret,
			      "failed to register V4L2 subdev\n");
		goto subdev_cleanup;
	}

	pm_runtime_set_autosuspend_delay(s5k5f1->dev, 1000);
	pm_runtime_use_autosuspend(s5k5f1->dev);
	pm_runtime_idle(s5k5f1->dev);

	return 0;

subdev_cleanup:
	v4l2_subdev_cleanup(&s5k5f1->sd);
	pm_runtime_disable(s5k5f1->dev);
	pm_runtime_set_suspended(s5k5f1->dev);

media_entity_cleanup:
	media_entity_cleanup(&s5k5f1->sd.entity);

v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(s5k5f1->sd.ctrl_handler);

power_off:
	s5k5f1_power_off(s5k5f1->dev);

	return ret;
}

static void s5k5f1_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k5f1 *s5k5f1 = to_s5k5f1(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_disable(s5k5f1->dev);

	if (!pm_runtime_status_suspended(s5k5f1->dev)) {
		s5k5f1_power_off(s5k5f1->dev);
		pm_runtime_set_suspended(s5k5f1->dev);
	}
}

static DEFINE_RUNTIME_DEV_PM_OPS(s5k5f1_pm_ops, s5k5f1_power_off,
				 s5k5f1_power_on, NULL);

static const struct of_device_id s5k5f1_of_match[] = {
	{ .compatible = "samsung,s5k5f1" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5k5f1_of_match);

static struct i2c_driver s5k5f1_i2c_driver = {
	.driver = {
		.name = "s5k5f1",
		.pm = pm_ptr(&s5k5f1_pm_ops),
		.of_match_table = s5k5f1_of_match,
	},
	.probe = s5k5f1_probe,
	.remove = s5k5f1_remove,
};
module_i2c_driver(s5k5f1_i2c_driver);

MODULE_AUTHOR("William Floyd <git@notmy.space>");
MODULE_DESCRIPTION("Samsung S5K5F1 IR image sensor driver");
MODULE_LICENSE("GPL");
