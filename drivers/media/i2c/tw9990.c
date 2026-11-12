// SPDX-License-Identifier: GPL-2.0
/*
 * Techwell (Intersil/Renesas) TW9990 analog video decoder.
 *
 * Written for the rear camera of the Mobis D-Audio TCC803x head unit. The
 * init table is the vendor's (Telechips/Mobis, GPL): the CM4 early-camera
 * firmware in the TCC803x LK tree (dev/camera/earlycamera/dev/tw9990.c), also
 * in the vendor Linux driver (AVNT5W_TCC803XKK_kernel,
 * drivers/media/platform/tcccam/atv/tw9990.c). Register meanings below are
 * from the Renesas datasheet (FN8406 rev 6.01). Not run on hardware.
 *
 * Output is interlaced: 8-bit BT.656, one field at a time; the decoder has no
 * deinterlacer, so the receiver needs one.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-subdev.h>

#define TW9990_REG_ID		0x00
#define TW9990_ID_PRODUCT	GENMASK(7, 3)	/* 0 for the TW9990 (and TW9900) */
#define TW9990_ID_REV		GENMASK(2, 0)
#define TW9990_REG_STATUS	0x01
#define TW9990_REG_INPUT	0x02
#define TW9990_REG_OUTCTL	0x03
#define TW9990_REG_BRIGHT	0x10
#define TW9990_REG_CONTRAST	0x11
#define TW9990_REG_STD		0x1c

/*
 * The CM4 firmware's sensor_video_check(): VDLOSS (7) clear; HLOCK (6),
 * SLOCK (5), VLOCK (3) set; MONO (1, no colour burst) and DET50 (0, 50 Hz)
 * clear. FIELD (4) toggles and bit 2 is reserved.
 */
#define TW9990_STATUS_MASK	0xeb
#define TW9990_STATUS_LOCKED	0x68

/*
 * OPFORM: BT.656 (MODE), 8-bit, LLC clock; TRI_SEL/OEN 000 all outputs on,
 * 111 all tri-stated.
 */
#define TW9990_OUTCTL_RUN	0xa0
#define TW9990_OUTCTL_STANDBY	0xa7
/* Vendor sensor_close(): INFORM 27 MHz, composite, Y from MUX1 (unused here). */
#define TW9990_INPUT_PARKED	0x46

#define TW9990_WIDTH		720
#define TW9990_FIELD_HEIGHT	240

struct tw9990_reg {
	u8 reg;
	u8 val;
};

/* CVBS table, CM4 firmware cvbs_composite[] (the 2018-11 colour defaults). */
static const struct tw9990_reg tw9990_init[] = {
	{ 0x01, 0x78 }, { 0x02, 0x40 }, { 0x03, 0xa0 }, { 0x04, 0x00 },
	{ 0x05, 0x00 }, { 0x1c, 0x08 }, { 0x07, 0x02 }, { 0x08, 0x12 },
	{ 0x09, 0xf0 }, { 0x0a, 0x10 }, { 0x0b, 0xd0 }, { 0x0c, 0xcc },
	{ 0x0d, 0x00 }, { 0x0f, 0x00 }, { 0x10, 0xfb }, { 0x11, 0x5f },
	{ 0x12, 0x21 }, { 0x13, 0x64 }, { 0x14, 0x64 }, { 0x15, 0x00 },
	{ 0x16, 0x00 }, { 0x17, 0x30 }, { 0x18, 0x44 }, { 0x19, 0x58 },
	{ 0x1a, 0x0a }, { 0x1b, 0x00 }, { 0x1d, 0x7f }, { 0x1e, 0x08 },
	{ 0x1f, 0x00 }, { 0x20, 0xa0 }, { 0x21, 0x42 }, { 0x22, 0xf0 },
	{ 0x23, 0xd8 }, { 0x24, 0xbc }, { 0x25, 0xb8 }, { 0x26, 0x44 },
	{ 0x27, 0x2a }, { 0x28, 0x00 }, { 0x29, 0x00 }, { 0x2a, 0x78 },
	{ 0x2b, 0x44 }, { 0x2c, 0x40 }, { 0x2d, 0x14 }, { 0x2e, 0xa5 },
	{ 0x2f, 0xe0 }, { 0x30, 0x00 }, { 0x31, 0x10 }, { 0x32, 0xff },
	{ 0x33, 0x05 }, { 0x34, 0x1a }, { 0x35, 0xa0 }, { 0x4c, 0x05 },
	{ 0x4d, 0x40 }, { 0x4e, 0x00 }, { 0x4f, 0x00 }, { 0x50, 0xa0 },
	{ 0x51, 0x22 }, { 0x52, 0x31 }, { 0x53, 0x80 }, { 0x54, 0x00 },
	{ 0x55, 0x00 }, { 0x6b, 0x06 }, { 0x6c, 0x24 }, { 0x6d, 0x00 },
	{ 0x6e, 0x20 }, { 0x6f, 0x13 }, { 0xae, 0x1a }, { 0xaf, 0x80 },
	/*
	 * SRESET: resets the decoder state, keeps the registers. Vendor note:
	 * fixes an image shift (2018-08).
	 */
	{ 0x06, 0x80 },
};

/*
 * Per-input differences between the CM4 tables cvbs_composite[] and
 * s_video[]. INFORM: 27 MHz crystal, composite or S-Video, Y on MUX0. SDT:
 * CVBS forces NTSC(M) with the shadow registers off, S-Video auto-detects.
 * Which input this car uses is a variant strap (Mobis daudio_pinctrl.c
 * get_camera_variant(), GPE15 = S-Video).
 */
enum tw9990_input {
	TW9990_IN_CVBS,
	TW9990_IN_SVIDEO,
};

static const struct {
	u8 input;
	u8 std;
} tw9990_inputs[] = {
	[TW9990_IN_CVBS]	= { 0x40, 0x08 },
	[TW9990_IN_SVIDEO]	= { 0x50, 0x07 },
};

struct tw9990 {
	struct i2c_client *client;
	struct gpio_desc *reset_gpio;
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
	struct media_pad pad;
	struct mutex lock;	/* serializes register access; also the ctrl lock */
	bool streaming;
	enum tw9990_input input;
};

static inline struct tw9990 *to_tw9990(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tw9990, sd);
}

static int tw9990_write(struct tw9990 *tw, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(tw->client, reg, val);

	if (ret)
		dev_err(&tw->client->dev, "write %#04x=%#04x: %d\n", reg, val, ret);
	return ret;
}

static int tw9990_write_table(struct tw9990 *tw, const struct tw9990_reg *t,
			      size_t n)
{
	int ret;

	for (; n; n--, t++) {
		ret = tw9990_write(tw, t->reg, t->val);
		if (ret)
			return ret;
	}
	return 0;
}

/* Called with tw->lock held (it is the handler's lock). */
static int tw9990_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct tw9990 *tw = container_of(ctrl->handler, struct tw9990, hdl);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return tw9990_write(tw, TW9990_REG_BRIGHT, (u8)ctrl->val);
	case V4L2_CID_CONTRAST:
		return tw9990_write(tw, TW9990_REG_CONTRAST, ctrl->val);
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops tw9990_ctrl_ops = {
	.s_ctrl = tw9990_s_ctrl,
};

static int tw9990_select_input(struct tw9990 *tw)
{
	int ret;

	ret = tw9990_write(tw, TW9990_REG_INPUT, tw9990_inputs[tw->input].input);
	if (ret)
		return ret;
	return tw9990_write(tw, TW9990_REG_STD, tw9990_inputs[tw->input].std);
}

static int tw9990_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct tw9990 *tw = to_tw9990(sd);
	int ret;

	mutex_lock(&tw->lock);
	if (enable) {
		ret = tw9990_select_input(tw);
		if (!ret)
			ret = tw9990_write(tw, TW9990_REG_OUTCTL, TW9990_OUTCTL_RUN);
	} else {
		ret = tw9990_write(tw, TW9990_REG_INPUT, TW9990_INPUT_PARKED);
		if (!ret)
			ret = tw9990_write(tw, TW9990_REG_OUTCTL, TW9990_OUTCTL_STANDBY);
	}
	if (!ret)
		tw->streaming = enable;
	mutex_unlock(&tw->lock);

	return ret;
}

/* Allowed while streaming: the rear-view bridge polls lock with video on. */
static int tw9990_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct tw9990 *tw = to_tw9990(sd);
	int ret;

	mutex_lock(&tw->lock);
	ret = i2c_smbus_read_byte_data(tw->client, TW9990_REG_STATUS);
	mutex_unlock(&tw->lock);
	if (ret < 0)
		return ret;

	*status = (ret & TW9990_STATUS_MASK) == TW9990_STATUS_LOCKED ?
		  0 : V4L2_IN_ST_NO_SIGNAL;
	return 0;
}

static int tw9990_s_routing(struct v4l2_subdev *sd, u32 input, u32 output,
			    u32 config)
{
	struct tw9990 *tw = to_tw9990(sd);
	int ret = 0;

	if (input >= ARRAY_SIZE(tw9990_inputs))
		return -EINVAL;

	mutex_lock(&tw->lock);
	tw->input = input;
	if (tw->streaming)
		ret = tw9990_select_input(tw);
	mutex_unlock(&tw->lock);

	return ret;
}

/* The vendor tables are NTSC only; PAL appears only for its AUX input. */
static int tw9990_g_tvnorms(struct v4l2_subdev *sd, v4l2_std_id *std)
{
	*std = V4L2_STD_NTSC;
	return 0;
}

static int tw9990_g_std(struct v4l2_subdev *sd, v4l2_std_id *std)
{
	*std = V4L2_STD_NTSC;
	return 0;
}

static int tw9990_s_std(struct v4l2_subdev *sd, v4l2_std_id std)
{
	return std & V4L2_STD_NTSC ? 0 : -EINVAL;
}

static void tw9990_fill_fmt(struct v4l2_mbus_framefmt *fmt)
{
	/*
	 * Field height as the vendor captures it (720x240, VIN interlace mode
	 * on). The datasheet's NTSC field is 244/243 active lines with
	 * VACTIVE 0xf0 cropping it to 240.
	 */
	fmt->code = MEDIA_BUS_FMT_UYVY8_2X8;
	fmt->width = TW9990_WIDTH;
	fmt->height = TW9990_FIELD_HEIGHT;
	fmt->field = V4L2_FIELD_ALTERNATE;
	fmt->colorspace = V4L2_COLORSPACE_SMPTE170M;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_LIM_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_709;
}

static int tw9990_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	tw9990_fill_fmt(&fmt->format);
	return 0;
}

static int tw9990_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_UYVY8_2X8;
	return 0;
}

static const struct v4l2_subdev_core_ops tw9990_core_ops = {
	.subscribe_event	= v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event	= v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops tw9990_video_ops = {
	.s_routing	= tw9990_s_routing,
	.g_std		= tw9990_g_std,
	.s_std		= tw9990_s_std,
	.g_tvnorms	= tw9990_g_tvnorms,
	.g_input_status	= tw9990_g_input_status,
	.s_stream	= tw9990_s_stream,
};

static const struct v4l2_subdev_pad_ops tw9990_pad_ops = {
	.enum_mbus_code	= tw9990_enum_mbus_code,
	.get_fmt	= tw9990_get_fmt,
	.set_fmt	= tw9990_get_fmt,
};

static const struct v4l2_subdev_ops tw9990_ops = {
	.core	= &tw9990_core_ops,
	.video	= &tw9990_video_ops,
	.pad	= &tw9990_pad_ops,
};

static int tw9990_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct gpio_desc *pdn;
	struct tw9990 *tw;
	int id, ret;

	tw = devm_kzalloc(dev, sizeof(*tw), GFP_KERNEL);
	if (!tw)
		return -ENOMEM;
	tw->client = client;
	mutex_init(&tw->lock);

	pdn = devm_gpiod_get_optional(dev, "powerdown", GPIOD_OUT_LOW);
	if (IS_ERR(pdn))
		return dev_err_probe(dev, PTR_ERR(pdn), "powerdown gpio\n");

	tw->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(tw->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(tw->reset_gpio),
				     "reset gpio\n");
	if (tw->reset_gpio) {
		/*
		 * Vendor sensor_open(): 10 ms in reset, 15 ms to come up (the
		 * datasheet gives no figure). Reset also latches the SIAD0 strap
		 * on VD0, which picks 0x44 or 0x45.
		 */
		msleep(10);
		gpiod_set_value_cansleep(tw->reset_gpio, 0);
		msleep(15);
	}

	id = i2c_smbus_read_byte_data(client, TW9990_REG_ID);
	if (id < 0)
		return dev_err_probe(dev, id, "no response at %#04x\n",
				     client->addr);
	if (FIELD_GET(TW9990_ID_PRODUCT, id))
		return dev_err_probe(dev, -ENODEV, "unexpected id %#04x\n", id);
	dev_info(dev, "revision %lu\n", FIELD_GET(TW9990_ID_REV, id));

	ret = tw9990_write_table(tw, tw9990_init, ARRAY_SIZE(tw9990_init));
	if (!ret)
		ret = tw9990_s_stream(&tw->sd, 0);
	if (ret)
		return dev_err_probe(dev, ret, "init\n");

	v4l2_i2c_subdev_init(&tw->sd, client, &tw9990_ops);
	tw->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	tw->sd.entity.function = MEDIA_ENT_F_ATV_DECODER;

	/* Defaults match the init table's 0x10/0x11. */
	v4l2_ctrl_handler_init(&tw->hdl, 2);
	tw->hdl.lock = &tw->lock;
	v4l2_ctrl_new_std(&tw->hdl, &tw9990_ctrl_ops, V4L2_CID_BRIGHTNESS,
			  -128, 127, 1, -5);
	v4l2_ctrl_new_std(&tw->hdl, &tw9990_ctrl_ops, V4L2_CID_CONTRAST,
			  0, 255, 1, 0x5f);
	tw->sd.ctrl_handler = &tw->hdl;
	ret = tw->hdl.error;
	if (!ret)
		ret = v4l2_ctrl_handler_setup(&tw->hdl);
	if (ret)
		goto err_ctrls;

	tw->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&tw->sd.entity, 1, &tw->pad);
	if (ret)
		goto err_ctrls;

	ret = v4l2_async_register_subdev(&tw->sd);
	if (ret)
		goto err_entity;
	return 0;

err_entity:
	media_entity_cleanup(&tw->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&tw->hdl);
	return ret;
}

static void tw9990_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
}

static const struct of_device_id tw9990_of_match[] = {
	{ .compatible = "techwell,tw9990" },
	{ }
};
MODULE_DEVICE_TABLE(of, tw9990_of_match);

static const struct i2c_device_id tw9990_id[] = {
	{ "tw9990" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tw9990_id);

static struct i2c_driver tw9990_driver = {
	.driver = {
		.name		= "tw9990",
		.of_match_table	= tw9990_of_match,
	},
	.probe		= tw9990_probe,
	.remove		= tw9990_remove,
	.id_table	= tw9990_id,
};
module_i2c_driver(tw9990_driver);

MODULE_DESCRIPTION("Techwell TW9990 video decoder");
MODULE_LICENSE("GPL");
