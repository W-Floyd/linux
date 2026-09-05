// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2026 William Floyd <git@notmy.space>
//
// Flash LED driver for the Samsung S2MPB02 camera sub-PMIC.
//
// The FLED block has two independent channels, each with its own mode,
// current and timeout registers. On the Snapdragon Galaxy S9 channel 1
// drives the rear camera's white flash and torch, and channel 2 the
// infrared illuminator that lights the scene for the iris camera.
//
// Each channel can be strobed either over I2C or by asserting an external
// pin. This driver uses I2C only, so the board is expected to leave the
// FLASH_EN and TORCH_EN pins deasserted.
//
// The register semantics come from Samsung's own vendor driver for this
// chip, which is the only available description of the block.

#include <linux/bitfield.h>
#include <linux/jiffies.h>
#include <linux/led-class-flash.h>
#include <linux/mfd/samsung/core.h>
#include <linux/mfd/samsung/s2mpb02.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <media/v4l2-flash-led-class.h>

#define S2MPB02_FLED_CHANNELS		2

/* FLED_CTRLn: with the enable bit clear the channel follows its strobe pins */
#define S2MPB02_FLED_EN			BIT(7)
#define S2MPB02_FLED_MODE_TORCH		BIT(6)
#define S2MPB02_FLED_EN_MODE_MASK	(S2MPB02_FLED_EN | S2MPB02_FLED_MODE_TORCH)

/* Undervoltage cut-off, 2.9V plus 100mV per step */
#define S2MPB02_FLED_LV_SEL_MASK	GENMASK(2, 0)
#define S2MPB02_FLED_LV_SEL(mV)		(((mV) - 2900) / 100)
#define S2MPB02_FLED_LV_SEL_MV		3100

/* FLED_CURn and FLED_TIMEn each split into a flash and a torch nibble */
#define S2MPB02_FLED_FLASH_MASK		GENMASK(7, 4)
#define S2MPB02_FLED_TORCH_MASK		GENMASK(3, 0)

#define S2MPB02_FLASH_UA_MIN		100000
#define S2MPB02_FLASH_UA_STEP		100000
#define S2MPB02_FLASH_UA_MAX		1500000

#define S2MPB02_TORCH_UA_MIN		20000
#define S2MPB02_TORCH_UA_STEP		20000
#define S2MPB02_TORCH_UA_MAX		300000

#define S2MPB02_TIMEOUT_US_MIN		62500
#define S2MPB02_TIMEOUT_US_STEP		62500
#define S2MPB02_TIMEOUT_US_MAX		1000000

/*
 * The torch cut-off runs from 1 to 16 seconds and cannot be switched off, so
 * take the longest setting and let the LED class re-arm it.
 */
#define S2MPB02_TORCH_TIMEOUT_MAX	0x0f

struct s2mpb02_led {
	struct led_classdev_flash fled;
	struct v4l2_flash *v4l2_flash;
	struct s2mpb02_leds *chip;

	/* Per-channel register addresses */
	unsigned int ctrl;
	unsigned int cur;
	unsigned int time;

	/* When the running strobe ends, since the chip cannot report it */
	unsigned long strobe_end;

	u32 torch_ua_max;
};

struct s2mpb02_leds {
	struct device *dev;
	struct regmap *regmap;

	/* Serialises the read-modify-write of the shared channel registers */
	struct mutex lock;

	struct s2mpb02_led leds[S2MPB02_FLED_CHANNELS];
	unsigned int num_leds;
};

static struct s2mpb02_led *fled_to_led(struct led_classdev_flash *fled)
{
	return container_of(fled, struct s2mpb02_led, fled);
}

static int s2mpb02_led_set_mode(struct s2mpb02_led *led, bool enable, bool torch)
{
	unsigned int val = 0;

	if (enable)
		val = S2MPB02_FLED_EN | (torch ? S2MPB02_FLED_MODE_TORCH : 0);

	return regmap_update_bits(led->chip->regmap, led->ctrl,
				  S2MPB02_FLED_EN_MODE_MASK, val);
}

static int s2mpb02_torch_brightness_set(struct led_classdev *lcdev,
					enum led_brightness brightness)
{
	struct s2mpb02_led *led = fled_to_led(lcdev_to_flcdev(lcdev));
	int ret;

	guard(mutex)(&led->chip->lock);

	if (!brightness)
		return s2mpb02_led_set_mode(led, false, true);

	ret = regmap_update_bits(led->chip->regmap, led->cur,
				 S2MPB02_FLED_TORCH_MASK,
				 FIELD_PREP(S2MPB02_FLED_TORCH_MASK, brightness));
	if (ret)
		return ret;

	return s2mpb02_led_set_mode(led, true, true);
}

static int s2mpb02_flash_brightness_set(struct led_classdev_flash *fled,
					u32 brightness)
{
	struct s2mpb02_led *led = fled_to_led(fled);
	unsigned int level = brightness / S2MPB02_FLASH_UA_STEP;

	guard(mutex)(&led->chip->lock);

	return regmap_update_bits(led->chip->regmap, led->cur,
				  S2MPB02_FLED_FLASH_MASK,
				  FIELD_PREP(S2MPB02_FLED_FLASH_MASK, level));
}

static int s2mpb02_flash_timeout_set(struct led_classdev_flash *fled,
				     u32 timeout)
{
	struct s2mpb02_led *led = fled_to_led(fled);
	unsigned int level = timeout / S2MPB02_TIMEOUT_US_STEP - 1;

	guard(mutex)(&led->chip->lock);

	return regmap_update_bits(led->chip->regmap, led->time,
				  S2MPB02_FLED_FLASH_MASK,
				  FIELD_PREP(S2MPB02_FLED_FLASH_MASK, level));
}

static int s2mpb02_flash_strobe_set(struct led_classdev_flash *fled, bool state)
{
	struct s2mpb02_led *led = fled_to_led(fled);
	int ret;

	guard(mutex)(&led->chip->lock);

	if (!state) {
		led->strobe_end = jiffies;
		return s2mpb02_led_set_mode(led, false, false);
	}

	/*
	 * A channel fires on the rising edge of its enable bit, and the chip
	 * leaves that bit set once its own timer has ended the flash. Drop it
	 * before asserting it, so that a repeated strobe still produces an
	 * edge instead of a write that regmap elides as redundant.
	 */
	ret = s2mpb02_led_set_mode(led, false, false);
	if (ret)
		return ret;

	ret = s2mpb02_led_set_mode(led, true, false);
	if (ret)
		return ret;

	led->strobe_end = jiffies + usecs_to_jiffies(fled->timeout.val);

	return 0;
}

static int s2mpb02_flash_strobe_get(struct led_classdev_flash *fled, bool *state)
{
	struct s2mpb02_led *led = fled_to_led(fled);

	guard(mutex)(&led->chip->lock);

	/*
	 * The enable bit stays set after the hardware timer has ended the
	 * flash, so it cannot answer this; track the duration instead.
	 */
	*state = time_before(jiffies, led->strobe_end);

	return 0;
}

static const struct led_flash_ops s2mpb02_flash_ops = {
	.flash_brightness_set = s2mpb02_flash_brightness_set,
	.strobe_set = s2mpb02_flash_strobe_set,
	.strobe_get = s2mpb02_flash_strobe_get,
	.timeout_set = s2mpb02_flash_timeout_set,
};

static void s2mpb02_init_setting(struct led_flash_setting *s, u32 min, u32 max,
				 u32 step)
{
	s->min = min;
	s->max = max;
	s->step = step;
	s->val = min;
}

static int s2mpb02_led_parse(struct s2mpb02_led *led, struct fwnode_handle *fwnode)
{
	struct led_classdev_flash *fled = &led->fled;
	struct led_classdev *lcdev = &fled->led_cdev;
	struct device *dev = led->chip->dev;
	u32 flash_ua_max = S2MPB02_FLASH_UA_MAX;
	u32 timeout_us_max = S2MPB02_TIMEOUT_US_MAX;
	u32 reg;
	int ret;

	ret = fwnode_property_read_u32(fwnode, "reg", &reg);
	if (ret)
		return dev_err_probe(dev, ret, "missing reg property\n");

	if (reg >= S2MPB02_FLED_CHANNELS)
		return dev_err_probe(dev, -EINVAL, "invalid channel %u\n", reg);

	/*
	 * The two channels' registers are interleaved: CTRL1, CTRL2, CUR1,
	 * TIME1, CUR2, TIME2.
	 */
	/* jiffies starts negative, so an unset deadline must not read as past */
	led->strobe_end = jiffies;

	led->ctrl = S2MPB02_REG_FLED_CTRL1 + reg;
	led->cur = S2MPB02_REG_FLED_CUR1 + 2 * reg;
	led->time = S2MPB02_REG_FLED_TIME1 + 2 * reg;

	/*
	 * The board limits are optional; without them assume the LED can take
	 * everything the chip can source.
	 */
	led->torch_ua_max = S2MPB02_TORCH_UA_MAX;
	fwnode_property_read_u32(fwnode, "led-max-microamp", &led->torch_ua_max);
	fwnode_property_read_u32(fwnode, "flash-max-microamp", &flash_ua_max);
	fwnode_property_read_u32(fwnode, "flash-max-timeout-us", &timeout_us_max);

	if (led->torch_ua_max < S2MPB02_TORCH_UA_MIN ||
	    led->torch_ua_max > S2MPB02_TORCH_UA_MAX ||
	    flash_ua_max < S2MPB02_FLASH_UA_MIN ||
	    flash_ua_max > S2MPB02_FLASH_UA_MAX ||
	    timeout_us_max < S2MPB02_TIMEOUT_US_MIN ||
	    timeout_us_max > S2MPB02_TIMEOUT_US_MAX)
		return dev_err_probe(dev, -EINVAL,
				     "channel %u limits out of range\n", reg);

	lcdev->max_brightness = led->torch_ua_max / S2MPB02_TORCH_UA_STEP;
	lcdev->brightness_set_blocking = s2mpb02_torch_brightness_set;
	lcdev->flags |= LED_DEV_CAP_FLASH;

	fled->ops = &s2mpb02_flash_ops;
	s2mpb02_init_setting(&fled->brightness, S2MPB02_FLASH_UA_MIN,
			     flash_ua_max, S2MPB02_FLASH_UA_STEP);
	s2mpb02_init_setting(&fled->timeout, S2MPB02_TIMEOUT_US_MIN,
			     timeout_us_max, S2MPB02_TIMEOUT_US_STEP);

	return 0;
}

static int s2mpb02_led_setup(struct s2mpb02_led *led)
{
	struct regmap *regmap = led->chip->regmap;
	int ret;

	ret = s2mpb02_led_set_mode(led, false, false);
	if (ret)
		return ret;

	ret = regmap_update_bits(regmap, led->ctrl, S2MPB02_FLED_LV_SEL_MASK,
				 S2MPB02_FLED_LV_SEL(S2MPB02_FLED_LV_SEL_MV));
	if (ret)
		return ret;

	ret = regmap_update_bits(regmap, led->time, S2MPB02_FLED_TORCH_MASK,
				 FIELD_PREP(S2MPB02_FLED_TORCH_MASK,
					    S2MPB02_TORCH_TIMEOUT_MAX));
	if (ret)
		return ret;

	return s2mpb02_flash_timeout_set(&led->fled, led->fled.timeout.val);
}

static int s2mpb02_led_register(struct s2mpb02_led *led,
				struct fwnode_handle *fwnode)
{
	struct led_init_data init_data = { .fwnode = fwnode };
	struct led_classdev *lcdev = &led->fled.led_cdev;
	struct v4l2_flash_config v4l2_cfg = { };
	struct device *dev = led->chip->dev;
	int ret;

	ret = devm_led_classdev_flash_register_ext(dev, &led->fled, &init_data);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register LED\n");

	strscpy(v4l2_cfg.dev_name, lcdev->dev->kobj.name,
		sizeof(v4l2_cfg.dev_name));
	s2mpb02_init_setting(&v4l2_cfg.intensity, S2MPB02_TORCH_UA_MIN,
			     led->torch_ua_max, S2MPB02_TORCH_UA_STEP);

	led->v4l2_flash = v4l2_flash_init(dev, fwnode, &led->fled, NULL,
					  &v4l2_cfg);
	if (IS_ERR(led->v4l2_flash))
		return dev_err_probe(dev, PTR_ERR(led->v4l2_flash),
				     "failed to register V4L2 flash device\n");

	return 0;
}

static void s2mpb02_leds_release(struct s2mpb02_leds *chip)
{
	unsigned int i;

	for (i = 0; i < chip->num_leds; i++)
		v4l2_flash_release(chip->leds[i].v4l2_flash);
}

static int s2mpb02_leds_probe(struct platform_device *pdev)
{
	struct sec_pmic_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	struct fwnode_handle *child, *leds;
	struct s2mpb02_leds *chip;
	int ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	chip->regmap = iodev->regmap_pmic;
	platform_set_drvdata(pdev, chip);

	ret = devm_mutex_init(dev, &chip->lock);
	if (ret)
		return ret;

	if (!dev->of_node)
		device_set_of_node_from_dev(dev, dev->parent);

	leds = device_get_named_child_node(dev, "leds");
	if (!leds)
		return dev_err_probe(dev, -ENODEV, "no leds node\n");

	fwnode_for_each_available_child_node(leds, child) {
		struct s2mpb02_led *led = &chip->leds[chip->num_leds];

		if (chip->num_leds >= S2MPB02_FLED_CHANNELS) {
			ret = dev_err_probe(dev, -EINVAL, "too many LEDs\n");
			goto err_release;
		}

		led->chip = chip;

		ret = s2mpb02_led_parse(led, child);
		if (ret)
			goto err_child;

		ret = s2mpb02_led_setup(led);
		if (ret)
			goto err_child;

		ret = s2mpb02_led_register(led, child);
		if (ret)
			goto err_child;

		chip->num_leds++;
	}

	fwnode_handle_put(leds);

	return 0;

err_child:
	fwnode_handle_put(child);
err_release:
	fwnode_handle_put(leds);
	s2mpb02_leds_release(chip);

	return ret;
}

static void s2mpb02_leds_remove(struct platform_device *pdev)
{
	s2mpb02_leds_release(platform_get_drvdata(pdev));
}

static const struct platform_device_id s2mpb02_leds_id[] = {
	{ "s2mpb02-led" },
	{ }
};
MODULE_DEVICE_TABLE(platform, s2mpb02_leds_id);

static struct platform_driver s2mpb02_leds_driver = {
	.driver = {
		.name = "s2mpb02-led",
	},
	.probe = s2mpb02_leds_probe,
	.remove = s2mpb02_leds_remove,
	.id_table = s2mpb02_leds_id,
};
module_platform_driver(s2mpb02_leds_driver);

MODULE_AUTHOR("William Floyd <git@notmy.space>");
MODULE_DESCRIPTION("Samsung S2MPB02 flash LED driver");
MODULE_LICENSE("GPL");
