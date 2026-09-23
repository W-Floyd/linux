// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI TPS61280 boost converter with bypass
 *
 * Copyright (c) 2026 William Floyd <git@notmy.space>
 *
 * A single boost converter, 2.85 V to 4.4 V in 50 mV steps, with an I2C
 * interface. Two output registers, VOUTFLOOR and VOUTROOF, are selected by the
 * VSEL pin; the EN pin enables the converter and the nBYP pin forces
 * pass-through. All three pins may be strapped on the board or driven by the
 * host, and when the host does not drive them the CONFIG register offers the
 * same controls.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#include <dt-bindings/regulator/ti,tps61280.h>

#define TPS61280_CONFIG			0x01
#define TPS61280_CONFIG_MODE		GENMASK(1, 0)
#define TPS61280_CONFIG_MODE_DEVICE	0
#define TPS61280_CONFIG_MODE_AUTO_PWM	1
#define TPS61280_CONFIG_MODE_FORCE_PWM	2
#define TPS61280_CONFIG_MODE_VSEL	3
#define TPS61280_CONFIG_ENABLE		GENMASK(6, 5)
#define TPS61280_CONFIG_ENABLE_PINS	0	/* EN and nBYP pins are in control */
#define TPS61280_CONFIG_ENABLE_AUTO	1	/* boost, bypass when VIN allows */
#define TPS61280_CONFIG_ENABLE_BYPASS	2	/* forced pass-through */
#define TPS61280_CONFIG_ENABLE_OFF	3	/* shutdown */

#define TPS61280_VOUTFLOORSET		0x02
#define TPS61280_VOUTROOFSET		0x03
#define TPS61280_VOUT			GENMASK(4, 0)
#define TPS61280_VOUT_MIN_UV		2850000
#define TPS61280_VOUT_STEP_UV		50000
#define TPS61280_VOUT_N			32

#define TPS61280_ILIMSET		0x04
#define TPS61280_ILIM			GENMASK(3, 0)
#define TPS61280_ILIM_SEL		GENMASK(2, 0)
#define TPS61280_ILIM_BASE		BIT(3)
#define TPS61280_ILIM_OFF		BIT(5)
#define TPS61280_ILIM_MIN_UA		1500000
#define TPS61280_ILIM_STEP_UA		500000

#define TPS61280_STATUS			0x05
#define TPS61280_STATUS_PGOOD		BIT(0)
#define TPS61280_STATUS_FAULT		BIT(1)
#define TPS61280_STATUS_ILIMBST		BIT(2)
#define TPS61280_STATUS_ILIMPT		BIT(3)
#define TPS61280_STATUS_OPMODE		BIT(4)
#define TPS61280_STATUS_DCDCMODE	BIT(5)
#define TPS61280_STATUS_HOTDIE		BIT(6)
#define TPS61280_STATUS_THERMALSD	BIT(7)

#define TPS61280_E2PROMCTRL		0xff

struct tps61280 {
	struct regmap *regmap;
	struct gpio_desc *bypass_gpiod;
	/* the output register the VSEL pin level selects */
	unsigned int vout_reg;
	/* the EN pin is driven by the host, not by CONFIG */
	bool ena_gpio;
	bool bypass;
};

static bool tps61280_volatile_reg(struct device *dev, unsigned int reg)
{
	return reg == TPS61280_STATUS;
}

static const struct regmap_config tps61280_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = TPS61280_E2PROMCTRL,
	.cache_type = REGCACHE_MAPLE,
	.volatile_reg = tps61280_volatile_reg,
};

static int tps61280_set_voltage_sel(struct regulator_dev *rdev, unsigned int sel)
{
	struct tps61280 *tps = rdev_get_drvdata(rdev);

	return regmap_update_bits(tps->regmap, tps->vout_reg, TPS61280_VOUT, sel);
}

static int tps61280_get_voltage_sel(struct regulator_dev *rdev)
{
	struct tps61280 *tps = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret;

	ret = regmap_read(tps->regmap, tps->vout_reg, &val);
	if (ret)
		return ret;

	return FIELD_GET(TPS61280_VOUT, val);
}

static int tps61280_set_config_enable(struct tps61280 *tps, unsigned int state)
{
	return regmap_update_bits(tps->regmap, TPS61280_CONFIG,
				  TPS61280_CONFIG_ENABLE,
				  FIELD_PREP(TPS61280_CONFIG_ENABLE, state));
}

/*
 * Register-based enable, used only when no EN GPIO is described: the core
 * drives the GPIO itself when there is one.
 */
static int tps61280_enable(struct regulator_dev *rdev)
{
	struct tps61280 *tps = rdev_get_drvdata(rdev);

	return tps61280_set_config_enable(tps, tps->bypass ?
					  TPS61280_CONFIG_ENABLE_BYPASS :
					  TPS61280_CONFIG_ENABLE_AUTO);
}

static int tps61280_disable(struct regulator_dev *rdev)
{
	struct tps61280 *tps = rdev_get_drvdata(rdev);

	return tps61280_set_config_enable(tps, TPS61280_CONFIG_ENABLE_OFF);
}

static int tps61280_is_enabled(struct regulator_dev *rdev)
{
	struct tps61280 *tps = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret;

	ret = regmap_read(tps->regmap, TPS61280_CONFIG, &val);
	if (ret)
		return ret;

	return FIELD_GET(TPS61280_CONFIG_ENABLE, val) != TPS61280_CONFIG_ENABLE_OFF;
}

static int tps61280_set_bypass(struct regulator_dev *rdev, bool enable)
{
	struct tps61280 *tps = rdev_get_drvdata(rdev);
	int ret;

	if (tps->bypass_gpiod) {
		gpiod_set_value_cansleep(tps->bypass_gpiod, enable);
	} else if (tps->ena_gpio) {
		/*
		 * The pins are in control and nBYP is strapped: the register
		 * bypass modes are not in effect.
		 */
		return -EOPNOTSUPP;
	} else if (tps61280_is_enabled(rdev) > 0) {
		ret = tps61280_set_config_enable(tps, enable ?
						 TPS61280_CONFIG_ENABLE_BYPASS :
						 TPS61280_CONFIG_ENABLE_AUTO);
		if (ret)
			return ret;
	}

	tps->bypass = enable;

	return 0;
}

static int tps61280_get_bypass(struct regulator_dev *rdev, bool *enable)
{
	struct tps61280 *tps = rdev_get_drvdata(rdev);

	*enable = tps->bypass;

	return 0;
}

static int tps61280_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
	struct tps61280 *tps = rdev_get_drvdata(rdev);
	unsigned int val;

	switch (mode) {
	case REGULATOR_MODE_NORMAL:
		val = TPS61280_CONFIG_MODE_AUTO_PWM;
		break;
	case REGULATOR_MODE_FAST:
		val = TPS61280_CONFIG_MODE_FORCE_PWM;
		break;
	default:
		return -EINVAL;
	}

	return regmap_update_bits(tps->regmap, TPS61280_CONFIG,
				  TPS61280_CONFIG_MODE,
				  FIELD_PREP(TPS61280_CONFIG_MODE, val));
}

static unsigned int tps61280_get_mode(struct regulator_dev *rdev)
{
	struct tps61280 *tps = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret;

	ret = regmap_read(tps->regmap, TPS61280_CONFIG, &val);
	if (ret)
		return 0;

	if (FIELD_GET(TPS61280_CONFIG_MODE, val) == TPS61280_CONFIG_MODE_FORCE_PWM)
		return REGULATOR_MODE_FAST;

	return REGULATOR_MODE_NORMAL;
}

static int tps61280_set_current_limit(struct regulator_dev *rdev,
				      int min_uA, int max_uA)
{
	struct tps61280 *tps = rdev_get_drvdata(rdev);
	int sel;

	for (sel = TPS61280_ILIM_SEL; sel >= 0; sel--) {
		int limit = TPS61280_ILIM_MIN_UA + sel * TPS61280_ILIM_STEP_UA;

		if (limit <= max_uA && limit >= min_uA)
			return regmap_update_bits(tps->regmap, TPS61280_ILIMSET,
						  TPS61280_ILIM | TPS61280_ILIM_OFF,
						  TPS61280_ILIM_BASE | sel);
	}

	return -EINVAL;
}

static int tps61280_get_current_limit(struct regulator_dev *rdev)
{
	struct tps61280 *tps = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret;

	ret = regmap_read(tps->regmap, TPS61280_ILIMSET, &val);
	if (ret)
		return ret;

	return TPS61280_ILIM_MIN_UA +
	       FIELD_GET(TPS61280_ILIM_SEL, val) * TPS61280_ILIM_STEP_UA;
}

static int tps61280_get_error_flags(struct regulator_dev *rdev,
				    unsigned int *flags)
{
	struct tps61280 *tps = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret;

	ret = regmap_read(tps->regmap, TPS61280_STATUS, &val);
	if (ret)
		return ret;

	*flags = 0;
	if (val & (TPS61280_STATUS_ILIMBST | TPS61280_STATUS_ILIMPT))
		*flags |= REGULATOR_ERROR_OVER_CURRENT;
	if (val & TPS61280_STATUS_HOTDIE)
		*flags |= REGULATOR_ERROR_OVER_TEMP_WARN;
	if (val & TPS61280_STATUS_THERMALSD)
		*flags |= REGULATOR_ERROR_OVER_TEMP;
	if (val & TPS61280_STATUS_FAULT)
		*flags |= REGULATOR_ERROR_FAIL;

	return 0;
}

static const struct regulator_ops tps61280_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.set_voltage_sel = tps61280_set_voltage_sel,
	.get_voltage_sel = tps61280_get_voltage_sel,
	.enable = tps61280_enable,
	.disable = tps61280_disable,
	.is_enabled = tps61280_is_enabled,
	.set_bypass = tps61280_set_bypass,
	.get_bypass = tps61280_get_bypass,
	.set_mode = tps61280_set_mode,
	.get_mode = tps61280_get_mode,
	.set_current_limit = tps61280_set_current_limit,
	.get_current_limit = tps61280_get_current_limit,
	.get_error_flags = tps61280_get_error_flags,
};

static unsigned int tps61280_of_map_mode(unsigned int mode)
{
	switch (mode) {
	case TPS61280_MODE_NORMAL:
		return REGULATOR_MODE_NORMAL;
	case TPS61280_MODE_FPWM:
		return REGULATOR_MODE_FAST;
	default:
		return REGULATOR_MODE_INVALID;
	}
}

static const struct regulator_desc tps61280_desc = {
	.name = "tps61280",
	.owner = THIS_MODULE,
	.ops = &tps61280_ops,
	.of_map_mode = tps61280_of_map_mode,
	.type = REGULATOR_VOLTAGE,
	.n_voltages = TPS61280_VOUT_N,
	.min_uV = TPS61280_VOUT_MIN_UV,
	.uV_step = TPS61280_VOUT_STEP_UV,
};

static int tps61280_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regulator_config config = {};
	struct regulator_init_data *init_data;
	struct regulator_dev *rdev;
	struct gpio_desc *vsel;
	enum gpiod_flags flags;
	struct tps61280 *tps;
	bool vsel_high;

	tps = devm_kzalloc(dev, sizeof(*tps), GFP_KERNEL);
	if (!tps)
		return -ENOMEM;

	tps->regmap = devm_regmap_init_i2c(client, &tps61280_regmap_config);
	if (IS_ERR(tps->regmap))
		return dev_err_probe(dev, PTR_ERR(tps->regmap),
				     "failed to initialise regmap\n");

	init_data = of_get_regulator_init_data(dev, dev->of_node, &tps61280_desc);
	if (!init_data)
		return -ENOMEM;

	/*
	 * VSEL picks the output register. Drive the pin when the host owns it,
	 * otherwise trust the strap the binding describes.
	 */
	vsel_high = device_property_read_bool(dev, "ti,vsel-state-high");
	vsel = devm_gpiod_get_optional(dev, "ti,vsel",
				       vsel_high ? GPIOD_OUT_HIGH : GPIOD_OUT_LOW);
	if (IS_ERR(vsel))
		return dev_err_probe(dev, PTR_ERR(vsel), "failed to get VSEL GPIO\n");
	tps->vout_reg = vsel_high ? TPS61280_VOUTROOFSET : TPS61280_VOUTFLOORSET;

	/* nBYP is active low; the descriptor's polarity flag inverts it. */
	tps->bypass_gpiod = devm_gpiod_get_optional(dev, "ti,bypass", GPIOD_OUT_LOW);
	if (IS_ERR(tps->bypass_gpiod))
		return dev_err_probe(dev, PTR_ERR(tps->bypass_gpiod),
				     "failed to get nBYP GPIO\n");

	if (init_data->constraints.boot_on || init_data->constraints.always_on)
		flags = GPIOD_OUT_HIGH;
	else
		flags = GPIOD_OUT_LOW;
	config.ena_gpiod = devm_gpiod_get_optional(dev, "enable", flags);
	if (IS_ERR(config.ena_gpiod))
		return dev_err_probe(dev, PTR_ERR(config.ena_gpiod),
				     "failed to get EN GPIO\n");
	tps->ena_gpio = !!config.ena_gpiod;

	config.dev = dev;
	config.of_node = dev->of_node;
	config.init_data = init_data;
	config.regmap = tps->regmap;
	config.driver_data = tps;

	rdev = devm_regulator_register(dev, &tps61280_desc, &config);
	if (IS_ERR(rdev))
		return dev_err_probe(dev, PTR_ERR(rdev),
				     "failed to register regulator\n");

	return 0;
}

static const struct of_device_id tps61280_of_match[] = {
	{ .compatible = "ti,tps61280" },
	{ }
};
MODULE_DEVICE_TABLE(of, tps61280_of_match);

static const struct i2c_device_id tps61280_i2c_id[] = {
	{ .name = "tps61280" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tps61280_i2c_id);

static struct i2c_driver tps61280_driver = {
	.driver = {
		.name = "tps61280",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = tps61280_of_match,
	},
	.probe = tps61280_probe,
	.id_table = tps61280_i2c_id,
};
module_i2c_driver(tps61280_driver);

MODULE_DESCRIPTION("TI TPS61280 boost converter driver");
MODULE_AUTHOR("William Floyd <git@notmy.space>");
MODULE_LICENSE("GPL");
