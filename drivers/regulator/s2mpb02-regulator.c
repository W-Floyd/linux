// SPDX-License-Identifier: GPL-2.0+
//
// s2mpb02-regulator.c - Regulator driver for the Samsung S2MPB02
//
// Copyright (c) 2017 Samsung Electronics Co., Ltd
//              http://www.samsung.com
// Copyright (C) 2026 William Floyd <git@notmy.space>

#include <linux/err.h>
#include <linux/mfd/samsung/core.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/s2mpb02.h>
#include <linux/slab.h>

struct s2mpb02_data {
	struct regmap *regmap;
	struct device *dev;
};

#define _BUCK(macro)	S2MPB02_BUCK##macro
#define _LDO(macro)	S2MPB02_LDO##macro
#define _REG(ctrl)	S2MPB02_REG##ctrl
#define _ops(num)	s2mpb02_ops##num
#define _TIME(macro)	S2MPB02_ENABLE_TIME##macro

#define LDO_DESC(_name, _id, _ops, m, s, v, e, t) {		\
	.name		= _name,				\
	.id		= _id,					\
	.ops		= _ops,					\
	.of_match = of_match_ptr(_name),			\
	.of_match_full_name = true,				\
	.regulators_node = of_match_ptr("regulators"),		\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= m,					\
	.uV_step	= s,					\
	.n_voltages	= S2MPB02_LDO_N_VOLTAGES,		\
	.vsel_reg	= v,					\
	.vsel_mask	= S2MPB02_LDO_VSEL_MASK,		\
	.enable_reg	= e,					\
	.enable_mask	= S2MPB02_LDO_ENABLE_MASK,		\
	.enable_time	= t,					\
	.ramp_delay	= S2MPB02_RAMP_DELAY			\
}

#define BUCK_DESC(_name, _id, _ops, m, s, v, e, t) {		\
	.name		= _name,				\
	.id		= _id,					\
	.ops		= _ops,					\
	.of_match = of_match_ptr(_name),			\
	.of_match_full_name = true,				\
	.regulators_node = of_match_ptr("regulators"),		\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= m,					\
	.uV_step	= s,					\
	.n_voltages	= S2MPB02_BUCK_N_VOLTAGES,		\
	.vsel_reg	= v,					\
	.vsel_mask	= S2MPB02_BUCK_VSEL_MASK,		\
	.enable_reg	= e,					\
	.enable_mask	= S2MPB02_BUCK_ENABLE_MASK,		\
	.enable_time	= t,					\
	.ramp_delay	= S2MPB02_RAMP_DELAY			\
}

static const struct regulator_ops s2mpb02_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel	= regulator_set_voltage_time_sel,
};

static const struct regulator_desc regulators[S2MPB02_REGULATOR_MAX] = {
		// name, id, ops, min_uv, uV_step, vsel_reg, enable_reg
		LDO_DESC("ldo1", _LDO(1), &_ops(), _LDO(_MIN1),
			_LDO(_STEP1), _REG(_L1CTRL),
			_REG(_L1CTRL), _TIME(_LDO)),
		LDO_DESC("ldo2", _LDO(2), &_ops(), _LDO(_MIN1),
			_LDO(_STEP1), _REG(_L2CTRL),
			_REG(_L2CTRL), _TIME(_LDO)),
		LDO_DESC("ldo3", _LDO(3), &_ops(), _LDO(_MIN1),
			_LDO(_STEP1), _REG(_L3CTRL),
			_REG(_L3CTRL), _TIME(_LDO)),
		LDO_DESC("ldo4", _LDO(4), &_ops(), _LDO(_MIN1),
			_LDO(_STEP1), _REG(_L4CTRL),
			_REG(_L4CTRL), _TIME(_LDO)),
		LDO_DESC("ldo5", _LDO(5), &_ops(), _LDO(_MIN1),
			_LDO(_STEP1), _REG(_L5CTRL),
			_REG(_L5CTRL), _TIME(_LDO)),
		LDO_DESC("ldo6", _LDO(6), &_ops(), _LDO(_MIN1),
			_LDO(_STEP2), _REG(_L6CTRL),
			_REG(_L6CTRL), _TIME(_LDO)),
		LDO_DESC("ldo7", _LDO(7), &_ops(), _LDO(_MIN1),
			_LDO(_STEP2), _REG(_L7CTRL),
			_REG(_L7CTRL), _TIME(_LDO)),
		LDO_DESC("ldo8", _LDO(8), &_ops(), _LDO(_MIN1),
			_LDO(_STEP2), _REG(_L8CTRL),
			_REG(_L8CTRL), _TIME(_LDO)),
		LDO_DESC("ldo9", _LDO(9), &_ops(), _LDO(_MIN1),
			_LDO(_STEP2), _REG(_L9CTRL),
			_REG(_L9CTRL), _TIME(_LDO)),
		LDO_DESC("ldo10", _LDO(10), &_ops(), _LDO(_MIN1),
			_LDO(_STEP2), _REG(_L10CTRL),
			_REG(_L10CTRL), _TIME(_LDO)),
		LDO_DESC("ldo11", _LDO(11), &_ops(), _LDO(_MIN1),
			_LDO(_STEP2), _REG(_L11CTRL),
			_REG(_L11CTRL), _TIME(_LDO)),
		LDO_DESC("ldo12", _LDO(12), &_ops(), _LDO(_MIN1),
			_LDO(_STEP2), _REG(_L12CTRL),
			_REG(_L12CTRL), _TIME(_LDO)),
		LDO_DESC("ldo13", _LDO(13), &_ops(), _LDO(_MIN1),
			_LDO(_STEP2), _REG(_L13CTRL),
			_REG(_L13CTRL), _TIME(_LDO)),
		LDO_DESC("ldo14", _LDO(14), &_ops(), _LDO(_MIN1),
			_LDO(_STEP2), _REG(_L14CTRL),
			_REG(_L14CTRL), _TIME(_LDO)),
		LDO_DESC("ldo15", _LDO(15), &_ops(), _LDO(_MIN1),
			_LDO(_STEP2), _REG(_L15CTRL),
			_REG(_L15CTRL), _TIME(_LDO)),
		LDO_DESC("ldo16", _LDO(16), &_ops(), _LDO(_MIN1),
			_LDO(_STEP2), _REG(_L16CTRL),
			_REG(_L16CTRL), _TIME(_LDO)),
		LDO_DESC("ldo17", _LDO(17), &_ops(), _LDO(_MIN1),
			_LDO(_STEP2), _REG(_L17CTRL),
			_REG(_L17CTRL), _TIME(_LDO)),
		LDO_DESC("ldo18", _LDO(18), &_ops(), _LDO(_MIN1),
			_LDO(_STEP2), _REG(_L18CTRL),
			_REG(_L18CTRL), _TIME(_LDO)),
		BUCK_DESC("buck1", _BUCK(1), &_ops(), _BUCK(_MIN1),
			_BUCK(_STEP1), _REG(_B1CTRL2),
			_REG(_B1CTRL1), _TIME(_BUCK)),
		BUCK_DESC("buck2", _BUCK(2), &_ops(), _BUCK(_MIN1),
			_BUCK(_STEP1), _REG(_B2CTRL2),
			_REG(_B2CTRL1), _TIME(_BUCK)),
		BUCK_DESC("bb", S2MPB02_BB1, &_ops(), _BUCK(_MIN2),
			_BUCK(_STEP2), _REG(_BB1CTRL2),
			_REG(_BB1CTRL1), _TIME(_BB)),
};

static int s2mpb02_pmic_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sec_pmic_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct s2mpb02_data *s2mpb02;
	struct regulator_config config = { };
	unsigned int rdev_num = ARRAY_SIZE(regulators);

	s2mpb02 = devm_kzalloc(dev, sizeof(*s2mpb02), GFP_KERNEL);
	if (!s2mpb02)
		return -ENOMEM;

	platform_set_drvdata(pdev, s2mpb02);

	s2mpb02->regmap = iodev->regmap_pmic;
	s2mpb02->dev = dev;
	if (!dev->of_node)
		device_set_of_node_from_dev(dev, dev->parent);

	config.dev = dev;
	config.driver_data = s2mpb02;

	for (int i = 0; i < rdev_num; i++) {
		struct regulator_dev *regulator;

		regulator = devm_regulator_register(&pdev->dev,
						&regulators[i], &config);
		if (IS_ERR(regulator)) {
			return dev_err_probe(&pdev->dev, PTR_ERR(regulator),
					"regulator init failed for %d\n", i);
		}
	}

	return 0;
}

static const struct platform_device_id s2mpb02_pmic_id[] = {
	{ "s2mpb02-regulator" },
	{ },
};
MODULE_DEVICE_TABLE(platform, s2mpb02_pmic_id);

static struct platform_driver s2mpb02_platform_driver = {
	.driver = {
		.name = "s2mpb02",
	},
	.probe = s2mpb02_pmic_probe,
	.id_table = s2mpb02_pmic_id,
};
module_platform_driver(s2mpb02_platform_driver);

MODULE_AUTHOR("William Floyd <git@notmy.space>");
MODULE_DESCRIPTION("Samsung S2MPB02 Regulator Driver");
MODULE_LICENSE("GPL");
