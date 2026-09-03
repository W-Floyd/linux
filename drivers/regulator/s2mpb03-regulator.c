// SPDX-License-Identifier: GPL-2.0+
//
// s2mpb03-regulator.c - Regulator driver for the Samsung S2MPB03
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
#include <linux/regulator/s2mpb03.h>
#include <linux/slab.h>

struct s2mpb03_data {
	struct regmap *regmap;
	struct device *dev;
};

#define _LDO(macro)	S2MPB03_LDO##macro
#define _REG(ctrl)	S2MPB03_REG##ctrl
#define _ldo_ops(num)	s2mpb03_ops##num
#define _TIME(macro)	S2MPB03_ENABLE_TIME##macro

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
	.n_voltages	= S2MPB03_LDO_N_VOLTAGES,		\
	.vsel_reg	= v,					\
	.vsel_mask	= S2MPB03_LDO_VSEL_MASK,		\
	.enable_reg	= e,					\
	.enable_mask	= S2MPB03_LDO_ENABLE_MASK,		\
	.enable_time	= t,					\
	.ramp_delay	= S2MPB03_RAMP_DELAY			\
}

static const struct regulator_ops s2mpb03_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel	= regulator_set_voltage_time_sel,
};

static const struct regulator_desc regulators[S2MPB03_REGULATOR_MAX] = {
		// name, id, ops, min_uv, uV_step, vsel_reg, enable_reg
		LDO_DESC("ldo1", _LDO(1), &_ldo_ops(), _LDO(_MIN1),
			_LDO(_STEP2), _REG(_LDO1_CTRL),
			_REG(_LDO1_CTRL), _TIME(_LDO)),
		LDO_DESC("ldo2", _LDO(2), &_ldo_ops(), _LDO(_MIN1),
			_LDO(_STEP2), _REG(_LDO2_CTRL),
			_REG(_LDO2_CTRL), _TIME(_LDO)),
		LDO_DESC("ldo3", _LDO(3), &_ldo_ops(), _LDO(_MIN1),
			_LDO(_STEP1), _REG(_LDO3_CTRL),
			_REG(_LDO3_CTRL), _TIME(_LDO)),
		LDO_DESC("ldo4", _LDO(4), &_ldo_ops(), _LDO(_MIN1),
			_LDO(_STEP2), _REG(_LDO4_CTRL),
			_REG(_LDO4_CTRL), _TIME(_LDO)),
		LDO_DESC("ldo5", _LDO(5), &_ldo_ops(), _LDO(_MIN2),
			_LDO(_STEP1), _REG(_LDO5_CTRL),
			_REG(_LDO5_CTRL), _TIME(_LDO)),
		LDO_DESC("ldo6", _LDO(6), &_ldo_ops(), _LDO(_MIN2),
			_LDO(_STEP1), _REG(_LDO6_CTRL),
			_REG(_LDO6_CTRL), _TIME(_LDO)),
		LDO_DESC("ldo7", _LDO(7), &_ldo_ops(), _LDO(_MIN2),
			_LDO(_STEP1), _REG(_LDO7_CTRL),
			_REG(_LDO7_CTRL), _TIME(_LDO)),
};

static int s2mpb03_pmic_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sec_pmic_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct s2mpb03_data *s2mpb03;
	struct regulator_config config = { };
	unsigned int rdev_num = ARRAY_SIZE(regulators);

	s2mpb03 = devm_kzalloc(dev, sizeof(*s2mpb03), GFP_KERNEL);
	if (!s2mpb03)
		return -ENOMEM;

	platform_set_drvdata(pdev, s2mpb03);

	s2mpb03->regmap = iodev->regmap_pmic;
	s2mpb03->dev = dev;
	if (!dev->of_node)
		device_set_of_node_from_dev(dev, dev->parent);

	config.dev = dev;
	config.driver_data = s2mpb03;

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

static const struct platform_device_id s2mpb03_pmic_id[] = {
	{ "s2mpb03-regulator" },
	{ },
};
MODULE_DEVICE_TABLE(platform, s2mpb03_pmic_id);

static struct platform_driver s2mpb03_platform_driver = {
	.driver = {
		.name = "s2mpb03",
	},
	.probe = s2mpb03_pmic_probe,
	.id_table = s2mpb03_pmic_id,
};
module_platform_driver(s2mpb03_platform_driver);

MODULE_AUTHOR("William Floyd <git@notmy.space>");
MODULE_DESCRIPTION("Samsung S2MPB03 Regulator Driver");
MODULE_LICENSE("GPL");
