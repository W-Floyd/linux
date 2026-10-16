// SPDX-License-Identifier: GPL-2.0-only
/*
 * Pin controller / GPIO driver for the Telechips TCC803x SoC.
 *
 * This is a from-scratch mainline driver, not a port of the vendor driver.
 * Every register offset, bank layout and pin-function value below is cited
 * against the real Telechips vendor source (Linux 3.18, GPLv2), as released
 * through Hyundai Mobis's open-source compliance portal, reference tree
 * AVNT5W_TCC803XKK_kernel/ (paths below are relative to that tree's root):
 *
 *   - Per-bank register layout (GPIO_DATA/OUTPUT_ENABLE/DATA_OR/DATA_BIC/
 *     FUNC offsets): drivers/pinctrl/tcc/pinctrl-tcc803x.c:44-54
 *   - GPIO get/set/direction/function-select algorithms:
 *     drivers/pinctrl/tcc/pinctrl-tcc803x.c:178-259
 *     (tcc803x_gpio_get, tcc803x_gpio_set, tcc803x_gpio_set_direction,
 *     tcc803x_gpio_set_function)
 *   - Bank byte offsets and pin counts (gpa..gpsd1):
 *     arch/arm/boot/dts/mobis/tcc8031-pinctrl.dtsi:46-121
 *     (TCC8031 is this SoC's internal die name -- CONFIG_ARCH_TCC803X
 *     builds board files literally named tcc8030/tcc8031 in the vendor
 *     tree, e.g. arch/arm/boot/dts/mobis/tcc8030-android-lpd4321_sv0.1.dts
 *     including "tcc8031-android-avn.dtsi")
 *   - uart0 pin group (pins + function value 7):
 *     arch/arm/boot/dts/mobis/tcc8031-pinctrl.dtsi:808-811
 *   - Controller base address and reg window (0x14200000, size 0x300 for
 *     the main block; 0x14800800, size 0x100 for the PM-domain block):
 *     arch/arm/boot/dts/mobis/tcc803x.dtsi:892-894
 *
 * KNOWN GAP: the vendor driver also drives banks GPSD2/GPH/GPK/GPMA, at
 * byte offsets 0x600/0x640/0x680/0x6c0 (pinctrl-tcc803x.c:35-42) -- outside
 * the 0x300-byte "main" reg window the vendor's own devicetree declares for
 * this node. That inconsistency is unresolved in the vendor source itself.
 * Rather than guess which of the two vendor-provided facts (the reg size,
 * or the register offsets the driver actually touches) is wrong, this
 * driver only implements the banks that fit inside the declared window
 * (GPA..GPSD1). See the binding doc for the same caveat.
 *
 * Also out of scope for this skeleton: pin bias (pull-up/down), drive
 * strength and slew rate. Their register formats are cited in the vendor
 * source (pinctrl-tcc803x.c:299-323) but wiring up generic pinconf is left
 * for a follow-up; this driver only does pinmux (function select) and
 * plain GPIO.
 */

#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "../pinctrl-utils.h"

/* Per-bank register offsets, relative to each bank's own base address.
 * Cited: drivers/pinctrl/tcc/pinctrl-tcc803x.c:44-54.
 */
#define TCC803X_GPIO_DATA		0x00
#define TCC803X_GPIO_OUTPUT_ENABLE	0x04
#define TCC803X_GPIO_DATA_OR		0x08
#define TCC803X_GPIO_DATA_BIC		0x0c
#define TCC803X_GPIO_FUNC		0x30

/* FUNC is a 4-bit field per pin, 8 pins per 32-bit register.
 * Cited: drivers/pinctrl/tcc/pinctrl-tcc803x.c:240,252-257.
 */
#define TCC803X_FUNC_BITS		4
#define TCC803X_FUNC_PINS_PER_REG	8

/*
 * Bank byte offsets and pin counts, banks GPA..GPSD1 only (see the KNOWN
 * GAP note above for why GPSD2/GPH/GPK/GPMA are absent).
 * Cited: arch/arm/boot/dts/mobis/tcc8031-pinctrl.dtsi:46-121.
 */
struct tcc803x_bank {
	const char *name;
	u32 reg_offset;
	unsigned int npins;
	unsigned int pin_base;	/* filled in at probe time */
};

static struct tcc803x_bank tcc803x_banks[] = {
	{ .name = "gpa",   .reg_offset = 0x000, .npins = 32 },
	{ .name = "gpb",   .reg_offset = 0x040, .npins = 29 },
	{ .name = "gpc",   .reg_offset = 0x080, .npins = 30 },
	{ .name = "gpd",   .reg_offset = 0x0c0, .npins = 22 },
	{ .name = "gpe",   .reg_offset = 0x100, .npins = 20 },
	{ .name = "gpf",   .reg_offset = 0x140, .npins = 32 },
	{ .name = "gpg",   .reg_offset = 0x180, .npins = 11 },
	{ .name = "gpsd0", .reg_offset = 0x200, .npins = 15 },
	{ .name = "gpsd1", .reg_offset = 0x240, .npins = 11 },
};

#define TCC803X_NBANKS ARRAY_SIZE(tcc803x_banks)

/*
 * uart0 pin group: gpsd0-11, gpsd0-12, function value 7.
 * Cited: arch/arm/boot/dts/mobis/tcc8031-pinctrl.dtsi:808-811.
 *
 * Global pin numbers are filled in at probe time once bank pin_base is
 * known (gpsd0 is the 8th bank, after gpa..gpg: 32+29+30+22+20+32+11 = 176,
 * so gpsd0-11/-12 are global pins 187/188 -- computed, not hardcoded, so
 * it can't drift out of sync with the bank table above).
 */
static unsigned int tcc803x_uart0_pins[2];

static const char * const tcc803x_uart0_groups[] = { "uart0-grp" };

#define TCC803X_UART0_FUNC_VAL	7

struct tcc803x_group {
	const char *name;
	const unsigned int *pins;
	unsigned int npins;
	unsigned int func_val;
};

static struct tcc803x_group tcc803x_groups[] = {
	{
		.name = "uart0-grp",
		.pins = tcc803x_uart0_pins,
		.npins = ARRAY_SIZE(tcc803x_uart0_pins),
		.func_val = TCC803X_UART0_FUNC_VAL,
	},
};

static const struct pinfunction tcc803x_functions[] = {
	PINCTRL_PINFUNCTION("uart0", tcc803x_uart0_groups,
			     ARRAY_SIZE(tcc803x_uart0_groups)),
};

struct tcc803x_pinctrl {
	struct device *dev;
	void __iomem *base;		/* main block, banks GPA..GPSD1 */
	struct pinctrl_dev *pctldev;
	struct gpio_chip gpio_chip;
	struct pinctrl_desc desc;
	struct pinctrl_pin_desc *pins;
	unsigned int npins;
	/*
	 * Guards the OUTPUT_ENABLE read-modify-write in
	 * tcc803x_gpio_direction_set(): the register packs every pin of a
	 * bank into one 32-bit word, so two threads changing direction on
	 * different pins of the same bank would otherwise race on the same
	 * readl()/writel() pair. Also taken around the DATA_OR/DATA_BIC
	 * write in tcc803x_gpio_set() for the same reason, even though that
	 * one write is itself atomic - it keeps a direction_set() and a
	 * set() on sibling pins from interleaving unpredictably.
	 */
	spinlock_t lock;
};

static const struct tcc803x_bank *
tcc803x_pin_to_bank(unsigned int pin, unsigned int *local)
{
	unsigned int i;

	for (i = 0; i < TCC803X_NBANKS; i++) {
		const struct tcc803x_bank *bank = &tcc803x_banks[i];

		if (pin >= bank->pin_base &&
		    pin < bank->pin_base + bank->npins) {
			*local = pin - bank->pin_base;
			return bank;
		}
	}

	return NULL;
}

/* --- pinmux_ops --- */

static int tcc803x_pmx_get_functions_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(tcc803x_functions);
}

static const char *tcc803x_pmx_get_function_name(struct pinctrl_dev *pctldev,
						   unsigned int selector)
{
	return tcc803x_functions[selector].name;
}

static int tcc803x_pmx_get_function_groups(struct pinctrl_dev *pctldev,
					     unsigned int selector,
					     const char * const **groups,
					     unsigned int *num_groups)
{
	*groups = tcc803x_functions[selector].groups;
	*num_groups = tcc803x_functions[selector].ngroups;
	return 0;
}

static int tcc803x_pmx_set_mux(struct pinctrl_dev *pctldev,
				 unsigned int selector, unsigned int group)
{
	struct tcc803x_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	struct tcc803x_group *grp = &tcc803x_groups[group];
	unsigned int i;

	for (i = 0; i < grp->npins; i++) {
		unsigned int local;
		const struct tcc803x_bank *bank =
			tcc803x_pin_to_bank(grp->pins[i], &local);
		void __iomem *reg;
		unsigned int shift;
		u32 val;

		if (!bank)
			return -EINVAL;

		/* reg = bank_base + FUNC + 4*(local/8); Cited above. */
		reg = pctl->base + bank->reg_offset + TCC803X_GPIO_FUNC +
			4 * (local / TCC803X_FUNC_PINS_PER_REG);
		shift = TCC803X_FUNC_BITS *
			(local % TCC803X_FUNC_PINS_PER_REG);

		val = readl(reg);
		val &= ~(0xfU << shift);
		val |= (grp->func_val & 0xfU) << shift;
		writel(val, reg);
	}

	return 0;
}

static const struct pinmux_ops tcc803x_pmx_ops = {
	.get_functions_count	= tcc803x_pmx_get_functions_count,
	.get_function_name	= tcc803x_pmx_get_function_name,
	.get_function_groups	= tcc803x_pmx_get_function_groups,
	.set_mux		= tcc803x_pmx_set_mux,
};

/* --- pinctrl_ops --- */

static int tcc803x_pctl_get_groups_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(tcc803x_groups);
}

static const char *tcc803x_pctl_get_group_name(struct pinctrl_dev *pctldev,
						  unsigned int selector)
{
	return tcc803x_groups[selector].name;
}

static int tcc803x_pctl_get_group_pins(struct pinctrl_dev *pctldev,
					  unsigned int selector,
					  const unsigned int **pins,
					  unsigned int *num_pins)
{
	*pins = tcc803x_groups[selector].pins;
	*num_pins = tcc803x_groups[selector].npins;
	return 0;
}

static int tcc803x_pctl_dt_node_to_map(struct pinctrl_dev *pctldev,
					 struct device_node *np,
					 struct pinctrl_map **map,
					 unsigned int *num_maps)
{
	/*
	 * Groups are static (see KNOWN GAP note): the DT only names which
	 * group to apply to a device via "telechips,pins"/pin-function",
	 * matched against the compiled-in group table above by name.
	 */
	unsigned int reserved = 0;
	int ret;

	*map = NULL;
	*num_maps = 0;

	ret = pinctrl_utils_reserve_map(pctldev, map, &reserved, num_maps, 1);
	if (ret)
		return ret;

	return pinctrl_utils_add_map_mux(pctldev, map, &reserved, num_maps,
					  np->name, "uart0");
}

static const struct pinctrl_ops tcc803x_pctl_ops = {
	.get_groups_count	= tcc803x_pctl_get_groups_count,
	.get_group_name		= tcc803x_pctl_get_group_name,
	.get_group_pins		= tcc803x_pctl_get_group_pins,
	.dt_node_to_map		= tcc803x_pctl_dt_node_to_map,
	.dt_free_map		= pinctrl_utils_free_map,
};

/* --- gpio_chip --- */

static int tcc803x_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct tcc803x_pinctrl *pctl = gpiochip_get_data(chip);
	unsigned int local;
	const struct tcc803x_bank *bank = tcc803x_pin_to_bank(offset, &local);
	u32 val;

	if (!bank)
		return -EINVAL;

	/* Cited: drivers/pinctrl/tcc/pinctrl-tcc803x.c:178-182. */
	val = readl(pctl->base + bank->reg_offset + TCC803X_GPIO_DATA);
	return !!(val & BIT(local));
}

static int tcc803x_gpio_set(struct gpio_chip *chip, unsigned int offset,
			     int value)
{
	struct tcc803x_pinctrl *pctl = gpiochip_get_data(chip);
	unsigned int local;
	const struct tcc803x_bank *bank = tcc803x_pin_to_bank(offset, &local);
	unsigned long flags;

	if (!bank)
		return -EINVAL;

	/* Cited: drivers/pinctrl/tcc/pinctrl-tcc803x.c:184-190. */
	spin_lock_irqsave(&pctl->lock, flags);
	writel(BIT(local), pctl->base + bank->reg_offset +
		(value ? TCC803X_GPIO_DATA_OR : TCC803X_GPIO_DATA_BIC));
	spin_unlock_irqrestore(&pctl->lock, flags);

	return 0;
}

static int tcc803x_gpio_direction_set(struct gpio_chip *chip,
				       unsigned int offset, bool input)
{
	struct tcc803x_pinctrl *pctl = gpiochip_get_data(chip);
	unsigned int local;
	const struct tcc803x_bank *bank = tcc803x_pin_to_bank(offset, &local);
	unsigned long flags;
	void __iomem *reg;
	u32 val;

	if (!bank)
		return -EINVAL;

	/*
	 * Cited: drivers/pinctrl/tcc/pinctrl-tcc803x.c:212-224. Bit set in
	 * OUTPUT_ENABLE means output, cleared means input -- note this is
	 * the opposite sense of its name read naively.
	 */
	reg = pctl->base + bank->reg_offset + TCC803X_GPIO_OUTPUT_ENABLE;

	spin_lock_irqsave(&pctl->lock, flags);
	val = readl(reg);
	val &= ~BIT(local);
	if (!input)
		val |= BIT(local);
	writel(val, reg);
	spin_unlock_irqrestore(&pctl->lock, flags);

	return 0;
}

static int tcc803x_gpio_direction_input(struct gpio_chip *chip,
					 unsigned int offset)
{
	return tcc803x_gpio_direction_set(chip, offset, true);
}

static int tcc803x_gpio_direction_output(struct gpio_chip *chip,
					  unsigned int offset, int value)
{
	tcc803x_gpio_set(chip, offset, value);
	return tcc803x_gpio_direction_set(chip, offset, false);
}

static int tcc803x_gpio_get_direction(struct gpio_chip *chip,
				       unsigned int offset)
{
	struct tcc803x_pinctrl *pctl = gpiochip_get_data(chip);
	unsigned int local;
	const struct tcc803x_bank *bank = tcc803x_pin_to_bank(offset, &local);
	u32 val;

	if (!bank)
		return -EINVAL;

	/* Cited: drivers/pinctrl/tcc/pinctrl-tcc803x.c:226-234. */
	val = readl(pctl->base + bank->reg_offset +
		    TCC803X_GPIO_OUTPUT_ENABLE);
	return (val & BIT(local)) ? GPIO_LINE_DIRECTION_OUT :
				     GPIO_LINE_DIRECTION_IN;
}

static int tcc803x_build_pins(struct tcc803x_pinctrl *pctl)
{
	unsigned int bank_idx, pin_idx, n = 0;

	pctl->npins = 0;
	for (bank_idx = 0; bank_idx < TCC803X_NBANKS; bank_idx++)
		pctl->npins += tcc803x_banks[bank_idx].npins;

	pctl->pins = devm_kcalloc(pctl->dev, pctl->npins, sizeof(*pctl->pins),
				   GFP_KERNEL);
	if (!pctl->pins)
		return -ENOMEM;

	for (bank_idx = 0; bank_idx < TCC803X_NBANKS; bank_idx++) {
		struct tcc803x_bank *bank = &tcc803x_banks[bank_idx];

		bank->pin_base = n;

		for (pin_idx = 0; pin_idx < bank->npins; pin_idx++, n++) {
			pctl->pins[n].number = n;
			pctl->pins[n].name = devm_kasprintf(pctl->dev,
				GFP_KERNEL, "%s-%u", bank->name, pin_idx);
			if (!pctl->pins[n].name)
				return -ENOMEM;
		}
	}

	/* gpsd0 is tcc803x_banks[7]; see the comment above the pins array. */
	tcc803x_uart0_pins[0] = tcc803x_banks[7].pin_base + 11;
	tcc803x_uart0_pins[1] = tcc803x_banks[7].pin_base + 12;

	return 0;
}

static int tcc803x_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tcc803x_pinctrl *pctl;
	int ret;

	pctl = devm_kzalloc(dev, sizeof(*pctl), GFP_KERNEL);
	if (!pctl)
		return -ENOMEM;

	pctl->dev = dev;
	spin_lock_init(&pctl->lock);

	/* reg[0] = main block (GPA..GPSD1). reg[1] (PM block) is unused by
	 * this skeleton; see the KNOWN GAP note at the top of this file.
	 */
	pctl->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pctl->base))
		return PTR_ERR(pctl->base);

	ret = tcc803x_build_pins(pctl);
	if (ret)
		return ret;

	pctl->desc.name = dev_name(dev);
	pctl->desc.owner = THIS_MODULE;
	pctl->desc.pctlops = &tcc803x_pctl_ops;
	pctl->desc.pmxops = &tcc803x_pmx_ops;
	pctl->desc.pins = pctl->pins;
	pctl->desc.npins = pctl->npins;

	pctl->pctldev = devm_pinctrl_register(dev, &pctl->desc, pctl);
	if (IS_ERR(pctl->pctldev))
		return PTR_ERR(pctl->pctldev);

	pctl->gpio_chip.label = dev_name(dev);
	pctl->gpio_chip.parent = dev;
	pctl->gpio_chip.owner = THIS_MODULE;
	pctl->gpio_chip.request = gpiochip_generic_request;
	pctl->gpio_chip.free = gpiochip_generic_free;
	pctl->gpio_chip.get_direction = tcc803x_gpio_get_direction;
	pctl->gpio_chip.direction_input = tcc803x_gpio_direction_input;
	pctl->gpio_chip.direction_output = tcc803x_gpio_direction_output;
	pctl->gpio_chip.get = tcc803x_gpio_get;
	pctl->gpio_chip.set = tcc803x_gpio_set;
	pctl->gpio_chip.base = -1;
	pctl->gpio_chip.ngpio = pctl->npins;
	pctl->gpio_chip.fwnode = dev_fwnode(dev);

	return devm_gpiochip_add_data(dev, &pctl->gpio_chip, pctl);
}

static const struct of_device_id tcc803x_pinctrl_of_match[] = {
	{ .compatible = "telechips,tcc803x-pinctrl" },
	{ }
};
MODULE_DEVICE_TABLE(of, tcc803x_pinctrl_of_match);

static struct platform_driver tcc803x_pinctrl_driver = {
	.probe = tcc803x_pinctrl_probe,
	.driver = {
		.name = "tcc803x-pinctrl",
		.of_match_table = tcc803x_pinctrl_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(tcc803x_pinctrl_driver);
