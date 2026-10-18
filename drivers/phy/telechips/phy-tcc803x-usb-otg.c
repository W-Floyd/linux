// SPDX-License-Identifier: GPL-2.0-only
/*
 * Telechips TCC803x USB OTG PHY.
 *
 * Register layout and init sequence follow the vendor's tcc_dwc_otg_phy_init()
 * (drivers/usb/phy/phy-tcc-dwc_otg.c, CONFIG_ARCH_TCC803X and
 * CONFIG_TCC_DWC_OTG_HOST_MUX paths) in AVNT5W_TCC803XKK_kernel. Untested on
 * hardware. Unlike the vendor, it does not pulse the HSIO USB20DH bus reset
 * (clk_reset(hclk)): that clock is owned by the firmware here.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

#define PCFG0		0x00
#define  PCFG0_PHYVALID_EN	BIT(20)
#define  PCFG0_SIDDQ		BIT(24)
#define  PCFG0_UNKNOWN25	BIT(25)	/* vendor clears it after the valid wait */
#define  PCFG0_POR		BIT(31)
#define PCFG1		0x04
#define PCFG2		0x08
#define PCFG3		0x0c
#define PCFG4		0x10
#define  PCFG4_DPDM_PULLDOWN	0x1400
#define  PCFG4_PHYVALID		BIT(27)
#define LCFG0		0x18
#define  LCFG0_RESETS		0x30000000	/* prstn, adp_reset_n */
#define OTGMUX		0x28
#define  MUX_OPSEL		BIT(0)	/* 1: OTG core owns the PHY pins */
#define  MUX_O_CLKEN		BIT(1)
#define  MUX_O_RUN		BIT(2)
#define  MUX_H_CLKEN		BIT(3)
#define  MUX_H_RUN		BIT(4)

#define PCFG0_INIT	0x83000025
#define PCFG1_INIT	0xe31c243a
#define PCFG2_INIT	0x75852004

struct tcc803x_otg_phy {
	void __iomem *base;
};

static int tcc803x_otg_phy_init(struct phy *phy)
{
	struct tcc803x_otg_phy *p = phy_get_drvdata(phy);
	u32 val;
	int ret;

	val = readl(p->base + OTGMUX);
	val |= MUX_OPSEL | MUX_O_CLKEN | MUX_O_RUN | MUX_H_CLKEN | MUX_H_RUN;
	writel(val, p->base + OTGMUX);

	writel(PCFG0_INIT, p->base + PCFG0);
	writel(PCFG1_INIT, p->base + PCFG1);
	writel(PCFG2_INIT, p->base + PCFG2);
	writel(0, p->base + PCFG3);
	writel(0, p->base + PCFG4);
	writel(LCFG0_RESETS, p->base + LCFG0);

	/* Power-on reset, with the link held in reset */
	val = readl(p->base + PCFG0);
	writel(val | PCFG0_POR, p->base + PCFG0);
	writel(0, p->base + LCFG0);
	udelay(30);

	val = readl(p->base + PCFG0);
	writel(val & ~PCFG0_POR, p->base + PCFG0);
	val = readl(p->base + PCFG0);
	writel(val & ~PCFG0_SIDDQ, p->base + PCFG0);
	val = readl(p->base + PCFG0);
	writel(val | PCFG0_PHYVALID_EN, p->base + PCFG0);
	val = readl(p->base + PCFG4);
	writel(val | PCFG4_DPDM_PULLDOWN, p->base + PCFG4);

	ret = readl_poll_timeout(p->base + PCFG4, val, val & PCFG4_PHYVALID,
				 5, 50000);
	if (ret)
		dev_err(&phy->dev, "PHY did not report valid\n");

	val = readl(p->base + PCFG0);
	writel(val & ~(PCFG0_PHYVALID_EN | PCFG0_UNKNOWN25), p->base + PCFG0);
	val = readl(p->base + LCFG0);
	writel(val | LCFG0_RESETS, p->base + LCFG0);

	return ret;
}

static int tcc803x_otg_phy_exit(struct phy *phy)
{
	struct tcc803x_otg_phy *p = phy_get_drvdata(phy);
	u32 val;

	val = readl(p->base + PCFG0);
	writel(val | PCFG0_POR | PCFG0_SIDDQ, p->base + PCFG0);

	return 0;
}

static const struct phy_ops tcc803x_otg_phy_ops = {
	.init	= tcc803x_otg_phy_init,
	.exit	= tcc803x_otg_phy_exit,
	.owner	= THIS_MODULE,
};

static int tcc803x_otg_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tcc803x_otg_phy *p;
	struct phy_provider *provider;
	struct gpio_desc *vbus;
	struct phy *phy;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(p->base))
		return PTR_ERR(p->base);

	/* Peripheral only: the host supplies VBUS, never back-feed it. */
	vbus = devm_gpiod_get_optional(dev, "vbus", GPIOD_OUT_LOW);
	if (IS_ERR(vbus))
		return dev_err_probe(dev, PTR_ERR(vbus), "vbus gpio\n");

	phy = devm_phy_create(dev, NULL, &tcc803x_otg_phy_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);
	phy_set_drvdata(phy, p);

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id tcc803x_otg_phy_of_match[] = {
	{ .compatible = "telechips,tcc803x-dwc-otg-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, tcc803x_otg_phy_of_match);

static struct platform_driver tcc803x_otg_phy_driver = {
	.probe = tcc803x_otg_phy_probe,
	.driver = {
		.name = "phy-tcc803x-usb-otg",
		.of_match_table = tcc803x_otg_phy_of_match,
	},
};
module_platform_driver(tcc803x_otg_phy_driver);

MODULE_DESCRIPTION("Telechips TCC803x USB OTG PHY driver");
MODULE_LICENSE("GPL");
