// SPDX-License-Identifier: GPL-2.0-only
/*
 * Telechips TCC803x RTC.
 *
 * Register layout and access sequences follow the vendor driver
 * (drivers/rtc/tcc_rtc.c, drivers/rtc/tcc/tca_rtc.[ch]). Time only: the vendor
 * driver leaves the alarm interrupt unrequested too.
 *
 * The block's two clock gates (IOBUS_RTC, ISOIP_TOP_RTC) sit behind the secure
 * monitor, reachable only through SiP SMC calls (vendor drivers/clk/tcc/clk.c,
 * CONFIG_TEE path). No clock provider models them, so the driver asks the
 * monitor itself, and only touches the registers once the monitor says the
 * block is powered.
 */
#include <linux/arm-smccc.h>
#include <linux/bcd.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>

#define SIP_CLK_ENABLE_IOBUS	0x82000011
#define SIP_CLK_IS_IOBUS	0x82000013
#define SIP_CLK_ENABLE_ISOTOP	0x82000020
#define SIP_CLK_IS_ISOTOP	0x82000022

#define IOBUS_RTC		138
#define ISOIP_TOP_RTC		5

#define RTCCON			0x00
#define  RTCCON_HALT		BIT(0)
#define  RTCCON_WREN		BIT(1)
#define  RTCCON_CNTRST		BIT(4)
#define INTCON			0x04
#define  INTCON_WREN		BIT(0)
#define  INTCON_PROT		BIT(15)
#define BCDSEC			0x28
#define BCDMIN			0x2c
#define BCDHOUR			0x30
#define BCDDATE			0x34
#define BCDDAY			0x38
#define BCDMON			0x3c
#define BCDYEAR			0x40

struct tcc803x_rtc {
	void __iomem *base;
};

static void tcc803x_rtc_set(struct tcc803x_rtc *rtc, unsigned int reg, u32 bits)
{
	writel(readl(rtc->base + reg) | bits, rtc->base + reg);
}

static void tcc803x_rtc_clr(struct tcc803x_rtc *rtc, unsigned int reg, u32 bits)
{
	writel(readl(rtc->base + reg) & ~bits, rtc->base + reg);
}

static u8 tcc803x_rtc_bcd(struct tcc803x_rtc *rtc, unsigned int reg, u32 mask)
{
	u32 v = readl(rtc->base + reg) & mask;

	/* A nibble above 9 is garbage; 0xff fails rtc_valid_tm(). */
	if ((v & 0xf) > 9 || ((v >> 4) & 0xf) > 9)
		return 0xff;
	return bcd2bin(v);
}

static int tcc803x_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct tcc803x_rtc *rtc = dev_get_drvdata(dev);
	u32 year;
	u8 sec;

	tcc803x_rtc_set(rtc, RTCCON, RTCCON_WREN);
	tcc803x_rtc_set(rtc, INTCON, INTCON_WREN);
	tcc803x_rtc_clr(rtc, RTCCON, RTCCON_CNTRST);

	/* Re-read until the seconds hold still across the other fields. */
	do {
		sec = tcc803x_rtc_bcd(rtc, BCDSEC, 0x7f);
		year = readl(rtc->base + BCDYEAR) & 0xffff;
		tm->tm_mon = tcc803x_rtc_bcd(rtc, BCDMON, 0x1f) - 1;
		tm->tm_mday = tcc803x_rtc_bcd(rtc, BCDDATE, 0x3f);
		tm->tm_hour = tcc803x_rtc_bcd(rtc, BCDHOUR, 0x3f);
		tm->tm_min = tcc803x_rtc_bcd(rtc, BCDMIN, 0x7f);
		tm->tm_sec = tcc803x_rtc_bcd(rtc, BCDSEC, 0x7f);
	} while (tm->tm_sec != sec);

	tcc803x_rtc_clr(rtc, INTCON, INTCON_WREN);
	tcc803x_rtc_clr(rtc, RTCCON, RTCCON_WREN);

	/* Century in the high byte, year in the low byte. */
	tm->tm_year = bcd2bin(year >> 8) * 100 + bcd2bin(year & 0xff) - 1900;

	if (rtc_valid_tm(tm))
		return -EINVAL;

	/* The weekday register's convention is not worth trusting. */
	tm->tm_wday = (int)((rtc_tm_to_time64(tm) / 86400 + 4) % 7);
	return 0;
}

static int tcc803x_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct tcc803x_rtc *rtc = dev_get_drvdata(dev);
	int year = tm->tm_year + 1900;

	tcc803x_rtc_set(rtc, RTCCON, RTCCON_WREN);
	tcc803x_rtc_set(rtc, INTCON, INTCON_WREN);
	tcc803x_rtc_set(rtc, RTCCON, RTCCON_HALT);

	/* Reset the sub-second divider so the count starts on the write. */
	tcc803x_rtc_set(rtc, RTCCON, RTCCON_CNTRST);
	tcc803x_rtc_clr(rtc, RTCCON, RTCCON_CNTRST);
	tcc803x_rtc_set(rtc, RTCCON, RTCCON_CNTRST);

	/* Protection is lifted around the BCD writes (vendor order). */
	tcc803x_rtc_set(rtc, INTCON, INTCON_PROT);
	tcc803x_rtc_clr(rtc, INTCON, INTCON_PROT);

	writel(bin2bcd(tm->tm_sec), rtc->base + BCDSEC);
	writel(bin2bcd(tm->tm_min), rtc->base + BCDMIN);
	writel(bin2bcd(tm->tm_hour), rtc->base + BCDHOUR);
	writel(bin2bcd(tm->tm_wday), rtc->base + BCDDAY);
	writel(bin2bcd(tm->tm_mday), rtc->base + BCDDATE);
	writel(bin2bcd(tm->tm_mon + 1), rtc->base + BCDMON);
	writel(bin2bcd(year / 100) << 8 | bin2bcd(year % 100), rtc->base + BCDYEAR);

	tcc803x_rtc_set(rtc, INTCON, INTCON_PROT);
	/* The vendor leaves CNTRST set until the next read clears it. */
	tcc803x_rtc_clr(rtc, RTCCON, RTCCON_CNTRST);
	tcc803x_rtc_clr(rtc, RTCCON, RTCCON_HALT);

	tcc803x_rtc_clr(rtc, INTCON, INTCON_WREN);
	tcc803x_rtc_clr(rtc, RTCCON, RTCCON_WREN);
	return 0;
}

static const struct rtc_class_ops tcc803x_rtc_ops = {
	.read_time = tcc803x_rtc_read_time,
	.set_time = tcc803x_rtc_set_time,
};

/*
 * IS_* answers 1 for a gate that is powered down (vendor clk.c: is_enabled
 * returns a0 == 1 ? 0 : 1), 0 for one that is on. Anything else is an error
 * or an unknown call (SMCCC returns -1).
 */
static int tcc803x_rtc_gate(struct device *dev, const char *name,
			    unsigned long is_fn, unsigned long enable_fn,
			    unsigned long id)
{
	struct arm_smccc_res res;

	arm_smccc_smc(is_fn, id, 0, 0, 0, 0, 0, 0, &res);
	dev_info(dev, "%s gate: monitor says %#lx (1 = powered down)\n", name, res.a0);
	if (res.a0 == 0)
		return 0;
	if (res.a0 != 1)
		return -ENODEV;

	arm_smccc_smc(enable_fn, id, 0, 0, 0, 0, 0, 0, &res);
	arm_smccc_smc(is_fn, id, 0, 0, 0, 0, 0, 0, &res);
	dev_info(dev, "%s gate after enable: %#lx\n", name, res.a0);
	return res.a0 == 0 ? 0 : -EIO;
}

static int tcc803x_rtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tcc803x_rtc *rtc;
	struct rtc_device *rd;
	int ret;

	rtc = devm_kzalloc(dev, sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	rtc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(rtc->base))
		return PTR_ERR(rtc->base);

	/* No register access before both gates are confirmed on: an unpowered block can hang the bus. */
	ret = tcc803x_rtc_gate(dev, "iobus", SIP_CLK_IS_IOBUS,
			       SIP_CLK_ENABLE_IOBUS, IOBUS_RTC);
	if (ret)
		return dev_err_probe(dev, ret, "RTC bus clock unavailable\n");
	ret = tcc803x_rtc_gate(dev, "isoip", SIP_CLK_IS_ISOTOP,
			       SIP_CLK_ENABLE_ISOTOP, ISOIP_TOP_RTC);
	if (ret)
		return dev_err_probe(dev, ret, "RTC isolation gate unavailable\n");

	platform_set_drvdata(pdev, rtc);

	rd = devm_rtc_allocate_device(dev);
	if (IS_ERR(rd))
		return PTR_ERR(rd);

	rd->ops = &tcc803x_rtc_ops;
	/* The vendor driver's validity window. */
	rd->range_min = mktime64(1980, 1, 1, 0, 0, 0);
	rd->range_max = mktime64(2037, 12, 31, 23, 59, 59);

	dev_info(dev, "first register access\n");
	return devm_rtc_register_device(rd);
}

static const struct of_device_id tcc803x_rtc_of_match[] = {
	{ .compatible = "telechips,tcc803x-rtc" },
	{ }
};
MODULE_DEVICE_TABLE(of, tcc803x_rtc_of_match);

static struct platform_driver tcc803x_rtc_driver = {
	.probe = tcc803x_rtc_probe,
	.driver = {
		.name = "rtc-tcc803x",
		.of_match_table = tcc803x_rtc_of_match,
	},
};
module_platform_driver(tcc803x_rtc_driver);

MODULE_DESCRIPTION("Telechips TCC803x RTC driver");
MODULE_LICENSE("GPL");
