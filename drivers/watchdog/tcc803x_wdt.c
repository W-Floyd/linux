// SPDX-License-Identifier: GPL-2.0
/*
 * Telechips TCC803x PMU watchdog.
 *
 * The watchdog belongs to the secure monitor: the vendor driver
 * (drivers/watchdog/tcc_wdt.c) sets it up, starts, stops and pings it with
 * SiP calls (SIP_DEV_WATCHDOG, 0x8200_2000 + n), not register writes. The
 * vendor kernel starts it before a kexec'd kernel runs, and a kernel that
 * does not ping it gets reset by the PMU, which also clears RAM. This driver
 * takes the running watchdog over: the watchdog core keeps pinging it while
 * nothing holds /dev/watchdog.
 */

#include <linux/arm-smccc.h>
#include <linux/bits.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>

#define SIP_WATCHDOG_SETUP	0x82002000
#define SIP_WATCHDOG_START	0x82002001
#define SIP_WATCHDOG_STOP	0x82002002
#define SIP_WATCHDOG_PING	0x82002003

#define WDT_PMU_EN		BIT(31)

/* The vendor programs 20 s as about 6 s of real time: ping well inside that. */
#define TCC_WDT_TIMEOUT		20
#define TCC_WDT_HW_MS		2000

struct tcc803x_wdt {
	struct watchdog_device wdd;
	u32 rate;
	u32 clear_bit;
};

/*
 * Besides the PMU watchdog there is a timer watchdog (TWDCFG), which the
 * vendor kernel enables with its interrupt (SPI 39) and services by pinging
 * the PMU one. Its interrupt has no handler here, so it is switched off, as
 * the vendor's stop does: bit 0 enables the timer, bit 3 its interrupt.
 */
#define TWDCFG_EN_IEN	(BIT(0) | BIT(3))

static int tcc803x_wdt_ping(struct watchdog_device *wdd)
{
	struct tcc803x_wdt *w = watchdog_get_drvdata(wdd);
	struct arm_smccc_res res;

	arm_smccc_smc(SIP_WATCHDOG_PING, 0, BIT(w->clear_bit), 0, 0, 0, 0, 0, &res);
	return 0;
}

static int tcc803x_wdt_set_timeout(struct watchdog_device *wdd, unsigned int t)
{
	struct tcc803x_wdt *w = watchdog_get_drvdata(wdd);
	struct arm_smccc_res res;

	arm_smccc_smc(SIP_WATCHDOG_SETUP, 0, 0, 0, t * w->rate, 0, 0, 0, &res);
	wdd->timeout = t;
	return 0;
}

static int tcc803x_wdt_start(struct watchdog_device *wdd)
{
	struct arm_smccc_res res;

	tcc803x_wdt_set_timeout(wdd, wdd->timeout);
	tcc803x_wdt_ping(wdd);
	arm_smccc_smc(SIP_WATCHDOG_START, 0, WDT_PMU_EN, 0, 0, 0, 0, 0, &res);
	return 0;
}

static int tcc803x_wdt_stop(struct watchdog_device *wdd)
{
	struct arm_smccc_res res;

	tcc803x_wdt_ping(wdd);
	arm_smccc_smc(SIP_WATCHDOG_STOP, 0, WDT_PMU_EN, 0, 0, 0, 0, 0, &res);
	return 0;
}

static const struct watchdog_info tcc803x_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity = "TCC803x PMU watchdog",
};

static const struct watchdog_ops tcc803x_wdt_ops = {
	.owner = THIS_MODULE,
	.start = tcc803x_wdt_start,
	.stop = tcc803x_wdt_stop,
	.ping = tcc803x_wdt_ping,
	.set_timeout = tcc803x_wdt_set_timeout,
};

static int tcc803x_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tcc803x_wdt *w;
	struct resource *res;

	w = devm_kzalloc(dev, sizeof(*w), GFP_KERNEL);
	if (!w)
		return -ENOMEM;

	if (of_property_read_u32(dev->of_node, "clock-frequency", &w->rate))
		w->rate = 24000000;
	of_property_read_u32(dev->of_node, "telechips,clear-bit", &w->clear_bit);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res) {
		void __iomem *twd = devm_ioremap(dev, res->start, resource_size(res));

		if (twd) {
			dev_info(dev, "timer watchdog config %#x, switching it off\n", readl(twd));
			writel(readl(twd) & ~TWDCFG_EN_IEN, twd);
			writel(0, twd);
		}
	}

	w->wdd.info = &tcc803x_wdt_info;
	w->wdd.ops = &tcc803x_wdt_ops;
	w->wdd.timeout = TCC_WDT_TIMEOUT;
	w->wdd.min_timeout = 1;
	w->wdd.max_hw_heartbeat_ms = TCC_WDT_HW_MS;
	w->wdd.parent = dev;
	watchdog_set_drvdata(&w->wdd, w);
	watchdog_stop_on_reboot(&w->wdd);

	/*
	 * Running already (the vendor kernel started it): have the core ping
	 * it until something opens /dev/watchdog, and restart it in the
	 * driver's own terms now so a reset is never close.
	 */
	set_bit(WDOG_HW_RUNNING, &w->wdd.status);
	tcc803x_wdt_start(&w->wdd);

	return devm_watchdog_register_device(dev, &w->wdd);
}

static const struct of_device_id tcc803x_wdt_of_match[] = {
	{ .compatible = "telechips,tcc803x-wdt" },
	{}
};
MODULE_DEVICE_TABLE(of, tcc803x_wdt_of_match);

static struct platform_driver tcc803x_wdt_driver = {
	.driver = {
		.name = "tcc803x-wdt",
		.of_match_table = tcc803x_wdt_of_match,
	},
	.probe = tcc803x_wdt_probe,
};
module_platform_driver(tcc803x_wdt_driver);

MODULE_DESCRIPTION("Telechips TCC803x PMU watchdog (secure monitor)");
MODULE_AUTHOR("William Floyd <git@notmy.space>");
MODULE_LICENSE("GPL");
