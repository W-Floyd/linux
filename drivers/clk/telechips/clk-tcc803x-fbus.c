// SPDX-License-Identifier: GPL-2.0-only
/*
 * Telechips TCC803x: the CPU and bus clocks (the CKC's CLKCTRL channels) belong
 * to the secure monitor. The vendor kernel reads and sets them only through SiP
 * SMC calls (SIP_CLK_*); the registers at the CKC base are not the kernel's to
 * touch.
 *
 * This is the read-only half: it asks the monitor for each channel's rate and
 * enable state and says what it got, to find what the CPU runs at after the
 * previous kernel left it.
 */
#include <linux/arm-smccc.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#define SIP_CLK_GET_CLKCTRL	0x82000004
#define SIP_CLK_IS_CLKCTRL	0x82000007

static const struct {
	unsigned int id;
	const char *name;
} tcc803x_fbus[] = {
	{ 0, "cpu0" },
	{ 2, "cbus" },
	{ 4, "mem" },
	{ 5, "vbus" },
	{ 6, "hsio" },
	{ 7, "smu" },
	{ 8, "gpu" },
	{ 9, "ddi" },
	{ 11, "io" },
};

static int tcc803x_fbus_probe(struct platform_device *pdev)
{
	struct arm_smccc_res rate, on;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(tcc803x_fbus); i++) {
		arm_smccc_smc(SIP_CLK_GET_CLKCTRL, tcc803x_fbus[i].id,
			      0, 0, 0, 0, 0, 0, &rate);
		arm_smccc_smc(SIP_CLK_IS_CLKCTRL, tcc803x_fbus[i].id,
			      0, 0, 0, 0, 0, 0, &on);
		dev_info(&pdev->dev, "%-5s %10lu Hz, enable %#lx\n",
			 tcc803x_fbus[i].name, rate.a0, on.a0);
	}

	return 0;
}

static const struct of_device_id tcc803x_fbus_of_match[] = {
	{ .compatible = "telechips,tcc803x-fbus" },
	{ }
};
MODULE_DEVICE_TABLE(of, tcc803x_fbus_of_match);

static struct platform_driver tcc803x_fbus_driver = {
	.probe = tcc803x_fbus_probe,
	.driver = {
		.name = "tcc803x-fbus",
		.of_match_table = tcc803x_fbus_of_match,
	},
};
module_platform_driver(tcc803x_fbus_driver);

MODULE_DESCRIPTION("Telechips TCC803x CPU and bus clocks (secure monitor)");
MODULE_LICENSE("GPL");
