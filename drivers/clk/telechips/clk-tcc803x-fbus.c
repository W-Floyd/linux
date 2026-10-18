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
#include <linux/ktime.h>
#include <linux/math64.h>
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

/*
 * What the CPU really runs at, from its own cycle counter against the arch
 * timer: the monitor's figure is only what it was told. Also retired
 * instructions (event 0x08) over the same time, for the IPC of the loop. The
 * PMU is left as found.
 */
static void tcc803x_fbus_measure(struct device *dev)
{
	u32 pmcr, cntens, inst, cycles, ipc100;
	u64 t0, t1;

	preempt_disable();
	asm volatile("mrc p15, 0, %0, c9, c12, 0" : "=r" (pmcr));
	asm volatile("mrc p15, 0, %0, c9, c12, 1" : "=r" (cntens));
	asm volatile("mcr p15, 0, %0, c9, c12, 5" : : "r" (0));	/* PMSELR: counter 0 */
	asm volatile("mcr p15, 0, %0, c9, c13, 1" : : "r" (0x08));	/* INST_RETIRED */
	asm volatile("mcr p15, 0, %0, c9, c12, 1" : : "r" (BIT(31) | BIT(0)));
	asm volatile("mcr p15, 0, %0, c9, c12, 0" : : "r" (pmcr | BIT(0) | BIT(1) | BIT(2)));

	t0 = ktime_get_ns();
	do {
		t1 = ktime_get_ns();
	} while (t1 - t0 < 20 * NSEC_PER_MSEC);

	asm volatile("mrc p15, 0, %0, c9, c13, 0" : "=r" (cycles));
	asm volatile("mrc p15, 0, %0, c9, c13, 2" : "=r" (inst));
	asm volatile("mcr p15, 0, %0, c9, c12, 0" : : "r" (pmcr));
	asm volatile("mcr p15, 0, %0, c9, c12, 2" : : "r" (~cntens & (BIT(31) | BIT(0))));
	preempt_enable();

	ipc100 = cycles ? div_u64((u64)inst * 100, cycles) : 0;
	dev_info(dev, "cpu0 measured: %llu kHz, %u instructions in %llu us (IPC %u.%02u)\n",
		 div64_u64((u64)cycles * 1000000, t1 - t0), inst, div_u64(t1 - t0, 1000),
		 ipc100 / 100, ipc100 % 100);
}

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

	tcc803x_fbus_measure(&pdev->dev);

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
