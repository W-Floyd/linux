// SPDX-License-Identifier: GPL-2.0-only
/*
 * Telechips TCC803x: the CPU and bus clocks (the CKC's CLKCTRL channels) belong
 * to the secure monitor. The vendor kernel reads and sets them only through SiP
 * SMC calls (SIP_CLK_*); the registers at the CKC base are not the kernel's to
 * touch.
 *
 * This is the read-only half, a diagnostic for bring-up: at probe it asks the
 * monitor for each channel's rate and enable state, and for the state of the
 * clock gates the monitor owns (IOBUS, VPUBUS, ISOTOP), and says what it got.
 * It then times the CPU with its own cycle counter: the real clock, the cost of
 * a dependent add, and the latency of loads from L1, L2 and DRAM.
 */
#include <linux/arm-smccc.h>
#include <linux/irqflags.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/random.h>
#include <linux/vmalloc.h>

#define SIP_CLK_GET_CLKCTRL	0x82000004
#define SIP_CLK_IS_CLKCTRL	0x82000007
#define SIP_CLK_IS_IOBUS	0x82000013
#define SIP_CLK_IS_VPUBUS	0x8200001b
#define SIP_CLK_IS_ISOTOP	0x82000022

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

/* The gates the RTC, audio, UARTs and the video codecs sit behind. */
static const struct {
	u32 fn;
	unsigned int id;
	const char *name;
} tcc803x_gates[] = {
	{ SIP_CLK_IS_IOBUS, 138, "iobus rtc" },
	{ SIP_CLK_IS_ISOTOP, 5, "isotop rtc" },
	{ SIP_CLK_IS_IOBUS, 21, "iobus adma0" },
	{ SIP_CLK_IS_IOBUS, 22, "iobus dai0" },
	{ SIP_CLK_IS_IOBUS, 25, "iobus sr-adma0" },
	{ SIP_CLK_IS_IOBUS, 26, "iobus sr-dai0" },
	{ SIP_CLK_IS_IOBUS, 64, "iobus uart0" },
	{ SIP_CLK_IS_IOBUS, 65, "iobus uart1" },
	{ SIP_CLK_IS_IOBUS, 66, "iobus uart2" },
	{ SIP_CLK_IS_IOBUS, 67, "iobus uart3" },
	{ SIP_CLK_IS_VPUBUS, 0, "vpubus jpeg" },
	{ SIP_CLK_IS_VPUBUS, 1, "vpubus coda-bus" },
	{ SIP_CLK_IS_VPUBUS, 2, "vpubus coda-core" },
	{ SIP_CLK_IS_VPUBUS, 3, "vpubus hevc-bus" },
	{ SIP_CLK_IS_VPUBUS, 4, "vpubus hevc-core" },
};

struct tcc803x_pmu {
	u32 pmcr, cntens;
};

/* The cycle counter and event counter 0 (INST_RETIRED) on, as found otherwise. */
static void tcc803x_pmu_on(struct tcc803x_pmu *s)
{
	asm volatile("mrc p15, 0, %0, c9, c12, 0" : "=r" (s->pmcr));
	asm volatile("mrc p15, 0, %0, c9, c12, 1" : "=r" (s->cntens));
	asm volatile("mcr p15, 0, %0, c9, c12, 5" : : "r" (0));	/* PMSELR: counter 0 */
	asm volatile("mcr p15, 0, %0, c9, c13, 1" : : "r" (0x08));	/* INST_RETIRED */
	asm volatile("mcr p15, 0, %0, c9, c12, 1" : : "r" (BIT(31) | BIT(0)));
	asm volatile("mcr p15, 0, %0, c9, c12, 0" : : "r" (s->pmcr | BIT(0) | BIT(1) | BIT(2)));
}

static void tcc803x_pmu_off(const struct tcc803x_pmu *s)
{
	asm volatile("mcr p15, 0, %0, c9, c12, 0" : : "r" (s->pmcr));
	asm volatile("mcr p15, 0, %0, c9, c12, 2" : : "r" (~s->cntens & (BIT(31) | BIT(0))));
}

static inline u32 tcc803x_cycles(void)
{
	u32 c;

	asm volatile("mrc p15, 0, %0, c9, c13, 0" : "=r" (c));
	return c;
}

/* Cycles for 8 * n dependent adds: the core's best case, one per cycle. */
static u32 tcc803x_alu(unsigned long n)
{
	unsigned long flags, x = 0, i;
	u32 c0, c1;

	local_irq_save(flags);
	c0 = tcc803x_cycles();
	for (i = 0; i < n; i++)
		asm volatile("add %0, %0, #1\n\tadd %0, %0, #1\n\t"
			     "add %0, %0, #1\n\tadd %0, %0, #1\n\t"
			     "add %0, %0, #1\n\tadd %0, %0, #1\n\t"
			     "add %0, %0, #1\n\tadd %0, %0, #1" : "+r" (x));
	c1 = tcc803x_cycles();
	local_irq_restore(flags);
	return c1 - c0;
}

/*
 * Cycles per dependent load through a random cycle of cache lines in a buffer
 * of the given size: L1 for a few KiB, then L2, then DRAM.
 */
static void tcc803x_chase(struct device *dev, size_t bytes)
{
	const size_t line = 64, words = line / sizeof(u32);
	const unsigned long steps = 100000;
	size_t lines = bytes / line, i;
	u32 *buf, *order, *p, cyc, c0;
	unsigned long flags;

	buf = vmalloc(bytes);
	order = vmalloc(lines * sizeof(*order));
	if (!buf || !order)
		goto out;

	for (i = 0; i < lines; i++)
		order[i] = i;
	for (i = lines - 1; i > 0; i--) {	/* Sattolo: one cycle through every line */
		u32 j = get_random_u32_below(i);

		swap(order[i], order[j]);
	}
	for (i = 0; i < lines; i++)
		buf[order[i] * words] = (u32)(uintptr_t)&buf[order[(i + 1) % lines] * words];

	p = buf;
	for (i = 0; i < lines; i++)	/* warm */
		p = (u32 *)(uintptr_t)*p;

	local_irq_save(flags);
	c0 = tcc803x_cycles();
	for (i = 0; i < steps; i++)
		p = (u32 *)(uintptr_t)*p;
	cyc = tcc803x_cycles() - c0;
	local_irq_restore(flags);

	dev_info(dev, "chase %5zu KiB: %u.%02u cycles per load (%p)\n", bytes / 1024,
		 cyc / (u32)steps, (u32)div_u64((u64)cyc * 100, steps) % 100, p);
out:
	vfree(order);
	vfree(buf);
}

/*
 * What the CPU really runs at, from its own cycle counter against the arch
 * timer: the monitor's figure is only what it was told. Then the loop IPC, the
 * add and load costs. The PMU is left as found.
 */
static void tcc803x_fbus_measure(struct device *dev)
{
	static const size_t sizes[] = { 16 << 10, 128 << 10, 1 << 20, 8 << 20 };
	struct tcc803x_pmu pmu;
	u32 inst, cycles, ipc100, alu;
	const unsigned long alu_n = 1000000;
	u64 t0, t1;
	unsigned int i;

	preempt_disable();
	tcc803x_pmu_on(&pmu);

	t0 = ktime_get_ns();
	do {
		t1 = ktime_get_ns();
	} while (t1 - t0 < 20 * NSEC_PER_MSEC);

	cycles = tcc803x_cycles();
	asm volatile("mrc p15, 0, %0, c9, c13, 2" : "=r" (inst));
	ipc100 = cycles ? div_u64((u64)inst * 100, cycles) : 0;
	dev_info(dev, "cpu0 measured: %llu kHz, %u instructions in %llu us (IPC %u.%02u)\n",
		 div64_u64((u64)cycles * 1000000, t1 - t0), inst, div_u64(t1 - t0, 1000),
		 ipc100 / 100, ipc100 % 100);

	alu = tcc803x_alu(alu_n);
	dev_info(dev, "alu: %u cycles for %lu dependent adds (%u.%02u per add)\n", alu,
		 alu_n * 8, alu / (u32)(alu_n * 8),
		 (u32)div_u64((u64)alu * 100, alu_n * 8) % 100);

	for (i = 0; i < ARRAY_SIZE(sizes); i++)
		tcc803x_chase(dev, sizes[i]);

	tcc803x_pmu_off(&pmu);
	preempt_enable();
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

	/* IS_* answers 1 for a gate that is powered down (the vendor's clk.c). */
	for (i = 0; i < ARRAY_SIZE(tcc803x_gates); i++) {
		arm_smccc_smc(tcc803x_gates[i].fn, tcc803x_gates[i].id,
			      0, 0, 0, 0, 0, 0, &on);
		dev_info(&pdev->dev, "gate %-17s %s (a0 %#lx)\n", tcc803x_gates[i].name,
			 on.a0 == 1 ? "powered down" : "up", on.a0);
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
