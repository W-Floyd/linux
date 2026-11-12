// SPDX-License-Identifier: GPL-2.0-only
/*
 * Telechips TCC803x: the CPU and bus clocks (the CKC's CLKCTRL channels) belong
 * to the secure monitor. The vendor kernel reads and sets them only through SiP
 * SMC calls (SIP_CLK_*); the registers at the CKC base are not the kernel's to
 * touch.
 *
 * Each channel is registered as a clk_hw, indexed by its CLKCTRL id (one-cell
 * specifier): the rate is read from the monitor, never cached. No enable or
 * disable op, so the framework never gates a channel. Rate setting is off
 * unless fbus_set_rate is given (below).
 *
 * At probe it also logs the state of the clock gates the monitor owns (IOBUS,
 * VPUBUS, ISOTOP) and times the CPU with its own cycle counter: the real clock,
 * the cost of a dependent add, and the latency of loads from L1, L2 and DRAM.
 */
#include <linux/arm-smccc.h>
#include <linux/clk-provider.h>
#include <linux/irqflags.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/random.h>
#include <linux/vmalloc.h>

#define SIP_CLK_SET_CLKCTRL	0x82000003
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
	{ 3, "cmbus" },		/* Cortex-M4 */
	{ 4, "mem" },
	{ 5, "vbus" },
	{ 6, "hsio" },
	{ 7, "smu" },
	{ 8, "gpu" },
	{ 9, "ddi" },
	{ 11, "io" },
};

#define TCC803X_FBUS_NR		12	/* CLKCTRL ids 0-11; 1 and 10 unregistered */

/*
 * EXPERIMENTAL, default off: let the framework set a channel's rate with the
 * monitor's SET_CLKCTRL (id, 1, rate) -- the (id, enable, rate) argument
 * shape is the real vendor calling convention, confirmed against vendor
 * clock-driver source, not copied over from SET_PCLKCTRL by assumption.
 * Still gated: neither call has ever been run on this unit for these
 * channels, and a bad CPU or memory-bus rate can hang the head unit of a
 * running car.
 *
 * fbus_set_rate=0: read-only. determine_rate answers the current rate and
 * set_rate refuses, so nothing but GET/IS calls reach the monitor.
 */
static bool fbus_set_rate;
/* 0444, not 0644: a live sysfs flip at runtime could retune the memory bus
 * under a running kernel. Set only at module load (modprobe fbus_set_rate=1).
 */
module_param(fbus_set_rate, bool, 0444);
MODULE_PARM_DESC(fbus_set_rate, "EXPERIMENTAL, untested on hardware: allow setting CPU/GPU/bus clock rates through the monitor (default 0, read-only)");

struct tcc803x_fbus_clk {
	struct clk_hw hw;
	u32 id;
};

#define to_fbus_clk(_hw) container_of(_hw, struct tcc803x_fbus_clk, hw)

static unsigned long sip_call(unsigned long fn, unsigned long a1,
			      unsigned long a2, unsigned long a3)
{
	struct arm_smccc_res res;

	arm_smccc_smc(fn, a1, a2, a3, 0, 0, 0, 0, &res);
	return res.a0;
}

/* The monitor answers an unknown call with a negative SMCCC code. */
static bool sip_failed(unsigned long ret)
{
	return (long)ret < 0;
}

/* IS_CLKCTRL answers 1 for enabled, unlike IS_IOBUS. */
static int fbus_is_enabled(struct clk_hw *hw)
{
	return sip_call(SIP_CLK_IS_CLKCTRL, to_fbus_clk(hw)->id, 0, 0) == 1;
}

static unsigned long fbus_recalc_rate(struct clk_hw *hw,
				      unsigned long parent_rate)
{
	unsigned long rate = sip_call(SIP_CLK_GET_CLKCTRL, to_fbus_clk(hw)->id, 0, 0);

	return sip_failed(rate) ? 0 : rate;
}

/*
 * The monitor picks the PLL and divider, and what it lands on for a given
 * request is unknown (unconfirmed whether it rounds up, down, or reprograms a
 * shared PLL): pass the request through and let recalc_rate read back the result.
 */
static int fbus_determine_rate(struct clk_hw *hw, struct clk_rate_request *req)
{
	if (!READ_ONCE(fbus_set_rate))
		req->rate = fbus_recalc_rate(hw, 0);
	return 0;
}

static int fbus_set_rate_op(struct clk_hw *hw, unsigned long rate,
			    unsigned long parent_rate)
{
	struct tcc803x_fbus_clk *c = to_fbus_clk(hw);

	if (!READ_ONCE(fbus_set_rate)) {
		pr_warn_once("tcc803x-fbus: rate change refused, fbus_set_rate=0\n");
		return -EPERM;
	}
	if (sip_failed(sip_call(SIP_CLK_SET_CLKCTRL, c->id, 1, rate)))
		return -EIO;
	return 0;
}

static const struct clk_ops fbus_ops = {
	.is_enabled = fbus_is_enabled,
	.recalc_rate = fbus_recalc_rate,
	.determine_rate = fbus_determine_rate,
	.set_rate = fbus_set_rate_op,
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
	struct device *dev = &pdev->dev;
	struct clk_hw_onecell_data *data;
	struct arm_smccc_res on;
	unsigned int i;
	int ret;

	data = devm_kzalloc(dev, struct_size(data, hws, TCC803X_FBUS_NR), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->num = TCC803X_FBUS_NR;
	for (i = 0; i < TCC803X_FBUS_NR; i++)
		data->hws[i] = ERR_PTR(-ENOENT);

	for (i = 0; i < ARRAY_SIZE(tcc803x_fbus); i++) {
		struct tcc803x_fbus_clk *c;
		struct clk_init_data init = {
			.name = devm_kasprintf(dev, GFP_KERNEL, "fbus_%s",
					       tcc803x_fbus[i].name),
			.ops = &fbus_ops,
			.flags = CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
		};

		c = devm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
		if (!c || !init.name)
			return -ENOMEM;
		c->id = tcc803x_fbus[i].id;
		c->hw.init = &init;

		ret = devm_clk_hw_register(dev, &c->hw);
		if (ret)
			return ret;
		data->hws[c->id] = &c->hw;

		dev_info(dev, "%-5s %10lu Hz, %s\n", tcc803x_fbus[i].name,
			 clk_hw_get_rate(&c->hw),
			 fbus_is_enabled(&c->hw) ? "enabled" : "disabled");
	}

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, data);
	if (ret)
		return ret;

	/* IS_* answers 1 for a gate that is powered down (the vendor's clk.c). */
	for (i = 0; i < ARRAY_SIZE(tcc803x_gates); i++) {
		arm_smccc_smc(tcc803x_gates[i].fn, tcc803x_gates[i].id,
			      0, 0, 0, 0, 0, 0, &on);
		dev_info(dev, "gate %-17s %s (a0 %#lx)\n", tcc803x_gates[i].name,
			 on.a0 == 1 ? "powered down" : "up", on.a0);
	}

	tcc803x_fbus_measure(dev);

	dev_info(dev, "CPU and bus clocks via the secure monitor%s\n",
		 fbus_set_rate ? ", rate setting ENABLED (experimental)" : ", read-only");
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
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(tcc803x_fbus_driver);

MODULE_DESCRIPTION("Telechips TCC803x CPU and bus clocks (secure monitor)");
MODULE_LICENSE("GPL");
