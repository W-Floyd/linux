// SPDX-License-Identifier: GPL-2.0-only
/*
 * Telechips TCC803x: the audio peripheral clocks and I/O-bus gates, through
 * the secure monitor.
 *
 * On the Cortex-A53 the vendor kernel (CONFIG_TEE=y, tcc_ckc_set_ops() forces
 * ckc_ops = NULL for CONFIG_ARCH_TCC803X) never touches the CKC/IOBUS config
 * registers: every enable, rate and gate goes out as a SiP SMC call
 * (drivers/clk/tcc/clk.c:325-420 PERI, :722-800 IOBUS). IDs are the SIP_CLK_*
 * enum in include/soc/tcc/tcc-sip.h:56-82 counted from SIP_CMD_TAG (0x82000000);
 * 0x82000004/0x82000007 match clk-tcc803x-fbus.c.
 *
 * The I/O-bus gates are never disabled: the vendor does not on AVN
 * (clk.c:70-72, IGNORE_CLK_DISABLE under CONFIG_PLATFORM_AVN).
 *
 * "telechips,read-only": enable/set_rate/disable do nothing and report
 * success, only the queries go to the monitor. For a first boot that must
 * leave the stock clock tree as the previous kernel left it.
 */
#include <linux/arm-smccc.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>

#include <dt-bindings/clock/telechips,tcc803x-clks.h>

#define SIP_CLK_SET_PCLKCTRL	0x82000008
#define SIP_CLK_GET_PCLKCTRL	0x82000009
#define SIP_CLK_ENABLE_PERI	0x8200000a
#define SIP_CLK_DISABLE_PERI	0x8200000b
#define SIP_CLK_IS_PERI		0x8200000c
#define SIP_CLK_ENABLE_IOBUS	0x82000011
#define SIP_CLK_IS_IOBUS	0x82000013
#define SIP_CLK_ENABLE_VPUBUS	0x82000019
#define SIP_CLK_DISABLE_VPUBUS	0x8200001a
#define SIP_CLK_IS_VPUBUS	0x8200001b
#define SIP_CLK_RESET_VPUBUS	0x8200001c

struct tcc803x_sipclk {
	struct clk_hw hw;
	u32 id;
	bool read_only;
};

#define to_sipclk(_hw) container_of(_hw, struct tcc803x_sipclk, hw)

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

static int sipclk_peri_enable(struct clk_hw *hw)
{
	struct tcc803x_sipclk *c = to_sipclk(hw);

	if (!c->read_only)
		sip_call(SIP_CLK_ENABLE_PERI, c->id, 0, 0);
	return 0;
}

static void sipclk_peri_disable(struct clk_hw *hw)
{
	struct tcc803x_sipclk *c = to_sipclk(hw);

	if (!c->read_only)
		sip_call(SIP_CLK_DISABLE_PERI, c->id, 0, 0);
}

static int sipclk_peri_is_enabled(struct clk_hw *hw)
{
	return sip_call(SIP_CLK_IS_PERI, to_sipclk(hw)->id, 0, 0) == 1;
}

static unsigned long sipclk_peri_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	return sip_call(SIP_CLK_GET_PCLKCTRL, to_sipclk(hw)->id, 0, 0);
}

static int sipclk_peri_determine_rate(struct clk_hw *hw,
				      struct clk_rate_request *req)
{
	/* The monitor picks the source and divider; it accepts any rate. */
	return 0;
}

static int sipclk_peri_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct tcc803x_sipclk *c = to_sipclk(hw);

	if (!c->read_only)
		sip_call(SIP_CLK_SET_PCLKCTRL, c->id, 1, rate);
	return 0;
}

static const struct clk_ops sipclk_peri_ops = {
	.enable = sipclk_peri_enable,
	.disable = sipclk_peri_disable,
	.is_enabled = sipclk_peri_is_enabled,
	.recalc_rate = sipclk_peri_recalc_rate,
	.determine_rate = sipclk_peri_determine_rate,
	.set_rate = sipclk_peri_set_rate,
};

static int sipclk_iobus_enable(struct clk_hw *hw)
{
	struct tcc803x_sipclk *c = to_sipclk(hw);

	if (!c->read_only)
		sip_call(SIP_CLK_ENABLE_IOBUS, c->id, 0, 0);
	return 0;
}

/* IS_IOBUS answers "powered down" (clk.c:765-775). */
static int sipclk_iobus_is_enabled(struct clk_hw *hw)
{
	return sip_call(SIP_CLK_IS_IOBUS, to_sipclk(hw)->id, 0, 0) != 1;
}

static const struct clk_ops sipclk_iobus_ops = {
	.enable = sipclk_iobus_enable,
	.is_enabled = sipclk_iobus_is_enabled,
};

/*
 * The video-bus gates, enabled for real even where the node is read-only: a
 * codec that asks for one needs it up (vendor clk.c tcc_vpubus_enable).
 * IS_VPUBUS answers "powered down", like IS_IOBUS.
 *
 * Never disabled. DISABLE_VPUBUS resets and powers the block down (vendor
 * tcc_vpubus_disable: swreset, pwdn), and the coda driver drops its clocks
 * right after starting the BIT processor, which on an i.MX only gates a clock:
 * here it killed the firmware before it ran its first command.
 */
static int sipclk_vpubus_enable(struct clk_hw *hw)
{
	sip_call(SIP_CLK_ENABLE_VPUBUS, to_sipclk(hw)->id, 0, 0);
	return 0;
}

static int sipclk_vpubus_is_enabled(struct clk_hw *hw)
{
	return sip_call(SIP_CLK_IS_VPUBUS, to_sipclk(hw)->id, 0, 0) != 1;
}

static const struct clk_ops sipclk_vpubus_ops = {
	.enable = sipclk_vpubus_enable,
	.is_enabled = sipclk_vpubus_is_enabled,
};

/*
 * One reset line, the CODA. EXPERIMENTAL default: the monitor's RESET_VPUBUS
 * call as the TOPST kernel's reset driver (drivers/reset/reset-telechips.c,
 * telechips,vpubus-reset) uses it: (id, 1) asserts and (id, 0) releases a line,
 * ids 1 and 2 for the CODA bus and core (the TCC805x manual says a codec's
 * resets are asserted and released together; the order and the 10 us here are
 * not from a vendor source). Untested on the unit: it may power the block down
 * the way DISABLE does, which kills a running firmware. A failed call, or a gate
 * left down, is handled: the gates are re-enabled and a failure falls back to
 * the gate power-cycle.
 *
 * vpubus_reset=0: power-cycle the bus and core gates, which brings the BIT
 * processor up cold (a start over a running one fails); the vendor's own
 * disable-then-enable (clk.c tcc_vpubus_disable, then tcc_vpubus_enable), and
 * the one tested on the unit.
 */
static bool vpubus_reset = true;
module_param(vpubus_reset, bool, 0644);
MODULE_PARM_DESC(vpubus_reset, "EXPERIMENTAL: reset the CODA with the monitor's RESET_VPUBUS (default) instead of cycling the gates (0, the tested path)");

static int sipclk_reset(struct reset_controller_dev *rcdev, unsigned long id)
{
	static const u32 gates[] = {
		TCC803X_SIPCLK_VPUBUS_CODA_CORE - TCC803X_SIPCLK_VPUBUS_BASE,
		TCC803X_SIPCLK_VPUBUS_CODA_BUS - TCC803X_SIPCLK_VPUBUS_BASE,
	};
	int i;

	if (id != TCC803X_SIPRST_CODA)
		return -EINVAL;

	if (vpubus_reset) {
		bool failed = false;

		for (i = 0; i < ARRAY_SIZE(gates); i++)
			failed |= sip_failed(sip_call(SIP_CLK_RESET_VPUBUS, gates[i], 1, 0));
		udelay(10);
		for (i = ARRAY_SIZE(gates) - 1; i >= 0; i--)
			failed |= sip_failed(sip_call(SIP_CLK_RESET_VPUBUS, gates[i], 0, 0));
		/* The reset must not leave a gate down: IS_VPUBUS is 1 for down. */
		for (i = 0; i < ARRAY_SIZE(gates); i++)
			if (sip_call(SIP_CLK_IS_VPUBUS, gates[i], 0, 0) == 1)
				sip_call(SIP_CLK_ENABLE_VPUBUS, gates[i], 0, 0);
		if (!failed)
			return 0;
		pr_warn("tcc803x-sipclk: RESET_VPUBUS failed, cycling the gates instead\n");
	}

	for (i = 0; i < ARRAY_SIZE(gates); i++)
		sip_call(SIP_CLK_DISABLE_VPUBUS, gates[i], 0, 0);
	udelay(10);
	for (i = ARRAY_SIZE(gates) - 1; i >= 0; i--)
		sip_call(SIP_CLK_ENABLE_VPUBUS, gates[i], 0, 0);
	return 0;
}

static const struct reset_control_ops sipclk_reset_ops = {
	.reset = sipclk_reset,
};

static const struct {
	const char *name;
	unsigned int idx;
} sipclk_peri[] = {
	{ "sip_mdai0", TCC803X_SIPCLK_PERI_MDAI0 },
	{ "sip_mflt0_dai", TCC803X_SIPCLK_PERI_MFLT0_DAI },
	{ "sip_srch0_core", TCC803X_SIPCLK_PERI_SRCH0_CORE },
	{ "sip_srch0_filter", TCC803X_SIPCLK_PERI_SRCH0_FILTER },
	{ "sip_mspdif2", TCC803X_SIPCLK_PERI_MSPDIF2 },
}, sipclk_iobus[] = {
	{ "sip_iobus_adma0", TCC803X_SIPCLK_IOBUS_ADMA0 },
	{ "sip_iobus_dai0", TCC803X_SIPCLK_IOBUS_DAI0 },
	{ "sip_iobus_sr_adma0", TCC803X_SIPCLK_IOBUS_SR_ADMA0 },
	{ "sip_iobus_sr_dai0", TCC803X_SIPCLK_IOBUS_SR_DAI0 },
	{ "sip_iobus_adma2", TCC803X_SIPCLK_IOBUS_ADMA2 },
	{ "sip_iobus_spdif2", TCC803X_SIPCLK_IOBUS_SPDIF2 },
}, sipclk_vpubus[] = {
	{ "sip_vpubus_coda_bus", TCC803X_SIPCLK_VPUBUS_CODA_BUS },
	{ "sip_vpubus_coda_core", TCC803X_SIPCLK_VPUBUS_CODA_CORE },
};

static int sipclk_register(struct device *dev, struct clk_hw_onecell_data *data,
			   const char *name, unsigned int idx, u32 id,
			   const struct clk_ops *ops, bool read_only)
{
	struct tcc803x_sipclk *c;
	struct clk_init_data init = {
		.name = name,
		.ops = ops,
		.flags = CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
	};
	int ret;

	c = devm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	c->id = id;
	c->read_only = read_only;
	c->hw.init = &init;

	ret = devm_clk_hw_register(dev, &c->hw);
	if (ret)
		return ret;

	data->hws[idx] = &c->hw;
	return 0;
}

static int sipclk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	bool read_only = of_property_read_bool(dev->of_node, "telechips,read-only");
	struct clk_hw_onecell_data *data;
	struct reset_controller_dev *rcdev;
	unsigned int i;
	int ret;

	data = devm_kzalloc(dev, struct_size(data, hws, TCC803X_SIPCLK_NR),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->num = TCC803X_SIPCLK_NR;
	for (i = 0; i < TCC803X_SIPCLK_NR; i++)
		data->hws[i] = ERR_PTR(-ENOENT);

	for (i = 0; i < ARRAY_SIZE(sipclk_peri); i++) {
		ret = sipclk_register(dev, data, sipclk_peri[i].name,
				      sipclk_peri[i].idx, sipclk_peri[i].idx,
				      &sipclk_peri_ops, read_only);
		if (ret)
			return ret;
	}

	for (i = 0; i < ARRAY_SIZE(sipclk_iobus); i++) {
		ret = sipclk_register(dev, data, sipclk_iobus[i].name,
				      sipclk_iobus[i].idx,
				      sipclk_iobus[i].idx - TCC803X_SIPCLK_IOBUS_BASE,
				      &sipclk_iobus_ops, read_only);
		if (ret)
			return ret;
	}

	for (i = 0; i < ARRAY_SIZE(sipclk_vpubus); i++) {
		ret = sipclk_register(dev, data, sipclk_vpubus[i].name,
				      sipclk_vpubus[i].idx,
				      sipclk_vpubus[i].idx - TCC803X_SIPCLK_VPUBUS_BASE,
				      &sipclk_vpubus_ops, false);
		if (ret)
			return ret;
	}

	rcdev = devm_kzalloc(dev, sizeof(*rcdev), GFP_KERNEL);
	if (!rcdev)
		return -ENOMEM;
	rcdev->owner = THIS_MODULE;
	rcdev->ops = &sipclk_reset_ops;
	rcdev->of_node = dev->of_node;
	rcdev->nr_resets = 1;
	ret = devm_reset_controller_register(dev, rcdev);
	if (ret)
		return ret;

	dev_info(dev, "audio and video clocks via the secure monitor%s\n",
		 read_only ? ", read-only" : "");

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, data);
}

static const struct of_device_id sipclk_of_match[] = {
	{ .compatible = "telechips,tcc803x-sipclk" },
	{ }
};
MODULE_DEVICE_TABLE(of, sipclk_of_match);

static struct platform_driver sipclk_driver = {
	.probe = sipclk_probe,
	.driver = {
		.name = "tcc803x-sipclk",
		.of_match_table = sipclk_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(sipclk_driver);

MODULE_DESCRIPTION("Telechips TCC803x audio clocks (secure monitor)");
MODULE_LICENSE("GPL");
