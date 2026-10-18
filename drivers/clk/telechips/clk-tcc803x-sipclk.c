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
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <dt-bindings/clock/telechips,tcc803x-clks.h>

#define SIP_CLK_SET_PCLKCTRL	0x82000008
#define SIP_CLK_GET_PCLKCTRL	0x82000009
#define SIP_CLK_ENABLE_PERI	0x8200000a
#define SIP_CLK_DISABLE_PERI	0x8200000b
#define SIP_CLK_IS_PERI		0x8200000c
#define SIP_CLK_ENABLE_IOBUS	0x82000011
#define SIP_CLK_IS_IOBUS	0x82000013

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

	dev_info(dev, "audio clocks via the secure monitor%s\n",
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
