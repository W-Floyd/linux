// SPDX-License-Identifier: GPL-2.0-only
/*
 * SDHCI platform glue for the Telechips TCC803x SD/eMMC host controllers.
 *
 * This is a from-scratch mainline driver, not a port of the vendor driver,
 * but every non-generic register/value below is cited against the real
 * vendor source (Linux 3.18, GPLv2), as released through Hyundai Mobis's
 * open-source compliance portal, reference tree AVNT5W_TCC803XKK_kernel/
 * (paths below are relative to that tree's root):
 *
 *   drivers/mmc/host/sdhci-tcc.c, drivers/mmc/host/sdhci-tcc.h
 *
 * The vendor driver is itself a thin platform-glue layer on top of the
 * same SDHCI core this driver uses (sdhci_pltfm_init()/sdhci_add_host()),
 * confirmed by its Kconfig dependency on MMC_SDHCI_PLTFM and its use of
 * struct sdhci_host -- not a from-scratch register interface.
 *
 * Scope of this driver (see arch/arm/boot/dts/telechips/tcc803x.dtsi and
 * Documentation/devicetree/bindings/mmc/telechips,tcc803x-sdhci.yaml for
 * the DT side of this same citation trail):
 *
 *  - reg[1] (the per-instance slice of the SoC's shared "channel control"
 *    block) is wired in DT as an already channel-offset sub-region
 *    (0x1647_0000 / _0050 / _00a0, stride 0x50), which only matches the
 *    *small, fixed* TCC_SDHC_* register offsets in sdhci-tcc.h (TAPDLY at
 *    0x00 through CD_WP at 0x4C -- all comfortably inside one 0x50 slice).
 *    The vendor driver's *other* layout, TCC803X_SDHC_* (large, channel-
 *    strided offsets reaching 0x12C into one shared, non-channel-offset
 *    iomap, selected at runtime via system_rev == 1, "TCC803x rev. 1"),
 *    does not fit this DT layout at all and is architecturally out of
 *    scope here, not merely deferred. That is also the only place the
 *    vendor driver pokes a custom core-clock divider register
 *    (TCC803X_SDHC_CORE_CLK_REG0/1, sdhci-tcc.c ~line 672-660) or an
 *    HS400 DQS tap register (TCC803X_SDHC_CORE_CLK_REG2) -- on the
 *    system_rev == 0 path this driver implements, the vendor driver sets
 *    the SDHC core clock with a plain clk_set_rate() on the fclk and
 *    never touches a divider register (sdhci-tcc.c ~line 934-936), and
 *    HS400/tuning is not reachable at all. Mainline has no equivalent of
 *    system_rev to select between the two layouts at runtime, so rev.1
 *    support (and the hardware this SoC family needs it for) is left for
 *    a follow-up once that's confirmed against real silicon/board data.
 *  - reg[2] (channel mux) and reg[3] (shared auto-tune-result register)
 *    are not mapped: reg[2] is unpopulated on this board (see the DT
 *    binding doc), and reg[3] only backs a debugfs tuning-result readout
 *    (sdhci_tcc_tune_result_show, sdhci-tcc.c ~line 886-905), not basic
 *    probe or I/O.
 *  - No HS200/HS400/UHS tuning (.platform_execute_tuning) is implemented.
 *    This is a default/legacy-speed driver: it programs the one set of
 *    TAPDLY/CAPREG/DELAY_CON registers the vendor driver itself writes
 *    unconditionally at probe (sdhci_tcc_set_channel_configs(), sdhci-
 *    tcc.c ~line 455-476) -- these are not optional even at legacy speed:
 *    CAPREG0/1 override the SDHCI core's otherwise-broken capabilities
 *    (hence SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN below), and the TAPDLY/
 *    DELAY_CON tap values are the command/data sampling delay the
 *    controller needs to not corrupt I/O even at its power-on defaults.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "sdhci-pltfm.h"

/*
 * Channel-control block register offsets, vendor sdhci-tcc.h ("Telechips
 * SDHC Specific Registers for others (such as TCC803x rev. 0, ...)"
 * section). Relative to reg[1], which DT already offsets per channel.
 */
#define TCC_SDHC_TAPDLY			0x00
#define TCC_SDHC_CAPREG0		0x04
#define TCC_SDHC_CAPREG1		0x08
#define TCC_SDHC_DELAY_CON0		0x28
#define TCC_SDHC_DELAY_CON1		0x2C
#define TCC_SDHC_DELAY_CON2		0x30
#define TCC_SDHC_DELAY_CON3		0x34
#define TCC_SDHC_DELAY_CON4		0x38
#define TCC_SDHC_CD_WP			0x4C

#define TCC_SDHC_TAPDLY_TUNE_CNT(x)	(((x) & 0x3F) << 16)
#define TCC_SDHC_TAPDLY_OTAP_SEL(x)	(((x) & 0x1F) << 8)
#define TCC_SDHC_TAPDLY_OTAP_SEL_MASK	TCC_SDHC_TAPDLY_OTAP_SEL(0x1F)
#define TCC_SDHC_TAPDLY_OTAP_EN(x)	(((x) & 0x1) << 13)
#define TCC_SDHC_TAPDLY_ASYNCWKUP_EN(x)	(((x) & 0x1) << 14)

#define TCC_SDHC_CMDDLY_IN(x)		(((x) & 0xF) << 16)
#define TCC_SDHC_CMDDLY_OUT(x)		(((x) & 0xF) << 20)
#define TCC_SDHC_CMDDLY_EN(x)		(((x) & 0xF) << 24)

#define TCC_SDHC_DATADLY_IN(n, x)	(((x) & 0xF) << (((n) & 0x1) * 12))
#define TCC_SDHC_DATADLY_OUT(n, x)	(((x) & 0xF) << ((((n) & 0x1) * 12) + 4))
#define TCC_SDHC_DATADLY_EN(n, x)	(((x) & 0xF) << ((((n) & 0x1) * 12) + 8))

/*
 * Capability-register override. The SDHCI core's own SDHCI_CAPABILITIES
 * base-clock field is unreliable on this controller (see
 * SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN below); the vendor driver always
 * rewrites CAPREG0/1 to this fixed value at probe (sdhci-tcc.c
 * TCC_SDHC_CAPARG0_DEF/1_DEF) regardless of board or speed mode.
 */
#define TCC_SDHC_CAPARG0_DEF		0xEDFE9970
#define TCC_SDHC_CAPARG1_DEF		0x00000007

/* Vendor sdhci-tcc.h TCC_SDHC_CLKOUTDLY_DEF_TAP/CMDDLY_DEF_TAP/DATADLY_DEF_TAP */
#define TCC_SDHC_CLKOUTDLY_DEF_TAP	8
#define TCC_SDHC_CMDDLY_DEF_TAP		7
#define TCC_SDHC_DATADLY_DEF_TAP	7

struct sdhci_tcc803x {
	void __iomem *chctrl_base;
	struct gpio_desc *reset_gpio;
	u32 controller_id;
};

static inline struct sdhci_tcc803x *to_tcc803x(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	return sdhci_pltfm_priv(pltfm_host);
}

/*
 * Programs the channel's TAPDLY/CMD-DATA delay-control registers and
 * reasserts the CAPREG override. Mirrors the system_rev == 0 branch of
 * sdhci_tcc803x_set_channel_configs() (sdhci-tcc.c ~line 455-476): the
 * only path reachable for the register layout this DT uses (see the
 * file comment above).
 */
static void sdhci_tcc803x_set_channel_configs(struct sdhci_host *host,
					       u32 clk_out_tap, u32 cmd_tap,
					       u32 data_tap)
{
	struct sdhci_tcc803x *tcc = to_tcc803x(host);
	void __iomem *base = tcc->chctrl_base;
	u32 vals;

	writel(TCC_SDHC_CAPARG0_DEF, base + TCC_SDHC_CAPREG0);
	writel(TCC_SDHC_CAPARG1_DEF, base + TCC_SDHC_CAPREG1);

	vals = TCC_SDHC_TAPDLY_TUNE_CNT(16) | TCC_SDHC_TAPDLY_OTAP_EN(1) |
	       TCC_SDHC_TAPDLY_ASYNCWKUP_EN(1);
	vals |= TCC_SDHC_TAPDLY_OTAP_SEL(clk_out_tap);
	writel(vals, base + TCC_SDHC_TAPDLY);

	vals = TCC_SDHC_CMDDLY_IN(cmd_tap) | TCC_SDHC_CMDDLY_OUT(cmd_tap) |
	       TCC_SDHC_CMDDLY_EN(cmd_tap);
	writel(vals, base + TCC_SDHC_DELAY_CON0);

	vals = TCC_SDHC_DATADLY_IN(0, data_tap) | TCC_SDHC_DATADLY_OUT(0, data_tap) |
	       TCC_SDHC_DATADLY_EN(0, data_tap) | TCC_SDHC_DATADLY_IN(1, data_tap) |
	       TCC_SDHC_DATADLY_OUT(1, data_tap) | TCC_SDHC_DATADLY_EN(1, data_tap);
	writel(vals, base + TCC_SDHC_DELAY_CON1);
	writel(vals, base + TCC_SDHC_DELAY_CON2);
	writel(vals, base + TCC_SDHC_DELAY_CON3);
	writel(vals, base + TCC_SDHC_DELAY_CON4);

	/* Clear CD/WP override; card detect comes from the SDHCI core. */
	writel(0, base + TCC_SDHC_CD_WP);
}

/*
 * GPIO-toggled eMMC hardware reset. Board-optional: the vendor driver
 * no-ops when the reset GPIO isn't wired (sdhci_tcc_hw_reset(), sdhci-
 * tcc.c ~line 704-716), same as this does via gpiod's NULL-safe calls.
 * Timing (10us assert, 300-1000us settle) is carried over unchanged.
 */
static void sdhci_tcc803x_hw_reset(struct sdhci_host *host)
{
	struct sdhci_tcc803x *tcc = to_tcc803x(host);

	if (!tcc->reset_gpio)
		return;

	gpiod_set_value_cansleep(tcc->reset_gpio, 1);
	/*
	 * udelay(), not usleep_range(), is intentional here despite the
	 * checkpatch style hint: this is the RST_n assert width, not a
	 * settle wait, and sdhci-tcc.c:715 (vendor) uses the same 10us
	 * busy-wait. usleep_range()'s lower bound is a hint, not a floor -
	 * the scheduler is free to run this task later, which could shrink
	 * the actual assert pulse below what the card needs. A fixed
	 * busy-wait guarantees the minimum width; sleeping is fine for the
	 * settle time after deassert below, where undershooting isn't a
	 * correctness risk.
	 */
	udelay(10);
	gpiod_set_value_cansleep(tcc->reset_gpio, 0);
	usleep_range(300, 1000);
}

static const struct sdhci_ops sdhci_tcc803x_ops = {
	.get_max_clock		= sdhci_pltfm_clk_get_max_clock,
	.set_clock		= sdhci_set_clock,
	.set_bus_width		= sdhci_set_bus_width,
	.reset			= sdhci_reset,
	.hw_reset		= sdhci_tcc803x_hw_reset,
	.set_uhs_signaling	= sdhci_set_uhs_signaling,
};

/*
 * Quirks carried over from sdhci_tcc803x_pdata (sdhci-tcc.c ~line 753-757):
 *  - CAP_CLOCK_BASE_BROKEN: SDHCI_CAPABILITIES' base-clock field is not
 *    trustworthy on this controller; CAPREG0/1 above are the real source
 *    of truth and the core clock rate comes from the fclk itself.
 *  - QUIRK2_PRESET_VALUE_BROKEN: the UHS preset-value registers are not
 *    usable (moot while UHS/tuning isn't implemented, but matches the
 *    vendor-confirmed hardware behaviour).
 *  - QUIRK2_STOP_WITH_TC: this controller needs a STOP command issued
 *    with the Transfer Complete bit, not the generic sequencing.
 */
static const struct sdhci_pltfm_data sdhci_tcc803x_pdata = {
	.ops	= &sdhci_tcc803x_ops,
	.quirks	= SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.quirks2 = SDHCI_QUIRK2_PRESET_VALUE_BROKEN | SDHCI_QUIRK2_STOP_WITH_TC,
};

static int sdhci_tcc803x_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_tcc803x *tcc;
	u32 clk_out_tap, cmd_tap, data_tap;
	int ret;

	host = sdhci_pltfm_init(pdev, &sdhci_tcc803x_pdata, sizeof(*tcc));
	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);
	tcc = sdhci_pltfm_priv(pltfm_host);

	ret = of_property_read_u32(dev->of_node, "telechips,controller-id",
				    &tcc->controller_id);
	if (ret) {
		dev_err(dev, "missing telechips,controller-id\n");
		return ret;
	}

	/*
	 * mmc_hclk (AHB/IO bus clock): always-on, no rate to manage.
	 * mmc_fclk (core/bit-rate clock): rate is set below, once
	 * mmc_of_parse() has read max-frequency into host->mmc->f_max --
	 * matches the vendor probe's ordering (sdhci-tcc.c ~line 870-942).
	 */
	ret = PTR_ERR_OR_ZERO(devm_clk_get_enabled(dev, "mmc_hclk"));
	if (ret) {
		dev_err(dev, "failed to get/enable mmc_hclk: %d\n", ret);
		return ret;
	}

	pltfm_host->clk = devm_clk_get_enabled(dev, "mmc_fclk");
	if (IS_ERR(pltfm_host->clk)) {
		ret = PTR_ERR(pltfm_host->clk);
		dev_err(dev, "failed to get/enable mmc_fclk: %d\n", ret);
		return ret;
	}

	tcc->chctrl_base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(tcc->chctrl_base)) {
		ret = PTR_ERR(tcc->chctrl_base);
		return ret;
	}

	tcc->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(tcc->reset_gpio)) {
		ret = PTR_ERR(tcc->reset_gpio);
		dev_err(dev, "failed to get reset gpio: %d\n", ret);
		return ret;
	}

	if (of_property_read_u32(dev->of_node, "telechips,clk-out-tap", &clk_out_tap))
		clk_out_tap = TCC_SDHC_CLKOUTDLY_DEF_TAP;
	if (of_property_read_u32(dev->of_node, "telechips,cmd-tap", &cmd_tap))
		cmd_tap = TCC_SDHC_CMDDLY_DEF_TAP;
	if (of_property_read_u32(dev->of_node, "telechips,data-tap", &data_tap))
		data_tap = TCC_SDHC_DATADLY_DEF_TAP;

	ret = mmc_of_parse(host->mmc);
	if (ret) {
		dev_err(dev, "mmc_of_parse failed: %d\n", ret);
		return ret;
	}

	if (host->mmc->f_max) {
		ret = clk_set_rate(pltfm_host->clk, host->mmc->f_max);
		if (ret) {
			dev_err(dev, "failed to set mmc_fclk to %u Hz: %d\n",
				host->mmc->f_max, ret);
			return ret;
		}
	}

	sdhci_tcc803x_set_channel_configs(host, clk_out_tap, cmd_tap, data_tap);

	ret = sdhci_add_host(host);
	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id sdhci_tcc803x_of_match[] = {
	{ .compatible = "telechips,tcc803x-sdhci" },
	{}
};
MODULE_DEVICE_TABLE(of, sdhci_tcc803x_of_match);

static struct platform_driver sdhci_tcc803x_driver = {
	.driver	= {
		.name		= "sdhci-tcc803x",
		.probe_type	= PROBE_PREFER_ASYNCHRONOUS,
		.pm		= &sdhci_pltfm_pmops,
		.of_match_table	= sdhci_tcc803x_of_match,
	},
	.probe	= sdhci_tcc803x_probe,
	.remove	= sdhci_pltfm_remove,
};
module_platform_driver(sdhci_tcc803x_driver);

MODULE_DESCRIPTION("SDHCI platform glue for Telechips TCC803x");
MODULE_AUTHOR("William Floyd <git@notmy.space>");
MODULE_LICENSE("GPL v2");
