// SPDX-License-Identifier: GPL-2.0-only
/*
 * Telechips TCC803x CKC ("Clock Controller") driver.
 *
 * Partial: registers the PERI_* peripheral-clock gates (PCLKCTRL
 * array) and the DDIBUS_* display-bus power-down/reset gates listed
 * below. The CKC also owns PLL configuration, the FBUS_* top-level bus
 * clocks, and separate I/O-bus/video-bus/HSIO-bus power-down and reset
 * gate arrays, plus the PMU-resident ISOIP_* gates -- none of that is
 * implemented here (ISOIP_DDB_HDMI specifically: its vendor register
 * offset could not be confirmed, see telechips,tcc803x-clks.h).
 *
 * Register layout confirmed against the real vendor Linux 3.18 tree
 * (AVNT5W_TCC803XKK_kernel, Hyundai Mobis supplier-portal release,
 * Telechips Inc., GPLv2); every concrete offset/shift below is cited
 * to it individually. This file is a clean-room reimplementation
 * against mainline's clk_ops, not a copy of the vendor driver.
 *
 * The vendor's later (kernel 5.10) driver for this same SoC
 * (github.com/topst-development/kernel-5.10,
 * drivers/clk/telechips/clk-legacy.c, selected for CONFIG_ARCH_TCC803X
 * by that tree's drivers/clk/telechips/Makefile) still controls the
 * CKC only indirectly, by trapping to firmware over SMC
 * (arm_smccc_smc(SIP_CLK_*, ...)); it was not usable as a register
 * reference. The direct-register PCLKCTRL access pattern below instead
 * follows drivers/clk/tcc/clk-tcc803x-a7s.c, the only vendor source
 * found that programs the CKC's MMIO registers directly.
 *
 * DDIBUS_*: tcc_ckc_ddibus_pwdn()/_swreset() (clk-tcc803x-a7s.c:1619-
 * 1669) bit-bang a *different* MMIO window than PCLKCTRL -- the "display
 * bus configuration" block, reached via a separate reg entry
 * (of_iomap(np, 3) there; vendor tcc803x.dtsi:726, 0x12380000 size 0x8).
 * DDIBUS_PWDN is offset 0x0 in that block, DDIBUS_RESET is offset 0x4
 * (clk-tcc803x-a7s-reg.h:58-59), each a bitmask indexed by the raw
 * vendor DDIBUS_* channel number (bit 0 = channel 0, etc), not the
 * PCLKCTRL EN/OUTEN two-bit-per-channel scheme. The vendor's enable
 * path also gates on the parent FBUS_DDI top-bus clock being on
 * (ckc_base+CKC_CLKCTRL+FBUS_DDI*4, bit 29) before touching DDIBUS_PWDN;
 * that FBUS_* layer isn't implemented in this driver, so this gate
 * assumes the bootloader already left FBUS_DDI enabled. PERI_*'s
 * .recalc_rate()/.set_rate()/.round_rate() have a narrower version of the
 * same gap: they decode/select only XIN and the direct PLL0-4 outputs
 * (reading PLLPMS live, since this driver still doesn't implement PLL
 * clk_hw's of its own), not PLLnDIV/HDMI/EXTCLK/PCI SEL values or DCO
 * mode -- see the PCLKCTRL_SEL_PLL0 and tcc803x_ckc_peri_find_div()
 * comments below.
 */

#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <dt-bindings/clock/telechips,tcc803x-clks.h>

/*
 * CKC_PCLKCTRL: base offset of the per-peripheral PCLKCTRL register
 * array. Each channel's register is CKC_PCLKCTRL + id * 4, id being
 * one of the PERI_* indices in telechips,tcc803x-clks.h.
 *   drivers/clk/tcc/clk-tcc803x-a7s-reg.h:14 (#define CKC_PCLKCTRL 0x0D0)
 *   drivers/clk/tcc/clk-tcc803x-a7s.c:1371   (reg = ckc_base+CKC_PCLKCTRL+id*4)
 */
#define TCC803X_CKC_PCLKCTRL		0x0D0

/*
 * CKC_PLLPMS: base offset of the 5-entry PLL P/M/S/EN/SRC array (PLL0-4,
 * one 32-bit word each at CKC_PLLPMS + id*4).
 *   drivers/clk/tcc/clk-tcc803x-a7s-reg.h:8  (#define CKC_PLLPMS 0x040)
 *   drivers/clk/tcc/clk-tcc803x-a7s.c:605    (reg = ckc_base+CKC_PLLPMS+id*4)
 * Read-only here: this driver only decodes an already-configured PLL's
 * live output rate (same formula as tcc_ckc_pll_get_rate(), a7s.c:603-
 * 651), to pick a PERI_* mux source in .set_rate()/.round_rate() below. It
 * never writes PLLPMS, so none of the DPLL_LOCKST lock-bit polling that
 * guards an actual PLL P/M/S/EN change (e.g. a7s.c tcc_ckc_dpll_set_val())
 * applies to anything in this file.
 */
#define TCC803X_CKC_PLLPMS		0x040
#define TCC803X_CKC_PLLPMS_COUNT	5

/*
 * PLLPMS field layout, clk-tcc803x-a7s-reg.h:145-152 (PLL0/PLL1, "normal")
 * vs :168-199 (PLL2-4, "DPLL"/dithered). Only the M field width differs
 * between the two (10 bits vs 9); P/S/SRC/EN are identical shifts+masks in
 * both, confirmed by direct comparison of the two reg.h blocks. Which
 * layout applies to which index: tcc_ckc_is_normal_pll(), a7s.c:378-397
 * (true only for id 0/1).
 */
#define PLL_P_SHIFT			0
#define PLL_P_MASK			0x3F
#define PLL_M_SHIFT			6
#define PLL_M_MASK_NORMAL		0x3FF	/* PLL0/PLL1 */
#define PLL_M_MASK_DPLL			0x1FF	/* PLL2/PLL3/PLL4 */
#define PLL_S_SHIFT			16
#define PLL_S_MASK			0x7
#define PLL_SRC_SHIFT			19
#define PLL_SRC_MASK			0x3
#define PLL_EN_SHIFT			31

/*
 * PLL reference-clock source select (PLL_SRC field above), clk-tcc803x-
 * a7s-reg.h:214-217. PLLSRC_XIN is this driver's "xin" parent; HDMIXI is
 * the HDMI input's 27MHz reference (HDMI_CLK_RATE, clk-tcc803x.h:180) --
 * no clk_hw for that ref exists in this driver, so it's a bare constant,
 * used only to decode a PLL's live rate, never to drive one.
 * PLLSRC_EXTCLK0/1 can't be decoded at all: clk-tcc803x.h (the header
 * clk-tcc803x-a7s.c itself includes) never defines EXT0_CLK_RATE/
 * EXT1_CLK_RATE for this SoC -- those macros exist only in the unrelated
 * clk-tcc898x.h -- so a PLL actually wired to an external ref on TCC803x
 * has no rate anywhere in this vendor tree; treated as unusable below
 * rather than guessed at.
 */
#define PLLSRC_XIN			0
#define PLLSRC_HDMIXI			1
#define TCC803X_HDMI_REF_CLK_RATE	27000000UL

/*
 * PCLKCTRL_EN / PCLKCTRL_OUTEN: clk-tcc803x-a7s-reg.h:241,242
 * (#define PCLKCTRL_EN_SHIFT 29, #define PCLKCTRL_OUTEN_SHIFT 30).
 *
 * clk-tcc803x-a7s.c's tcc_check_pclk_type() (lines 1334-1367) sorts
 * PERI_* channels into two PCLKCTRL flavours; PERI_UART0 (58) is not
 * in its "TYPE_YYY" case list (the audio MDAI/SPDIF/SRCH/AUX/TSRX
 * channels, lines 1337-1364), so it falls through to the default
 * TYPE_XXX. tcc_ckc_peri_enable()/_disable() (lines 1369-1390) set
 * both EN and OUTEN together on enable, and clear OUTEN before EN on
 * disable, for TYPE_XXX channels -- mirrored below. A single-bit
 * clk_gate (drivers/clk/clk-gate.c) can't express this two-bit
 * enable, hence the hand-rolled clk_ops.
 */
#define PCLKCTRL_EN			BIT(29)
#define PCLKCTRL_OUTEN			BIT(30)

/*
 * PCLKCTRL SEL/DIV/MD fields, shared by every PERI_* channel's PCLKCTRL
 * word (clk-tcc803x-a7s-reg.h:230-243):
 *   DIV   bits [11:0]  (PCLKCTRL_DIV_SHIFT 0, PCLKCTRL_DIV_XXX_MASK 0xFFF
 *                        -- the "TYPE_XXX" width; TYPE_YYY channels use a
 *                        24-bit field instead (DIV_YYY_MASK 0xFFFFFF), but
 *                        none of the channels this driver implements are
 *                        TYPE_YYY -- see tcc_check_pclk_type(),
 *                        clk-tcc803x-a7s.c:1333-1366: that list is audio
 *                        MDAI/SPDIF/SRCH/AUX/TSRX channels only)
 *   SEL   bits [28:24] (PCLKCTRL_SEL_SHIFT 24, PCLKCTRL_SEL_MASK 0x1F)
 *   MD    bit  31      (PCLKCTRL_MD_SHIFT 31; 0 = DCO/fractional mode,
 *                        1 = plain source/divider mode -- PCLKCTRL_MODE_DIVIDER)
 *
 * In divider mode, actual divisor = DIV field + 1 (clk-tcc803x-a7s.c
 * tcc_find_pclk(), ~line 1325, writes requested_divisor - 1). SEL is
 * *initialized* to PCLKCTRL_SEL_XIN by tcc_ckc_peri_set_rate()
 * (clk-tcc803x-a7s.c:1404-1408) but that's only the seed value: unless the
 * HDMI-pclk special case applies (freq == the dummy HDMI_PCLK_RATE
 * sentinel, LCD0/LCD1 only), it's immediately overwritten by whatever
 * tcc_find_pclk() (lines 1218-1332) picks -- a live runtime search across
 * every enabled PLL/PLLDIV/XIN/XTIN source for the closest achievable
 * rate, not a fixed per-channel default. This driver's .recalc_rate()/
 * .set_rate()/.round_rate() mirror that search, restricted to XIN and the
 * direct PLL0-4 outputs (DCO mode, PLLnDIV, and the remaining SEL values
 * aren't decoded -- see tcc803x_ckc_peri_find_div() below for why).
 */
#define PCLKCTRL_DIV_SHIFT		0
#define PCLKCTRL_DIV_XXX_MASK		0xFFF
#define PCLKCTRL_SEL_SHIFT		24
#define PCLKCTRL_SEL_MASK		0x1F
/*
 * PCLKCTRL SEL values 0-4 select PLL0-4 directly (clk-tcc803x-a7s-reg.h:
 * 261-265, "Peri. Clock Source" enum); 5 is XIN. This driver only
 * searches the direct PLL0-4 outputs and XIN for .set_rate()/
 * .round_rate() below -- not the PLLnDIV variants (SEL 10-14, a separate
 * divided tap off each PLL, tcc_ckc_plldiv_get_rate(), a7s.c ~line 740),
 * XTIN (32.768kHz, too slow to matter for any PERI_* rate this driver
 * cares about), or the HDMI/EXTCLK/PCI SEL values (18-26) -- all firmly
 * out of scope for reaching a ~194MHz display pixel clock.
 */
#define PCLKCTRL_SEL_PLL0		0
#define PCLKCTRL_SEL_PLL1		1
#define PCLKCTRL_SEL_PLL2		2
#define PCLKCTRL_SEL_PLL3		3
#define PCLKCTRL_SEL_PLL4		4
#define PCLKCTRL_SEL_XIN		5
#define PCLKCTRL_MD_SHIFT		31
#define PCLKCTRL_MD_DIVIDER		BIT(PCLKCTRL_MD_SHIFT)

/*
 * DDIBUS power-down/reset register pair, within the "display bus
 * configuration" MMIO window (ddibus_cfg_base), NOT CKC_PCLKCTRL's
 * window: clk-tcc803x-a7s-reg.h:58-59.
 *   DDIBUS_PWDN  0x0: 1 == powered up, bit-per-channel.
 *   DDIBUS_RESET 0x4: 1 == out of reset, bit-per-channel.
 * tcc_ckc_ddibus_enable() sets PWDN then clears RESET-asserted (i.e.
 * sets the RESET bit, since 1 == deasserted here); disable reverses
 * the order (clk-tcc803x-a7s.c:1619-1669), mirrored below.
 */
#define TCC803X_DDIBUS_PWDN		0x0
#define TCC803X_DDIBUS_RESET		0x4

struct tcc803x_ckc_gate {
	struct clk_hw	hw;
	void __iomem	*reg;
	void __iomem	*pllpms_base;	/* base + TCC803X_CKC_PLLPMS, PLL0-4 array */
};

#define to_tcc803x_ckc_gate(_hw) container_of(_hw, struct tcc803x_ckc_gate, hw)

struct tcc803x_ckc_ddibus_gate {
	struct clk_hw	hw;
	void __iomem	*pwdn_reg;
	void __iomem	*reset_reg;
	u32		bit;
};

#define to_tcc803x_ckc_ddibus_gate(_hw) \
	container_of(_hw, struct tcc803x_ckc_ddibus_gate, hw)

static int tcc803x_ckc_peri_enable(struct clk_hw *hw)
{
	struct tcc803x_ckc_gate *gate = to_tcc803x_ckc_gate(hw);
	u32 val = readl(gate->reg);

	val |= PCLKCTRL_EN | PCLKCTRL_OUTEN;
	writel(val, gate->reg);

	return 0;
}

static void tcc803x_ckc_peri_disable(struct clk_hw *hw)
{
	struct tcc803x_ckc_gate *gate = to_tcc803x_ckc_gate(hw);
	u32 val = readl(gate->reg);

	val &= ~PCLKCTRL_OUTEN;
	writel(val, gate->reg);
	val &= ~PCLKCTRL_EN;
	writel(val, gate->reg);
}

static int tcc803x_ckc_peri_is_enabled(struct clk_hw *hw)
{
	struct tcc803x_ckc_gate *gate = to_tcc803x_ckc_gate(hw);

	return !!(readl(gate->reg) & PCLKCTRL_EN);
}

/*
 * Live-reads CKC_PLLPMS[pll_id] and decodes it with the same P/M/S/SRC
 * formula as the vendor's tcc_ckc_pll_get_rate() (clk-tcc803x-a7s.c:603-
 * 651): freq = (src_freq * M / P) >> S. Returns 0 if the PLL is disabled
 * (EN bit clear) or its SRC field is a value this driver can't decode
 * (PLLSRC_EXTCLK0/1 -- see the PLLSRC_* comment above); 0 is treated by
 * every caller below as "this source isn't usable right now", matching
 * what an unavailable clock would mean to the clk framework.
 */
static unsigned long tcc803x_ckc_pll_rate(struct tcc803x_ckc_gate *gate, u32 pll_id)
{
	u32 val = readl(gate->pllpms_base + pll_id * 4);
	u32 m_mask = (pll_id < 2) ? PLL_M_MASK_NORMAL : PLL_M_MASK_DPLL;
	u32 p, m, s, src;
	u64 rate;

	if (!(val & BIT(PLL_EN_SHIFT)))
		return 0;

	p = (val >> PLL_P_SHIFT) & PLL_P_MASK;
	m = (val >> PLL_M_SHIFT) & m_mask;
	s = (val >> PLL_S_SHIFT) & PLL_S_MASK;
	src = (val >> PLL_SRC_SHIFT) & PLL_SRC_MASK;

	switch (src) {
	case PLLSRC_XIN:
		rate = clk_hw_get_rate(clk_hw_get_parent(&gate->hw));
		break;
	case PLLSRC_HDMIXI:
		rate = TCC803X_HDMI_REF_CLK_RATE;
		break;
	default:
		return 0;
	}

	if (!rate || !p)
		return 0;

	rate = div_u64(rate * m, p);

	return (unsigned long)(rate >> s);
}

/*
 * Decodes PCLKCTRL's SEL+DIV fields when MD is in divider mode -- see the
 * PCLKCTRL_* field comment above for the vendor citations and the DCO-mode
 * scope limit. SEL==XIN uses the real parent clock; SEL==PLL0-4 decodes
 * the live PLL via tcc803x_ckc_pll_rate() above, so a rate .set_rate()
 * picked below round-trips correctly through this function afterwards.
 * Any other SEL (PLLnDIV, HDMI, EXTCLK, ...) isn't modeled; the parent
 * rate is passed through unchanged, same as before DIV decoding existed.
 */
static unsigned long tcc803x_ckc_peri_recalc_rate(struct clk_hw *hw,
						    unsigned long parent_rate)
{
	struct tcc803x_ckc_gate *gate = to_tcc803x_ckc_gate(hw);
	u32 val = readl(gate->reg);
	u32 sel = (val >> PCLKCTRL_SEL_SHIFT) & PCLKCTRL_SEL_MASK;
	u32 div = (val >> PCLKCTRL_DIV_SHIFT) & PCLKCTRL_DIV_XXX_MASK;
	unsigned long src;

	if (!(val & PCLKCTRL_MD_DIVIDER))
		return parent_rate;

	if (sel == PCLKCTRL_SEL_XIN)
		src = parent_rate;
	else if (sel <= PCLKCTRL_SEL_PLL4)
		src = tcc803x_ckc_pll_rate(gate, sel);
	else
		return parent_rate;

	return src ? src / (div + 1) : parent_rate;
}

/*
 * Searches XIN and the live PLL0-4 outputs for whichever gives the
 * closest divider-mode (actual divisor = DIV field + 1, 12-bit field,
 * clk-tcc803x-a7s-reg.h:233-234) result to @rate, mirroring the vendor's
 * tcc_find_pclk()/tcc_ckc_pclk_divider() search (clk-tcc803x-a7s.c:1151-
 * 1179, 1218-1332) restricted to TYPE_XXX's divider-only mode (every
 * PERI_* channel this driver implements is TYPE_XXX per
 * tcc_check_pclk_type(), a7s.c:1334-1367 -- none are in its TYPE_YYY
 * audio-channel list). DCO mode is deliberately not searched: recalc_rate
 * above doesn't decode it, so picking it here would make the rate this
 * function reports and the rate .recalc_rate() reads back disagree.
 *
 * A disabled/undecodable PLL reads back 0 from tcc803x_ckc_pll_rate() and
 * is skipped. Which PLLs are actually enabled, and at what rate, is
 * board/bootloader-configured (tcc_ckc_pll_get_rate() reads it from
 * registers at runtime, a7s.c:603-651 -- there's no static "PLL2 = *this*
 * many MHz" anywhere in the vendor source for this SoC); this search
 * finds the best real answer at call time instead of assuming one PLL in
 * particular is already running at the right rate.
 */
static unsigned long tcc803x_ckc_peri_find_div(struct clk_hw *hw, unsigned long rate,
						u32 *best_sel, u32 *best_div)
{
	struct tcc803x_ckc_gate *gate = to_tcc803x_ckc_gate(hw);
	unsigned long best_rate = 0;
	unsigned long best_err = ULONG_MAX;
	u32 sel;

	if (!rate)
		return 0;

	for (sel = PCLKCTRL_SEL_PLL0; sel <= PCLKCTRL_SEL_XIN; sel++) {
		unsigned long src, div, achieved, err;

		src = (sel == PCLKCTRL_SEL_XIN) ?
			clk_hw_get_rate(clk_hw_get_parent(hw)) :
			tcc803x_ckc_pll_rate(gate, sel);
		if (!src)
			continue;

		div = clamp_val(DIV_ROUND_CLOSEST(src, rate), 1,
				 PCLKCTRL_DIV_XXX_MASK + 1);
		achieved = src / div;
		err = abs_diff(achieved, rate);

		if (err < best_err) {
			best_err = err;
			best_rate = achieved;
			*best_sel = sel;
			*best_div = div - 1;
		}
	}

	return best_rate;
}

/*
 * This tree's clk_ops uses .determine_rate(struct clk_rate_request *),
 * not the older .round_rate(unsigned long, unsigned long *parent_rate) --
 * confirmed against this tree's own include/linux/clk-provider.h (struct
 * clk_ops, no .round_rate member; struct clk_rate_request).
 */
static int tcc803x_ckc_peri_determine_rate(struct clk_hw *hw,
					    struct clk_rate_request *req)
{
	u32 sel, div;
	unsigned long achieved = tcc803x_ckc_peri_find_div(hw, req->rate, &sel, &div);

	if (!achieved)
		return -EINVAL;

	req->rate = achieved;
	return 0;
}

/*
 * Programs SEL+DIV for the source tcc803x_ckc_peri_find_div() picked.
 * Write order mirrors tcc_pclkctrl_write()'s PCLKCTRL_TYPE_XXX path
 * exactly (clk-tcc803x-a7s.c:142-154): drop OUTEN, drop EN, write DIV,
 * write SEL, restore EN, restore OUTEN (the two restores use this
 * channel's enable state as read from the live register before the
 * sequence starts, matching nPCLKCTRL.en at a7s.c:1423). No busy/lock bit
 * is polled: the only register-change poll in the vendor file guards a
 * PLL's own P/M/S/EN change (DPLL_LOCKST), not this mux/divider; the one
 * poll that vendor code has for PCLKCTRL itself (tcc_ckc_peri_disable(),
 * a7s.c:1384-1385) is commented out in the vendor source itself.
 *
 * MD is deliberately left untouched: the vendor's own TYPE_XXX write path
 * never writes PCLKCTRL_MD either (same lines above), even though
 * tcc_find_pclk() unconditionally sets md=PCLKCTRL_MODE_DIVIDER for
 * TYPE_XXX channels (a7s.c:1228) -- i.e. the vendor driver itself relies
 * on MD already reading divider mode for a TYPE_XXX channel and doesn't
 * assert it in the write sequence. This driver matches that as-is rather
 * than adding a write the vendor sequence doesn't have.
 */
static int tcc803x_ckc_peri_set_rate(struct clk_hw *hw, unsigned long rate,
				      unsigned long parent_rate)
{
	struct tcc803x_ckc_gate *gate = to_tcc803x_ckc_gate(hw);
	u32 sel, div, val, en;

	if (!tcc803x_ckc_peri_find_div(hw, rate, &sel, &div))
		return -EINVAL;

	val = readl(gate->reg);
	en = val & PCLKCTRL_EN;

	val &= ~PCLKCTRL_OUTEN;
	writel(val, gate->reg);
	val &= ~PCLKCTRL_EN;
	writel(val, gate->reg);

	val &= ~(PCLKCTRL_DIV_XXX_MASK << PCLKCTRL_DIV_SHIFT);
	val |= (div & PCLKCTRL_DIV_XXX_MASK) << PCLKCTRL_DIV_SHIFT;
	writel(val, gate->reg);

	val &= ~(PCLKCTRL_SEL_MASK << PCLKCTRL_SEL_SHIFT);
	val |= (sel & PCLKCTRL_SEL_MASK) << PCLKCTRL_SEL_SHIFT;
	writel(val, gate->reg);

	if (en) {
		val |= PCLKCTRL_EN;
		writel(val, gate->reg);
		val |= PCLKCTRL_OUTEN;
		writel(val, gate->reg);
	}

	return 0;
}

static const struct clk_ops tcc803x_ckc_peri_gate_ops = {
	.enable		= tcc803x_ckc_peri_enable,
	.disable	= tcc803x_ckc_peri_disable,
	.is_enabled	= tcc803x_ckc_peri_is_enabled,
	.recalc_rate	= tcc803x_ckc_peri_recalc_rate,
	.determine_rate	= tcc803x_ckc_peri_determine_rate,
	.set_rate	= tcc803x_ckc_peri_set_rate,
};

static int tcc803x_ckc_ddibus_enable(struct clk_hw *hw)
{
	struct tcc803x_ckc_ddibus_gate *gate = to_tcc803x_ckc_ddibus_gate(hw);
	u32 bit = BIT(gate->bit);

	writel(readl(gate->pwdn_reg) | bit, gate->pwdn_reg);
	writel(readl(gate->reset_reg) | bit, gate->reset_reg);

	return 0;
}

static void tcc803x_ckc_ddibus_disable(struct clk_hw *hw)
{
	struct tcc803x_ckc_ddibus_gate *gate = to_tcc803x_ckc_ddibus_gate(hw);
	u32 bit = BIT(gate->bit);

	writel(readl(gate->reset_reg) & ~bit, gate->reset_reg);
	writel(readl(gate->pwdn_reg) & ~bit, gate->pwdn_reg);
}

static int tcc803x_ckc_ddibus_is_enabled(struct clk_hw *hw)
{
	struct tcc803x_ckc_ddibus_gate *gate = to_tcc803x_ckc_ddibus_gate(hw);

	return !!(readl(gate->pwdn_reg) & BIT(gate->bit));
}

static const struct clk_ops tcc803x_ckc_ddibus_gate_ops = {
	.enable		= tcc803x_ckc_ddibus_enable,
	.disable	= tcc803x_ckc_ddibus_disable,
	.is_enabled	= tcc803x_ckc_ddibus_is_enabled,
};

struct tcc803x_ckc_peri_desc {
	u32		id;
	const char	*name;
};

/*
 * Every PERI_* channel this driver currently implements. clock-names
 * are this driver's own choice (the vendor doesn't name these), kept
 * short and consistent with the pre-existing "peri-uart0".
 */
static const struct tcc803x_ckc_peri_desc tcc803x_ckc_peri_clocks[] = {
	{ TCC803X_CLK_PERI_UART0,	"peri-uart0" },
	{ TCC803X_CLK_PERI_UART2,	"peri-uart2" },
	{ TCC803X_CLK_PERI_LCD0,	"peri-lcd0" },
	{ TCC803X_CLK_PERI_HDMI_CORE,	"peri-hdmi-core" },
	{ TCC803X_CLK_PERI_HDMI_AUDIO,	"peri-hdmi-audio" },
	{ TCC803X_CLK_PERI_HDMI_PCLK,	"peri-hdmi-pclk" },
	{ TCC803X_CLK_PERI_HDMI_CEC,	"peri-hdmi-cec" },
	{ TCC803X_CLK_PERI_HDMI_HDCP14,	"peri-hdmi-hdcp14" },
	{ TCC803X_CLK_PERI_HDMI_HDCP22,	"peri-hdmi-hdcp22" },
	{ TCC803X_CLK_PERI_LVDS_PHY,	"peri-lvds-phy" },
};

struct tcc803x_ckc_ddibus_desc {
	u32		onecell_id;
	u32		bit;
	const char	*name;
};

static const struct tcc803x_ckc_ddibus_desc tcc803x_ckc_ddibus_clocks[] = {
	{ TCC803X_CLK_DDIBUS_VIOC, 0, "ddibus-vioc" },
	{ TCC803X_CLK_DDIBUS_HDMI, 2, "ddibus-hdmi" },
};

static int tcc803x_ckc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk_hw_onecell_data *data;
	struct clk_init_data init = {};
	void __iomem *base, *ddibus_base;
	const char *parent_name;
	u32 max_id = 0;
	int i, ret;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	/*
	 * Second reg entry: the "display bus configuration" window the
	 * DDIBUS_* gates live in (vendor tcc803x.dtsi:726, 0x12380000
	 * size 0x8), separate from the CKC's own PCLKCTRL window above.
	 */
	ddibus_base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(ddibus_base))
		return PTR_ERR(ddibus_base);

	parent_name = of_clk_get_parent_name(dev->of_node, 0);
	if (!parent_name)
		return dev_err_probe(dev, -EINVAL, "missing \"xin\" parent clock\n");

	for (i = 0; i < ARRAY_SIZE(tcc803x_ckc_peri_clocks); i++)
		max_id = max(max_id, tcc803x_ckc_peri_clocks[i].id);
	for (i = 0; i < ARRAY_SIZE(tcc803x_ckc_ddibus_clocks); i++)
		max_id = max(max_id, tcc803x_ckc_ddibus_clocks[i].onecell_id);

	data = devm_kzalloc(dev, struct_size(data, hws, max_id + 1), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/*
	 * Sized/populated only for the channels below: every other index
	 * is left NULL, so of_clk_hw_onecell_get() correctly fails
	 * lookups for channels this driver doesn't implement yet.
	 */
	data->num = max_id + 1;

	for (i = 0; i < ARRAY_SIZE(tcc803x_ckc_peri_clocks); i++) {
		const struct tcc803x_ckc_peri_desc *desc = &tcc803x_ckc_peri_clocks[i];
		struct tcc803x_ckc_gate *gate;

		gate = devm_kzalloc(dev, sizeof(*gate), GFP_KERNEL);
		if (!gate)
			return -ENOMEM;

		gate->reg = base + TCC803X_CKC_PCLKCTRL + desc->id * 4;
		gate->pllpms_base = base + TCC803X_CKC_PLLPMS;

		init = (struct clk_init_data){
			.name		= desc->name,
			.ops		= &tcc803x_ckc_peri_gate_ops,
			.parent_names	= &parent_name,
			.num_parents	= 1,
		};
		gate->hw.init = &init;

		ret = devm_clk_hw_register(dev, &gate->hw);
		if (ret)
			return ret;

		data->hws[desc->id] = &gate->hw;
	}

	for (i = 0; i < ARRAY_SIZE(tcc803x_ckc_ddibus_clocks); i++) {
		const struct tcc803x_ckc_ddibus_desc *desc = &tcc803x_ckc_ddibus_clocks[i];
		struct tcc803x_ckc_ddibus_gate *gate;

		gate = devm_kzalloc(dev, sizeof(*gate), GFP_KERNEL);
		if (!gate)
			return -ENOMEM;

		gate->pwdn_reg = ddibus_base + TCC803X_DDIBUS_PWDN;
		gate->reset_reg = ddibus_base + TCC803X_DDIBUS_RESET;
		gate->bit = desc->bit;

		init = (struct clk_init_data){
			.name		= desc->name,
			.ops		= &tcc803x_ckc_ddibus_gate_ops,
			.parent_names	= &parent_name,
			.num_parents	= 1,
		};
		gate->hw.init = &init;

		ret = devm_clk_hw_register(dev, &gate->hw);
		if (ret)
			return ret;

		data->hws[desc->onecell_id] = &gate->hw;
	}

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, data);
}

static const struct of_device_id tcc803x_ckc_of_match[] = {
	{ .compatible = "telechips,tcc803x-ckc" },
	{ }
};
MODULE_DEVICE_TABLE(of, tcc803x_ckc_of_match);

static struct platform_driver tcc803x_ckc_driver = {
	.probe = tcc803x_ckc_probe,
	.driver = {
		.name = "tcc803x-ckc",
		.of_match_table = tcc803x_ckc_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(tcc803x_ckc_driver);
