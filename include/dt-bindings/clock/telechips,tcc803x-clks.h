/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Clock indices for the Telechips TCC803x CKC (drivers/clk/telechips).
 *
 * These numbers are the "PERI_*" peripheral-clock channel indices used
 * by the vendor's own CKC register map: each index selects a 4-byte
 * PCLKCTRL register at CKC_PCLKCTRL (CKC base + 0xD0) + index * 4.
 * Confirmed against the real vendor Linux 3.18 tree
 * (AVNT5W_TCC803XKK_kernel, Hyundai Mobis supplier-portal release,
 * Telechips Inc., GPLv2):
 *   drivers/clk/tcc/clk-tcc803x-a7s-reg.h:14   (CKC_PCLKCTRL offset 0xD0)
 *   drivers/clk/tcc/clk-tcc803x-a7s.c:1371     (register = base + CKC_PCLKCTRL + id*4)
 *   include/dt-bindings/clock/tcc803x-clks.h:98 (PERI_UART0 == 58)
 *
 * Only the PERI_* indices below are register offsets into that same
 * PCLKCTRL array; add entries here, matching the vendor numbering
 * exactly, as drivers/clk/telechips/clk-tcc803x-ckc.c gains support for
 * them. Do not renumber: these indices are register offsets, not
 * arbitrary IDs.
 *
 * DDIBUS_* indices are a *different* namespace: they are per-bit
 * channel numbers into the separate "display bus configuration"
 * power-down/reset register pair (not CKC_PCLKCTRL, not base+id*4).
 * Confirmed:
 *   include/dt-bindings/clock/tcc803x-clks.h:292 (DDIBUS_VIOC == 0)
 *   include/dt-bindings/clock/tcc803x-clks.h:293 (DDIBUS_HDMI == 2)
 *   drivers/clk/tcc/clk-tcc803x-a7s.c:1619-1669 (tcc_ckc_ddibus_pwdn()/
 *     _swreset(), bit-per-channel on ddibus_cfg_base+DDIBUS_PWDN/RESET)
 *   drivers/clk/tcc/clk-tcc803x-a7s-reg.h:58-59 (DDIBUS_PWDN 0x0,
 *     DDIBUS_RESET 0x4, offsets within that block)
 *   drivers/clk/tcc/clk-tcc803x-a7s.c:2177 (ddibus_cfg_base =
 *     of_iomap(np, 3), the ckc node's 4th reg entry)
 *   arch/arm/boot/dts/tcc/tcc803x.dtsi:726 (that reg entry is
 *     0x12380000 0x8, "display bus configuration")
 *
 * The mainline ckc node (unlike the vendor's separate clk_peri/clk_ddi/
 * clk_isoip_ddi provider nodes) is a single clock-controller with one
 * #clock-cells = <1> onecell table, so DDIBUS_* raw bit numbers (0, 2 --
 * same numeric range as PERI_*) cannot be used directly as onecell IDs
 * here without colliding with PERI_* channels 0 and 2. The
 * TCC803X_CLK_DDIBUS_* macros below are therefore mainline-only onecell
 * indices (vendor bit number + TCC803X_CLK_DDIBUS_BASE), not vendor
 * register values themselves; the driver subtracts the base back out to
 * get the real bit position.
 *
 * ISOIP_DDB_HDMI is NOT included here: the vendor source references
 * pmu_base+PMU_ISOIP_DDI (drivers/clk/tcc/clk-tcc803x-a7s.c:1873) but
 * PMU_ISOIP_DDI's offset is never #defined anywhere reachable by that
 * file in this tree (grepped clk-tcc803x-a7s-reg.h, clk-tcc803x.h, and
 * the whole extracted source tree -- only the unrelated tcc802x/tcc898x
 * drivers define PMU_ISOIP_DDI, at a different offset, 0x068, that
 * cannot be assumed correct for tcc803x's PMU layout without its own
 * citation). Left unimplemented rather than guessed.
 */

#ifndef _DT_BINDINGS_CLOCK_TELECHIPS_TCC803X_H
#define _DT_BINDINGS_CLOCK_TELECHIPS_TCC803X_H

#define TCC803X_CLK_PERI_UART0		58
#define TCC803X_CLK_PERI_UART1		59
#define TCC803X_CLK_PERI_UART2		60
#define TCC803X_CLK_PERI_LCD0		9
#define TCC803X_CLK_PERI_HDMI_CORE	14
#define TCC803X_CLK_PERI_HDMI_AUDIO	15
#define TCC803X_CLK_PERI_HDMI_PCLK	16
#define TCC803X_CLK_PERI_HDMI_CEC	17
#define TCC803X_CLK_PERI_HDMI_HDCP14	18
#define TCC803X_CLK_PERI_HDMI_HDCP22	19
/*
 * PERI_LVDS_PHY == 105: confirmed live, not from static vendor source --
 * see the lvds_phy node's comment in tcc803x.dtsi for the register
 * capture this index is read from (CKC_PCLKCTRL(105), 0x14000274).
 */
#define TCC803X_CLK_PERI_LVDS_PHY	105

/*
 * DDIBUS_* onecell IDs -- mainline-only (base + vendor bit number), see
 * above. Raw vendor bit numbers: VIOC == 0, HDMI == 2.
 */
#define TCC803X_CLK_DDIBUS_BASE	128
#define TCC803X_CLK_DDIBUS_VIOC	(TCC803X_CLK_DDIBUS_BASE + 0)
#define TCC803X_CLK_DDIBUS_HDMI	(TCC803X_CLK_DDIBUS_BASE + 2)

#endif /* _DT_BINDINGS_CLOCK_TELECHIPS_TCC803X_H */
