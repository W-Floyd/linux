// SPDX-License-Identifier: GPL-2.0-only
/*
 * Telechips TCC803x on-die LVDS PHY bridge.
 *
 * The PHY init sequence is ported from Telechips' LK bootloader
 * (AVNT5W_TCC803XKK_bootloadernew, platform/tcc803x/vioc/lvds_phy.c and
 * include/platform/lvds_phy.h, GPLv2), in the order the vendor panel code
 * calls it. Its register bases match this node's 4 regions exactly for the
 * vendor's system_rev == 0 map (HwVIOC_BASE 0x12000000 + 0x370000 / 0x360000,
 * port << 10): lvds0 = single port S0, lvds1 = S1 (NULL in the vendor's port
 * table, never used), lvds-cfg0/lvds-cfg1 = dual ports D0/D1. A live read
 * of these regions on a displaying unit returned all zeros, which the vendor
 * init would not leave behind: this unit may be a system_rev != 0 part whose
 * PHY sits at 0x1b941000/0x1b942800/0x1b942c00 behind an LVDS wrapper
 * (0x1b940000) this driver does not program.
 *
 * The init runs only when the DT sets telechips,phy-init. Without it the
 * driver leaves the PHY as the bootloader left it, which is what makes the
 * display work across a kexec from stock today.
 *
 * Not done here, all needed for a cold boot and all unverified on this unit:
 *  - the VIOC PXDEMUX lane/bit mapping the vendor programs before the PHY on
 *    system_rev == 0 (VIOC_PXDEMUX_SetConfigure/SetMuxOutput/SetDataArray);
 *  - panel/serializer power, reset and standby GPIOs;
 *  - the DISP0 pixel clock: the PHY's coarse tuning is sized from the mode's
 *    clock, so it only locks if the CRTC really runs at that rate.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/drm_modes.h>
#include <drm/drm_probe_helper.h>

#define LVDS_RESETB			0x00
#define  LVDS_RESETB_ALL_RELEASE	0x00001fff
/* Vendor value: holds CT, the FIFOs and the pixel/SSCG/NCO clocks in reset. */
#define  LVDS_RESETB_FIFO_RESET		0x0000118f
#define LVDS_CLK_SET			0x04
/* Vendor value: DAC clock and every internal clock enabled, none inverted. */
#define  LVDS_CLK_SET_ENABLE		0x000100ff
#define LVDS_AUTO_STB_SET		0x08
#define  LVDS_AUTO_STB_SET_AUTO_EN	BIT(0)
#define LVDS_STB_EN			0x1c
#define  LVDS_STB_EN_EN			BIT(0)
#define LVDS_AUTO_STB_DONE		0x2c
#define LVDS_AUTO_STB_RDATA		0x30
#define LVDS_EN				0x40
#define  LVDS_EN_ALL			GENMASK(3, 0)	/* FIFO2..0, DATA */
#define LVDS_FORMAT			0x44
#define  LVDS_FORMAT_UPSAMPLE		GENMASK(5, 3)
#define  LVDS_FORMAT_BALANCED		BIT(2)
#define  LVDS_FORMAT_DEPTH_8BIT		BIT(1)
#define  LVDS_FORMAT_JEIDA		BIT(0)
#define LVDS_PORT			0x48
#define  LVDS_PORT_SYNC_SRC		GENMASK(14, 12)
#define  LVDS_PORT_LANE_EN		GENMASK(8, 4)
#define  LVDS_PORT_USE_OTHER_SYNC	BIT(2)
#define  LVDS_PORT_VH_SWAP		BIT(1)
#define  LVDS_PORT_SECONDARY		BIT(0)
/* The vendor clears 0x1f at bit 0 for the secondary-port field. */
#define  LVDS_PORT_SECONDARY_CLR	GENMASK(4, 0)
#define LVDS_STARTUP_MODE		0x4c
#define  LVDS_STARTUP_FIFO2_RD_TIMING	GENMASK(14, 13)
#define LVDS_PHY_IF_SET1		0x70
#define  LVDS_PHY_IF_SET1_LANE(n)	(GENMASK(2, 0) << ((n) * 4))
#define LVDS_MONITOR_DEBUG1		0x80
#define  LVDS_MON_PLL_STATUS		GENMASK(3, 0)
#define  LVDS_MON_LKVDET		(BIT(2) | BIT(3))	/* LOW | HIGH */
#define LVDS_CTSET0			0x90
#define  LVDS_CTSET0_RUN		BIT(1)
#define  LVDS_CTSET0_ENABLE		BIT(0)
#define LVDS_CTSET1			0x94
#define  LVDS_CTSET1_CLOSED_LOOP	BIT(13)
#define  LVDS_CTSET1_DIVN		GENMASK(12, 10)
#define  LVDS_CTSET1_FCON		GENMASK(9, 0)
#define LVDS_FCCNTR0			0x98
#define LVDS_FCOPT			0x9c
#define  LVDS_FCOPT_CLK_DET_SEL		BIT(1)
#define  LVDS_FCOPT_CT_AUTO		BIT(0)
#define LVDS_FCSTAT			0xa0
#define  LVDS_FCSTAT_CLK_OK		BIT(0)
#define  LVDS_FCSTAT_DONE		BIT(2)
#define  LVDS_FCSTAT_ERROR		BIT(3)
#define LVDS_FCCNTR1			0xb0
#define  LVDS_FCCNTR1_TARGET_TH		GENMASK(31, 16)
#define  LVDS_FCCNTR1_REF_CNT		GENMASK(15, 0)
#define LVDS_FCCONTINSET0		0xb4
#define LVDS_FCCONTINSET1		0xb8
#define LVDS_FCCONTINSET2		0xbc

/*
 * Offsets >= 0x200 are the analog block's registers, reached through the
 * strobe interface: a write or read posts a request, AUTO_STB_DONE says when
 * it completed, and a read's value lands in AUTO_STB_RDATA.
 */
#define LVDS_STB_LANE(lane, addr)	(0x200 + (lane) * 0x40 + (addr) * 4)
#define LVDS_STB_PLL(addr)		(0x380 + (addr) * 4)

#define LVDS_NUM_LANES			5	/* CLK, DATA0..3 */
#define LVDS_VCO_MIN_HZ			560000000ULL
#define LVDS_VCO_MAX_HZ			1120000000ULL
#define LVDS_UPSAMPLE_MAX		4

enum tcc803x_lvds_region { LVDS_S0, LVDS_S1, LVDS_D0, LVDS_D1, LVDS_NUM_REGIONS };

struct tcc803x_lvds_port {
	void __iomem *base;
	u32 lane_sel[LVDS_NUM_LANES];
};

struct tcc803x_lvds {
	struct device *dev;
	struct drm_bridge bridge;
	struct clk *phy_clk;
	void __iomem *regs[LVDS_NUM_REGIONS];

	bool phy_init;
	bool dual;
	struct tcc803x_lvds_port pri, sec;
	u8 stb_lane_cal;	/* swing [3:0], vcm [6:4] */
	u32 format;		/* LVDS_FORMAT depth/format bits */
};

static const char * const tcc803x_lvds_reg_names[LVDS_NUM_REGIONS] = {
	"lvds0", "lvds1", "lvds-cfg0", "lvds-cfg1",
};

static inline struct tcc803x_lvds *to_tcc803x_lvds(struct drm_bridge *bridge)
{
	return container_of(bridge, struct tcc803x_lvds, bridge);
}

static void lvds_rmw(void __iomem *base, u32 off, u32 clr, u32 set)
{
	writel((readl(base + off) & ~clr) | set, base + off);
}

static int lvds_stb_write(void __iomem *base, u32 off, u32 val)
{
	u32 done;

	writel(val, base + off);
	return readl_poll_timeout_atomic(base + LVDS_AUTO_STB_DONE, done,
					 done == 1, 1, 1000);
}

static int lvds_stb_read(void __iomem *base, u32 off, u32 *val)
{
	u32 done;
	int ret;

	readl(base + off);
	ret = readl_poll_timeout_atomic(base + LVDS_AUTO_STB_DONE, done,
					done & 1, 1, 1000);
	if (!ret)
		*val = readl(base + LVDS_AUTO_STB_RDATA);
	return ret;
}

/*
 * Vendor LVDS_PHY_GetCalibrationLevel(): pick the VCM code from the panel's
 * typical common-mode voltage, then the first of 16 swing steps within that
 * code's range that reaches the typical swing. Ranges are the vendor's, kept
 * verbatim even where they disagree with its own comments (code 3: 540, not
 * 570; code 7: 560, not 460).
 */
static u8 lvds_calibration(u32 vcm_mv, u32 vsw_mv)
{
	static const struct { u32 vcm_below, sw_min, sw_max; } tbl[] = {
		{  770, 190, 370 }, {  870, 210, 470 }, {  960, 210, 540 },
		{ 1050, 210, 540 }, { 1130, 210, 560 }, { 1210, 210, 530 },
		{ 1290, 210, 500 }, { U32_MAX, 210, 560 },
	};
	unsigned int vcm, swing;
	u32 step;

	for (vcm = 0; vcm < ARRAY_SIZE(tbl) - 1; vcm++)
		if (vcm_mv < tbl[vcm].vcm_below)
			break;

	step = (tbl[vcm].sw_max - tbl[vcm].sw_min) / 16;
	for (swing = 0; swing < 15; swing++)
		if (vsw_mv <= tbl[vcm].sw_min + step * swing)
			break;

	return (vcm << 4) | swing;
}

/* Per-link rate: in dual-link mode each port carries every other pixel. */
static u64 lvds_link_hz(struct tcc803x_lvds *lvds, int clock_khz)
{
	u64 hz = (u64)clock_khz * 1000;

	return lvds->dual ? div_u64(hz, 2) : hz;
}

/* Smallest 2^n oversampling that puts the PLL VCO above its minimum. */
static int lvds_upsample_ratio(u64 link_hz)
{
	int n;

	for (n = 0; n <= LVDS_UPSAMPLE_MAX; n++)
		if ((link_hz * 7) << n > LVDS_VCO_MIN_HZ)
			return n;
	return -ERANGE;
}

/* Vendor formula: ((link * 7 * 2^n) / 20) / 24 * 16 * 16, in MHz. */
static u32 lvds_ref_cnt(u64 link_hz, int upsample)
{
	u64 v = div_u64((link_hz * 7) << upsample, 20);

	return div_u64(div_u64(v, 24) * 256, 1000000);
}

static int lvds_stb_init(struct tcc803x_lvds_port *p, bool dual, bool primary)
{
	/* PHY vendor ("Alphachips guide") values, not panel-specific. */
	static const u8 pll[] = {
		0xff, 0x1f, 0x32, 0x0f, 0x02, 0x13, 0x29, 0x25,
		0xd4, 0x00, 0x0c, 0x00, 0x00,
	};
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(pll); i++) {
		ret = lvds_stb_write(p->base, LVDS_STB_PLL(i), pll[i]);
		if (ret)
			return ret;
	}
	ret = lvds_stb_write(p->base, LVDS_STB_PLL(0xd),
			     !dual ? 0x01 : primary ? 0x07 : 0x0c);
	return ret ?: lvds_stb_write(p->base, LVDS_STB_PLL(0xe), 0x01);
}

static int lvds_stb_lanes(struct tcc803x_lvds_port *p, u32 addr, u32 val)
{
	unsigned int lane;
	int ret;

	for (lane = 0; lane < LVDS_NUM_LANES; lane++) {
		ret = lvds_stb_write(p->base, LVDS_STB_LANE(lane, addr), val);
		if (ret)
			return ret;
	}
	return 0;
}

static int lvds_stb_ready(struct tcc803x_lvds_port *p, int upsample)
{
	int ret;

	ret = lvds_stb_write(p->base, LVDS_STB_PLL(0), 0x00);
	ret = ret ?: lvds_stb_write(p->base, LVDS_STB_PLL(0), 0xff);
	ret = ret ?: lvds_stb_write(p->base, LVDS_STB_PLL(1), 0x1f);
	ret = ret ?: lvds_stb_write(p->base, LVDS_STB_PLL(9), upsample & 0x7);
	return ret ?: lvds_stb_write(p->base, LVDS_STB_PLL(0xb), upsample & 0x7);
}

static int lvds_stb_pll_d_clear(struct tcc803x_lvds_port *p)
{
	u32 v;
	int ret;

	ret = lvds_stb_read(p->base, LVDS_STB_PLL(0xd), &v);
	return ret ?: lvds_stb_write(p->base, LVDS_STB_PLL(0xd), v & ~0xf0);
}

static int lvds_stb_start(struct tcc803x_lvds *lvds)
{
	struct tcc803x_lvds_port *p = &lvds->pri, *s = &lvds->sec;
	u32 v;
	int ret;

	if (lvds->dual) {
		ret = lvds_stb_pll_d_clear(p);
		ret = ret ?: lvds_stb_pll_d_clear(s);
		ret = ret ?: lvds_stb_write(p->base, LVDS_STB_PLL(0xe), 0x00);
		ret = ret ?: lvds_stb_write(s->base, LVDS_STB_PLL(0xe), 0x00);
		ret = ret ?: lvds_stb_write(s->base, LVDS_STB_PLL(0xe), 0x01);
		ret = ret ?: lvds_stb_write(s->base, LVDS_STB_PLL(0xd), 0x3c);
		ret = ret ?: lvds_stb_write(p->base, LVDS_STB_PLL(0xe), 0x01);
		ret = ret ?: lvds_stb_write(p->base, LVDS_STB_PLL(0xd), 0xd7);
	} else {
		ret = lvds_stb_pll_d_clear(p);
		ret = ret ?: lvds_stb_write(p->base, LVDS_STB_PLL(0xe), 0x00);
		ret = ret ?: lvds_stb_write(p->base, LVDS_STB_PLL(0xe), 0x01);
		ret = ret ?: lvds_stb_read(p->base, LVDS_STB_PLL(0xd), &v);
		ret = ret ?: lvds_stb_write(p->base, LVDS_STB_PLL(0xd),
					    (v & ~0xf0) | 0x10);
	}
	if (ret)
		return ret;

	ret = lvds_stb_lanes(p, 0, 0x79);
	if (!ret && lvds->dual)
		ret = lvds_stb_lanes(s, 0, 0x79);
	return ret;
}

/* Vendor LVDS_PHY_SetFcon(AUTOMATIC, open loop, div 0) + FConEnable. */
static void lvds_fcon_auto(struct tcc803x_lvds_port *p, u32 ref_cnt)
{
	void __iomem *b = p->base;

	lvds_rmw(b, LVDS_FCCNTR1, LVDS_FCCNTR1_TARGET_TH | LVDS_FCCNTR1_REF_CNT,
		 FIELD_PREP(LVDS_FCCNTR1_TARGET_TH, 0x2020) |
		 FIELD_PREP(LVDS_FCCNTR1_REF_CNT, ref_cnt));
	lvds_rmw(b, LVDS_CTSET1, LVDS_CTSET1_CLOSED_LOOP | LVDS_CTSET1_DIVN |
		 LVDS_CTSET1_FCON, 0);
	lvds_rmw(b, LVDS_CTSET0, LVDS_CTSET0_ENABLE | LVDS_CTSET0_RUN,
		 LVDS_CTSET0_ENABLE);
	lvds_rmw(b, LVDS_FCOPT, LVDS_FCOPT_CLK_DET_SEL | LVDS_FCOPT_CT_AUTO,
		 LVDS_FCOPT_CT_AUTO | LVDS_FCOPT_CLK_DET_SEL);
	writel(0x10101010, b + LVDS_FCCNTR0);

	lvds_rmw(b, LVDS_CTSET0, LVDS_CTSET0_ENABLE | LVDS_CTSET0_RUN,
		 LVDS_CTSET0_ENABLE | LVDS_CTSET0_RUN);
}

/*
 * Vendor LVDS_PHY_SetCFcon(AUTOMATIC, enable): wait for coarse tuning, turn
 * on continuous tuning, close the loop. The vendor spins forever; a timeout
 * here most likely means the pixel clock is not what the mode claims.
 */
static int lvds_cfcon(struct tcc803x_lvds *lvds, struct tcc803x_lvds_port *p)
{
	const u32 m = LVDS_FCSTAT_CLK_OK | LVDS_FCSTAT_DONE | LVDS_FCSTAT_ERROR;
	void __iomem *b = p->base;
	u32 st;
	int ret;

	ret = readl_poll_timeout(b + LVDS_FCSTAT, st,
				 (st & m) == (LVDS_FCSTAT_CLK_OK | LVDS_FCSTAT_DONE),
				 10, 100000);
	if (ret) {
		dev_err(lvds->dev, "coarse tuning did not finish (FCSTAT %#x)\n", st);
		return ret;
	}

	writel(0x0000002c, b + LVDS_FCCONTINSET0);
	writel(0x00000960, b + LVDS_FCCONTINSET1);
	writel(0x003ff005, b + LVDS_FCCONTINSET2);
	writel(0x0000002d, b + LVDS_FCCONTINSET0);

	lvds_rmw(b, LVDS_CTSET1, LVDS_CTSET1_CLOSED_LOOP, LVDS_CTSET1_CLOSED_LOOP);
	return 0;
}

static int lvds_wait_pll(struct tcc803x_lvds *lvds, struct tcc803x_lvds_port *p)
{
	u32 st;
	int ret;

	ret = readl_poll_timeout(p->base + LVDS_MONITOR_DEBUG1, st,
				 (st & LVDS_MON_PLL_STATUS) == LVDS_MON_PLL_STATUS,
				 10, 100000);
	if (ret)
		dev_err(lvds->dev, "PLL did not lock (MONITOR_DEBUG1 %#x)\n", st);
	return ret;
}

#define lvds_for_each_port(lvds, p) \
	for (p = &(lvds)->pri; p; p = ((p) == &(lvds)->pri && (lvds)->dual) ? &(lvds)->sec : NULL)

/* Vendor order: lanes off before FIFOs, FIFOs on before lanes. */
static void lvds_lanes_fifos(struct tcc803x_lvds *lvds, bool on)
{
	struct tcc803x_lvds_port *p;

	if (on)
		lvds_for_each_port(lvds, p)
			lvds_rmw(p->base, LVDS_EN, LVDS_EN_ALL, LVDS_EN_ALL);
	lvds_for_each_port(lvds, p)
		lvds_rmw(p->base, LVDS_PORT, LVDS_PORT_LANE_EN,
			 on ? LVDS_PORT_LANE_EN : 0);
	if (!on)
		lvds_for_each_port(lvds, p)
			lvds_rmw(p->base, LVDS_EN, LVDS_EN_ALL, 0);
}

/* Vendor order from its LVDS panel power-on path; delays are the vendor's. */
static int tcc803x_lvds_phy_init(struct tcc803x_lvds *lvds, int clock_khz)
{
	struct tcc803x_lvds_port *p;
	u64 link_hz = lvds_link_hz(lvds, clock_khz);
	int upsample = lvds_upsample_ratio(link_hz);
	u32 ref_cnt;
	unsigned int lane;
	int ret;

	if (upsample < 0)
		return upsample;
	ref_cnt = lvds_ref_cnt(link_hz, upsample);

	lvds_for_each_port(lvds, p)
		writel(LVDS_CLK_SET_ENABLE, p->base + LVDS_CLK_SET);
	lvds_for_each_port(lvds, p)
		writel(0, p->base + LVDS_RESETB);
	usleep_range(1000, 1100);
	lvds_for_each_port(lvds, p)
		writel(LVDS_RESETB_ALL_RELEASE, p->base + LVDS_RESETB);

	lvds_for_each_port(lvds, p) {
		lvds_rmw(p->base, LVDS_STB_EN, LVDS_STB_EN_EN, LVDS_STB_EN_EN);
		lvds_rmw(p->base, LVDS_AUTO_STB_SET, LVDS_AUTO_STB_SET_AUTO_EN,
			 LVDS_AUTO_STB_SET_AUTO_EN);
	}

	ret = lvds_stb_init(&lvds->pri, lvds->dual, true);
	if (!ret && lvds->dual)
		ret = lvds_stb_init(&lvds->sec, true, false);
	lvds_for_each_port(lvds, p)
		ret = ret ?: lvds_stb_lanes(p, 1, lvds->stb_lane_cal);
	if (ret)
		goto stb_timeout;

	lvds_for_each_port(lvds, p)
		lvds_rmw(p->base, LVDS_PORT, LVDS_PORT_LANE_EN, 0);
	lvds_rmw(lvds->pri.base, LVDS_PORT,
		 LVDS_PORT_SYNC_SRC | LVDS_PORT_LANE_EN | LVDS_PORT_USE_OTHER_SYNC |
		 LVDS_PORT_VH_SWAP | LVDS_PORT_SECONDARY_CLR, 0);
	if (lvds->dual)
		lvds_rmw(lvds->sec.base, LVDS_PORT,
			 LVDS_PORT_SYNC_SRC | LVDS_PORT_LANE_EN |
			 LVDS_PORT_USE_OTHER_SYNC | LVDS_PORT_VH_SWAP |
			 LVDS_PORT_SECONDARY_CLR,
			 FIELD_PREP(LVDS_PORT_SYNC_SRC, 7) |
			 LVDS_PORT_USE_OTHER_SYNC | LVDS_PORT_SECONDARY);

	lvds_for_each_port(lvds, p)
		for (lane = 0; lane < LVDS_NUM_LANES; lane++)
			lvds_rmw(p->base, LVDS_PHY_IF_SET1,
				 LVDS_PHY_IF_SET1_LANE(lane),
				 p->lane_sel[lane] << (lane * 4));

	lvds_for_each_port(lvds, p) {
		ret = lvds_stb_ready(p, upsample);
		if (ret)
			goto stb_timeout;
	}

	lvds_for_each_port(lvds, p)
		lvds_fcon_auto(p, ref_cnt);
	lvds_for_each_port(lvds, p) {
		ret = lvds_cfcon(lvds, p);
		if (ret)
			return ret;
	}
	lvds_for_each_port(lvds, p) {
		ret = lvds_wait_pll(lvds, p);
		if (ret)
			return ret;
	}
	msleep(10);

	ret = lvds_stb_start(lvds);
	if (ret)
		goto stb_timeout;

	lvds_for_each_port(lvds, p) {
		lvds_rmw(p->base, LVDS_FORMAT,
			 LVDS_FORMAT_UPSAMPLE | LVDS_FORMAT_BALANCED |
			 LVDS_FORMAT_DEPTH_8BIT | LVDS_FORMAT_JEIDA,
			 lvds->format | FIELD_PREP(LVDS_FORMAT_UPSAMPLE, upsample));
		writel(FIELD_PREP(LVDS_STARTUP_FIFO2_RD_TIMING, 3),
		       p->base + LVDS_STARTUP_MODE);
	}

	lvds_lanes_fifos(lvds, false);
	lvds_for_each_port(lvds, p)
		writel(LVDS_RESETB_FIFO_RESET, p->base + LVDS_RESETB);
	usleep_range(1000, 1100);
	lvds_for_each_port(lvds, p)
		writel(LVDS_RESETB_ALL_RELEASE, p->base + LVDS_RESETB);
	lvds_lanes_fifos(lvds, true);

	lvds_for_each_port(lvds, p)
		if (!(readl(p->base + LVDS_MONITOR_DEBUG1) & LVDS_MON_LKVDET))
			dev_warn(lvds->dev, "%s port reports no lock detect\n",
				 p == &lvds->pri ? "primary" : "secondary");

	dev_info(lvds->dev, "PHY up: %s link, %llu Hz/link, upsample %d, ref_cnt %u\n",
		 lvds->dual ? "dual" : "single", link_hz, upsample, ref_cnt);
	return 0;

stb_timeout:
	dev_err(lvds->dev, "strobe access timed out\n");
	return ret;
}

static int tcc803x_lvds_attach(struct drm_bridge *bridge,
				struct drm_encoder *encoder,
				enum drm_bridge_attach_flags flags)
{
	/* Terminal bridge: no drm_panel driver exists for the panel node. */
	return 0;
}

/*
 * The one panel mode: live DISP0 timing captured off the running unit
 * (tcc803x-avn.dts lvds-panel, tcc803x_drm_vioc.c DISP0_*). Sync polarity
 * is the live CTRL.IP/IH/IV = 0, i.e. active high. The pixel clock is
 * LK's for this mode (vic 0x402); nothing here programs it.
 */
static int tcc803x_lvds_get_modes(struct drm_bridge *bridge,
				  struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_create(connector->dev);
	if (!mode)
		return 0;

	mode->clock = 99700;
	mode->hdisplay = 1920;
	mode->hsync_start = mode->hdisplay + 50;
	mode->hsync_end = mode->hsync_start + 32;
	mode->htotal = mode->hsync_end + 160;
	mode->vdisplay = 720;
	mode->vsync_start = mode->vdisplay + 15;
	mode->vsync_end = mode->vsync_start + 5;
	mode->vtotal = mode->vsync_end + 28;
	mode->flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC;
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = 243;
	connector->display_info.height_mm = 92;

	return 1;
}

static enum drm_mode_status
tcc803x_lvds_mode_valid(struct drm_bridge *bridge,
			 const struct drm_display_info *info,
			 const struct drm_display_mode *mode)
{
	struct tcc803x_lvds *lvds = to_tcc803x_lvds(bridge);
	u64 link_hz;

	if (!lvds->phy_init)
		return MODE_OK;

	link_hz = lvds_link_hz(lvds, mode->clock);
	if (link_hz * 7 > LVDS_VCO_MAX_HZ)
		return MODE_CLOCK_HIGH;
	if (lvds_upsample_ratio(link_hz) < 0)
		return MODE_CLOCK_LOW;
	return MODE_OK;
}

static void tcc803x_lvds_atomic_enable(struct drm_bridge *bridge,
					struct drm_atomic_commit *state)
{
	struct tcc803x_lvds *lvds = to_tcc803x_lvds(bridge);
	const struct drm_crtc_state *crtc_state;
	struct drm_connector *connector;
	struct drm_crtc *crtc;

	if (!lvds->phy_init)
		return;

	connector = drm_atomic_get_new_connector_for_encoder(state,
							     bridge->encoder);
	crtc = drm_atomic_get_new_connector_state(state, connector)->crtc;
	crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	/* After the CRTC is running: the vendor starts DISP before the PHY. */
	if (tcc803x_lvds_phy_init(lvds, crtc_state->adjusted_mode.clock))
		dev_err(lvds->dev, "PHY init failed, no LVDS output\n");
}

static void tcc803x_lvds_atomic_disable(struct drm_bridge *bridge,
					 struct drm_atomic_commit *state)
{
	struct tcc803x_lvds *lvds = to_tcc803x_lvds(bridge);

	/*
	 * Without phy-init nothing could bring the PHY back, so leave it. The
	 * vendor never powers it down; dropping lanes and FIFOs is the inverse
	 * of the last init step, and enable re-runs the whole sequence.
	 */
	if (lvds->phy_init)
		lvds_lanes_fifos(lvds, false);
}

static const struct drm_bridge_funcs tcc803x_lvds_bridge_funcs = {
	.attach			= tcc803x_lvds_attach,
	.get_modes		= tcc803x_lvds_get_modes,
	.mode_valid		= tcc803x_lvds_mode_valid,
	.atomic_enable		= tcc803x_lvds_atomic_enable,
	.atomic_disable		= tcc803x_lvds_atomic_disable,
	.atomic_duplicate_state	= drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_bridge_destroy_state,
	.atomic_create_state	= drm_atomic_helper_bridge_create_state,
};

static int tcc803x_lvds_parse_lanes(struct tcc803x_lvds *lvds,
				    const char *prop, u32 *sel)
{
	struct device *dev = lvds->dev;
	unsigned int i;
	int ret;

	ret = of_property_read_u32_array(dev->of_node, prop, sel, LVDS_NUM_LANES);
	if (ret == -EINVAL) {
		/* Absent: identity. Real boards route lanes, see the binding. */
		for (i = 0; i < LVDS_NUM_LANES; i++)
			sel[i] = i;
		return 0;
	}
	if (ret)
		return dev_err_probe(dev, ret, "invalid %s\n", prop);
	for (i = 0; i < LVDS_NUM_LANES; i++)
		if (sel[i] >= LVDS_NUM_LANES)
			return dev_err_probe(dev, -EINVAL, "%s[%u] out of range\n",
					     prop, i);
	return 0;
}

static int tcc803x_lvds_parse_phy(struct tcc803x_lvds *lvds)
{
	struct device *dev = lvds->dev;
	struct device_node *np = dev->of_node;
	const char *mapping = "vesa-24";
	u32 vcm, vsw;
	int ret;

	lvds->phy_init = of_property_read_bool(np, "telechips,phy-init");
	if (!lvds->phy_init)
		return 0;

	/* Panel-specific: no default, a wrong guess silently degrades the link. */
	if (of_property_read_u32(np, "telechips,vcm-millivolt", &vcm) ||
	    of_property_read_u32(np, "telechips,vsw-millivolt", &vsw))
		return dev_err_probe(dev, -EINVAL,
				     "phy-init needs telechips,vcm-millivolt and telechips,vsw-millivolt\n");
	lvds->stb_lane_cal = lvds_calibration(vcm, vsw);

	of_property_read_string(np, "telechips,data-mapping", &mapping);
	if (!strcmp(mapping, "vesa-24"))
		lvds->format = LVDS_FORMAT_DEPTH_8BIT;
	else if (!strcmp(mapping, "jeida-24"))
		lvds->format = LVDS_FORMAT_DEPTH_8BIT | LVDS_FORMAT_JEIDA;
	else if (!strcmp(mapping, "jeida-18"))
		lvds->format = LVDS_FORMAT_JEIDA;
	else
		return dev_err_probe(dev, -EINVAL, "unknown data-mapping %s\n",
				     mapping);

	lvds->dual = of_property_read_bool(np, "telechips,dual-link");
	if (lvds->dual) {
		lvds->pri.base = lvds->regs[LVDS_D0];
		lvds->sec.base = lvds->regs[LVDS_D1];
	} else {
		lvds->pri.base = lvds->regs[LVDS_S0];
	}

	ret = tcc803x_lvds_parse_lanes(lvds, "telechips,lane-select",
				       lvds->pri.lane_sel);
	if (!ret && lvds->dual)
		ret = tcc803x_lvds_parse_lanes(lvds,
					       "telechips,secondary-lane-select",
					       lvds->sec.lane_sel);
	return ret;
}

static int tcc803x_lvds_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tcc803x_lvds *lvds;
	unsigned int i;
	int ret;

	lvds = devm_drm_bridge_alloc(dev, struct tcc803x_lvds, bridge,
				      &tcc803x_lvds_bridge_funcs);
	if (IS_ERR(lvds))
		return PTR_ERR(lvds);

	lvds->dev = dev;

	lvds->phy_clk = devm_clk_get(dev, "lvds-phy");
	if (IS_ERR(lvds->phy_clk))
		return dev_err_probe(dev, PTR_ERR(lvds->phy_clk),
				      "Unable to get \"lvds-phy\" clock\n");

	/*
	 * Register the enable with the clock framework but never drop it:
	 * the bootloader already runs the gate (live read 0xe5000000), and
	 * gating it on disable or unload would take the panel down with no
	 * way to bring the PHY back.
	 */
	ret = clk_prepare_enable(lvds->phy_clk);
	if (ret)
		return dev_err_probe(dev, ret,
				     "PERI_LVDS_PHY clk_prepare_enable failed\n");

	for (i = 0; i < ARRAY_SIZE(lvds->regs); i++) {
		lvds->regs[i] = devm_platform_ioremap_resource_byname(pdev,
							tcc803x_lvds_reg_names[i]);
		if (IS_ERR(lvds->regs[i]))
			return dev_err_probe(dev, PTR_ERR(lvds->regs[i]),
					      "Unable to map \"%s\" region\n",
					      tcc803x_lvds_reg_names[i]);
	}

	ret = tcc803x_lvds_parse_phy(lvds);
	if (ret)
		return ret;

	lvds->bridge.of_node = dev->of_node;
	lvds->bridge.type = DRM_MODE_CONNECTOR_LVDS;
	lvds->bridge.ops = DRM_BRIDGE_OP_MODES;
	drm_bridge_add(&lvds->bridge);

	platform_set_drvdata(pdev, lvds);

	if (!lvds->phy_init)
		dev_info(dev, "telechips,phy-init unset: PHY left as the bootloader set it\n");

	return 0;
}

static void tcc803x_lvds_remove(struct platform_device *pdev)
{
	struct tcc803x_lvds *lvds = platform_get_drvdata(pdev);

	drm_bridge_remove(&lvds->bridge);
}

static const struct of_device_id tcc803x_lvds_of_table[] = {
	{ .compatible = "telechips,tcc803x-lvds" },
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, tcc803x_lvds_of_table);

static struct platform_driver tcc803x_lvds_platform_driver = {
	.probe	= tcc803x_lvds_probe,
	.remove	= tcc803x_lvds_remove,
	.driver	= {
		.name		= "tcc803x-lvds",
		.of_match_table	= tcc803x_lvds_of_table,
	},
};
module_platform_driver(tcc803x_lvds_platform_driver);

MODULE_AUTHOR("William Floyd <git@notmy.space>");
MODULE_DESCRIPTION("Telechips TCC803x on-die LVDS PHY bridge");
MODULE_LICENSE("GPL");
