// SPDX-License-Identifier: GPL-2.0-only
/*
 * TCC803x VIOC CRTC+plane driver (RDMA -> WMIX -> DISP0 single-plane path)
 *
 * Binds as a component (see tcc803x_drm_drv.c) against the
 * "telechips,tcc803x-vioc-crtc" DT node (vioc_crtc0, SoC .dtsi). That one
 * node's own ->bind() creates BOTH the DRM CRTC (WMIX compositing +
 * DISP0 timing) and its single primary plane (RDMA0, reached via the
 * node's "telechips,planes" phandle) in one call, rather than registering
 * the plane as a separate top-level component. Reasoning: the top-level
 * driver's "ports" list (tcc803x-avn.dts, display-subsystem node) only
 * names &vioc_crtc0 and &hdmi_tx, not &vioc_rdma0 -- so vioc_rdma0 is
 * never independently matched by component_bind_all(), and splitting the
 * plane into its own ->bind() would hit a real ordering problem: a CRTC
 * needs its primary plane object to already exist when
 * drm_crtc_init_with_planes() runs, but component bind order between two
 * independently-matched devices is not guaranteed. One component owning
 * both avoids that instead of papering over it.
 *
 * Register offsets/bit positions below are all confirmed against the
 * real vendor Linux 3.18 tree (AVNT5W_TCC803XKK_kernel, Hyundai Mobis
 * supplier-portal release, Telechips Inc., GPLv2) -- see the per-field
 * comments. This is a clean-room reimplementation against DRM's atomic
 * helpers, not a copy of the vendor's own vioc_rdma.c/vioc_wmix.c/
 * vioc_disp.c (which are register-struct/bREG style, not Linux clk/DRM
 * API shaped).
 *
 * HONEST STATUS (read before assuming this lights up a screen):
 *
 * DONE / real:
 *  - RDMA0 plane: atomic_update programs CTRL.FMT, SIZE, BASE0 and the
 *    IEN+UPD enable/latch bits from the real GEM-DMA framebuffer address
 *    DRM hands it (drm_fb_dma_get_gem_addr()) -- a real atomic plane, not
 *    a hardcoded test pattern. Only DRM_FORMAT_XRGB8888 is wired
 *    (VIOC_IMG_FMT_ARGB8888 = 12, alpha ignored on scanout).
 *  - WMIX0: OVP (overlay priority) and UPD bits programmed so RDMA0's
 *    layer is actually composited and latched.
 *  - DISP0: CTRL.LEN (display-enable) and uLSIZE programmed on
 *    atomic_enable/disable.
 *
 * NOT DONE / blocked, in priority order:
 *  1. [DONE as of this patch, SUPERSEDES the previous hdmi_1920x720.c
 *     version] DISP0 timing generator (LHTIME1/2, LVTIME1/2 @ offsets
 *     0x00c/0x010/0x014/0x018), LCLKDIV (0x008) and CTRL polarity are
 *     programmed in atomic_enable() for 1920x720. The timing numbers
 *     are no longer taken from drivers/video/fbdev/tcc-fb/
 *     hdmi_1920x720.c's hdmi1920x720_panel struct -- a live /dev/mem
 *     register capture off the real, running AVN head unit (DISP0
 *     base 0x12000000) proved that vendor file WRONG for this board on
 *     every timing field except resolution:
 *       SIZE     0x1200004c = 0x02d00780 (1920x720, matches)
 *       LCLKDIV  0x12000008 = 0x00000000 (PXCLKDIV=0, ACDIV=0 -- NOT
 *                the PXCLKDIV=1/ACDIV=1 hdmi_1920x720.c + tca_lcdc.c
 *                would set)
 *       LHTIME1  0x1200000c = 0x001f077f (LPC=1919=1920-1, LPW=31 --
 *                not LPW=2)
 *       LHTIME2  0x12000010 = 0x009f0031 (LSWC=159, LEWC=49 -- not
 *                LSWC=12, LEWC=7)
 *       LVTIME1  0x12000014 = 0x000402cf (FLC=719=720-1, FPW=4 -- not
 *                FPW=0)
 *       LVTIME2  0x12000018 = 0x001b000e (FSWC=27, FEWC=14 -- not
 *                FSWC=6, FEWC=4)
 *       CTRL     0x12000000 = 0x004c0141 (LEN=1: display genuinely
 *                enabled live. Decoded against vioc_disp.h:92-116's
 *                VIOC_DISP_LCTRL bitfield: IP/IH/IV (bits 11-13, sync
 *                polarity invert) are all 0 -- NOT inverted, unlike
 *                hdmi_1920x720.c's sync_invert=IV_INVERT|IH_INVERT,
 *                which this file previously copied. Other set bits
 *                (TV#6, DP#8, PXDW=12 in bits16-20, Y2RMD=2 in
 *                bits21-23) are left alone below: this file only
 *                read-modify-writes LEN, never clobbering whatever the
 *                bootloader already has there.)
 *     hdmi_1920x720.c is not referenced for timing values anymore --
 *     it may still be right about the general IP/IH/IV convention
 *     *existing*, just not about this board's actual polarity. Target
 *     is 1920x720, not 1920x1080: three independent touch-controller
 *     coord definitions on the real AVN board
 *     (arch/arm/boot/dts/mobis/tcc8031-android-avn.dtsi: siw,sw17700
 *     max_x/max_y, focaltech,fts display-coords, himax,hxcommon
 *     panel-coords/display-coords) all agree on 1920x720, and the live
 *     SIZE register above confirms it directly.
 *  2. [OPEN, not fabricated] Pixel clock source for DISP0/LCLK is
 *     UNRESOLVED. PXCLKDIV=0 almost certainly means passthrough
 *     (PXCLK=LCLK, bypassing the "/(2*PXCLKDIV)" divide that only
 *     applies for PXCLKDIV>=1 -- see vioc_disp.h:139's own field
 *     comment, which only defines the formula, not a PXCLKDIV=0 case),
 *     but a live clk_summary capture the same night showed PERI_LCD0
 *     (peri_9, the clock this driver's "disp0" DT binding names) at
 *     only 12MHz -- (1920+159+31+49) x (720+27+14+4) at 12MHz is a
 *     ~7Hz refresh, not plausible for a working panel. Vendor source
 *     (drivers/clk/tcc/clk-tcc803x-a7s.c:1396-1417,
 *     tcc_ckc_peri_set_rate()) shows PERI_LCD0's PCLKCTRL mux (SEL
 *     field, bits[28:24] of CKC_PCLKCTRL(9), i.e. offset
 *     0x0D0+9*4=0xF4 from the CKC base) is NOT limited to
 *     XIN/PLL0-4(DIV): PCLKCTRL_SEL_HDMIPCLK/HDMITMDS/HDMIXIN/
 *     PCIPHY_CLKOUT/EXTCLK0-1 are all valid SEL values for this same
 *     mux (clk-tcc803x-a7s-reg.h:270-277), and
 *     tcc_ckc_peri_get_rate()'s own switch (clk-tcc803x-a7s.c:1478-
 *     1486) returns 0 for every one of them -- the vendor clk driver
 *     admits it cannot read back a rate for those sources. That is:
 *     clk_summary's "12MHz" for PERI_LCD0 is consistent with either
 *     (a) PERI_LCD0 genuinely being XIN-sourced as a register/bus
 *     clock distinct from DISP0's actual pixel-timing LCLK input, or
 *     (b) the common-clk core's cached/stale idea of PERI_LCD0's rate
 *     not reflecting a SEL that's actually parked on one of the
 *     external, unmodeled sources above. Vendor source does not
 *     resolve which. Does NOT say so independently from a guess: this
 *     is read off tcc_ckc_peri_set_rate()/_get_rate()'s real switch
 *     statements, not inferred.
 *     ADDITIONAL LIVE READ THAT WOULD RESOLVE THIS: CKC_PCLKCTRL(9)
 *     at physical 0x140000F4 (CKC base 0x14000000 confirmed in this
 *     tree's tcc803x.dtsi, + CKC_PCLKCTRL offset 0x0D0, + id=9*4 for
 *     PERI_LCD0 -- include/dt-bindings/clock/tcc803x-clks.h:49). Bits
 *     [28:24] are the SEL field (PCLKCTRL_SEL_MASK=0x1F at
 *     PCLKCTRL_SEL_SHIFT=24): SEL=5 is XIN (confirms the 12MHz read
 *     really is what's muxed in, meaning LCLK must come from a
 *     different physical pin/mux than this PERI_LCD0 gate entirely);
 *     SEL=20 is PCLKCTRL_SEL_HDMIPCLK, SEL=19 HDMITMDS, SEL=21
 *     HDMIXIN, SEL=18 PCIPHY_CLKOUT, SEL=25/26 EXTCLK0/1 (any of
 *     these would confirm LCLK bypasses the PLL ladder via an
 *     on-chip PHY/external source clk_summary can't see). Because the
 *     live reads were taken off an already-running, already-correct
 *     stock unit, this driver does NOT call clk_set_rate() on "disp0"
 *     below anymore -- whatever clock configuration the bootloader
 *     left in place is, by definition, the one that produces a working
 *     picture on this hardware tonight, and guessing a rate from the
 *     wrong PXCLKDIV=1 formula (as the previous version of this patch
 *     did, targeting a fabricated 194.68MHz) risks breaking a display
 *     this driver doesn't yet understand well enough to reprogram.
 *     clk-tcc803x-ckc.c's .set_rate() for PERI_LCD0 is unused by this
 *     file as of this patch pending the live PCLKCTRL(9) read above.
 *
 *     UPDATE (same-night follow-up, live CKC_PCLKCTRL(9) read obtained):
 *     0x140000F4 = 0xe5000001. SEL=5 (XIN) as predicted above -- but the
 *     DIV field (bits[11:0]) is 1, not 0: actual divisor = DIV+1 = 2
 *     (clk-tcc803x-a7s.c tcc_find_pclk()'s divider-mode path, PCLKCTRL->
 *     div -= 1 before the write). So PERI_LCD0's own gate outputs XIN/2
 *     = 6MHz, not a bare 12MHz passthrough -- the "~7Hz" estimate above
 *     already had two compensating errors (ignoring this /2, and
 *     ignoring CTRL.DP below), not one.
 *
 *     Also: PXDW (CTRL bits 16-20, live value 12) is NOT a pixel-per-
 *     clock field -- it's the output pixel *format* width (12 ==
 *     DCTRL_FMT_24BIT_RGB888, vioc_disp.h:598; same enum value as
 *     LCDC_IMG_FMT_RGB888, hdmi_1_4_hdmi.h:360). The real multi-pixel-
 *     per-clock control is CTRL bit 8, "DP" / "Double Pixel Data"
 *     (vioc_disp.h:570,595, struct field :100, comment :512). Live
 *     CTRL=0x004c0141 has DP=1 (confirmed by direct bit decode, not
 *     read off a macro table) -- DISP0 is in double-pixel mode.
 *
 *     hdmi_1920x720.c (drivers/video/fbdev/tcc-fb/, this exact 1920x720
 *     panel) requests its pixel clock through a *separate* clk handle,
 *     "lcdc0-clk" (hdmi_lcdc0_clk, of_clk_get_by_name), distinct from
 *     "hdmi-pclk" (PERI_LCD0) -- clk_freq=97,340,000, clk_div=2. (Note:
 *     tca_lcdc.c:176 writes PXCLKDIV_hw = clk_div/2 = 1 into LCLKDIV,
 *     not clk_div itself -- the "/2" is baked into that write, separate
 *     from the "/(2*PXCLKDIV)" hardware formula.) That a second,
 *     differently-named clock exists at all supports the "LCLK reaches
 *     DISP0 by a path PCLKCTRL(9)/PERI_LCD0 doesn't gate" hypothesis the
 *     live SEL=5 read is consistent with. Applying hdmi_1920x720.c's
 *     numbers: PXCLK = 97.34MHz / (2*1) = 48.67MHz; with that file's own
 *     htotal=1941 (1920+12+7+2), vtotal=730 (720+6+4+0) that's a ~34Hz
 *     refresh -- and DP's pixel-doubling would roughly double that again
 *     into a plausible 60-70Hz-class rate.
 *
 *     CAUTION, do not act on the above as if it resolves this board:
 *     hdmi_1920x720.c's own LHTIME/LVTIME numbers (LPW=2, LSWC=12,
 *     LEWC=7, FPW=0, FSWC=6, FEWC=4) do NOT match the live blanking read
 *     in item 1 above (LPW=31, LSWC=159, LEWC=49, FPW=4, FSWC=27,
 *     FEWC=14) either -- not a small rounding gap, a >10x difference on
 *     LPW alone. Whatever table this board's real firmware uses, it
 *     isn't this GPL reference file's, for timing OR (per the DIV/DP
 *     findings above) necessarily for clock routing. The "lcdc0-clk"
 *     DT clock-cell this driver's actual DISP0 LCLK input resolves to
 *     could not be traced further in this vendor tree (no tcc803x
 *     board .dtsi here instantiates an "hdmi1920x720"-compatible node
 *     with that clock-names list -- the AVN/Mobis board files use a
 *     different, not-present-in-this-release panel/display path).
 *     NEXT LIVE READ TO SETTLE THIS: CKC_PLLPMS[0..4], physical
 *     0x14000040 + n*4 for n=0..4 (TCC803X_CKC_PLLPMS, this tree's
 *     clk-tcc803x-ckc.c:83-84). PLLPMS field layout: P bits[5:0], M
 *     bits[15:6] (10 bits for PLL0/1, 9 for PLL2-4), S bits[18:16], SRC
 *     bit[19] (0=XIN), EN bit[31] (clk-tcc803x-ckc.c:94-103). PLL rate =
 *     (SRC_clk / P) * M / 2^S. A PLL with EN=1 and a computed rate near
 *     48.67MHz, 97.34MHz, or 24.3MHz (common PXCLKDIV candidates off a
 *     ~97MHz-class source) would confirm a dedicated display PLL feeds
 *     DISP0 directly, bypassing PERI_LCD0/PCLKCTRL(9) entirely -- and
 *     would give a concrete number to retarget clk-tcc803x-ckc.c's
 *     PERI_LCD0 .set_rate() against instead of guessing.
 *  3. No vblank/IRQ handling: VIOC0_IRQI (SPI 151, see tcc803x.dtsi's
 *     vioc-crtc node comment for why not the vendor-documented 119) is
 *     declared in DT but this driver never requests it. No
 *     frame-complete signaling, no drm_crtc_vblank_* hooks in the crtc
 *     funcs.
 *  4. No IOMMU/address-space wiring, and this has now been checked, not
 *     just assumed: RDMA0's BASE0 is written with the dma_addr_t DRM
 *     gives this driver as-is. Vendor evidence (AVNT5W_TCC803XKK_kernel):
 *     grep for iommu/smmu/sysmmu/#iommu-cells across
 *     arch/arm/mach-tcc803x/vioc/, drivers/video/fbdev/tcc-fb/ (incl.
 *     vioc/vioc_rdma.c's VIOC_RDMA_SetImageBase()), and both
 *     arch/arm/boot/dts/{mobis,tcc}/tcc803x.dtsi is all-zero-match -- no
 *     #iommu-cells anywhere in the SoC dtsi, not even on unrelated
 *     peripherals. The production fbdev driver (tcc_vioc_fb.c
 *     tccfb_map_video_memory(), CONFIG_ARCH_TCC803X path) allocates with
 *     plain dma_alloc_writecombine() and feeds the resulting dma_addr_t
 *     straight into VIOC_RDMA_SetImageBase() with no iommu_map()/
 *     dma-iommu glue in between. The vendor tree does carry an IOMMU-
 *     aware DRM stack (drivers/gpu/drm/tcc/tcc_drm_iommu.c), but it's
 *     for a different, unused DRM_TCC path gated on CONFIG_TCC_IOMMU -- a
 *     symbol referenced only in drivers/gpu/drm/tcc/Kconfig and defined
 *     nowhere in the tree, i.e. permanently unselectable dead
 *     scaffolding. DRM_TCC never appears in any tcc803x_*_defconfig;
 *     the shipped display stack is FB_TCC (CONFIG_FB_TCC_OVERLAY=y /
 *     CONFIG_FB_TCC_USE_VSYNC_INTERRUPT=y in tcc803x_android_avn_defconfig),
 *     and no CONFIG_IOMMU/CONFIG_*SMMU symbol is defined or enabled
 *     anywhere in this vendor tree's Kconfig/defconfigs at all. Verdict:
 *     TCC803x's VIOC has no IOMMU in front of RDMA/WDMA; the raw
 *     physical-dma_addr_t BASE0 write in this driver is correct as-is
 *     and needs no IOMMU/dma-iommu support added.
 *  5. hdmi-tx has no probe driver yet (DT node/binding only) -- so even
 *     if 1-3 above were fixed, component_bind_all() in tcc803x_drm_drv.c
 *     still never completes, and this code path is never reached by a
 *     running kernel tonight.
 *
 * Bottom line: this compiles and is real glue (real register offsets,
 * real atomic plane/CRTC wiring, no invented clock IDs), but does not
 * produce a visible image on real hardware yet. Timing (1) is now the
 * live-captured values, not a guess at which vendor panel table applies.
 * The pixel-clock source (2) is explicitly still open -- see that point
 * for the exact register read that would close it. This driver no
 * longer touches "disp0"'s (PERI_LCD0's) clock rate at all, trusting
 * whatever the bootloader already has running, so it cannot itself make
 * the clock situation worse than it already is on a stock boot.
 * Whether the downstream physical interface is parallel RGB/LVDS or an
 * on-chip HDMI encoder feeding a bridge is still not fully resolved (see
 * point 1 above); that is drivers/gpu/drm/bridge/tcc803x-dw-hdmi.c's and
 * the output_config investigation's territory, not this file's. Points
 * 3-5 are still open.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_plane.h>
#include <drm/drm_plane_helper.h>

/*
 * RDMA CTRL (offset 0x00): FMT bits[4:0], UPD bit16, IEN bit28.
 * Confirmed: arch/arm/mach-tcc803x/include/mach/vioc_rdma.h:236-254
 * (HwDMA_FMT/HwDMA_UPD/HwDMA_IEN), vioc_rdma.c:40-55
 * (VIOC_RDMA_SetImageUpdate/SetImageEnable, bit-identical).
 */
#define RDMA_CTRL		0x00
#define RDMA_CTRL_FMT_MASK	GENMASK(4, 0)
#define RDMA_CTRL_UPD		BIT(16)
#define RDMA_CTRL_IEN		BIT(28)
#define RDMA_SIZE		0x08	/* (height << 16) | width -- vioc_rdma.c:191 */
#define RDMA_BASE0		0x0C	/* vioc_rdma.c:202-211 */

/* VIOC_IMG_FMT_ARGB8888 = 12: include/video/tcc/vioc_global.h:58 (vendor path). */
#define VIOC_IMG_FMT_ARGB8888	12

/*
 * WMIX CTRL (offset 0x00): OVP bits[4:0], UPD bit16.
 * Confirmed: arch/arm/mach-tcc803x/include/mach/vioc_wmix.h:231-245,
 * vioc_wmix.c:41-57 (VIOC_WMIX_SetOverlayPriority/SetUpdate).
 */
#define WMIX_CTRL		0x00
#define WMIX_CTRL_OVP_MASK	GENMASK(4, 0)
#define WMIX_CTRL_UPD		BIT(16)
#define WMIX_SIZE		0x08	/* (height << 16) | width -- vioc_wmix.c:74-78 */

/*
 * DISP CTRL (offset 0x000): bit0 = LEN (display enable).
 * Confirmed: arch/arm/mach-tcc803x/include/mach/vioc_disp.h:424-452,
 * vioc_disp.c:431-435 (VIOC_DISP_TurnOn/TurnOff, bit-identical).
 */
#define DISP_CTRL		0x000
#define DISP_CTRL_LEN		BIT(0)
/*
 * CTRL.IP/IH/IV (sync polarity invert, active-low when set): struct
 * VIOC_DISP_LCTRL bit layout, include/video/tcc/vioc_disp.h:92-116
 * (IP bit11, IH bit12, IV bit13), applied identically by
 * tca_lcdc.c:160-170 (DCTRL_IP/IH/IV) for any lcd_panel's
 * ->sync_invert flags.
 */
#define DISP_CTRL_IP		BIT(11)
#define DISP_CTRL_IH		BIT(12)
#define DISP_CTRL_IV		BIT(13)
#define DISP_SIZE		0x04C	/* uLSIZE, (height << 16) | width -- vioc_disp.c:96-103 */

/*
 * DISP0 timing generator + pixel-clock divider. Field layout and the
 * (xres-1)/(yres-1) convention confirmed against
 * include/video/tcc/vioc_disp.h:137-207 (VIOC_DISP_LCLKDIV/LHTIME1/
 * LHTIME2/LVTIME1/LVTIME2 bitfield structs) and the real board-init
 * code that programs them, drivers/video/fbdev/tcc-fb/vioc/
 * tca_lcdc.c:175-206 (lcdc_initialize()), both in the vendor tree
 * (AVNT5W_TCC803XKK_kernel).
 *
 * LCLKDIV (0x008): PXCLKDIV bits[7:0], ACDIV bits[23:16].
 * PXCLK = LCLK / (2 * PXCLKDIV) -- vioc_disp.h:139 field comment.
 * tca_lcdc.c:176 sets PXCLKDIV = lcd_panel->clk_div / 2; ACDIV is
 * always forced to 1 (tca_lcdc.c:175), independent of the panel.
 */
#define DISP_LCLKDIV			0x008
#define DISP_LCLKDIV_PXCLKDIV_MASK	GENMASK(7, 0)
#define DISP_LCLKDIV_ACDIV_MASK		GENMASK(23, 16)

/* LHTIME1 (0x00c): LPC bits[13:0] = hdisplay - 1, LPW bits[24:16] = hsync width. */
#define DISP_LHTIME1		0x00C
#define DISP_LHTIME1_LPC_MASK	GENMASK(13, 0)
#define DISP_LHTIME1_LPW_MASK	GENMASK(24, 16)

/* LHTIME2 (0x010): LEWC bits[8:0] = h front porch, LSWC bits[24:16] = h back porch. */
#define DISP_LHTIME2		0x010
#define DISP_LHTIME2_LEWC_MASK	GENMASK(8, 0)
#define DISP_LHTIME2_LSWC_MASK	GENMASK(24, 16)

/* LVTIME1 (0x014): FLC bits[13:0] = vdisplay - 1, FPW bits[21:16] = vsync width. */
#define DISP_LVTIME1		0x014
#define DISP_LVTIME1_FLC_MASK	GENMASK(13, 0)
#define DISP_LVTIME1_FPW_MASK	GENMASK(21, 16)

/* LVTIME2 (0x018): FEWC bits[8:0] = v front porch, FSWC bits[24:16] = v back porch. */
#define DISP_LVTIME2		0x018
#define DISP_LVTIME2_FEWC_MASK	GENMASK(8, 0)
#define DISP_LVTIME2_FSWC_MASK	GENMASK(24, 16)

/*
 * LVTIME3/4 (0x01c/0x020): second-field mirror of LVTIME1/2, used for
 * interlaced scan (vioc_disp.h "in Interlaced, even field timing,
 * otherwise should be same with LVTIME1/2"). This panel is
 * progressive, so these are programmed identically to LVTIME1/2
 * (tca_lcdc.c:199-204 does the same for every non-interlaced panel).
 */
#define DISP_LVTIME3		0x01C
#define DISP_LVTIME4		0x020

/*
 * Real 1920x720 DISP0 timing -- NOT the vendor hdmi_1920x720.c table,
 * NOT derived/estimated. Source: a live /dev/mem register capture off
 * DISP0 (base 0x12000000) on the real, running AVN head unit:
 *   SIZE    0x1200004c = 0x02d00780
 *   LCLKDIV 0x12000008 = 0x00000000
 *   LHTIME1 0x1200000c = 0x001f077f
 *   LHTIME2 0x12000010 = 0x009f0031
 *   LVTIME1 0x12000014 = 0x000402cf
 *   LVTIME2 0x12000018 = 0x001b000e
 * decomposed per the field layout above. This disagrees with
 * hdmi_1920x720.c's hdmi1920x720_panel struct on every timing field
 * except resolution (see the status note at point 1, top of file) --
 * that struct is not referenced here anymore.
 *
 * PXCLKDIV=0 and ACDIV=0 both read back as 0, not the 1/1 that
 * tca_lcdc.c:175-176 (ACDIV forced to 1, PXCLKDIV = vendor
 * clk_div/2) would produce for any lcd_panel table including
 * hdmi_1920x720.c's -- i.e. this board's real init path does not
 * match tca_lcdc.c's generic lcdc_initialize() either. Point 2 (top
 * of file) covers why PXCLKDIV=0 is read as passthrough rather than
 * literal divide-by-zero, and why the resulting pixel clock rate is
 * still an open question this file deliberately does not guess at.
 */
#define DISP0_HDISPLAY		1920
#define DISP0_VDISPLAY		720
#define DISP0_HSYNC_WIDTH	31	/* LPW, LHTIME1[24:16] */
#define DISP0_HBACK_PORCH	159	/* LSWC, LHTIME2[24:16] */
#define DISP0_HFRONT_PORCH	49	/* LEWC, LHTIME2[8:0] */
#define DISP0_VSYNC_WIDTH	4	/* FPW, LVTIME1[21:16] */
#define DISP0_VBACK_PORCH	27	/* FSWC, LVTIME2[24:16] */
#define DISP0_VFRONT_PORCH	14	/* FEWC, LVTIME2[8:0] */
#define DISP0_PXCLKDIV		0	/* live LCLKDIV read: passthrough, not /(2*0) */
#define DISP0_ACDIV		0	/* live LCLKDIV read: NOT the vendor-forced 1 */

struct tcc803x_vioc_crtc {
	struct drm_crtc base;
	struct drm_plane primary;
	void __iomem *wmix;
	void __iomem *disp;
	void __iomem *rdma;
	struct clk *disp0_clk;	/* DT clock-names "disp0" == PERI_LCD0, tcc803x.dtsi:518-519 */
};

#define to_tcc803x_crtc(c) container_of(c, struct tcc803x_vioc_crtc, base)
#define plane_to_tcc803x_crtc(p) container_of(p, struct tcc803x_vioc_crtc, primary)

static void vioc_write_size(void __iomem *base, unsigned int reg,
			     unsigned int width, unsigned int height)
{
	writel((height << 16) | width, base + reg);
}

/* --- Plane (RDMA0) --- */

static const u32 tcc803x_plane_formats[] = {
	DRM_FORMAT_XRGB8888,
};

static void tcc803x_plane_atomic_update(struct drm_plane *plane,
					 struct drm_atomic_commit *state)
{
	struct tcc803x_vioc_crtc *crtc = plane_to_tcc803x_crtc(plane);
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_framebuffer *fb = new_state->fb;
	dma_addr_t addr;
	u32 ctrl;

	if (!fb)
		return;

	addr = drm_fb_dma_get_gem_addr(fb, new_state, 0);

	writel(lower_32_bits(addr), crtc->rdma + RDMA_BASE0);
	vioc_write_size(crtc->rdma, RDMA_SIZE, fb->width, fb->height);

	ctrl = readl(crtc->rdma + RDMA_CTRL);
	ctrl &= ~RDMA_CTRL_FMT_MASK;
	ctrl |= FIELD_PREP(RDMA_CTRL_FMT_MASK, VIOC_IMG_FMT_ARGB8888);
	ctrl |= RDMA_CTRL_IEN;
	writel(ctrl, crtc->rdma + RDMA_CTRL);
	/* UPD latches the above into the live register set on next vsync
	 * (vioc_rdma.c:40-45) -- written separately, after IEN/FMT/BASE0/
	 * SIZE, matching VIOC_RDMA_SetImageEnable()'s own two-write order.
	 */
	writel(readl(crtc->rdma + RDMA_CTRL) | RDMA_CTRL_UPD, crtc->rdma + RDMA_CTRL);
}

static void tcc803x_plane_atomic_disable(struct drm_plane *plane,
					  struct drm_atomic_commit *state)
{
	struct tcc803x_vioc_crtc *crtc = plane_to_tcc803x_crtc(plane);
	u32 ctrl = readl(crtc->rdma + RDMA_CTRL);

	ctrl &= ~RDMA_CTRL_IEN;
	writel(ctrl, crtc->rdma + RDMA_CTRL);
}

/*
 * No .atomic_check: this kernel has no drm_plane_helper_atomic_check()
 * generic implementation to reuse, and this driver doesn't yet enforce
 * any plane-specific constraint (scaling, position, format-per-CRTC)
 * beyond what drm_atomic_helper_check() already does at the mode_config
 * level (see tcc803x_drm_drv.c). Add one if/when RDMA's real limits
 * (max size, required alignment, etc.) get confirmed from the vendor
 * source and need enforcing.
 */
static const struct drm_plane_helper_funcs tcc803x_plane_helper_funcs = {
	.atomic_update	= tcc803x_plane_atomic_update,
	.atomic_disable	= tcc803x_plane_atomic_disable,
};

static const struct drm_plane_funcs tcc803x_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= drm_plane_cleanup,
	.reset			= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state	= drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
};

/* --- CRTC (WMIX0 + DISP0) --- */

static void tcc803x_crtc_atomic_enable(struct drm_crtc *crtc_base,
					struct drm_atomic_commit *state)
{
	struct tcc803x_vioc_crtc *crtc = to_tcc803x_crtc(crtc_base);
	struct drm_crtc_state *crtc_state = crtc_base->state;
	unsigned int w = crtc_state->mode.hdisplay;
	unsigned int h = crtc_state->mode.vdisplay;
	u32 ctrl;

	/*
	 * Deliberately NOT calling clk_set_rate() on crtc->disp0_clk
	 * ("disp0" == PERI_LCD0) here. The previous version of this patch
	 * did, targeting a fabricated 194.68MHz derived from the wrong
	 * (hdmi_1920x720.c-sourced) PXCLKDIV=1 formula -- see this file's
	 * top-of-file status note, point 2, for why that formula doesn't
	 * apply (live LCLKDIV reads back PXCLKDIV=0) and why the real LCLK
	 * source/rate is still an open question. The live register capture
	 * this patch is based on was taken off an already-running, already-
	 * displaying stock unit, so whatever clock configuration the
	 * bootloader already left in place is -- by construction -- the one
	 * that works; reprogramming it from a half-understood model risks
	 * making things worse, not better.
	 */

	/* WMIX0: layer 0 (RDMA0) on top, nothing else composited. */
	ctrl = readl(crtc->wmix + WMIX_CTRL);
	ctrl &= ~WMIX_CTRL_OVP_MASK;
	writel(ctrl, crtc->wmix + WMIX_CTRL);
	vioc_write_size(crtc->wmix, WMIX_SIZE, w, h);
	writel(readl(crtc->wmix + WMIX_CTRL) | WMIX_CTRL_UPD, crtc->wmix + WMIX_CTRL);

	/*
	 * DISP0 timing generator + pixel-clock divider, fixed 1920x720
	 * live-captured timing (see DISP0_* defines above). w/h come from
	 * the CRTC's drm_display_mode, which for now is pinned to 1920x720
	 * by the only mode this component offers; this is not yet a
	 * generic mode-timing path (no conversion from an arbitrary
	 * drm_display_mode's hsync_start/hsync_end/htotal etc.) -- it
	 * writes exactly the DISP0_* constants, independent of w/h.
	 */
	writel(FIELD_PREP(DISP_LCLKDIV_PXCLKDIV_MASK, DISP0_PXCLKDIV) |
	       FIELD_PREP(DISP_LCLKDIV_ACDIV_MASK, DISP0_ACDIV),
	       crtc->disp + DISP_LCLKDIV);

	writel(FIELD_PREP(DISP_LHTIME1_LPC_MASK, DISP0_HDISPLAY - 1) |
	       FIELD_PREP(DISP_LHTIME1_LPW_MASK, DISP0_HSYNC_WIDTH),
	       crtc->disp + DISP_LHTIME1);
	writel(FIELD_PREP(DISP_LHTIME2_LEWC_MASK, DISP0_HFRONT_PORCH) |
	       FIELD_PREP(DISP_LHTIME2_LSWC_MASK, DISP0_HBACK_PORCH),
	       crtc->disp + DISP_LHTIME2);

	writel(FIELD_PREP(DISP_LVTIME1_FLC_MASK, DISP0_VDISPLAY - 1) |
	       FIELD_PREP(DISP_LVTIME1_FPW_MASK, DISP0_VSYNC_WIDTH),
	       crtc->disp + DISP_LVTIME1);
	writel(FIELD_PREP(DISP_LVTIME2_FEWC_MASK, DISP0_VFRONT_PORCH) |
	       FIELD_PREP(DISP_LVTIME2_FSWC_MASK, DISP0_VBACK_PORCH),
	       crtc->disp + DISP_LVTIME2);
	/* Progressive scan: LVTIME3/4 mirror LVTIME1/2 (tca_lcdc.c:199-204). */
	writel(FIELD_PREP(DISP_LVTIME1_FLC_MASK, DISP0_VDISPLAY - 1) |
	       FIELD_PREP(DISP_LVTIME1_FPW_MASK, DISP0_VSYNC_WIDTH),
	       crtc->disp + DISP_LVTIME3);
	writel(FIELD_PREP(DISP_LVTIME2_FEWC_MASK, DISP0_VFRONT_PORCH) |
	       FIELD_PREP(DISP_LVTIME2_FSWC_MASK, DISP0_VBACK_PORCH),
	       crtc->disp + DISP_LVTIME4);

	/*
	 * Sync polarity: nothing to write here. Live CTRL=0x004c0141 off
	 * the real board decodes (vioc_disp.h:92-116 VIOC_DISP_LCTRL) to
	 * IP/IH/IV all 0 -- i.e. NOT inverted, unlike hdmi_1920x720.c's
	 * sync_invert=IV_INVERT|IH_INVERT this file previously copied.
	 * Leaving CTRL's IP/IH/IV bits untouched (the read-modify-write
	 * below only ever sets LEN) matches that live state.
	 */

	vioc_write_size(crtc->disp, DISP_SIZE, w, h);
	writel(readl(crtc->disp + DISP_CTRL) | DISP_CTRL_LEN, crtc->disp + DISP_CTRL);
}

static void tcc803x_crtc_atomic_disable(struct drm_crtc *crtc_base,
					 struct drm_atomic_commit *state)
{
	struct tcc803x_vioc_crtc *crtc = to_tcc803x_crtc(crtc_base);

	writel(readl(crtc->disp + DISP_CTRL) & ~DISP_CTRL_LEN, crtc->disp + DISP_CTRL);
}

static const struct drm_crtc_helper_funcs tcc803x_crtc_helper_funcs = {
	.atomic_enable	= tcc803x_crtc_atomic_enable,
	.atomic_disable	= tcc803x_crtc_atomic_disable,
};

static const struct drm_crtc_funcs tcc803x_crtc_funcs = {
	.reset			= drm_atomic_helper_crtc_reset,
	.destroy		= drm_crtc_cleanup,
	.set_config		= drm_atomic_helper_set_config,
	.page_flip		= drm_atomic_helper_page_flip,
	.atomic_duplicate_state	= drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
};

/* --- component glue --- */

static int tcc803x_vioc_bind(struct device *dev, struct device *master, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct drm_device *drm = data;
	struct tcc803x_vioc_crtc *crtc;
	struct device_node *rdma_np;
	struct resource rdma_res;
	int ret;

	crtc = drmm_kzalloc(drm, sizeof(*crtc), GFP_KERNEL);
	if (!crtc)
		return -ENOMEM;

	crtc->wmix = devm_platform_ioremap_resource_byname(pdev, "wmix");
	if (IS_ERR(crtc->wmix))
		return PTR_ERR(crtc->wmix);

	crtc->disp = devm_platform_ioremap_resource_byname(pdev, "disp");
	if (IS_ERR(crtc->disp))
		return PTR_ERR(crtc->disp);

	/*
	 * tcc803x.dtsi:518-519 already names this node's second clock
	 * "disp0" (PERI_LCD0) -- the clk-tcc803x-ckc driver implements that
	 * ID (TCC803X_CLK_PERI_LCD0), so this resolves once both drivers are
	 * built in, no DT change needed here.
	 */
	crtc->disp0_clk = devm_clk_get(dev, "disp0");
	if (IS_ERR(crtc->disp0_clk))
		return dev_err_probe(dev, PTR_ERR(crtc->disp0_clk),
				      "failed to get \"disp0\" (PERI_LCD0) clock\n");

	rdma_np = of_parse_phandle(dev->of_node, "telechips,planes", 0);
	if (!rdma_np)
		return dev_err_probe(dev, -EINVAL, "missing telechips,planes phandle\n");

	ret = of_address_to_resource(rdma_np, 0, &rdma_res);
	of_node_put(rdma_np);
	if (ret)
		return dev_err_probe(dev, ret, "failed to resolve RDMA0 reg\n");

	crtc->rdma = devm_ioremap_resource(dev, &rdma_res);
	if (IS_ERR(crtc->rdma))
		return PTR_ERR(crtc->rdma);

	ret = drm_universal_plane_init(drm, &crtc->primary, 0,
					&tcc803x_plane_funcs,
					tcc803x_plane_formats,
					ARRAY_SIZE(tcc803x_plane_formats),
					NULL, DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(&crtc->primary, &tcc803x_plane_helper_funcs);

	ret = drm_crtc_init_with_planes(drm, &crtc->base, &crtc->primary, NULL,
					 &tcc803x_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(&crtc->base, &tcc803x_crtc_helper_funcs);

	/* Now that the CRTC exists, the primary plane may target it. */
	crtc->primary.possible_crtcs = drm_crtc_mask(&crtc->base);

	dev_set_drvdata(dev, crtc);

	return 0;
}

static void tcc803x_vioc_unbind(struct device *dev, struct device *master, void *data)
{
	/* CRTC/plane are drm-managed (drmm_kzalloc'd container); DRM core
	 * tears them down with the drm_device itself. Nothing to do here.
	 */
}

static const struct component_ops tcc803x_vioc_component_ops = {
	.bind	= tcc803x_vioc_bind,
	.unbind	= tcc803x_vioc_unbind,
};

static int tcc803x_vioc_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &tcc803x_vioc_component_ops);
}

static void tcc803x_vioc_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &tcc803x_vioc_component_ops);
}

static const struct of_device_id tcc803x_vioc_of_match[] = {
	{ .compatible = "telechips,tcc803x-vioc-crtc" },
	{}
};
MODULE_DEVICE_TABLE(of, tcc803x_vioc_of_match);

static struct platform_driver tcc803x_vioc_driver = {
	.probe = tcc803x_vioc_probe,
	.remove = tcc803x_vioc_remove,
	.driver = {
		.name = "tcc803x-vioc",
		.of_match_table = tcc803x_vioc_of_match,
	},
};
module_platform_driver(tcc803x_vioc_driver);

MODULE_DESCRIPTION("Telechips TCC803x VIOC CRTC/plane driver");
MODULE_LICENSE("GPL");
