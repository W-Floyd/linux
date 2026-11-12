// SPDX-License-Identifier: GPL-2.0-only
/*
 * TCC803x rear-view camera path, entirely in the VIOC:
 *
 *   decoder -> VIN0 -> DEINTL_S -> SC2 -> WMIX5 -> WDMA5 -> 4 frame buffers
 *           -> async FIFO -> RDMA1 = image 1 of the display WMIX0
 *
 * Shown by raising image 1 above the UI with WMIX0's overlay priority and
 * enabling RDMA1; hidden by restoring the priority and disabling RDMA1. That
 * is the vendor CM4 early-camera firmware's switch (TCC803x LK,
 * dev/camera/earlycamera/source/lcdc.c tcc_lcd_rear_camera_display(): OVP 19
 * on, 24 off), driven like its main loop (cm4_main.c): poll the reverse-gear
 * GPIO, start the capture path, show; restart the path if WDMA stalls.
 *
 * Block indices, path-select values and register sequences: Mobis
 * AVNT5W_TCC803XKK_kernel drivers/media/platform/tcccam/
 * tcc_cam_direct_display_if.c (direct_display_start/stop) and
 * arch/arm/mach-tcc803x/vioc/; board config from the LK tree's
 * dev/camera/tw9990.c (VIN0, SC2, WMIX5, WDMA5, display RDMA1, gear GPA31
 * active high, OVP 19/24). Not run on hardware.
 *
 * The TW9990 sends interlaced fields (datasheet: no deinterlacer of its own).
 * Deinterlacing uses DEINTL_S, the vendor's compile-time alternative
 * (FEATURE_USE_DEINTLS_REAR_CAMERA) to the VIQE it ships: no carve-out and
 * none of VIQE's undocumented tuning writes. Expect lower quality than stock.
 *
 * WMIX0 also belongs to tcc803x-drm, which read-modify-writes its CTRL (it
 * keeps OVP). A lost update could drop the raised priority, so the poll loop
 * re-asserts it while shown.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/workqueue.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

/* VIOC config block, Mobis mach/vioc_ireq.h TYPEDEF_VIOC_IREQ_CONFIG. */
#define CFG_MISC0		0x040
#define CFG_MISC0_WMIX50	BIT(26)	/* 1: WMIX path, 0: bypass */
#define CFG_PATH_SC2		0x04c
#define CFG_PATH_DEINTLS	0x058
#define CFG_PATH_SEL		GENMASK(7, 0)
#define CFG_PATH_STS		GENMASK(17, 16)
#define CFG_PATH_STS_OFF	0
#define CFG_PATH_STS_ON		2
#define CFG_PATH_ERR		BIT(18)
#define CFG_PATH_EN		BIT(31)
#define CFG_SWR0		0x0d8
#define CFG_SWR1		0x0dc
#define CFG_SWR3		0x12c

/* Path-select values for VIN0, Mobis mach/vioc_global.h. */
#define SEL_SC_VIN00		0x10
#define SEL_DEINTLS_VIN00	0x0a

/* VIN, mach/vioc_vin.h and vioc_vin.c. */
#define VIN_CTRL		0x00
#define VIN_CTRL_EN		BIT(0)
/* "conv_en"; on with the TW9990's BT.656 output, so presumably SAV/EAV decode. */
#define VIN_CTRL_CONV		BIT(1)
#define VIN_CTRL_INTL		BIT(2)
#define VIN_CTRL_INTPL		BIT(3)
#define VIN_CTRL_HSDE		BIT(4)
#define VIN_CTRL_VS_MASK	BIT(6)
#define VIN_CTRL_POL		GENMASK(13, 8)	/* pclk, hs, vs, field, de, gen-field */
#define VIN_CTRL_FMT		GENMASK(19, 16)
#define VIN_CTRL_ORDER		GENMASK(22, 20)
#define VIN_CTRL_CAP		BIT(31)
#define VIN_FMT_YUV422_8BIT	1
#define VIN_MISC		0x04
#define VIN_MISC_LUT_EN		GENMASK(2, 0)
#define VIN_MISC_Y2R_EN		BIT(4)
#define VIN_MISC_Y2RMD		GENMASK(6, 5)
#define VIN_SIZE		0x10
#define VIN_OFFS		0x14
#define VIN_OFFS_INTL		0x18
#define VIN_CROP_SIZE		0x1c
#define VIN_CROP_OFFS		0x20

/* DEINTL_S, vioc_deintls.c: mode in bits 3:0, 0 on the 803x. */
#define DEINTLS_CTRL		0x00
#define DEINTLS_MODE		GENMASK(3, 0)

/* Scaler, mach/vioc_scaler.h. */
#define SC_CTRL			0x00
#define SC_CTRL_BYPASS		BIT(0)
#define SC_CTRL_UPD		BIT(16)
#define SC_DSTSIZE		0x0c
#define SC_OUTPOS		0x10
#define SC_OUTSIZE		0x14

/* WMIX, mach/vioc_wmix.h. */
#define WMIX_CTRL		0x00
#define WMIX_CTRL_OVP		GENMASK(4, 0)
#define WMIX_CTRL_UPD		BIT(16)
#define WMIX_BG			0x04
#define WMIX_SIZE		0x08
/* Position of mixer layer @n, not of RDMA @n: they coincide only on WMIX0,
 * whose four layers are RDMA00-03.
 */
#define WMIX_POS(n)		(0x10 + 4 * (n))

/* WDMA, mach/vioc_wdma.h. */
#define WDMA_CTRL		0x00
#define WDMA_CTRL_FMT		GENMASK(4, 0)
#define WDMA_CTRL_UPD		BIT(16)
#define WDMA_CTRL_CONT		BIT(23)
#define WDMA_CTRL_IEN		BIT(28)
#define WDMA_SIZE		0x08
#define WDMA_BASE0		0x0c
#define WDMA_CBASE		0x10
#define WDMA_OFFS		0x1c
#define WDMA_IRQSTS		0x40
#define WDMA_IRQSTS_EOFR	BIT(5)
#define WDMA_IRQMSK		0x44
#define WDMA_IRQMSK_ALL		GENMASK(8, 0)

/* RDMA, mach/vioc_rdma.h. */
#define RDMA_CTRL		0x00
#define RDMA_CTRL_FMT		GENMASK(4, 0)
#define RDMA_CTRL_Y2R		BIT(8)
#define RDMA_CTRL_UPD		BIT(16)
#define RDMA_CTRL_R2Y		BIT(17)
#define RDMA_CTRL_IEN		BIT(28)
#define RDMA_SIZE		0x08
#define RDMA_BASE0		0x0c
#define RDMA_OFFSET		0x1c
#define RDMA_STATUS		0x28
#define RDMA_STATUS_EOFRISE	BIT(1)
#define RDMA_STATUS_DEVEOF	BIT(19)

/* Async FIFO channel 0, mach/vioc_fifo.h. */
#define FIFO_CTRL0		0x00
#define FIFO_CTRL0_EN		GENMASK(3, 0)	/* WEN, REN0..2 */
#define FIFO_CTRL0_WEN		BIT(0)
#define FIFO_CTRL0_REN0		BIT(1)
#define FIFO_CTRL0_CFG		GENMASK(19, 4)
#define FIFO_CTRL0_NENTRY	GENMASK(13, 8)
#define FIFO_CTRL0_EFULL	GENMASK(17, 16)
#define FIFO_CTRL0_EEMPTY	GENMASK(19, 18)
#define FIFO_CTRL1		0x04
#define FIFO_CTRL1_DMAS		GENMASK(15, 0)
#define FIFO_BASE(n)		(0x20 + 4 * (n))

/*
 * DDI_CONFIG CIFPORT0 (0x12380008): the CIF port each VIN reads, a 3-bit field
 * every 4 bits. The vendor writes it with masks spanning every VIN at once
 * (0x00077777, 0xFFE77777), clobbering the other ports' fields; write only ours.
 */
#define CIFPORT_VIN0		GENMASK(2, 0)

/* DISP size, mach/vioc_disp.h uLSIZE. */
#define DISP_SIZE		0x4c

#define FMT_ARGB8888		12	/* VIOC_IMG_FMT_ARGB8888, vioc_global.h */
#define NBUF			4	/* FIFO NENTRY, VIOC_ASYNC_FIFO_ConfigEntry() */

/* Block indices, from the LK dev/camera/tw9990.c board config. */
#define IDX_RDMA		1
#define IDX_WDMA		5
#define IDX_WMIX		5
#define IDX_SC			2
#define IDX_VIN			0

/* Decoder field size, vendor sensor_info_init() for the TW9990. */
#define SRC_WIDTH		720
#define SRC_FIELD_HEIGHT	240
/*
 * CM4 firmware: scale 7 + 1 lines taller than the panel and start the output
 * window 7 lines down (early_cam_preview_additional_height/crop_y, plus its
 * "scaler margin" line), trimming the top of the analog field.
 */
#define SC_EXTRA_LINES		8
#define SC_CROP_Y		7

#define OVP_SHOWN		19	/* early_cam_ovp */
#define OVP_NORMAL		24	/* main_display_ovp */

#define POLL_MS			50
#define HEALTH_TICKS		20	/* WDMA stall check once a second */

struct tcc_rearview {
	struct device *dev;
	void __iomem *cfg, *vin, *deintls, *sc, *wmix, *wdma, *rdma, *fifo;
	void __iomem *cifport, *disp_wmix, *disp;
	struct gpio_desc *gear;

	struct v4l2_device v4l2_dev;
	struct v4l2_async_notifier notifier;
	struct v4l2_subdev *decoder;

	dma_addr_t buf[NBUF];
	size_t buf_size;
	unsigned int width, height;

	struct delayed_work poll;
	struct mutex lock;	/* everything below and the hardware */
	u32 ovp_normal;
	bool manual;
	bool gear_raw, gear_on;
	bool started, shown;
	unsigned int ticks;
};

static void rv_rmw(void __iomem *reg, u32 clr, u32 set)
{
	writel((readl(reg) & ~clr) | set, reg);
}

/* VIOC_CONFIG_PlugIn() */
static int rv_plug_in(struct tcc_rearview *rv, unsigned int path, u32 sel)
{
	void __iomem *reg = rv->cfg + path;
	u32 v;
	int ret;

	rv_rmw(reg, CFG_PATH_SEL, sel);
	rv_rmw(reg, 0, CFG_PATH_EN);
	if (readl(reg) & CFG_PATH_ERR) {
		rv_rmw(reg, CFG_PATH_EN, 0);
		return -EBUSY;
	}
	ret = readl_poll_timeout(reg, v,
				 FIELD_GET(CFG_PATH_STS, v) == CFG_PATH_STS_ON,
				 1000, 50000);
	if (ret)
		dev_err(rv->dev, "plug-in %#x timed out (%#x)\n", path, v);
	return ret;
}

/* VIOC_CONFIG_PlugOut() */
static void rv_plug_out(struct tcc_rearview *rv, unsigned int path)
{
	void __iomem *reg = rv->cfg + path;
	u32 v;

	if (FIELD_GET(CFG_PATH_STS, readl(reg)) != CFG_PATH_STS_OFF) {
		rv_rmw(reg, CFG_PATH_EN, 0);
		if (readl_poll_timeout(reg, v,
				       FIELD_GET(CFG_PATH_STS, v) == CFG_PATH_STS_OFF,
				       1000, 100000))
			dev_err(rv->dev, "plug-out %#x timed out (%#x)\n", path, v);
	}
	writel(0, reg);
}

/*
 * direct_display_reset_vioc_path(): pulse the software resets of every block
 * the path owns. Bits per VIOC_CONFIG_SWReset().
 */
static void rv_reset_blocks(struct tcc_rearview *rv)
{
	static const struct { u16 reg; u8 bit; } rst[] = {
		{ CFG_SWR1, 31 },		/* FIFO 0 */
		{ CFG_SWR0, IDX_RDMA },
		{ CFG_SWR1, IDX_WDMA },
		{ CFG_SWR1, IDX_WMIX + 9 },
		{ CFG_SWR3, IDX_SC + 16 },
		{ CFG_SWR1, 17 },		/* DEINTL_S */
		{ CFG_SWR0, IDX_VIN + 24 },
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(rst); i++)
		rv_rmw(rv->cfg + rst[i].reg, 0, BIT(rst[i].bit));
	usleep_range(1000, 2000);
	for (i = ARRAY_SIZE(rst) - 1; i >= 0; i--)
		rv_rmw(rv->cfg + rst[i].reg, BIT(rst[i].bit), 0);
}

/* VIOC_RDMA_SetImageDisable(): clear IEN, then wait for the frame to end. */
static void rv_rdma_disable(struct tcc_rearview *rv)
{
	u32 v;

	if (!(readl(rv->rdma + RDMA_CTRL) & RDMA_CTRL_IEN))
		return;
	writel(RDMA_STATUS_EOFRISE, rv->rdma + RDMA_STATUS);
	rv_rmw(rv->rdma + RDMA_CTRL, RDMA_CTRL_IEN, RDMA_CTRL_UPD);
	if (readl_poll_timeout(rv->rdma + RDMA_STATUS, v,
			       v & RDMA_STATUS_DEVEOF, 1000, 60000))
		dev_warn(rv->dev, "RDMA%d did not stop\n", IDX_RDMA);
}

static void rv_set_ovp(struct tcc_rearview *rv, u32 ovp)
{
	rv_rmw(rv->disp_wmix + WMIX_CTRL, WMIX_CTRL_OVP,
	       FIELD_PREP(WMIX_CTRL_OVP, ovp) | WMIX_CTRL_UPD);
}

/* tcc_lcd_rear_camera_display(1) */
static void rv_show(struct tcc_rearview *rv)
{
	writel(0, rv->disp_wmix + WMIX_POS(IDX_RDMA));
	rv_rmw(rv->disp_wmix + WMIX_CTRL, WMIX_CTRL_OVP,
	       FIELD_PREP(WMIX_CTRL_OVP, OVP_SHOWN));
	rv_rmw(rv->rdma + RDMA_CTRL, 0, RDMA_CTRL_IEN | RDMA_CTRL_UPD);
	rv_rmw(rv->disp_wmix + WMIX_CTRL, 0, WMIX_CTRL_UPD);
	rv->shown = true;
	dev_info(rv->dev, "shown\n");
}

/* tcc_lcd_rear_camera_display(0) */
static void rv_hide(struct tcc_rearview *rv)
{
	rv_rmw(rv->disp_wmix + WMIX_CTRL, WMIX_CTRL_OVP,
	       FIELD_PREP(WMIX_CTRL_OVP, rv->ovp_normal));
	rv_rdma_disable(rv);
	rv_rmw(rv->disp_wmix + WMIX_CTRL, 0, WMIX_CTRL_UPD);
	rv->shown = false;
	dev_info(rv->dev, "hidden\n");
}

static void rv_program_vin(struct tcc_rearview *rv)
{
	/*
	 * TW9990 sensor_info_init(): pclk negative edge, hs/vs active high,
	 * de active low, no generated field; direct_display_set_vin() inverts
	 * the polarities into the register, which leaves all six bits 0.
	 */
	rv_rmw(rv->vin + VIN_CTRL,
	       VIN_CTRL_EN | VIN_CTRL_CONV | VIN_CTRL_INTL | VIN_CTRL_INTPL |
	       VIN_CTRL_HSDE | VIN_CTRL_VS_MASK | VIN_CTRL_POL | VIN_CTRL_FMT |
	       VIN_CTRL_ORDER | VIN_CTRL_CAP,
	       VIN_CTRL_CONV | VIN_CTRL_INTL |
	       FIELD_PREP(VIN_CTRL_FMT, VIN_FMT_YUV422_8BIT));
	writel(SRC_FIELD_HEIGHT << 16 | SRC_WIDTH, rv->vin + VIN_SIZE);
	writel(0, rv->vin + VIN_OFFS);
	rv_rmw(rv->vin + VIN_OFFS_INTL, GENMASK(31, 16), 0);
	writel(0, rv->vin + VIN_CROP_OFFS);
	writel(SRC_FIELD_HEIGHT << 16 | SRC_WIDTH, rv->vin + VIN_CROP_SIZE);
	/* DEINTL_S path: VIN converts to RGB itself (Y2R mode 2), no LUT. */
	rv_rmw(rv->vin + VIN_MISC,
	       VIN_MISC_LUT_EN | VIN_MISC_Y2R_EN | VIN_MISC_Y2RMD,
	       VIN_MISC_Y2R_EN | FIELD_PREP(VIN_MISC_Y2RMD, 2));
	rv_rmw(rv->vin + VIN_CTRL, 0, VIN_CTRL_EN);
}

static void rv_program_scaler(struct tcc_rearview *rv)
{
	u32 w = rv->width, h = rv->height;

	writel((h + SC_EXTRA_LINES) << 16 | w, rv->sc + SC_DSTSIZE);
	writel(SC_CROP_Y << 16, rv->sc + SC_OUTPOS);
	writel(h << 16 | w, rv->sc + SC_OUTSIZE);
	rv_rmw(rv->sc + SC_CTRL, SC_CTRL_BYPASS, SC_CTRL_UPD);
}

static int rv_start(struct tcc_rearview *rv)
{
	u32 size, w, h;
	int i, ret;

	if (!rv->decoder)
		return -ENODEV;

	size = readl(rv->disp + DISP_SIZE);
	w = size & 0xffff;
	h = size >> 16;
	if (!w || !h || w * h * 4 > rv->buf_size) {
		dev_err(rv->dev, "unusable display size %ux%u\n", w, h);
		return -EINVAL;
	}
	rv->width = w;
	rv->height = h;

	ret = v4l2_subdev_enable_streams(rv->decoder, 0, BIT_ULL(0));
	if (ret && ret != -EALREADY)
		return ret;

	rv_reset_blocks(rv);

	rv_rmw(rv->cifport, CIFPORT_VIN0, 0);	/* CIF port 0 */

	rv_program_vin(rv);

	ret = rv_plug_in(rv, CFG_PATH_DEINTLS, SEL_DEINTLS_VIN00);
	if (ret)
		goto err;
	rv_rmw(rv->deintls + DEINTLS_CTRL, DEINTLS_MODE, 0);

	ret = rv_plug_in(rv, CFG_PATH_SC2, SEL_SC_VIN00);
	if (ret)
		goto err;
	rv_program_scaler(rv);

	writel(h << 16 | w, rv->wmix + WMIX_SIZE);
	writel(0, rv->wmix + WMIX_POS(0));
	writel(0, rv->wmix + WMIX_BG);		/* black */
	rv_rmw(rv->cfg + CFG_MISC0, 0, CFG_MISC0_WMIX50);
	rv_rmw(rv->wmix + WMIX_CTRL, 0, WMIX_CTRL_UPD);

	rv_rmw(rv->wdma + WDMA_CTRL, WDMA_CTRL_FMT,
	       FIELD_PREP(WDMA_CTRL_FMT, FMT_ARGB8888));
	writel(h << 16 | w, rv->wdma + WDMA_SIZE);
	writel(4 * w, rv->wdma + WDMA_OFFS);
	writel(lower_32_bits(rv->buf[0]), rv->wdma + WDMA_BASE0);
	rv_rmw(rv->wdma + WDMA_CTRL, 0,
	       WDMA_CTRL_IEN | WDMA_CTRL_CONT | WDMA_CTRL_UPD);

	/* CM4 waits 100 ms before pointing the display RDMA at the buffers. */
	msleep(100);

	/* Plain RGB to the display path: no colour conversion on RDMA1. */
	rv_rmw(rv->rdma + RDMA_CTRL,
	       RDMA_CTRL_FMT | RDMA_CTRL_Y2R | RDMA_CTRL_R2Y,
	       FIELD_PREP(RDMA_CTRL_FMT, FMT_ARGB8888));
	writel(h << 16 | w, rv->rdma + RDMA_SIZE);
	writel(4 * w, rv->rdma + RDMA_OFFSET);
	writel(lower_32_bits(rv->buf[0]), rv->rdma + RDMA_BASE0);

	/* WDMA5 fills the NBUF entries in turn, RDMA1 follows. */
	rv_rmw(rv->fifo + FIFO_CTRL1, FIFO_CTRL1_DMAS,
	       IDX_RDMA << 12 | IDX_RDMA << 8 | IDX_RDMA << 4 | IDX_WDMA);
	for (i = 0; i < NBUF; i++)
		writel(lower_32_bits(rv->buf[i]), rv->fifo + FIFO_BASE(i));
	rv_rmw(rv->fifo + FIFO_CTRL0, FIFO_CTRL0_CFG,
	       FIELD_PREP(FIFO_CTRL0_EEMPTY, 1) |
	       FIELD_PREP(FIFO_CTRL0_EFULL, 1) |
	       FIELD_PREP(FIFO_CTRL0_NENTRY, NBUF));
	rv_rmw(rv->fifo + FIFO_CTRL0, FIFO_CTRL0_EN,
	       FIFO_CTRL0_WEN | FIFO_CTRL0_REN0);

	rv->started = true;
	return 0;

err:
	rv_plug_out(rv, CFG_PATH_SC2);
	rv_plug_out(rv, CFG_PATH_DEINTLS);
	rv_rmw(rv->vin + VIN_CTRL, VIN_CTRL_EN, 0);
	rv_reset_blocks(rv);
	v4l2_subdev_disable_streams(rv->decoder, 0, BIT_ULL(0));
	return ret;
}

/* direct_display_stop() / tcc_cif_stop_stream() */
static void rv_stop(struct tcc_rearview *rv)
{
	u32 v;

	rv_rmw(rv->fifo + FIFO_CTRL0, FIFO_CTRL0_EN, 0);
	rv_rdma_disable(rv);

	writel(WDMA_IRQMSK_ALL, rv->wdma + WDMA_IRQMSK);
	rv_rmw(rv->wdma + WDMA_CTRL, WDMA_CTRL_IEN, WDMA_CTRL_UPD);
	if (readl_poll_timeout(rv->wdma + WDMA_IRQSTS, v,
			       v & WDMA_IRQSTS_EOFR, 5000, 100000))
		dev_warn(rv->dev, "WDMA%d did not stop\n", IDX_WDMA);

	rv_plug_out(rv, CFG_PATH_SC2);
	rv_plug_out(rv, CFG_PATH_DEINTLS);
	rv_rmw(rv->vin + VIN_CTRL, VIN_CTRL_EN, 0);
	rv_reset_blocks(rv);

	if (rv->decoder)
		v4l2_subdev_disable_streams(rv->decoder, 0, BIT_ULL(0));
	rv->started = false;
}

/* tcc_cif_wdma_counter(): the write pointer moves while a field is written. */
static bool rv_wdma_moving(struct tcc_rearview *rv)
{
	u32 prev, cur = readl(rv->wdma + WDMA_CBASE);
	int i;

	for (i = 0; i < 5; i++) {
		usleep_range(3000, 4000);
		prev = cur;
		cur = readl(rv->wdma + WDMA_CBASE);
		if (cur != prev)
			return true;
	}
	return false;
}

static bool rv_signal_locked(struct tcc_rearview *rv)
{
	u32 status;

	if (!rv->decoder ||
	    v4l2_subdev_call(rv->decoder, video, g_input_status, &status))
		return false;
	return !(status & V4L2_IN_ST_NO_SIGNAL);
}

static void rv_poll(struct work_struct *work)
{
	struct tcc_rearview *rv = container_of(to_delayed_work(work),
					       struct tcc_rearview, poll);
	bool raw, want;
	int ret;

	mutex_lock(&rv->lock);

	/* Two equal samples in a row (POLL_MS apart) before the state moves. */
	raw = rv->gear && gpiod_get_value_cansleep(rv->gear) > 0;
	if (raw == rv->gear_raw)
		rv->gear_on = raw;
	rv->gear_raw = raw;

	want = rv->gear_on || rv->manual;

	if (want && !rv->started) {
		ret = rv_start(rv);
		if (ret)
			dev_err_ratelimited(rv->dev, "start: %d\n", ret);
	}
	if (want && rv->started && !rv->shown)
		rv_show(rv);
	if (!want && rv->shown)
		rv_hide(rv);
	if (!want && rv->started)
		rv_stop(rv);

	if (rv->shown) {
		u32 ovp = FIELD_GET(WMIX_CTRL_OVP,
				    readl(rv->disp_wmix + WMIX_CTRL));

		if (ovp != OVP_SHOWN) {
			dev_warn(rv->dev, "overlay priority lost (%u)\n", ovp);
			rv_set_ovp(rv, OVP_SHOWN);
		}

		/* cm4_main.c: restart only if the decoder has a picture. */
		if (++rv->ticks >= HEALTH_TICKS) {
			rv->ticks = 0;
			if (rv_signal_locked(rv) && !rv_wdma_moving(rv)) {
				dev_warn(rv->dev, "WDMA stalled, restarting\n");
				rv_hide(rv);
				rv_stop(rv);
				if (!rv_start(rv))
					rv_show(rv);
			}
		}
	}

	mutex_unlock(&rv->lock);

	schedule_delayed_work(&rv->poll, msecs_to_jiffies(POLL_MS));
}

/*
 * Request from userspace, independent of the gear: the micom's rear-camera
 * button (frame 01 c4 01 81, cmd 51) arrives over the micom UART in
 * userspace, so this is the hook for it. Reverse gear shows the camera
 * whatever this says.
 */
static ssize_t manual_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct tcc_rearview *rv = dev_get_drvdata(dev);
	bool on;
	int ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;

	mutex_lock(&rv->lock);
	rv->manual = on;
	mutex_unlock(&rv->lock);
	mod_delayed_work(system_percpu_wq, &rv->poll, 0);

	return count;
}

static ssize_t manual_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct tcc_rearview *rv = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", READ_ONCE(rv->manual));
}
static DEVICE_ATTR_RW(manual);

static ssize_t gear_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct tcc_rearview *rv = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", READ_ONCE(rv->gear_on));
}
static DEVICE_ATTR_RO(gear);

static ssize_t shown_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct tcc_rearview *rv = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", READ_ONCE(rv->shown));
}
static DEVICE_ATTR_RO(shown);

static ssize_t signal_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct tcc_rearview *rv = dev_get_drvdata(dev);
	bool locked;

	mutex_lock(&rv->lock);
	locked = rv_signal_locked(rv);
	mutex_unlock(&rv->lock);

	return sysfs_emit(buf, "%d\n", locked);
}
static DEVICE_ATTR_RO(signal);

static struct attribute *rv_attrs[] = {
	&dev_attr_manual.attr,
	&dev_attr_gear.attr,
	&dev_attr_shown.attr,
	&dev_attr_signal.attr,
	NULL
};
ATTRIBUTE_GROUPS(rv);

static int rv_bound(struct v4l2_async_notifier *notifier,
		    struct v4l2_subdev *sd, struct v4l2_async_connection *asc)
{
	struct tcc_rearview *rv = container_of(notifier, struct tcc_rearview,
					       notifier);

	mutex_lock(&rv->lock);
	rv->decoder = sd;
	mutex_unlock(&rv->lock);
	dev_info(rv->dev, "decoder %s bound\n", sd->name);

	return 0;
}

static void rv_unbind(struct v4l2_async_notifier *notifier,
		      struct v4l2_subdev *sd, struct v4l2_async_connection *asc)
{
	struct tcc_rearview *rv = container_of(notifier, struct tcc_rearview,
					       notifier);

	mutex_lock(&rv->lock);
	if (rv->shown)
		rv_hide(rv);
	if (rv->started)
		rv_stop(rv);
	rv->decoder = NULL;
	mutex_unlock(&rv->lock);
}

/* No video node here, so the decoder's controls are reachable only this way. */
static int rv_complete(struct v4l2_async_notifier *notifier)
{
	struct tcc_rearview *rv = container_of(notifier, struct tcc_rearview,
					       notifier);

	return v4l2_device_register_subdev_nodes(&rv->v4l2_dev);
}

static const struct v4l2_async_notifier_operations rv_notifier_ops = {
	.bound		= rv_bound,
	.unbind		= rv_unbind,
	.complete	= rv_complete,
};

/* A register window named @name in another node, mapped without claiming it. */
static void __iomem *rv_map_shared(struct device *dev, const char *phandle,
				   const char *name)
{
	struct device_node *np;
	struct resource res;
	int idx = 0, ret;

	np = of_parse_phandle(dev->of_node, phandle, 0);
	if (!np)
		return IOMEM_ERR_PTR(-EINVAL);
	if (name)
		idx = of_property_match_string(np, "reg-names", name);
	ret = idx < 0 ? idx : of_address_to_resource(np, idx, &res);
	of_node_put(np);
	if (ret)
		return IOMEM_ERR_PTR(ret);

	return devm_ioremap(dev, res.start, resource_size(&res)) ?:
	       IOMEM_ERR_PTR(-ENOMEM);
}

static int rv_probe(struct platform_device *pdev)
{
	static const char * const names[] = {
		"vin", "deintls", "scaler", "wmix", "wdma", "rdma", "fifo",
		"cifport",
	};
	struct device *dev = &pdev->dev;
	struct v4l2_async_connection *asc;
	struct fwnode_handle *ep;
	struct tcc_rearview *rv;
	void __iomem **regs[ARRAY_SIZE(names)];
	struct clk *clk;
	int i, ret;

	rv = devm_kzalloc(dev, sizeof(*rv), GFP_KERNEL);
	if (!rv)
		return -ENOMEM;
	rv->dev = dev;
	mutex_init(&rv->lock);
	INIT_DELAYED_WORK(&rv->poll, rv_poll);

	regs[0] = &rv->vin;
	regs[1] = &rv->deintls;
	regs[2] = &rv->sc;
	regs[3] = &rv->wmix;
	regs[4] = &rv->wdma;
	regs[5] = &rv->rdma;
	regs[6] = &rv->fifo;
	regs[7] = &rv->cifport;
	for (i = 0; i < ARRAY_SIZE(names); i++) {
		*regs[i] = devm_platform_ioremap_resource_byname(pdev, names[i]);
		if (IS_ERR(*regs[i]))
			return PTR_ERR(*regs[i]);
	}

	/* Shared with tcc803x-drm, which claims them. */
	rv->cfg = rv_map_shared(dev, "telechips,vioc-config", NULL);
	rv->disp_wmix = rv_map_shared(dev, "telechips,display", "wmix");
	rv->disp = rv_map_shared(dev, "telechips,display", "disp");
	if (IS_ERR(rv->cfg) || IS_ERR(rv->disp_wmix) || IS_ERR(rv->disp))
		return dev_err_probe(dev, -EINVAL, "shared VIOC windows\n");

	clk = devm_clk_get_optional_enabled(dev, "ddi");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "ddi clock\n");

	rv->gear = devm_gpiod_get_optional(dev, "gear", GPIOD_IN);
	if (IS_ERR(rv->gear))
		return dev_err_probe(dev, PTR_ERR(rv->gear), "gear gpio\n");
	if (!rv->gear)
		dev_warn(dev, "no gear-gpios: shown only on request\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	/* Sized for the one mode tcc803x-drm drives (1920x720). */
	rv->buf_size = 1920 * 720 * 4;
	for (i = 0; i < NBUF; i++) {
		if (!dmam_alloc_coherent(dev, rv->buf_size, &rv->buf[i],
					 GFP_KERNEL))
			return -ENOMEM;
	}

	/*
	 * Normal stacking as the bootloader/DRM left it. OVP_SHOWN means the
	 * M4 early camera's overlay is still up (it didn't take its STOP, or was
	 * told to exit with the path live): take that down.
	 */
	rv->ovp_normal = FIELD_GET(WMIX_CTRL_OVP, readl(rv->disp_wmix + WMIX_CTRL));
	if (rv->ovp_normal == OVP_SHOWN) {
		dev_warn(dev, "camera overlay left up, taking it down\n");
		rv->ovp_normal = OVP_NORMAL;
		rv_hide(rv);
		rv_stop(rv);
	}

	platform_set_drvdata(pdev, rv);

	ret = v4l2_device_register(dev, &rv->v4l2_dev);
	if (ret)
		return ret;

	v4l2_async_nf_init(&rv->notifier, &rv->v4l2_dev);
	rv->notifier.ops = &rv_notifier_ops;
	ep = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!ep) {
		ret = dev_err_probe(dev, -ENODEV, "no decoder endpoint\n");
		goto err_v4l2;
	}
	asc = v4l2_async_nf_add_fwnode_remote(&rv->notifier, ep,
					      struct v4l2_async_connection);
	fwnode_handle_put(ep);
	if (IS_ERR(asc)) {
		ret = PTR_ERR(asc);
		goto err_nf;
	}
	ret = v4l2_async_nf_register(&rv->notifier);
	if (ret)
		goto err_nf;

	schedule_delayed_work(&rv->poll, 0);
	return 0;

err_nf:
	v4l2_async_nf_cleanup(&rv->notifier);
err_v4l2:
	v4l2_device_unregister(&rv->v4l2_dev);
	return ret;
}

static void rv_remove(struct platform_device *pdev)
{
	struct tcc_rearview *rv = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&rv->poll);

	mutex_lock(&rv->lock);
	if (rv->shown)
		rv_hide(rv);
	if (rv->started)
		rv_stop(rv);
	mutex_unlock(&rv->lock);

	v4l2_async_nf_unregister(&rv->notifier);
	v4l2_async_nf_cleanup(&rv->notifier);
	v4l2_device_unregister(&rv->v4l2_dev);
}

static const struct of_device_id rv_of_match[] = {
	{ .compatible = "telechips,tcc803x-vioc-rearview" },
	{ }
};
MODULE_DEVICE_TABLE(of, rv_of_match);

static struct platform_driver rv_driver = {
	.probe	= rv_probe,
	.remove	= rv_remove,
	.driver	= {
		.name		= "tcc803x-rearview",
		.of_match_table	= rv_of_match,
		.dev_groups	= rv_groups,
	},
};
module_platform_driver(rv_driver);

MODULE_DESCRIPTION("TCC803x VIOC rear-view camera overlay");
MODULE_LICENSE("GPL");
