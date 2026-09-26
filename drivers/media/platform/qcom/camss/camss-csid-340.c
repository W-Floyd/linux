// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm MSM Camera Subsystem - CSID (CSI Decoder) Module 340
 *
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/completion.h>
#include <linux/bitfield.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>

#include "camss.h"
#include "camss-csid.h"
#include "camss-csid-gen2.h"

#define CSID_RST_STROBES					(0x010)
#define		CSID_RST_SW_REGS			BIT(0)
#define		CSID_RST_IRQ				BIT(1)
#define CSID_RST_IFE_CLK				BIT(2)
#define		CSID_RST_PHY_CLK			BIT(3)
#define		CSID_RST_CSID_CLK			BIT(4)

#define CSID_IRQ_STATUS						(0x070)
#define CSID_IRQ_MASK						(0x074)
#define		CSID_IRQ_MASK_RST_DONE			BIT(0)
#define CSID_IRQ_CLEAR						(0x078)
#define CSID_IRQ_CMD						(0x080)
#define		CSID_IRQ_CMD_CLEAR			BIT(0)

#define CSID_CSI2_RX_CFG0					(0x100)
#define		CSI2_RX_CFG0_NUM_ACTIVE_LANES_MASK	GENMASK(1, 0)
#define		CSI2_RX_CFG0_DLX_INPUT_SEL_MASK		GENMASK(17, 4)
#define		CSI2_RX_CFG0_PHY_NUM_SEL_MASK		GENMASK(21, 20)
#define		CSI2_RX_CFG0_PHY_NUM_SEL_BASE_IDX	1
#define		CSI2_RX_CFG0_PHY_TYPE_SEL		BIT(24)

#define CSID_CSI2_RX_CFG1					(0x104)
#define		CSI2_RX_CFG1_PACKET_ECC_CORRECTION_EN	BIT(0)
#define		CSI2_RX_CFG1_MISR_EN			BIT(6)
#define		CSI2_RX_CFG1_CGC_MODE			BIT(7)

#define CSID_CFG0(iface)					(0x200 + 0x100 * (iface))
#define		CSID_CFG0_BYTE_CNTR_EN			BIT(0)
#define		CSID_CFG0_TIMESTAMP_EN			BIT(1)
#define		CSID_CFG0_DECODE_FORMAT_MASK		GENMASK(15, 12)
#define		CSID_CFG0_DECODE_FORMAT_NOP		CSID_CFG0_DECODE_FORMAT_MASK
#define		CSID_CFG0_DT_MASK			GENMASK(21, 16)
#define		CSID_CFG0_VC_MASK			GENMASK(23, 22)
#define		CSID_CFG0_DTID_MASK			GENMASK(28, 27)
#define		CSID_CFG0_ENABLE			BIT(31)

#define CSID_CTRL(iface)					(0x208 + 0x100 * (iface))
#define CSID_CTRL_HALT_AT_FRAME_BOUNDARY		0
#define CSID_CTRL_RESUME_AT_FRAME_BOUNDARY		1

enum csid_iface {
	CSID_IFACE_PIX,
	CSID_IFACE_RDI0,
	CSID_IFACE_RDI1,
	CSID_IFACE_RDI2,
};

static enum csid_iface csid_port_iface_map[MSM_CSID_MAX_SRC_STREAMS] = {
	[0] = CSID_IFACE_RDI0,
	[1] = CSID_IFACE_RDI1,
	[2] = CSID_IFACE_RDI2,
	[3] = CSID_IFACE_PIX,
};

static void __csid_configure_rx(struct csid_device *csid, struct csid_phy_config *phy)
{
	u32 val;

	val = FIELD_PREP(CSI2_RX_CFG0_NUM_ACTIVE_LANES_MASK, phy->lane_cnt - 1);
	val |= FIELD_PREP(CSI2_RX_CFG0_DLX_INPUT_SEL_MASK, phy->lane_assign);
	val |= FIELD_PREP(CSI2_RX_CFG0_PHY_NUM_SEL_MASK,
			  phy->csiphy_id + CSI2_RX_CFG0_PHY_NUM_SEL_BASE_IDX);
	writel_relaxed(val, csid->base + CSID_CSI2_RX_CFG0);

	val = CSI2_RX_CFG1_PACKET_ECC_CORRECTION_EN;
	writel_relaxed(val, csid->base + CSID_CSI2_RX_CFG1);
}

/* EXPERIMENT: RX packet capture control override (0x108), 0 = the port's VC/DT */
static uint csid_dbg_capture;
module_param(csid_dbg_capture, uint, 0644);
/* EXPERIMENT: override the VC the PD port matches, -1 = frame descriptor's */
static int csid_dbg_vc = -1;
module_param(csid_dbg_vc, int, 0644);

static const struct csid_format_info *csid_port_format(struct csid_device *csid, u8 port)
{
	struct v4l2_mbus_framefmt *input_format = &csid->fmt[MSM_CSID_PAD_FIRST_SRC + port];

	return csid_get_fmt_entry(csid->res->formats->formats,
				  csid->res->formats->nformats,
				  input_format->code);
}

static void __csid_configure_stream(struct csid_device *csid, u8 enable, u8 port, u8 vc, u8 dt)
{
	const struct csid_format_info *format = csid_port_format(csid, port);
	enum csid_iface iface = csid_port_iface_map[port];

	u8 dt_id;
	u32 val;

	/*
	 * Only RST_DONE is unmasked, so the ISR never runs while streaming and
	 * anything the receiver latched is still sitting in the status
	 * register. Read it on the way out rather than unmasking error
	 * interrupts, which would mean enabling bits this driver has no
	 * handling for on a path that runs per packet.
	 */
	if (!enable) {
		u32 irq = readl_relaxed(csid->base + CSID_IRQ_STATUS);

		if (irq & ~CSID_IRQ_MASK_RST_DONE)
			dev_warn(csid->camss->dev,
				 "CSID%u: latched IRQ status 0x%08x at stream stop\n",
				 csid->id, irq);
	}

	/* EXPERIMENT: what the path and the receiver saw, as camera.md §38 */
	if (iface != CSID_IFACE_PIX) {
		void __iomem *b = csid->base + CSID_CFG0(iface);

		if (!enable)
			dev_warn(csid->camss->dev,
				 "CSID%u port %u: irq %08x status %08x bytes %u/%u rx irq %08x long %08x %08x short %08x %08x\n",
				 csid->id, port,
				 readl_relaxed(csid->base + 0x30 + 0x10 * iface),
				 readl_relaxed(b + 0x50), readl_relaxed(b + 0xe0),
				 readl_relaxed(b + 0xe4),
				 readl_relaxed(csid->base + 0x20),
				 readl_relaxed(csid->base + 0x130),
				 readl_relaxed(csid->base + 0x134),
				 readl_relaxed(csid->base + 0x128),
				 readl_relaxed(csid->base + 0x12c));
		else
			writel_relaxed(csid_dbg_capture ? csid_dbg_capture :
				       BIT(0) | BIT(1) | (dt << 4) | (vc << 10) | (vc << 12),
				       csid->base + 0x108);
	}

	/*
	 * DT_ID is a two bit bitfield that is concatenated with
	 * the four least significant bits of the five bit VC
	 * bitfield to generate an internal CID value.
	 *
	 * CSID_CFG0(port)
	 * DT_ID : 28:27
	 * VC    : 26:22
	 * DT    : 21:16
	 *
	 * CID   : VC 3:0 << 2 | DT_ID 1:0
	 */
	dt_id = port & 0x03;

	if (iface == CSID_IFACE_PIX)
		val = FIELD_PREP(CSID_CFG0_DECODE_FORMAT_MASK, format->decode_format);
	else /* RDI is raw, no decoding */
		val = CSID_CFG0_DECODE_FORMAT_NOP;

	val |= FIELD_PREP(CSID_CFG0_DT_MASK, dt);
	val |= FIELD_PREP(CSID_CFG0_VC_MASK, vc);
	val |= FIELD_PREP(CSID_CFG0_DTID_MASK, dt_id);

	if (enable)
		val |= CSID_CFG0_ENABLE;

	/* EXPERIMENT: byte counter and timestamps */
	if (enable && iface != CSID_IFACE_PIX)
		val |= BIT(0) | BIT(2);

	dev_dbg(csid->camss->dev, "CSID%u: Stream %s (dt:0x%x df=0x%x port=%u vc=%u)\n",
		csid->id, enable ? "enable" : "disable", dt,
		format->decode_format, port, vc);

	writel_relaxed(val, csid->base + CSID_CFG0(iface));
	writel_relaxed(enable, csid->base + CSID_CTRL(iface));
}

static void csid_configure_streams(struct csid_device *csid, u8 enable)
{
	int i;

	__csid_configure_rx(csid, &csid->phy);

	for (i = 0; i < MSM_CSID_MAX_SRC_STREAMS; i++) {
		if (csid->phy.en_vc & BIT(i))
			__csid_configure_stream(csid, !!enable, i, 0,
						csid_port_format(csid, i)->data_type);
	}
}

static void csid_configure_rx(struct csid_device *csid)
{
	__csid_configure_rx(csid, &csid->phy);
}

/*
 * With the streams API the virtual channel and data type of each port come
 * from the transmitter's frame descriptor, so one CSI-2 link can feed several
 * ports with different data: a sensor's image data to RDI0 and its phase
 * detection data to RDI1, for instance. stream_id is the port.
 */
static void csid_enable_stream(struct csid_device *csid, u32 stream_id, u8 vc, u8 dt)
{
	if (csid_dbg_vc >= 0 && stream_id)
		vc = csid_dbg_vc;
	__csid_configure_stream(csid, 1, stream_id, vc, dt);
}

static void csid_disable_stream(struct csid_device *csid, u32 stream_id)
{
	__csid_configure_stream(csid, 0, stream_id, 0, 0);
}

static int csid_reset(struct csid_device *csid)
{
	unsigned long time;

	writel_relaxed(CSID_IRQ_MASK_RST_DONE, csid->base + CSID_IRQ_MASK);
	writel_relaxed(CSID_IRQ_MASK_RST_DONE, csid->base + CSID_IRQ_CLEAR);
	writel_relaxed(CSID_IRQ_CMD_CLEAR, csid->base + CSID_IRQ_CMD);

	reinit_completion(&csid->reset_complete);

	/* Reset with registers preserved */
	writel(CSID_RST_IRQ | CSID_RST_IFE_CLK | CSID_RST_PHY_CLK | CSID_RST_CSID_CLK,
	       csid->base + CSID_RST_STROBES);

	time = wait_for_completion_timeout(&csid->reset_complete,
					   msecs_to_jiffies(CSID_RESET_TIMEOUT_MS));
	if (!time) {
		dev_err(csid->camss->dev, "CSID%u: reset timeout\n", csid->id);
		return -EIO;
	}

	dev_dbg(csid->camss->dev, "CSID%u: reset done\n", csid->id);

	return 0;
}

static irqreturn_t csid_isr(int irq, void *dev)
{
	struct csid_device *csid = dev;
	u32 val;

	val = readl_relaxed(csid->base + CSID_IRQ_STATUS);
	writel_relaxed(val, csid->base + CSID_IRQ_CLEAR);
	writel_relaxed(CSID_IRQ_CMD_CLEAR, csid->base + CSID_IRQ_CMD);

	if (val & CSID_IRQ_MASK_RST_DONE)
		complete(&csid->reset_complete);
	else
		dev_warn_ratelimited(csid->camss->dev,
				     "CSID%u: unhandled interrupt, status 0x%08x\n",
				     csid->id, val);

	return IRQ_HANDLED;
}

static int csid_configure_testgen_pattern(struct csid_device *csid, s32 val)
{
	return -EOPNOTSUPP; /* Not part of CSID */
}

static void csid_subdev_init(struct csid_device *csid) {}

const struct csid_hw_ops csid_ops_340 = {
	.configure_testgen_pattern = csid_configure_testgen_pattern,
	.configure_stream = csid_configure_streams,
	.configure_rx = csid_configure_rx,
	.enable_stream = csid_enable_stream,
	.disable_stream = csid_disable_stream,
	.hw_version = csid_hw_version,
	.isr = csid_isr,
	.reset = csid_reset,
	.src_pad_code = csid_src_pad_code,
	.subdev_init = csid_subdev_init,
};
