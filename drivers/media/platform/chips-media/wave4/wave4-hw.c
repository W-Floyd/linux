// SPDX-License-Identifier: GPL-2.0-only
/*
 * Chips&Media Wave410 HEVC decoder: firmware boot and commands
 *
 * The sequences follow what the TCC803x stock hevc_lib.ko does (static
 * analysis; no vendor code is reused). Unlike Wave5 there is no command
 * queue: one command runs at a time, its completion is the bit of its own
 * command code in VINT_REASON_USR, and every decoder command carries the
 * instance's bitstream and buffer registers again.
 */
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>

#include "wave4.h"
#include "wave4-regs.h"

static inline u32 w4_read(struct wave4_dev *dev, u32 reg)
{
	return readl(dev->base + reg);
}

static inline void w4_write(struct wave4_dev *dev, u32 reg, u32 val)
{
	writel(val, dev->base + reg);
}

static void w4_fio_write(struct wave4_dev *dev, u32 addr, u32 val)
{
	w4_write(dev, W4_VPU_FIO_DATA, val);
	w4_write(dev, W4_VPU_FIO_CTRL_ADDR, W4_FIO_WRITE | (addr & 0xffff));
}

irqreturn_t wave4_irq(int irq, void *data)
{
	struct wave4_dev *dev = data;
	u32 reason = w4_read(dev, W4_VPU_VINT_REASON_USR);
	u32 usr;

	if (!reason)
		return IRQ_NONE;

	/* The vendor's order: reason clear, VINT clear, then the user bits. */
	w4_write(dev, W4_VPU_VINT_REASON_CLR, reason);
	w4_write(dev, W4_VPU_VINT_CLEAR, 1);
	/*
	 * USR is cleared by read-modify-write, not write-one-to-clear: record
	 * a completion bit the firmware set after the first read, or the write
	 * clears it unseen.
	 */
	usr = w4_read(dev, W4_VPU_VINT_REASON_USR);
	w4_write(dev, W4_VPU_VINT_REASON_USR, usr & ~0xffff);
	reason |= usr & 0xffff;

	spin_lock(&dev->irq_lock);
	dev->irq_reason |= reason;
	spin_unlock(&dev->irq_lock);
	complete(&dev->cmd_done);
	return IRQ_HANDLED;
}

static void w4_issue(struct wave4_dev *dev, u32 inst, u32 cmd)
{
	spin_lock_irq(&dev->irq_lock);
	dev->irq_reason = 0;
	reinit_completion(&dev->cmd_done);
	spin_unlock_irq(&dev->irq_lock);

	w4_write(dev, W4_VPU_BUSY_STATUS, 1);
	w4_write(dev, W4_RET_SUCCESS, 0);
	w4_write(dev, W4_CORE_INDEX, 0);
	w4_write(dev, W4_INST_INDEX, inst & 0xffff);	/* codec 0: HEVC */
	w4_write(dev, W4_COMMAND, cmd);
	if (cmd != W4_CMD_INIT_VPU)
		w4_write(dev, W4_VPU_HOST_INT_REQ, 1);
}

static int w4_wait(struct wave4_dev *dev, u32 cmd)
{
	u32 reason;

	if (!wait_for_completion_timeout(&dev->cmd_done,
					 msecs_to_jiffies(WAVE4_CMD_TIMEOUT_MS))) {
		dev_err(dev->dev, "command %#x timed out (pc %#x)\n", cmd,
			w4_read(dev, W4_VCPU_CUR_PC));
		return -ETIMEDOUT;
	}
	spin_lock_irq(&dev->irq_lock);
	reason = dev->irq_reason;
	spin_unlock_irq(&dev->irq_lock);
	if (!(reason & cmd)) {
		dev_err(dev->dev, "command %#x: interrupt reason %#x\n", cmd,
			reason);
		return -EIO;
	}
	return 0;
}

static int w4_cmd(struct wave4_dev *dev, u32 inst, u32 cmd, u32 vint)
{
	int ret;

	w4_write(dev, W4_VPU_VINT_ENABLE, vint);
	w4_issue(dev, inst, cmd);
	ret = w4_wait(dev, cmd);
	if (ret)
		return ret;
	if (!w4_read(dev, W4_RET_SUCCESS)) {
		dev_dbg(dev->dev, "command %#x failed: reason %#x\n", cmd,
			w4_read(dev, W4_RET_FAIL_REASON));
		return -EIO;
	}
	return 0;
}

static int w4_reset(struct wave4_dev *dev)
{
	u32 val;
	int ret;

	/* Let the bus drain first, as wave5 does (stock: 0x7ffff polls). */
	w4_fio_write(dev, W4_GDI_BUS_CTRL, 0x100);
	ret = readl_poll_timeout(dev->base + W4_GDI_BUS_STATUS, val,
				 val == W4_GDI_BUS_IDLE, 0, 10000);
	if (ret) {
		w4_fio_write(dev, W4_GDI_BUS_CTRL, 0);
		dev_err(dev->dev, "bus not idle: %#x\n", val);
		return ret;
	}

	w4_write(dev, W4_VPU_RESET_REQ, W4_RST_BLOCK_ALL);
	ret = readl_poll_timeout(dev->base + W4_VPU_RESET_STATUS, val, !val,
				 0, 10000);
	w4_write(dev, W4_VPU_RESET_REQ, 0);
	w4_fio_write(dev, W4_GDI_BUS_CTRL, 0);
	if (ret)
		dev_err(dev->dev, "reset did not complete: %#x\n", val);
	return ret;
}

int wave4_hw_boot(struct wave4_dev *dev, const void *fw, size_t fw_size)
{
	dma_addr_t base = dev->common_daddr;
	u32 reg, val;
	int ret;

	/* Plain copy: stock copies its bit_code array as-is, 16 bits at a time. */
	memcpy(dev->common_vaddr, fw, fw_size);
	memset(dev->common_vaddr + fw_size, 0, WAVE4_COMMON_SIZE - fw_size);
	wmb();

	ret = w4_reset(dev);
	if (ret)
		return ret;

	w4_write(dev, W4_PO_CONF, 0);
	w4_write(dev, W4_VPU_RESET_REQ, W4_RST_BLOCK_ALL);
	ret = readl_poll_timeout(dev->base + W4_VPU_RESET_STATUS, val, !val,
				 0, 10000);
	w4_write(dev, W4_VPU_RESET_REQ, 0);
	if (ret)
		return ret;

	for (reg = W4_COMMAND; reg < 0x300; reg += 4)
		w4_write(dev, reg, 0);

	w4_write(dev, W4_VPU_REMAP_CTRL, W4_REMAP_CTRL_CODE);
	w4_write(dev, W4_VPU_REMAP_VADDR, 0);
	w4_write(dev, W4_VPU_REMAP_PADDR, base);
	w4_write(dev, W4_ADDR_CODE_BASE, base);
	w4_write(dev, W4_CODE_SIZE, WAVE4_CODE_SIZE);
	w4_write(dev, W4_CODE_PARAM, 0);
	w4_write(dev, W4_TIMEOUT_CNT, 0xffff);
	w4_write(dev, W4_ADDR_STACK_BASE, base + WAVE4_STACK_OFFSET);
	w4_write(dev, W4_STACK_SIZE, WAVE4_STACK_SIZE);
	w4_write(dev, W4_HW_OPTION, 0);
	w4_write(dev, W4_VPU_VINT_ENABLE, W4_VINT_ENABLE_INIT);

	w4_issue(dev, 0, W4_CMD_INIT_VPU);
	w4_write(dev, W4_VPU_REMAP_CORE_START, 1);
	ret = w4_wait(dev, W4_CMD_INIT_VPU);
	if (ret)
		return ret;
	/* Stock wants exactly 1 here, not just non-zero. */
	val = w4_read(dev, W4_RET_SUCCESS);
	if (val != 1) {
		dev_err(dev->dev, "INIT_VPU failed: %#x, reason %#x\n", val,
			w4_read(dev, W4_RET_FAIL_REASON));
		return -EIO;
	}

	w4_write(dev, W4_VPU_VINT_ENABLE, W4_CMD_GET_FW_VERSION);
	w4_issue(dev, 0, W4_CMD_GET_FW_VERSION);
	ret = w4_wait(dev, W4_CMD_GET_FW_VERSION);
	if (ret)
		return ret;
	if (!w4_read(dev, W4_RET_SUCCESS))
		return -EIO;
	dev->fw_version = w4_read(dev, W4_RET_FW_VERSION);
	return 0;
}

/* No command after this one: remove() and shutdown() are on the way. */
void wave4_hw_halt(struct wave4_dev *dev)
{
	mutex_lock(&dev->hw_lock);
	if (!dev->gone)
		w4_reset(dev);
	dev->gone = true;
	mutex_unlock(&dev->hw_lock);
}

/*
 * Reset the block and boot the firmware again, as at probe. The vendor
 * resets on a timeout too, but also cycles the clocks; unverified whether
 * the reset alone frees a wedged VCPU. Every instance is lost with it: the
 * bumped generation fails their later commands before any reaches the
 * firmware.
 */
static void w4_recover(struct wave4_dev *dev)
{
	int ret;

	lockdep_assert_held(&dev->hw_lock);
	if (dev->gone)
		return;
	dev->generation++;
	dev_warn(dev->dev, "restarting the firmware\n");
	ret = wave4_restart(dev);
	if (ret) {
		/*
		 * A failed restart leaves the state unknown: hold the block
		 * in reset and stop issuing it commands, rather than let
		 * every later command time out again against whatever it's
		 * now in.
		 */
		dev_err(dev->dev, "firmware restart failed: %d, giving up on this block\n", ret);
		w4_reset(dev);
		dev->gone = true;
	}
}

void wave4_hw_recover(struct wave4_dev *dev)
{
	mutex_lock(&dev->hw_lock);
	w4_recover(dev);
	mutex_unlock(&dev->hw_lock);
}

/* Takes hw_lock for a command on ctx's instance, unless that is gone. */
static int w4_lock_ctx(struct wave4_ctx *ctx)
{
	struct wave4_dev *dev = ctx->dev;

	mutex_lock(&dev->hw_lock);
	if (dev->gone) {
		mutex_unlock(&dev->hw_lock);
		return -ENODEV;
	}
	if (ctx->generation != dev->generation) {
		mutex_unlock(&dev->hw_lock);
		return -ESTALE;
	}
	return 0;
}

/*
 * A command that timed out may still be running: the VCPU state is unknown,
 * and so is what the next command would do. Restart the firmware.
 */
static int w4_unlock(struct wave4_dev *dev, int ret)
{
	if (ret == -ETIMEDOUT)
		w4_recover(dev);
	mutex_unlock(&dev->hw_lock);
	return ret;
}

/* What stock writes ahead of every INIT_SEQ and DEC_PIC. */
static void w4_set_dec_params(struct wave4_ctx *ctx, bool dec_pic)
{
	struct wave4_dev *dev = ctx->dev;
	dma_addr_t common = dev->common_daddr;
	u32 opt = W4_BS_OPTION_PIC_END;

	if (ctx->eos)
		opt |= W4_BS_OPTION_EOS;

	w4_write(dev, W4_BS_PARAM, W4_ENDIAN_STREAM);
	w4_write(dev, W4_BS_START_ADDR, ctx->bs.daddr);
	w4_write(dev, W4_BS_SIZE, ctx->bs.size);
	w4_write(dev, W4_BS_RD_PTR, ctx->bs_rd);
	w4_write(dev, W4_BS_WR_PTR, ctx->bs_wr);
	w4_write(dev, W4_BS_OPTION, opt);
	/* Secondary AXI off, as stock with SEC_AXI_BUS_DISABLE_SDRAM. */
	w4_write(dev, W4_ADDR_SEC_AXI, 0);
	w4_write(dev, W4_SEC_AXI_SIZE, 0);
	w4_write(dev, W4_USE_SEC_AXI, 0);
	w4_write(dev, W4_ADDR_WORK_BASE, ctx->work.daddr);
	w4_write(dev, W4_WORK_SIZE, ctx->work.size);
	w4_write(dev, W4_WORK_PARAM, 0);
	w4_write(dev, W4_ADDR_TEMP_BASE, common + WAVE4_TEMP_OFFSET);
	w4_write(dev, W4_TEMP_SIZE, WAVE4_TEMP_SIZE);
	w4_write(dev, W4_TEMP_PARAM, 0);
	w4_write(dev, W4_CMD_DEC_USER_MASK, 0);
	w4_write(dev, W4_CMD_DEC_ADDR_USER_BASE, 0);
	w4_write(dev, W4_CMD_DEC_USER_SIZE, 0);
	w4_write(dev, W4_CMD_DEC_USER_PARAM, 0xf);
	if (dec_pic) {
		w4_write(dev, W4_CMD_DEC_ADDR_REPORT, 0);
		w4_write(dev, W4_CMD_DEC_REPORT_SIZE, 0);
		w4_write(dev, W4_CMD_DEC_REPORT_PARAM, 0xc);
	}
	w4_write(dev, W4_CMD_DEC_RESERVED_178, 0);
}

static void w4_set_work(struct wave4_ctx *ctx)
{
	w4_write(ctx->dev, W4_ADDR_WORK_BASE, ctx->work.daddr);
	w4_write(ctx->dev, W4_WORK_SIZE, ctx->work.size);
	w4_write(ctx->dev, W4_WORK_PARAM, 0);
}

int wave4_hw_create_instance(struct wave4_ctx *ctx)
{
	struct wave4_dev *dev = ctx->dev;
	int ret;

	mutex_lock(&dev->hw_lock);
	if (dev->gone) {
		mutex_unlock(&dev->hw_lock);
		return -ENODEV;
	}
	w4_set_work(ctx);
	ret = w4_cmd(dev, ctx->id, W4_CMD_CREATE_INSTANCE,
		     W4_CMD_CREATE_INSTANCE);
	if (!ret)
		ctx->generation = dev->generation;
	return w4_unlock(dev, ret);
}

static void w4_read_seq_info(struct wave4_ctx *ctx)
{
	struct wave4_dev *dev = ctx->dev;
	struct wave4_seq_info *s = &ctx->seq;
	u32 val;

	val = w4_read(dev, W4_RET_DEC_PIC_SIZE);
	s->width = val >> 16;
	s->height = val & 0xffff;
	s->min_fbs = w4_read(dev, W4_RET_DEC_NUM_REQUIRED_FB);

	val = w4_read(dev, W4_RET_DEC_CROP_LEFT_RIGHT);
	s->crop_left = val >> 16;
	s->crop_right = s->width - (val & 0xffff);
	val = w4_read(dev, W4_RET_DEC_CROP_TOP_BOTTOM);
	s->crop_top = val >> 16;
	s->crop_bottom = s->height - (val & 0xffff);

	val = w4_read(dev, W4_RET_DEC_SEQ_PARAM);
	s->level = val & 0xff;
	s->profile = (val >> 24) & 0x1f;
	s->tier = (val >> 29) & 1;

	/* Field order as wave5's W5_RET_DEC_COLOR_SAMPLE_INFO: not confirmed. */
	val = w4_read(dev, W4_RET_DEC_COLOR_SAMPLE_INFO);
	s->luma_depth = val & 0xf;
	s->chroma_depth = (val >> 4) & 0xf;
	s->chroma_format = (val >> 8) & 0xf;
}

int wave4_hw_init_seq(struct wave4_ctx *ctx)
{
	struct wave4_dev *dev = ctx->dev;
	int ret;

	ret = w4_lock_ctx(ctx);
	if (ret)
		return ret;
	w4_set_dec_params(ctx, false);
	w4_write(dev, W4_CMD_DEC_DISP_FLAG, 0);
	w4_write(dev, W4_COMMAND_OPTION, W4_INIT_SEQ_OPTION_NORMAL);
	w4_write(dev, W4_CMD_DEC_SKIP, 0);
	ret = w4_cmd(dev, ctx->id, W4_CMD_INIT_SEQ, W4_CMD_INIT_SEQ);
	if (ret != -ETIMEDOUT)
		ctx->bs_rd = w4_read(dev, W4_BS_RD_PTR);
	if (!ret)
		w4_read_seq_info(ctx);
	else if (ret == -EIO)
		dev_dbg(dev->dev, "INIT_SEQ: reason %#x, error %#x\n",
			w4_read(dev, W4_RET_FAIL_REASON),
			w4_read(dev, W4_RET_DEC_ERR_INFO));
	return w4_unlock(dev, ret);
}

/* One SET_FRAMEBUF per group of eight, START on the first, END on the last. */
static int w4_set_fb_group(struct wave4_ctx *ctx, bool linear,
			   const dma_addr_t *y, const dma_addr_t *cb)
{
	struct wave4_dev *dev = ctx->dev;
	unsigned int n = ctx->num_fbs, groups = DIV_ROUND_UP(n, 8);
	u32 endian = linear ? W4_ENDIAN_LINEAR : W4_ENDIAN_FBC;
	unsigned int g, i;
	int ret;

	for (g = 0; g < groups; g++) {
		unsigned int start = g * 8, cnt = min(n - start, 8U);
		u32 opt = endian << 16;

		if (g == 0)
			opt |= W4_SFB_OPTION_START;
		if (g == groups - 1)
			opt |= W4_SFB_OPTION_END;
		w4_write(dev, W4_COMMAND_OPTION, opt);
		w4_write(dev, W4_SET_FB_NUM, (start << 8) | (start + cnt - 1));

		for (i = 0; i < cnt; i++) {
			unsigned int k = start + i;
			u32 off = i * 16;

			w4_write(dev, W4_ADDR_LUMA_BASE0 + off, y[k]);
			w4_write(dev, W4_ADDR_CB_BASE0 + off, cb[k]);
			if (linear) {
				/* NV12: no separate Cr plane. */
				w4_write(dev, W4_ADDR_CR_BASE0 + off, 0);
				w4_write(dev, W4_ADDR_FBC_C_OFFSET0 + off, 0);
				w4_write(dev, W4_ADDR_MV_COL0 + i * 4, 0);
			} else {
				w4_write(dev, W4_ADDR_FBC_Y_OFFSET0 + off,
					 ctx->fbc_y_tbl[k].daddr);
				w4_write(dev, W4_ADDR_FBC_C_OFFSET0 + off,
					 ctx->fbc_c_tbl[k].daddr);
				w4_write(dev, W4_ADDR_MV_COL0 + i * 4,
					 ctx->mv_col[k].daddr);
			}
		}

		w4_set_work(ctx);
		w4_write(dev, W4_VPU_VINT_ENABLE, W4_CMD_SET_FRAMEBUF);
		w4_issue(dev, ctx->id, W4_CMD_SET_FRAMEBUF);
		ret = w4_wait(dev, W4_CMD_SET_FRAMEBUF);
		if (ret)
			return ret;
	}
	return w4_read(dev, W4_RET_SUCCESS) ? 0 : -EIO;
}

/*
 * Stock with write-to-linear on registers the linear set first, then the
 * compressed one (ProductVpuRegisterFramebuffer), the same count of each:
 * linear frame n is the display copy of compressed frame n.
 */
int wave4_hw_set_framebufs(struct wave4_ctx *ctx, const dma_addr_t *linear_y,
			   const dma_addr_t *linear_cb)
{
	struct wave4_dev *dev = ctx->dev;
	struct wave4_seq_info *s = &ctx->seq;
	dma_addr_t fbc_y[WAVE4_MAX_FBS], fbc_cb[WAVE4_MAX_FBS];
	u32 luma = wave4_fbc_luma_size(s->width, s->height);
	unsigned int i;
	int ret;

	for (i = 0; i < ctx->num_fbs; i++) {
		fbc_y[i] = ctx->fbc[i].daddr;
		fbc_cb[i] = ctx->fbc[i].daddr + luma;
	}

	ret = w4_lock_ctx(ctx);
	if (ret)
		return ret;
	w4_write(dev, W4_PIC_SIZE, (s->width << 16) | s->height);
	w4_write(dev, W4_COMMON_PIC_INFO,
		 W4_PIC_INFO_LINEAR | W4_PIC_INFO_PIXEL_ORDER |
		 W4_PIC_INFO_CBCR_INTERLEAVE | ctx->dst_fmt.bytesperline);
	ret = w4_set_fb_group(ctx, true, linear_y, linear_cb);
	if (ret)
		goto out;

	w4_write(dev, W4_PIC_SIZE, (s->width << 16) | s->height);
	w4_write(dev, W4_COMMON_PIC_INFO, wave4_fbc_stride(s->width));
	ret = w4_set_fb_group(ctx, false, fbc_y, fbc_cb);
out:
	return w4_unlock(dev, ret);
}

int wave4_hw_decode(struct wave4_ctx *ctx, struct wave4_dec_result *res)
{
	struct wave4_dev *dev = ctx->dev;
	u32 val, flag;
	int ret;

	ret = w4_lock_ctx(ctx);
	if (ret)
		return ret;
	w4_set_dec_params(ctx, true);
	flag = ctx->disp_flag & ~(u32)xchg(&ctx->disp_clr, 0);
	/* Frames released stay so unless a successful DEC_PIC returns flags. */
	ctx->disp_flag = flag;
	w4_write(dev, W4_CMD_DEC_DISP_FLAG, flag);
	w4_write(dev, W4_CMD_DEC_TEMPORAL_ID_PLUS1, W4_TEMPORAL_ID_ALL);
	w4_write(dev, W4_CMD_SEQ_CHANGE_ENABLE_FLAG,
		 W4_SEQ_CHANGE_ENABLE_ALL_HEVC);
	w4_write(dev, W4_CMD_DEC_SKIP, 0);
	w4_write(dev, W4_COMMAND_OPTION, W4_DEC_PIC_OPTION_NORMAL);
	w4_write(dev, W4_VPU_VINT_ENABLE, W4_VINT_ENABLE_DEC_PIC);
	w4_issue(dev, ctx->id, W4_CMD_DEC_PIC);
	ret = w4_wait(dev, W4_CMD_DEC_PIC);
	if (ret)
		goto out;

	res->success = w4_read(dev, W4_RET_SUCCESS);
	if (!res->success) {
		/* Unverified which results are valid on failure: read none. */
		res->fail_reason = w4_read(dev, W4_RET_FAIL_REASON);
		res->err_info = w4_read(dev, W4_RET_DEC_ERR_INFO);
		goto out;
	}
	res->pic_type = w4_read(dev, W4_RET_DEC_PIC_TYPE);
	res->seq_changed = w4_read(dev, W4_RET_DEC_NOTIFICATION) & 0x7fffffff;
	/* With linear frames registered the upper half is the linear index. */
	val = w4_read(dev, W4_RET_DEC_DISPLAY_INDEX);
	res->disp_idx = (s16)(val >> 16);
	val = w4_read(dev, W4_RET_DEC_DECODED_INDEX);
	res->dec_idx = (s16)(val >> 16);
	ctx->bs_rd = w4_read(dev, W4_BS_RD_PTR);
	ctx->disp_flag = w4_read(dev, W4_CMD_DEC_DISP_FLAG);
out:
	return w4_unlock(dev, ret);
}

int wave4_hw_fini_seq(struct wave4_ctx *ctx)
{
	struct wave4_dev *dev = ctx->dev;
	int ret;

	ret = w4_lock_ctx(ctx);
	if (ret)
		return ret;
	w4_set_work(ctx);
	ret = w4_cmd(dev, ctx->id, W4_CMD_FINI_SEQ, W4_CMD_FINI_SEQ);
	return w4_unlock(dev, ret);
}
