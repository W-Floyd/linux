/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Chips&Media Wave410 HEVC decoder
 */
#ifndef __WAVE4_H__
#define __WAVE4_H__

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/idr.h>
#include <linux/mutex.h>
#include <linux/reset.h>
#include <linux/sizes.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-mem2mem.h>

/*
 * The common area, laid out as the vendor's: code (the VCPU sees it at 0
 * through remap region 0), stack, temp buffer, secondary AXI buffer.
 */
#define WAVE4_CODE_SIZE		SZ_1M
#define WAVE4_STACK_OFFSET	WAVE4_CODE_SIZE
#define WAVE4_STACK_SIZE	SZ_8K
#define WAVE4_TEMP_OFFSET	(WAVE4_STACK_OFFSET + WAVE4_STACK_SIZE)
#define WAVE4_TEMP_SIZE		SZ_1M
#define WAVE4_SEC_AXI_OFFSET	(WAVE4_TEMP_OFFSET + WAVE4_TEMP_SIZE)
#define WAVE4_SEC_AXI_SIZE	SZ_128K
#define WAVE4_COMMON_SIZE	(WAVE4_SEC_AXI_OFFSET + WAVE4_SEC_AXI_SIZE)

#define WAVE4_WORK_SIZE		(3 * SZ_1M)	/* per instance */
#define WAVE4_MAX_INSTANCES	4
#define WAVE4_MAX_FBS		31	/* DISP_FLAG is a 32-bit mask */
#define WAVE4_TS_QUEUE		4

#define WAVE4_MIN_WIDTH		8
#define WAVE4_MIN_HEIGHT	8
#define WAVE4_MAX_WIDTH		4096
#define WAVE4_MAX_HEIGHT	2176

#define WAVE4_CMD_TIMEOUT_MS	500
/* Consecutive failed DEC_PICs on one instance before the firmware restarts. */
#define WAVE4_MAX_DEC_FAILS	3

struct firmware;

/*
 * Freed from the v4l2_device release callback, not by devm: an open file
 * can outlive remove(). Register access and devm memory are gone by then;
 * the 'gone' flag keeps the remaining file operations off the hardware.
 */
struct wave4_dev {
	struct device *dev;
	void __iomem *base;
	int irq;
	struct clk_bulk_data *clks;
	int num_clks;
	struct reset_control *rst;
	const struct firmware *fw;	/* kept for a restart */

	struct v4l2_device v4l2_dev;
	struct video_device vfd;
	struct v4l2_m2m_dev *m2m_dev;
	struct workqueue_struct *wq;

	/* One command at a time: the Wave410 has no command queue. */
	struct mutex hw_lock;
	struct completion cmd_done;
	spinlock_t irq_lock;	/* irq_reason, against the handler */
	u32 irq_reason;
	/* Under hw_lock: bumped by each firmware restart, see ctx->generation. */
	u32 generation;
	bool gone;		/* under hw_lock: halted for remove or shutdown */

	/* Serializes the vb2 queues and file operations. */
	struct mutex dev_lock;
	struct ida inst_ida;

	void *common_vaddr;
	dma_addr_t common_daddr;
	u32 fw_version;
};

struct wave4_seq_info {
	u32 width;
	u32 height;
	u32 min_fbs;
	u32 crop_left, crop_top, crop_right, crop_bottom;
	u32 profile, level, tier;
	u32 luma_depth, chroma_depth, chroma_format;
};

struct wave4_dpb_buf {
	void *vaddr;
	dma_addr_t daddr;
	size_t size;
	bool pages;	/* from the page allocator, streaming-mapped */
};

enum wave4_state {
	WAVE4_STATE_NEED_SEQ,	/* no sequence header parsed yet */
	WAVE4_STATE_SEQ_READY,	/* INIT_SEQ done, waiting for CAPTURE */
	WAVE4_STATE_DECODING,	/* frame buffers registered */
	WAVE4_STATE_ERROR,
};

struct wave4_ctx {
	struct v4l2_fh fh;
	struct wave4_dev *dev;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *min_bufs_ctrl;
	struct work_struct run_work;

	int id;
	/* dev->generation at CREATE_INSTANCE; a restart since loses the instance. */
	u32 generation;
	enum wave4_state state;
	bool abort;		/* job_abort() came before the worker ran the job */
	bool closing;		/* release() started: no more jobs */
	unsigned int dec_fails;	/* consecutive failed DEC_PICs */

	struct v4l2_pix_format src_fmt;
	struct v4l2_pix_format dst_fmt;
	struct wave4_seq_info seq;

	struct wave4_dpb_buf work;
	struct wave4_dpb_buf bs;	/* two halves, see wave4_feed() */
	u32 bs_half;
	bool bs_upper;		/* the next buffer goes to the upper half */
	u32 bs_rd, bs_wr;
	bool bs_pending;	/* bitstream left over from the last job */
	struct vb2_v4l2_buffer *src_held;	/* OUTPUT buffer of that bitstream */
	bool eos;		/* stream end flagged to the firmware */
	bool need_fb;		/* last DEC_PIC found no free frame buffer */

	unsigned int num_fbs;
	struct wave4_dpb_buf fbc[WAVE4_MAX_FBS];
	struct wave4_dpb_buf fbc_y_tbl[WAVE4_MAX_FBS];
	struct wave4_dpb_buf fbc_c_tbl[WAVE4_MAX_FBS];
	struct wave4_dpb_buf mv_col[WAVE4_MAX_FBS];
	u64 ts[WAVE4_MAX_FBS];	/* by decoded frame, for reordered output */
	/* OUTPUT buffers fed whose picture the firmware has not reported yet */
	u64 ts_q[WAVE4_TS_QUEUE];
	unsigned int ts_head, ts_count;
	/*
	 * DISP_FLAG as the firmware last returned it: a frame's bit is set from
	 * its decode until the host releases it, so a frame decoded but not
	 * shown yet is protected too. Only the worker touches it.
	 */
	u32 disp_flag;
	unsigned long disp_clr;	/* CAPTURE buffers queued since the last DEC_PIC */
};

static inline struct wave4_ctx *fh_to_ctx(struct v4l2_fh *fh)
{
	return container_of(fh, struct wave4_ctx, fh);
}

static inline struct wave4_ctx *file_to_ctx(struct file *filp)
{
	return fh_to_ctx(file_to_v4l2_fh(filp));
}

struct wave4_dec_result {
	int disp_idx;
	int dec_idx;
	u32 pic_type;
	u32 seq_changed;
	bool success;
	u32 fail_reason;
	u32 err_info;
};

/* wave4-drv.c */
int wave4_restart(struct wave4_dev *dev);

/* wave4-hw.c */
int wave4_hw_boot(struct wave4_dev *dev, const void *fw, size_t fw_size);
void wave4_hw_halt(struct wave4_dev *dev);
void wave4_hw_recover(struct wave4_dev *dev);
irqreturn_t wave4_irq(int irq, void *data);
int wave4_hw_create_instance(struct wave4_ctx *ctx);
int wave4_hw_init_seq(struct wave4_ctx *ctx);
int wave4_hw_set_framebufs(struct wave4_ctx *ctx, const dma_addr_t *linear_y,
			   const dma_addr_t *linear_cb);
int wave4_hw_decode(struct wave4_ctx *ctx, struct wave4_dec_result *res);
int wave4_hw_fini_seq(struct wave4_ctx *ctx);

/* Frame buffer geometry, as the vendor's CalcStride/CalcLumaSize/... */
static inline u32 wave4_fbc_stride(u32 width)
{
	return ALIGN(width, 32);
}

static inline u32 wave4_fbc_luma_size(u32 width, u32 height)
{
	return wave4_fbc_stride(width) * ALIGN(height, 16);
}

static inline u32 wave4_fbc_chroma_size(u32 width, u32 height)
{
	return ALIGN(height, 16) * ALIGN(wave4_fbc_stride(width) / 2, 16) / 2;
}

static inline u32 wave4_fbc_y_tbl_size(u32 width, u32 height)
{
	return DIV_ROUND_UP(width, 256) * DIV_ROUND_UP(height, 16) * 128;
}

static inline u32 wave4_fbc_c_tbl_size(u32 width, u32 height)
{
	return DIV_ROUND_UP(width / 2, 256) * DIV_ROUND_UP(height, 16) * 128;
}

static inline u32 wave4_mv_col_size(u32 width, u32 height)
{
	return (DIV_ROUND_UP(width, 64) * DIV_ROUND_UP(height, 64) * 4 + 1) * 64;
}

/* wave4-dec.c */
int wave4_dec_register(struct wave4_dev *dev);
void wave4_dec_unregister(struct wave4_dev *dev);

#endif
