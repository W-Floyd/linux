// SPDX-License-Identifier: GPL-2.0-only
/*
 * Chips&Media Wave410 HEVC decoder: V4L2 stateful (memory-to-memory) decoder
 *
 * The firmware decodes into its own compressed (FBC) frames and writes a
 * linear NV12 copy of each displayed frame into the frame registered at the
 * same index ("write to linear"). The CAPTURE buffers are those linear frames,
 * so their count is fixed at STREAMON: every CAPTURE buffer pairs with one
 * driver-owned FBC frame. The firmware marks a frame in DISP_FLAG when it
 * decodes it; the driver hands the flags back unchanged except for the
 * frames user space has queued again since, which it clears.
 *
 * Bitstream goes in "picture end" mode: each OUTPUT buffer is copied to one
 * half of a linear bitstream buffer and decoded up to its end; nothing wraps.
 */
#include <linux/dma-mapping.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#include "wave4.h"

#define WAVE4_NAME		"wave4-dec"
#define WAVE4_DEF_WIDTH		1920
#define WAVE4_DEF_HEIGHT	1088
#define WAVE4_MIN_BS_SIZE	SZ_1M
#define WAVE4_MAX_BS_SIZE	SZ_16M

/*
 * Lifecycle: OUTPUT data runs INIT_SEQ (NEED_SEQ -> SEQ_READY, SOURCE_CHANGE
 * event); CAPTURE STREAMON registers frames (-> DECODING); STOP drains with
 * the stream-end flag until the firmware reports no more frames to show.
 */

static void wave4_dpb_free(struct wave4_dev *dev, struct wave4_dpb_buf *b)
{
	if (!b->size)
		return;
	dma_free_attrs(dev->dev, b->size, b->vaddr, b->daddr,
		       DMA_ATTR_NO_KERNEL_MAPPING);
	memset(b, 0, sizeof(*b));
}

static int wave4_dpb_alloc(struct wave4_dev *dev, struct wave4_dpb_buf *b,
			   size_t size)
{
	b->size = PAGE_ALIGN(size);
	b->vaddr = dma_alloc_attrs(dev->dev, b->size, &b->daddr, GFP_KERNEL,
				   DMA_ATTR_NO_KERNEL_MAPPING);
	if (!b->vaddr) {
		b->size = 0;
		return -ENOMEM;
	}
	return 0;
}

static void wave4_coherent_free(struct wave4_dev *dev, struct wave4_dpb_buf *b)
{
	if (!b->size)
		return;
	dma_free_coherent(dev->dev, b->size, b->vaddr, b->daddr);
	memset(b, 0, sizeof(*b));
}

static int wave4_coherent_alloc(struct wave4_dev *dev, struct wave4_dpb_buf *b,
				size_t size)
{
	b->size = PAGE_ALIGN(size);
	b->vaddr = dma_alloc_coherent(dev->dev, b->size, &b->daddr, GFP_KERNEL);
	if (!b->vaddr) {
		b->size = 0;
		return -ENOMEM;
	}
	return 0;
}

static void wave4_free_fbs(struct wave4_ctx *ctx)
{
	unsigned int i;

	for (i = 0; i < WAVE4_MAX_FBS; i++) {
		wave4_dpb_free(ctx->dev, &ctx->fbc[i]);
		wave4_dpb_free(ctx->dev, &ctx->fbc_y_tbl[i]);
		wave4_dpb_free(ctx->dev, &ctx->fbc_c_tbl[i]);
		wave4_dpb_free(ctx->dev, &ctx->mv_col[i]);
	}
	ctx->num_fbs = 0;
}

static int wave4_alloc_fbs(struct wave4_ctx *ctx, unsigned int n)
{
	u32 w = ctx->seq.width, h = ctx->seq.height;
	size_t fb = wave4_fbc_luma_size(w, h) + 2 * wave4_fbc_chroma_size(w, h);
	unsigned int i;
	int ret = 0;

	for (i = 0; i < n && !ret; i++) {
		ret = wave4_dpb_alloc(ctx->dev, &ctx->fbc[i], fb);
		ret = ret ?: wave4_dpb_alloc(ctx->dev, &ctx->fbc_y_tbl[i],
					     wave4_fbc_y_tbl_size(w, h));
		ret = ret ?: wave4_dpb_alloc(ctx->dev, &ctx->fbc_c_tbl[i],
					     wave4_fbc_c_tbl_size(w, h));
		ret = ret ?: wave4_dpb_alloc(ctx->dev, &ctx->mv_col[i],
					     wave4_mv_col_size(w, h));
	}
	if (ret) {
		wave4_free_fbs(ctx);
		return ret;
	}
	ctx->num_fbs = n;
	return 0;
}

static void wave4_set_src_fmt(struct v4l2_pix_format *f)
{
	f->pixelformat = V4L2_PIX_FMT_HEVC;
	f->field = V4L2_FIELD_NONE;
	f->width = clamp_t(u32, f->width, WAVE4_MIN_WIDTH, WAVE4_MAX_WIDTH);
	f->height = clamp_t(u32, f->height, WAVE4_MIN_HEIGHT, WAVE4_MAX_HEIGHT);
	f->bytesperline = 0;
	f->sizeimage = clamp_t(u32, max_t(u32, f->sizeimage,
					   f->width * f->height * 3 / 4),
			       WAVE4_MIN_BS_SIZE, WAVE4_MAX_BS_SIZE);
	f->sizeimage = PAGE_ALIGN(f->sizeimage);
	f->flags = 0;
}

/* NV12 in one plane; CbCr follows the 16-line aligned luma, as stock lays it. */
static void wave4_set_dst_fmt(struct v4l2_pix_format *f, u32 width, u32 height)
{
	f->pixelformat = V4L2_PIX_FMT_NV12;
	f->field = V4L2_FIELD_NONE;
	f->width = width;
	f->height = ALIGN(height, 16);
	f->bytesperline = ALIGN(width, 32);
	f->sizeimage = f->bytesperline * f->height * 3 / 2;
	f->colorspace = V4L2_COLORSPACE_REC709;
	f->flags = 0;
}

static int wave4_querycap(struct file *file, void *priv,
			  struct v4l2_capability *cap)
{
	strscpy(cap->driver, WAVE4_NAME, sizeof(cap->driver));
	strscpy(cap->card, "Chips&Media Wave410 HEVC decoder", sizeof(cap->card));
	return 0;
}

static int wave4_enum_fmt_out(struct file *file, void *priv,
			      struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_HEVC;
	f->flags = V4L2_FMT_FLAG_COMPRESSED | V4L2_FMT_FLAG_DYN_RESOLUTION;
	return 0;
}

static int wave4_enum_fmt_cap(struct file *file, void *priv,
			      struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_NV12;
	return 0;
}

static int wave4_enum_framesizes(struct file *file, void *priv,
				 struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index || (fsize->pixel_format != V4L2_PIX_FMT_HEVC &&
			     fsize->pixel_format != V4L2_PIX_FMT_NV12))
		return -EINVAL;

	/* The conformance window crops the coded size to any size at all. */
	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	fsize->stepwise.min_width = WAVE4_MIN_WIDTH;
	fsize->stepwise.max_width = WAVE4_MAX_WIDTH;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.min_height = WAVE4_MIN_HEIGHT;
	fsize->stepwise.max_height = WAVE4_MAX_HEIGHT;
	fsize->stepwise.step_height = 1;
	return 0;
}

static int wave4_g_fmt_out(struct file *file, void *priv, struct v4l2_format *f)
{
	f->fmt.pix = file_to_ctx(file)->src_fmt;
	return 0;
}

static int wave4_try_fmt_out(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	wave4_set_src_fmt(&f->fmt.pix);
	return 0;
}

static int wave4_s_fmt_out(struct file *file, void *priv, struct v4l2_format *f)
{
	struct wave4_ctx *ctx = file_to_ctx(file);
	struct vb2_queue *vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);

	if (vb2_is_busy(vq))
		return -EBUSY;

	wave4_set_src_fmt(&f->fmt.pix);
	ctx->src_fmt = f->fmt.pix;
	/* Until a header says otherwise, CAPTURE follows the coded size. */
	if (ctx->state == WAVE4_STATE_NEED_SEQ)
		wave4_set_dst_fmt(&ctx->dst_fmt, f->fmt.pix.width,
				  f->fmt.pix.height);
	return 0;
}

/* The decoder picks the CAPTURE format; setting it only reads it back. */
static int wave4_g_fmt_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	f->fmt.pix = file_to_ctx(file)->dst_fmt;
	return 0;
}

static int wave4_selection(struct file *file, void *priv,
			   struct v4l2_selection *s)
{
	struct wave4_ctx *ctx = file_to_ctx(file);
	struct wave4_seq_info *seq = &ctx->seq;

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_PADDED:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = ctx->dst_fmt.width;
		s->r.height = ctx->dst_fmt.height;
		return 0;
	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
		if (ctx->state == WAVE4_STATE_NEED_SEQ) {
			s->r.left = 0;
			s->r.top = 0;
			s->r.width = ctx->dst_fmt.width;
			s->r.height = ctx->dst_fmt.height;
		} else {
			s->r.left = seq->crop_left;
			s->r.top = seq->crop_top;
			s->r.width = seq->crop_right - seq->crop_left;
			s->r.height = seq->crop_bottom - seq->crop_top;
		}
		return 0;
	default:
		return -EINVAL;
	}
}

static int wave4_decoder_cmd(struct file *file, void *priv,
			     struct v4l2_decoder_cmd *dc)
{
	struct wave4_ctx *ctx = file_to_ctx(file);
	struct v4l2_m2m_ctx *m2m = ctx->fh.m2m_ctx;
	int ret;

	ret = v4l2_m2m_ioctl_try_decoder_cmd(file, priv, dc);
	if (ret)
		return ret;

	if (dc->cmd == V4L2_DEC_CMD_START) {
		ctx->eos = false;
		return v4l2_m2m_ioctl_decoder_cmd(file, priv, dc);
	}

	/* Without frames nothing is held back: the generic STOP will do. */
	if (ctx->state == WAVE4_STATE_NEED_SEQ)
		return v4l2_m2m_ioctl_decoder_cmd(file, priv, dc);

	if (m2m->is_draining)
		return -EBUSY;
	if (m2m->has_stopped)
		return 0;
	m2m->is_draining = true;
	v4l2_m2m_try_schedule(m2m);
	return 0;
}

static int wave4_subscribe_event(struct v4l2_fh *fh,
				 const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	default:
		return v4l2_ctrl_subscribe_event(fh, sub);
	}
}

static const struct v4l2_ioctl_ops wave4_ioctl_ops = {
	.vidioc_querycap = wave4_querycap,
	.vidioc_enum_framesizes = wave4_enum_framesizes,

	.vidioc_enum_fmt_vid_out = wave4_enum_fmt_out,
	.vidioc_g_fmt_vid_out = wave4_g_fmt_out,
	.vidioc_try_fmt_vid_out = wave4_try_fmt_out,
	.vidioc_s_fmt_vid_out = wave4_s_fmt_out,

	.vidioc_enum_fmt_vid_cap = wave4_enum_fmt_cap,
	.vidioc_g_fmt_vid_cap = wave4_g_fmt_cap,
	.vidioc_try_fmt_vid_cap = wave4_g_fmt_cap,
	.vidioc_s_fmt_vid_cap = wave4_g_fmt_cap,

	.vidioc_g_selection = wave4_selection,

	/* No CREATE_BUFS: the frames are registered once, at STREAMON. */
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,

	.vidioc_try_decoder_cmd = v4l2_m2m_ioctl_try_decoder_cmd,
	.vidioc_decoder_cmd = wave4_decoder_cmd,

	.vidioc_subscribe_event = wave4_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static int wave4_queue_setup(struct vb2_queue *q, unsigned int *nbufs,
			     unsigned int *nplanes, unsigned int sizes[],
			     struct device *alloc_devs[])
{
	struct wave4_ctx *ctx = vb2_get_drv_priv(q);
	u32 size;

	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		size = ctx->src_fmt.sizeimage;
	} else {
		size = ctx->dst_fmt.sizeimage;
		if (ctx->state != WAVE4_STATE_NEED_SEQ)
			*nbufs = clamp_t(unsigned int, *nbufs,
					 ctx->seq.min_fbs, WAVE4_MAX_FBS);
		*nbufs = min_t(unsigned int, *nbufs, WAVE4_MAX_FBS);
	}

	if (*nplanes)
		return sizes[0] < size ? -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = size;
	return 0;
}

static int wave4_buf_prepare(struct vb2_buffer *vb)
{
	struct wave4_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	if (V4L2_TYPE_IS_CAPTURE(vb->type)) {
		if (vb2_plane_size(vb, 0) < ctx->dst_fmt.sizeimage)
			return -EINVAL;
		/* The firmware only knows the frames registered at STREAMON. */
		if (ctx->state == WAVE4_STATE_DECODING && vb->index >= ctx->num_fbs)
			return -EINVAL;
	}
	return 0;
}

static void wave4_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct wave4_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_m2m_ctx *m2m = ctx->fh.m2m_ctx;

	if (V4L2_TYPE_IS_CAPTURE(vb->type)) {
		if (v4l2_m2m_dst_buf_is_last(m2m)) {
			vbuf->field = V4L2_FIELD_NONE;
			vb2_set_plane_payload(vb, 0, 0);
			v4l2_m2m_last_buffer_done(m2m, vbuf);
			return;
		}
		set_bit(vb->index, &ctx->disp_clr);
		ctx->need_fb = false;
	}
	v4l2_m2m_buf_queue(m2m, vbuf);
}

static void wave4_return_bufs(struct wave4_ctx *ctx, bool output,
			      enum vb2_buffer_state state)
{
	struct v4l2_m2m_ctx *m2m = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *vb;

	while ((vb = output ? v4l2_m2m_src_buf_remove(m2m) :
			      v4l2_m2m_dst_buf_remove(m2m)))
		v4l2_m2m_buf_done(vb, state);
}

static int wave4_start_output(struct wave4_ctx *ctx)
{
	struct wave4_dev *dev = ctx->dev;
	int ret;

	ctx->bs_upper = false;
	if (ctx->bs_half >= ctx->src_fmt.sizeimage)
		return 0;
	wave4_coherent_free(dev, &ctx->bs);
	ret = wave4_coherent_alloc(dev, &ctx->bs, 2 * ctx->src_fmt.sizeimage);
	ctx->bs_half = ret ? 0 : ctx->src_fmt.sizeimage;
	return ret;
}

static int wave4_start_capture(struct wave4_ctx *ctx, struct vb2_queue *q)
{
	unsigned int n = vb2_get_num_buffers(q), i;
	dma_addr_t y[WAVE4_MAX_FBS], cb[WAVE4_MAX_FBS];
	struct vb2_v4l2_buffer *vbuf;
	struct v4l2_m2m_buffer *b;
	u32 cb_off = ctx->dst_fmt.bytesperline * ctx->dst_fmt.height;
	int ret;

	if (ctx->state != WAVE4_STATE_SEQ_READY)
		return 0;
	if (n < ctx->seq.min_fbs || n > WAVE4_MAX_FBS)
		return -EINVAL;

	for (i = 0; i < n; i++) {
		struct vb2_buffer *vb = vb2_get_buffer(q, i);

		if (!vb)
			return -EINVAL;
		y[i] = vb2_dma_contig_plane_dma_addr(vb, 0);
		cb[i] = y[i] + cb_off;
	}

	ret = wave4_alloc_fbs(ctx, n);
	if (ret)
		return ret;

	ret = wave4_hw_set_framebufs(ctx, y, cb);
	if (ret) {
		wave4_free_fbs(ctx);
		return ret;
	}

	/* Hold every frame user space has not queued yet. */
	ctx->disp_flag = GENMASK(n - 1, 0);
	ctx->disp_clr = 0;
	v4l2_m2m_for_each_dst_buf(ctx->fh.m2m_ctx, b) {
		vbuf = &b->vb;
		ctx->disp_flag &= ~BIT(vbuf->vb2_buf.index);
	}
	ctx->need_fb = false;
	ctx->state = WAVE4_STATE_DECODING;
	return 0;
}

static int wave4_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct wave4_ctx *ctx = vb2_get_drv_priv(q);
	int ret;

	v4l2_m2m_update_start_streaming_state(ctx->fh.m2m_ctx, q);

	ret = V4L2_TYPE_IS_OUTPUT(q->type) ? wave4_start_output(ctx) :
					     wave4_start_capture(ctx, q);
	if (ret)
		wave4_return_bufs(ctx, V4L2_TYPE_IS_OUTPUT(q->type),
				  VB2_BUF_STATE_QUEUED);
	return ret;
}

/* Return the OUTPUT buffer whose bitstream was being decoded (wave4_feed()). */
static void wave4_release_src(struct wave4_ctx *ctx, enum vb2_buffer_state state)
{
	if (ctx->src_held)
		v4l2_m2m_buf_done(ctx->src_held, state);
	ctx->src_held = NULL;
}

static void wave4_stop_streaming(struct vb2_queue *q)
{
	struct wave4_ctx *ctx = vb2_get_drv_priv(q);
	struct v4l2_m2m_ctx *m2m = ctx->fh.m2m_ctx;

	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		wave4_return_bufs(ctx, true, VB2_BUF_STATE_ERROR);
		wave4_release_src(ctx, VB2_BUF_STATE_ERROR);
		/* A seek: drop the leftover; the firmware resyncs on an IRAP. */
		ctx->bs_pending = false;
	} else {
		wave4_return_bufs(ctx, false, VB2_BUF_STATE_ERROR);
		if (ctx->state != WAVE4_STATE_NEED_SEQ) {
			/* Start over from the next sequence header. */
			wave4_hw_fini_seq(ctx);
			wave4_free_fbs(ctx);
			wave4_hw_create_instance(ctx);
			ctx->state = WAVE4_STATE_NEED_SEQ;
		}
		ctx->eos = false;
		ctx->ts_count = 0;
	}

	v4l2_m2m_update_stop_streaming_state(m2m, q);
}

static const struct vb2_ops wave4_qops = {
	.queue_setup = wave4_queue_setup,
	.buf_prepare = wave4_buf_prepare,
	.buf_queue = wave4_buf_queue,
	.start_streaming = wave4_start_streaming,
	.stop_streaming = wave4_stop_streaming,
};

static int wave4_queue_init(void *priv, struct vb2_queue *src,
			    struct vb2_queue *dst)
{
	struct wave4_ctx *ctx = priv;
	int ret;

	src->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src->io_modes = VB2_MMAP | VB2_DMABUF;
	src->drv_priv = ctx;
	src->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src->ops = &wave4_qops;
	/* The bitstream is copied by the CPU: it needs a kernel mapping. */
	src->mem_ops = &vb2_dma_contig_memops;
	src->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src->lock = &ctx->dev->dev_lock;
	src->dev = ctx->dev->dev;
	ret = vb2_queue_init(src);
	if (ret)
		return ret;

	dst->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst->io_modes = VB2_MMAP | VB2_DMABUF;
	dst->drv_priv = ctx;
	dst->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst->ops = &wave4_qops;
	dst->mem_ops = &vb2_dma_contig_memops;
	dst->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst->lock = &ctx->dev->dev_lock;
	dst->dev = ctx->dev->dev;
	return vb2_queue_init(dst);
}

/*
 * A DEC_PIC reports the picture of the buffer fed before it (see
 * wave4_feed()), so timestamps wait in a queue until their picture shows up.
 */
static void wave4_ts_push(struct wave4_ctx *ctx, u64 ts)
{
	if (ctx->ts_count == WAVE4_TS_QUEUE) {
		ctx->ts_head = (ctx->ts_head + 1) % WAVE4_TS_QUEUE;
		ctx->ts_count--;
	}
	ctx->ts_q[(ctx->ts_head + ctx->ts_count++) % WAVE4_TS_QUEUE] = ts;
}

static u64 wave4_ts_pop(struct wave4_ctx *ctx)
{
	u64 ts;

	if (!ctx->ts_count)
		return 0;
	ts = ctx->ts_q[ctx->ts_head];
	ctx->ts_head = (ctx->ts_head + 1) % WAVE4_TS_QUEUE;
	ctx->ts_count--;
	return ts;
}

/*
 * Copy the next OUTPUT buffer into the bitstream buffer. The buffer itself
 * is held until its data is consumed: with OUTPUT drained and CAPTURE not
 * streaming yet, poll() reports an error, and GStreamer, which waits for
 * the source change in poll() after dequeuing its header buffer, gives up.
 *
 * The firmware finishes a picture only when the next DEC_PIC shows it the
 * start of the following one (or the stream end), and it still reads the
 * previous buffer's data then. So consecutive buffers go to alternate
 * halves: the one before stays intact while the next is decoded.
 */
static bool wave4_feed(struct wave4_ctx *ctx)
{
	struct v4l2_m2m_ctx *m2m = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *src = v4l2_m2m_src_buf_remove(m2m);
	u32 off = ctx->bs_upper ? ctx->bs_half : 0;
	size_t len;
	void *va;

	if (!src)
		return false;
	wave4_release_src(ctx, VB2_BUF_STATE_DONE);

	len =vb2_get_plane_payload(&src->vb2_buf, 0);
	va = vb2_plane_vaddr(&src->vb2_buf, 0);
	if (!va || !len || len > ctx->bs_half) {
		v4l2_m2m_buf_done(src, VB2_BUF_STATE_ERROR);
		return false;
	}

	memcpy(ctx->bs.vaddr + off, va, len);
	ctx->bs_upper = !ctx->bs_upper;
	ctx->bs_rd = ctx->bs.daddr + off;
	ctx->bs_wr = ctx->bs_rd + len;
	ctx->bs_pending = true;
	wave4_ts_push(ctx, src->vb2_buf.timestamp);
	ctx->src_held = src;
	return true;
}

static void wave4_finish_drain(struct wave4_ctx *ctx);

static void wave4_run_init_seq(struct wave4_ctx *ctx)
{
	struct v4l2_m2m_ctx *m2m = ctx->fh.m2m_ctx;
	struct wave4_seq_info *seq = &ctx->seq;
	int ret;

	if (!ctx->bs_pending && !wave4_feed(ctx))
		return;

	ret = wave4_hw_init_seq(ctx);
	if (ret) {
		/* No usable header here: drop it and wait for the next buffer. */
		ctx->bs_pending = false;
		ctx->ts_count = 0;
		wave4_release_src(ctx, VB2_BUF_STATE_DONE);
		if (m2m->is_draining && !v4l2_m2m_num_src_bufs_ready(m2m))
			wave4_finish_drain(ctx);
		return;
	}
	ctx->bs_pending = ctx->bs_rd < ctx->bs_wr;

	if (seq->luma_depth != 8 || seq->chroma_depth != 8 ||
	    seq->chroma_format != 1 || seq->width > WAVE4_MAX_WIDTH ||
	    seq->height > WAVE4_MAX_HEIGHT || !seq->min_fbs ||
	    seq->min_fbs > WAVE4_MAX_FBS) {
		dev_err(ctx->dev->dev,
			"unsupported stream: %ux%u, %u/%u bit, format %u, %u frames\n",
			seq->width, seq->height, seq->luma_depth,
			seq->chroma_depth, seq->chroma_format, seq->min_fbs);
		ctx->state = WAVE4_STATE_ERROR;
		vb2_queue_error(v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx));
		return;
	}

	dev_dbg(ctx->dev->dev, "sequence: %ux%u, profile %u level %u, %u frames\n",
		seq->width, seq->height, seq->profile, seq->level, seq->min_fbs);

	wave4_set_dst_fmt(&ctx->dst_fmt, seq->width, seq->height);
	v4l2_ctrl_s_ctrl(ctx->min_bufs_ctrl, seq->min_fbs);
	ctx->state = WAVE4_STATE_SEQ_READY;
	v4l2_event_queue_fh(&ctx->fh, &(struct v4l2_event){
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
	});
}

static void wave4_finish_drain(struct wave4_ctx *ctx)
{
	struct v4l2_m2m_ctx *m2m = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *dst = v4l2_m2m_dst_buf_remove(m2m);

	if (!dst) {
		m2m->next_buf_last = true;
		return;
	}
	ctx->disp_flag |= BIT(dst->vb2_buf.index);
	dst->field = V4L2_FIELD_NONE;
	vb2_set_plane_payload(&dst->vb2_buf, 0, 0);
	v4l2_m2m_last_buffer_done(m2m, dst);
	v4l2_event_queue_fh(&ctx->fh, &(struct v4l2_event){
		.type = V4L2_EVENT_EOS });
}

static void wave4_run_decode(struct wave4_ctx *ctx)
{
	struct v4l2_m2m_ctx *m2m = ctx->fh.m2m_ctx;
	struct wave4_dec_result res = {};
	struct vb2_v4l2_buffer *dst;
	u32 rd;
	int ret;

	if (!ctx->bs_pending && !wave4_feed(ctx)) {
		if (!m2m->is_draining)
			return;
		/* Nothing left to feed: flag the end and empty the DPB. */
		ctx->eos = true;
		ctx->bs_rd = ctx->bs_wr;
	}

	rd = ctx->bs_rd;
	ret = wave4_hw_decode(ctx, &res);
	if (ret) {
		ctx->state = WAVE4_STATE_ERROR;
		vb2_queue_error(v4l2_m2m_get_src_vq(m2m));
		vb2_queue_error(v4l2_m2m_get_dst_vq(m2m));
		return;
	}

	if (res.seq_changed) {
		/* A mid-stream change needs new frames: not handled yet. */
		dev_warn(ctx->dev->dev, "sequence change %#x not supported\n",
			 res.seq_changed);
		ctx->state = WAVE4_STATE_ERROR;
		vb2_queue_error(v4l2_m2m_get_src_vq(m2m));
		vb2_queue_error(v4l2_m2m_get_dst_vq(m2m));
		return;
	}

	if (res.dec_idx >= 0 && res.dec_idx < ctx->num_fbs)
		ctx->ts[res.dec_idx] = wave4_ts_pop(ctx);
	else if (res.dec_idx == -2 && ctx->ts_count > 1)
		/* A skipped picture; with one queued it is still in progress. */
		wave4_ts_pop(ctx);
	else if (res.dec_idx == -1 && ctx->bs_rd == rd && res.disp_idx < 0)
		/* Every free frame is held; draining needs one as well. */
		ctx->need_fb = true;

	ctx->bs_pending = !ctx->eos && ctx->bs_rd < ctx->bs_wr;
	if (!ctx->bs_pending)
		wave4_release_src(ctx, VB2_BUF_STATE_DONE);

	if (res.disp_idx >= 0 && res.disp_idx < ctx->num_fbs) {
		dst = v4l2_m2m_dst_buf_remove_by_idx(m2m, res.disp_idx);
		if (!dst) {
			dev_warn(ctx->dev->dev, "frame %d shown but not queued\n",
				 res.disp_idx);
			return;
		}
		ctx->disp_flag |= BIT(res.disp_idx);
		dst->vb2_buf.timestamp = ctx->ts[res.disp_idx];
		dst->field = V4L2_FIELD_NONE;
		vb2_set_plane_payload(&dst->vb2_buf, 0, ctx->dst_fmt.sizeimage);
		v4l2_m2m_buf_done(dst, VB2_BUF_STATE_DONE);
	} else if (ctx->eos && res.disp_idx == -1) {
		wave4_finish_drain(ctx);
	}
}

static void wave4_run_work(struct work_struct *work)
{
	struct wave4_ctx *ctx = container_of(work, struct wave4_ctx, run_work);

	switch (ctx->state) {
	case WAVE4_STATE_NEED_SEQ:
		wave4_run_init_seq(ctx);
		break;
	case WAVE4_STATE_DECODING:
		wave4_run_decode(ctx);
		break;
	default:
		break;
	}
	v4l2_m2m_job_finish(ctx->dev->m2m_dev, ctx->fh.m2m_ctx);
}

static void wave4_device_run(void *priv)
{
	struct wave4_ctx *ctx = priv;

	queue_work(ctx->dev->wq, &ctx->run_work);
}

static int wave4_job_ready(void *priv)
{
	struct wave4_ctx *ctx = priv;
	struct v4l2_m2m_ctx *m2m = ctx->fh.m2m_ctx;
	bool have_src = ctx->bs_pending || v4l2_m2m_num_src_bufs_ready(m2m);

	switch (ctx->state) {
	case WAVE4_STATE_NEED_SEQ:
		return have_src;
	case WAVE4_STATE_DECODING:
		if (v4l2_m2m_has_stopped(m2m) || ctx->need_fb ||
		    !v4l2_m2m_num_dst_bufs_ready(m2m))
			return 0;
		return have_src || m2m->is_draining;
	default:
		return 0;
	}
}

static const struct v4l2_m2m_ops wave4_m2m_ops = {
	.device_run = wave4_device_run,
	.job_ready = wave4_job_ready,
};

static int wave4_open(struct file *file)
{
	struct wave4_dev *dev = video_drvdata(file);
	struct wave4_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;
	INIT_WORK(&ctx->run_work, wave4_run_work);

	ctx->id = ida_alloc_max(&dev->inst_ida, WAVE4_MAX_INSTANCES - 1,
				GFP_KERNEL);
	if (ctx->id < 0) {
		ret = -EBUSY;
		goto err_free;
	}

	ret = wave4_coherent_alloc(dev, &ctx->work, WAVE4_WORK_SIZE);
	if (ret)
		goto err_ida;

	ret = wave4_hw_create_instance(ctx);
	if (ret)
		goto err_work;

	v4l2_fh_init(&ctx->fh, &dev->vfd);

	v4l2_ctrl_handler_init(&ctx->ctrls, 1);
	ctx->min_bufs_ctrl = v4l2_ctrl_new_std(&ctx->ctrls, NULL,
					       V4L2_CID_MIN_BUFFERS_FOR_CAPTURE,
					       1, WAVE4_MAX_FBS, 1, 1);
	if (ctx->ctrls.error) {
		ret = ctx->ctrls.error;
		goto err_ctrls;
	}
	ctx->fh.ctrl_handler = &ctx->ctrls;

	ctx->src_fmt.width = WAVE4_DEF_WIDTH;
	ctx->src_fmt.height = WAVE4_DEF_HEIGHT;
	wave4_set_src_fmt(&ctx->src_fmt);
	wave4_set_dst_fmt(&ctx->dst_fmt, WAVE4_DEF_WIDTH, WAVE4_DEF_HEIGHT);

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev, ctx, wave4_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_ctrls;
	}
	/* INIT_SEQ must run before CAPTURE can be set up. */
	v4l2_m2m_set_src_buffered(ctx->fh.m2m_ctx, true);
	v4l2_m2m_set_dst_buffered(ctx->fh.m2m_ctx, true);
	ctx->fh.m2m_ctx->ignore_cap_streaming = true;

	v4l2_fh_add(&ctx->fh, file);
	return 0;

err_ctrls:
	v4l2_ctrl_handler_free(&ctx->ctrls);
	v4l2_fh_exit(&ctx->fh);
	wave4_hw_fini_seq(ctx);
err_work:
	wave4_coherent_free(dev, &ctx->work);
err_ida:
	ida_free(&dev->inst_ida, ctx->id);
err_free:
	kfree(ctx);
	return ret;
}

static int wave4_release(struct file *file)
{
	struct wave4_ctx *ctx = file_to_ctx(file);
	struct wave4_dev *dev = ctx->dev;

	mutex_lock(&dev->dev_lock);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	mutex_unlock(&dev->dev_lock);
	flush_work(&ctx->run_work);

	wave4_hw_fini_seq(ctx);
	wave4_free_fbs(ctx);
	wave4_coherent_free(dev, &ctx->bs);
	wave4_coherent_free(dev, &ctx->work);

	v4l2_ctrl_handler_free(&ctx->ctrls);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	ida_free(&dev->inst_ida, ctx->id);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations wave4_fops = {
	.owner = THIS_MODULE,
	.open = wave4_open,
	.release = wave4_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

int wave4_dec_register(struct wave4_dev *dev)
{
	struct video_device *vfd = &dev->vfd;
	int ret;

	dev->m2m_dev = v4l2_m2m_init(&wave4_m2m_ops);
	if (IS_ERR(dev->m2m_dev))
		return PTR_ERR(dev->m2m_dev);

	strscpy(vfd->name, WAVE4_NAME, sizeof(vfd->name));
	vfd->fops = &wave4_fops;
	vfd->ioctl_ops = &wave4_ioctl_ops;
	vfd->release = video_device_release_empty;
	vfd->lock = &dev->dev_lock;
	vfd->v4l2_dev = &dev->v4l2_dev;
	vfd->vfl_dir = VFL_DIR_M2M;
	vfd->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	video_set_drvdata(vfd, dev);

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, -1);
	if (ret)
		v4l2_m2m_release(dev->m2m_dev);
	return ret;
}

void wave4_dec_unregister(struct wave4_dev *dev)
{
	video_unregister_device(&dev->vfd);
	v4l2_m2m_release(dev->m2m_dev);
}
