/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Chips&Media Wave410 HEVC decoder: host interface registers.
 *
 * Offsets and values are from static analysis of the Telechips TCC803x stock
 * hevc_lib.ko (Wave4Vpu* functions) and the TOPST kernel-5.10 manager
 * (drivers/char/vpu_v3/vpu_hevc_dec_mgr.c, which names W4_BS_RD_PTR,
 * W4_BS_WR_PTR, W4_BS_OPTION, W4_BS_PARAM and the FIO pair). The names are
 * ours, chosen to match wave5-regdefine.h where a register has the same
 * function there; the 0x000-0x07c block is laid out as on Wave5.
 */
#ifndef __WAVE4_REGS_H__
#define __WAVE4_REGS_H__

#include <linux/bits.h>

/* Host interface, same offsets as W5_* */
#define W4_PO_CONF			0x000
#define W4_VCPU_CUR_PC			0x004
#define W4_VPU_FIO_CTRL_ADDR		0x020
#define   W4_FIO_WRITE			BIT(16)
#define   W4_FIO_READY			BIT(31)
#define W4_VPU_FIO_DATA			0x024
#define W4_VPU_VINT_REASON_USR		0x030
#define W4_VPU_VINT_REASON_CLR		0x034
#define W4_VPU_HOST_INT_REQ		0x038
#define W4_VPU_VINT_CLEAR		0x03c
#define W4_VPU_VINT_ENABLE		0x048
#define W4_VPU_RESET_REQ		0x050
#define   W4_RST_BLOCK_ALL		0x7ffffff
#define W4_VPU_RESET_STATUS		0x054
#define W4_VPU_REMAP_CTRL		0x060
/* enable, region 0, bit 11 (as wave5), 1 MiB page (0x100 4 KiB units) */
#define   W4_REMAP_CTRL_CODE		0x80000900
#define W4_VPU_REMAP_VADDR		0x064
#define W4_VPU_REMAP_PADDR		0x068
#define W4_VPU_REMAP_CORE_START		0x06c
#define W4_VPU_BUSY_STATUS		0x070

/* Behind the FIO window: written through W4_VPU_FIO_*, read directly */
#define W4_GDI_BUS_CTRL			0x88f0
#define W4_GDI_BUS_STATUS		0x88f4
#define   W4_GDI_BUS_IDLE		0x738

/* Command block */
#define W4_COMMAND			0x100
#define W4_CORE_INDEX			0x104
#define W4_INST_INDEX			0x108	/* inst | (codec << 16), codec 0 = HEVC */
#define W4_COMMAND_OPTION		0x10c	/* also SET_FB start/end and endian */
#define W4_RET_SUCCESS			0x110
#define W4_RET_FAIL_REASON		0x114

/* INIT_VPU parameters */
#define W4_ADDR_CODE_BASE		0x118
#define W4_CODE_SIZE			0x11c
#define W4_CODE_PARAM			0x120
#define W4_HW_OPTION			0x124
#define W4_ADDR_STACK_BASE		0x12c
#define W4_STACK_SIZE			0x130
#define W4_TIMEOUT_CNT			0x134

/* GET_FW_VERSION result */
#define W4_RET_FW_VERSION		0x118

/* Decoder: bitstream and per-instance buffers (INIT_SEQ, DEC_PIC) */
#define W4_BS_START_ADDR		0x120
#define W4_BS_SIZE			0x124
#define W4_BS_PARAM			0x128	/* [3:0] endian, [9] ring mode */
#define W4_BS_OPTION			0x12c	/* [0] pic-end mode, [1] stream end */
#define   W4_BS_OPTION_PIC_END		BIT(0)
#define   W4_BS_OPTION_EOS		BIT(1)
#define W4_BS_RD_PTR			0x130
#define W4_BS_WR_PTR			0x134
#define W4_ADDR_WORK_BASE		0x138
#define W4_WORK_SIZE			0x13c
#define W4_WORK_PARAM			0x140
#define W4_ADDR_TEMP_BASE		0x144
#define W4_TEMP_SIZE			0x148
#define W4_TEMP_PARAM			0x14c
#define W4_ADDR_SEC_AXI			0x150
#define W4_SEC_AXI_SIZE			0x154
#define W4_USE_SEC_AXI			0x158	/* the vendor sets bits 15, 9 and 0 */
#define   W4_USE_SEC_AXI_ALL		(BIT(15) | BIT(9) | BIT(0))
#define W4_CMD_DEC_ADDR_REPORT		0x15c	/* DEC_PIC only; meaning unknown */
#define W4_CMD_DEC_REPORT_SIZE		0x160
#define W4_CMD_DEC_REPORT_PARAM		0x164	/* the vendor writes 0xc */
#define W4_CMD_DEC_ADDR_USER_BASE	0x168
#define W4_CMD_DEC_USER_SIZE		0x16c
#define W4_CMD_DEC_USER_PARAM		0x170	/* the vendor writes 0xf */
#define W4_CMD_DEC_USER_MASK		0x174
#define W4_CMD_DEC_RESERVED_178		0x178	/* always 0 */
#define W4_CMD_DEC_SKIP			0x188	/* 1 with skip modes, else 0 */
#define W4_CMD_DEC_TEMPORAL_ID_PLUS1	0x18c
#define W4_CMD_DEC_DISP_FLAG		0x190	/* bit n: frame n held by the host */
#define W4_CMD_SEQ_CHANGE_ENABLE_FLAG	0x194

/* SET_FRAMEBUF */
#define W4_COMMON_PIC_INFO		0x120
#define   W4_PIC_INFO_CBCR_INTERLEAVE	BIT(16)
#define   W4_PIC_INFO_PIXEL_ORDER	BIT(23)
#define   W4_PIC_INFO_LINEAR		BIT(28)
#define   W4_PIC_INFO_NV21		BIT(29)
#define W4_PIC_SIZE			0x124
#define W4_SET_FB_NUM			0x128	/* start << 8 | end */
#define W4_SFB_OPTION_START		BIT(3)
#define W4_SFB_OPTION_END		BIT(4)
#define W4_ADDR_LUMA_BASE0		0x160	/* + 16 * n, n < 8 */
#define W4_ADDR_CB_BASE0		0x164
#define W4_ADDR_CR_BASE0		0x168	/* linear */
#define W4_ADDR_FBC_Y_OFFSET0		0x168	/* compressed */
#define W4_ADDR_FBC_C_OFFSET0		0x16c
#define W4_ADDR_MV_COL0			0x1e0	/* + 4 * n */

/* INIT_SEQ results (rewritten by DEC_PIC on a sequence change) */
#define W4_RET_DEC_BUF_DELAY		0x188
#define W4_RET_DEC_SEQ_PARAM		0x198	/* [7:0] level, [19:12] compat, [23:21] sub-layers, [28:24] profile, [29] tier */
#define W4_RET_DEC_COLOR_SAMPLE_INFO	0x1a0	/* [3:0] luma, [7:4] chroma bit depth, [11:8] chroma format, [23:16] aspect ratio idc */
#define W4_RET_DEC_ASPECT_RATIO		0x1a4
#define W4_RET_DEC_BIT_RATE		0x1a8
#define W4_RET_DEC_FRAME_RATE_NR	0x1ac
#define W4_RET_DEC_FRAME_RATE_DR	0x1b0
#define W4_RET_DEC_CROP_TOP_BOTTOM	0x1b4	/* top << 16 | bottom offset */
#define W4_RET_DEC_CROP_LEFT_RIGHT	0x1b8	/* left << 16 | right offset */
#define W4_RET_DEC_PIC_SIZE		0x1bc	/* width << 16 | height */
#define W4_RET_DEC_NUM_REQUIRED_FB	0x1c0
#define W4_RET_DEC_WARN_INFO		0x1d8
#define W4_RET_DEC_ERR_INFO		0x1f4

/* DEC_PIC results */
#define W4_RET_DEC_NOTIFICATION		0x1c4	/* [30:0] sequence changed */
#define W4_RET_DEC_DECODED_INDEX	0x1e0	/* WTL: linear << 16 | compressed */
#define W4_RET_DEC_DISPLAY_INDEX	0x1e4
#define W4_RET_DEC_PIC_TYPE		0x1e8	/* [2:0] I/P/B, [9:4] nal_unit_type */
#define W4_RET_DEC_PIC_POC		0x1ec
#define W4_RET_DEC_DISPLAY_SIZE		0x1f0

/* Commands: bit n of the code is bit n of VINT_REASON */
#define W4_CMD_INIT_VPU			BIT(0)
#define W4_CMD_INIT_SEQ			BIT(1)
#define W4_CMD_FINI_SEQ			BIT(2)
#define W4_CMD_DEC_PIC			BIT(3)
#define W4_CMD_SET_FRAMEBUF		BIT(4)
#define W4_CMD_FLUSH_DECODER		BIT(5)
#define W4_CMD_GET_FW_VERSION		BIT(8)
#define W4_CMD_SLEEP_VPU		BIT(10)
#define W4_CMD_CREATE_INSTANCE		BIT(14)
#define W4_INT_BSBUF_EMPTY		BIT(15)

/* The interrupt masks the vendor enables before each command */
#define W4_VINT_ENABLE_INIT		0x860b
#define W4_VINT_ENABLE_DEC_PIC		(W4_CMD_FINI_SEQ | W4_CMD_DEC_PIC | \
					 W4_CMD_FLUSH_DECODER | W4_INT_BSBUF_EMPTY)

/* W4_CMD_SEQ_CHANGE_ENABLE_FLAG: profile, size, bit depth, DPB count */
#define W4_SEQ_CHANGE_ENABLE_ALL_HEVC	(BIT(5) | BIT(16) | BIT(18) | BIT(19))

#define W4_INIT_SEQ_OPTION_NORMAL	1
#define W4_DEC_PIC_OPTION_NORMAL	0
#define W4_TEMPORAL_ID_ALL		7

/* Endian codes as the vendor programs them (its frame/stream endian 0x1f) */
#define W4_ENDIAN_STREAM		0xf
#define W4_ENDIAN_FBC			0xf
#define W4_ENDIAN_LINEAR		0x0

#endif
