/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Boot progress for kexec'd images, drawn into the stock firmware's live
 * scanout buffer (0x88000000, 3 pages of 1920x720 ARGB8888, 7680 bytes/row).
 *
 *   x   0..63    progress tiles: stage k is a 64x16 tile at y = 16*k
 *   x  68..99    16-bit initcall counter squares (see tcc803x_hb_counter())
 *   x 128..1919  text console (tcc803x-fbcon.c)
 *
 * Only meant to be expanded with the MMU off or with the buffer identity
 * mapped (head.S and mmu.c keep 0x88000000-0x88ffffff mapped until userspace).
 */
#ifndef __ASM_TCC803X_HB_H
#define __ASM_TCC803X_HB_H

#ifdef __ASSEMBLY__
#ifdef CONFIG_DEBUG_TCC803X_FB_HEARTBEAT

	.macro	tcc_hb_clear, t1, t2, t3
	mov	\t1, #0x88000000
	add	\t2, \t1, #0xfd0000
	add	\t2, \t2, #0x2000
	mov	\t3, #0xff000000
2:	str	\t3, [\t1], #4
	cmp	\t1, \t2
	blo	2b
	.endm

	@ \k: stage 0-35; \col: 0xRRGGBB (alpha forced to 0xff); t1-t4 scratch
	.macro	tcc_hb_band, k, col, t1, t2, t3, t4
	movw	\t3, #((\col) & 0xffff)
	movt	\t3, #(0xff00 | ((\col) >> 16))
	.irp	p, 0, 1, 2
	movw	\t1, #((0x88000000 + \p * 0x546000 + \k * 0x1e000) & 0xffff)
	movt	\t1, #((0x88000000 + \p * 0x546000 + \k * 0x1e000) >> 16)
	mov	\t4, #16
1:	add	\t2, \t1, #256
2:	str	\t3, [\t1], #4
	cmp	\t1, \t2
	blo	2b
	add	\t1, \t1, #(7680 - 256)
	subs	\t4, \t4, #1
	bne	1b
	.endr
	.endm

#else
	.macro	tcc_hb_clear, t1, t2, t3
	.endm
	.macro	tcc_hb_band, k, col, t1, t2, t3, t4
	.endm
#endif
#else
#ifdef CONFIG_DEBUG_TCC803X_FB_HEARTBEAT

#include <linux/sched.h>
#include <linux/mm_types.h>

/* The fb is only identity-mapped in init_mm; user mms would fault. */
static inline int tcc803x_hb_mm_ok(void)
{
	return current->active_mm == &init_mm;
}

/* false once a framebuffer driver (simplefb) owns the screen */
bool tcc803x_hb_active(void);

static inline int tcc803x_hb_ok(void)
{
	return tcc803x_hb_mm_ok() && tcc803x_hb_active();
}

/* Direct-write variant for C code (MMU on, fb identity-mapped). */
static inline void tcc803x_hb_band_direct(int k, unsigned int argb)
{
	int p, r, c;

	if (!tcc803x_hb_ok())
		return;
	for (p = 0; p < 3; p++) {
		for (r = 0; r < 16; r++) {
			volatile unsigned int *q = (volatile unsigned int *)
				(0x88000000 + p * 0x546000 + (k * 16 + r) * 7680);

			for (c = 0; c < 64; c++)
				q[c] = argb;
		}
	}
}

static inline void tcc803x_hb_band_any(int k, unsigned int argb)
{
	tcc803x_hb_band_direct(k, argb);
}
static inline void tcc803x_hb_late_on(void) { }

/*
 * 16-bit counter as a column of 32x32 px squares (x=68, y=8 + 36*bit; bit 0
 * at the top): white = 1, dark blue = 0. Only changed bits are repainted.
 */
static inline void tcc803x_hb_counter(unsigned int v)
{
	static unsigned int prev;
	static int first = 1;
	unsigned int diff = first ? 0xffff : ((v ^ prev) & 0xffff);
	unsigned int i, p, r, c;

	if (!tcc803x_hb_ok())
		return;
	for (i = 0; i < 16; i++) {
		unsigned int col;

		if (!(diff & (1u << i)))
			continue;
		col = (v & (1u << i)) ? 0xffffffff : 0xff0000a0;
		for (p = 0; p < 3; p++) {
			for (r = 0; r < 32; r++) {
				volatile unsigned int *q = (volatile unsigned int *)
					(0x88000000 + p * 0x546000 +
					 (8 + i * 36 + r) * 7680 + 68 * 4);

				for (c = 0; c < 32; c++)
					q[c] = col;
			}
		}
	}
	prev = v;
	first = 0;
}

void tcc803x_fbcon_register(void);

#else
static inline void tcc803x_hb_band_direct(int k, unsigned int argb) { }
static inline void tcc803x_hb_band_any(int k, unsigned int argb) { }
static inline void tcc803x_hb_late_on(void) { }
static inline void tcc803x_hb_counter(unsigned int v) { }
static inline void tcc803x_fbcon_register(void) { }
#endif
#endif

#endif
