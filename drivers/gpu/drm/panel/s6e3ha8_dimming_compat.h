/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Compatibility shim for Samsung's S6E3HA8 smart-dimming algorithm.
 *
 * The algorithm is imported near-verbatim from Samsung's downstream driver
 * rather than rewritten, because its output is verifiable: the max-brightness
 * gamma it generates is byte-identical to the 0xca payload this panel driver
 * already writes in its init sequence. Re-typing ~800 lines of fixed-point
 * maths would risk silent transcription errors for no functional gain.
 *
 * This header supplies the handful of vendor-kernel names it expects.
 */
#ifndef __S6E3HA8_DIMMING_COMPAT_H__
#define __S6E3HA8_DIMMING_COMPAT_H__

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/math64.h>

#define LCD_INFO(fmt, ...)	pr_debug("s6e3ha8: " fmt, ##__VA_ARGS__)
#define LCD_ERR(fmt, ...)	pr_err("s6e3ha8: " fmt, ##__VA_ARGS__)
#define LCD_DEBUG(fmt, ...)	pr_debug("s6e3ha8: " fmt, ##__VA_ARGS__)

/* Only normal-mode gamma is used; HBM is not wired up yet. */
#define HBM_MODE		1

#endif /* __S6E3HA8_DIMMING_COMPAT_H__ */
