// SPDX-License-Identifier: GPL-2.0
/*
 * Text console drawn straight into the stock firmware's live framebuffer, so
 * printk output of a kexec'd kernel is readable on the head unit's panel
 * without any UART. Uses the identity mapping head.S/mmu.c keep at
 * 0x88000000-0x88ffffff, so it only works until userspace starts.
 *
 * Text area: x 128..1919, y 0..575, two columns of 112 x 36 cells of 8x16
 * (logical line n is column n / 36, row n % 36). Output wraps to
 * the top; the line after the newest one is always blank. Lines 36..44 are a
 * separate area written directly by the heartbeat timer, bypassing printk.
 */
#include <linux/console.h>
#include <linux/fb.h>
#include <linux/font.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kdebug.h>
#include <linux/notifier.h>
#include <linux/pid.h>
#include <linux/sched.h>
#include <linux/sched/debug.h>
#include <linux/stacktrace.h>
#include <linux/timer.h>
#include <asm/irq_regs.h>
#include <linux/types.h>
#include <asm/tcc803x-hb.h>

#define FB_BASE		0x88000000UL
#define FB_PAGE		0x546000UL
#define FB_W		1920
#define X0		128
#define COLS		112	/* per column */
#define BLOCKS		2	/* text columns */
#define LINES		36	/* rows per column */
#define TOTAL_LINES	(LINES * BLOCKS)
#define DLINE0		0	/* rows below the text (y >= 576): direct, lock-free output */
#define DLINES		9
#define FG		0xffffffffu
#define BG		0xff000000u
/* RDMA0 BASE0 as stashed by the decompressor at its entry. */
#define BASE0_STASH	(*(volatile u32 *)(FB_BASE + 0xff0010))

static unsigned int cx, cy, first_page, last_page;

/*
 * Identity mapping (head.S/mmu.c) until tcc_hb_remap() swaps in an ioremap of
 * the same buffer, which, unlike the identity map, exists in every mm.
 */
static volatile u8 *fbv = (volatile u8 *)FB_BASE;
static bool fb_remapped;

/*
 * Once simplefb/fbcon own the screen, stop painting over them. The counter is
 * in fbmem.c but no longer declared in a header; this file is built in.
 */
extern int num_registered_fb;

bool tcc803x_hb_active(void)
{
	return !num_registered_fb;
}

static bool fb_usable(void)
{
	return (fb_remapped || tcc803x_hb_mm_ok()) && tcc803x_hb_active();
}

static volatile u32 *cell(unsigned int page, unsigned int y, unsigned int x)
{
	return (volatile u32 *)(fbv + page * FB_PAGE) + y * FB_W + x;
}

/* physical x of the left edge of logical line @line, and its pixel row */
static unsigned int line_x(unsigned int line)
{
	return X0 + (line / LINES) * COLS * 8;
}

static unsigned int line_y(unsigned int line)
{
	return (line % LINES) * 16;
}

static void clear_px(unsigned int x, unsigned int y)
{
	unsigned int p, r, c;

	for (p = first_page; p <= last_page; p++)
		for (r = 0; r < 16; r++) {
			volatile u32 *q = cell(p, y + r, x);

			for (c = 0; c < COLS * 8; c++)
				q[c] = BG;
		}
}

static void clear_lines(unsigned int line, unsigned int n)
{
	unsigned int l;

	for (l = line; l < line + n; l++)
		clear_px(line_x(l), line_y(l));
}

static void draw_px(unsigned int x, unsigned int y, unsigned char ch)
{
	const u8 *g = font_vga_8x16.data + ch * 16;
	unsigned int p, r, b;

	for (p = first_page; p <= last_page; p++)
		for (r = 0; r < 16; r++) {
			volatile u32 *q = cell(p, y + r, x);

			for (b = 0; b < 8; b++)
				q[b] = (g[r] & (0x80 >> b)) ? FG : BG;
		}
}

static void draw(unsigned int col, unsigned int line, unsigned char ch)
{
	draw_px(line_x(line) + col * 8, line_y(line), ch);
}

static void newline(void)
{
	cx = 0;
	cy = (cy + 1) % TOTAL_LINES;
	clear_lines(cy, 1);
}

static void tcc_hb_write(struct console *co, const char *s, unsigned int n)
{
	static bool entered, drew;

	if (!fb_usable())
		return;
	if (!entered) {
		entered = true;
		tcc803x_hb_band_direct(39, 0xffff8000);	/* write() called */
	}

	while (n--) {
		unsigned char ch = *s++;

		if (ch == '\n') {
			newline();
			continue;
		}
		if (ch == '\r')
			continue;
		if (ch == '\t' || ch < 0x20)
			ch = ' ';
		if (cx >= COLS)
			newline();
		draw(cx++, cy, ch);
		if (!drew) {
			drew = true;
			tcc803x_hb_band_direct(40, 0xff80ff00);	/* first glyph drawn */
		}
	}
}

/* Write one string into a direct-area line, without touching printk. */
static void put_line(unsigned int row, const char *str)
{
	unsigned int c = 0, y = (LINES + row) * 16;

	clear_px(X0, y);
	while (*str && c < COLS)
		draw_px(X0 + c++ * 8, y, (unsigned char)*str++);
}

/*
 * Heartbeat: if timer interrupts still fire it fills the direct area with the
 * interrupted PC/LR, the current task and PID 1's (kernel_init) stack, all
 * without printk (which cannot draw while a stuck thread holds console_lock).
 */
static struct timer_list tcc_hb_timer;
static unsigned int tcc_hb_ticks;

static void tcc_hb_timer_fn(struct timer_list *t)
{
	struct pt_regs *regs = get_irq_regs();
	char buf[COLS + 1];

	++tcc_hb_ticks;
	if (!fb_usable())
		goto rearm;
	snprintf(buf, sizeof(buf), "TCC alive tick %u jiffies=%lu", tcc_hb_ticks, jiffies);
	put_line(DLINE0, buf);
	if (regs) {
		snprintf(buf, sizeof(buf), "irq pc=%pS lr=%pS", (void *)regs->ARM_pc, (void *)regs->ARM_lr);
		put_line(DLINE0 + 1, buf);
	}
	snprintf(buf, sizeof(buf), "cur=%s pid=%d", current->comm, current->pid);
	put_line(DLINE0 + 2, buf);

	tcc803x_hb_band_direct(41, 0xff00ffff);	/* timer callback ran */
rearm:
	if (tcc_hb_ticks < 40)
		mod_timer(t, jiffies + 3 * HZ);
}

/*
 * Oops record in direct rows 3..8, which neither scrolling nor the heartbeat
 * timer touches: what, fault status, faulting address, PC/LR and registers.
 */
static int tcc_hb_die(struct notifier_block *nb, unsigned long val, void *data)
{
	struct die_args *a = data;
	struct pt_regs *r = a->regs;
	char buf[COLS + 1];

	if (val != DIE_OOPS || !r || !(fb_remapped || tcc803x_hb_mm_ok()))
		return NOTIFY_DONE;
	snprintf(buf, sizeof(buf), "OOPS %s err/fsr=%#lx trap=%d addr=%08lx",
		 a->str, a->err, a->trapnr, current->thread.address);
	put_line(DLINE0 + 3, buf);
	snprintf(buf, sizeof(buf), "pc=%08lx %pS", r->ARM_pc, (void *)r->ARM_pc);
	put_line(DLINE0 + 4, buf);
	snprintf(buf, sizeof(buf), "lr=%08lx %pS", r->ARM_lr, (void *)r->ARM_lr);
	put_line(DLINE0 + 5, buf);
	snprintf(buf, sizeof(buf), "r0=%08lx r1=%08lx r2=%08lx r3=%08lx r4=%08lx r5=%08lx",
		 r->ARM_r0, r->ARM_r1, r->ARM_r2, r->ARM_r3, r->ARM_r4, r->ARM_r5);
	put_line(DLINE0 + 6, buf);
	snprintf(buf, sizeof(buf), "r6=%08lx r7=%08lx r8=%08lx r9=%08lx r10=%08lx fp=%08lx",
		 r->ARM_r6, r->ARM_r7, r->ARM_r8, r->ARM_r9, r->ARM_r10, r->ARM_fp);
	put_line(DLINE0 + 7, buf);
	snprintf(buf, sizeof(buf), "ip=%08lx sp=%08lx psr=%08lx", r->ARM_ip, r->ARM_sp, r->ARM_cpsr);
	put_line(DLINE0 + 8, buf);
	return NOTIFY_DONE;
}

static struct notifier_block tcc_hb_die_nb = {
	.notifier_call = tcc_hb_die,
};

static struct console tcc_hb_console = {
	.name	= "tcchb",
	.write	= tcc_hb_write,
	.flags	= CON_PRINTBUFFER | CON_ENABLED,
	.index	= -1,
};

static int __init tcc_hb_remap(void)
{
	void __iomem *m = ioremap_wc(FB_BASE, 3 * FB_PAGE);

	if (m) {
		fbv = (volatile u8 *)m;
		fb_remapped = true;
	}
	return 0;
}
postcore_initcall(tcc_hb_remap);

void __init tcc803x_fbcon_register(void)
{
	u32 base = BASE0_STASH;

	/* Draw only into the page being scanned if we know it, else all three. */
	if (base >= FB_BASE && base < FB_BASE + 3 * FB_PAGE) {
		first_page = last_page = (base - FB_BASE) / FB_PAGE;
	} else {
		first_page = 0;
		last_page = 2;
	}
	tcc803x_hb_band_direct(36, 0xffff0000);	/* register: page chosen */
	clear_lines(0, TOTAL_LINES);
	tcc803x_hb_band_direct(37, 0xff00ff00);	/* register: text area cleared */
	register_die_notifier(&tcc_hb_die_nb);
	register_console(&tcc_hb_console);
	tcc803x_hb_band_direct(38, 0xff0000ff);	/* register_console returned */
	timer_setup(&tcc_hb_timer, tcc_hb_timer_fn, 0);
	mod_timer(&tcc_hb_timer, jiffies + 3 * HZ);
}
