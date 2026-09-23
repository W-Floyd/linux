// SPDX-License-Identifier: GPL-2.0-only
/*
 * Read the per-master sleep records the RPM firmware keeps in its message
 * RAM: for each master (APSS, MPSS, ...) the number of shutdowns, the
 * timestamps of the last shutdown request / wakeup indication / bringup
 * request and ack, and the XO shutdown accounting.
 *
 * The record layout is the "version 2" one downstream's rpm_master_stat.c
 * reads; the address and stride are the downstream devicetree values for
 * this SoC family (reg = <0x45f0150 0x5000>, master-offset 4096). Both are
 * module parameters so another SoC can be tried without a rebuild.
 *
 * Message RAM must not be touched from user space on this SoC (a /dev/mem
 * read of the same record reset the SoC); the kernel's ioremap is how
 * downstream reads it, and that is what this does. Debug aid, debugfs only.
 */
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <clocksource/arm_arch_timer.h>

static unsigned long base = 0x45f0150;
module_param(base, ulong, 0444);
MODULE_PARM_DESC(base, "physical address of the first master record");

static unsigned int stride = 4096;
module_param(stride, uint, 0444);
MODULE_PARM_DESC(stride, "bytes between master records");

static const char * const masters[] = { "APSS", "MPSS", "ADSP", "CDSP", "TZ" };

/* struct msm_rpm_master_stats, version 2 */
#define ACTIVE_CORES		0x00	/* u32 */
#define NUMSHUTDOWNS		0x04	/* u32 */
#define SHUTDOWN_REQ		0x08	/* u64 */
#define WAKEUP_IND		0x10	/* u64 */
#define BRINGUP_REQ		0x18	/* u64 */
#define BRINGUP_ACK		0x20	/* u64 */
#define WAKEUP_REASON		0x28	/* u32: 0 rude, 1 scheduled */
#define LAST_SLEEP_DURATION	0x2c	/* u32 */
#define LAST_WAKE_DURATION	0x30	/* u32 */
#define XO_COUNT		0x34	/* u32 */
#define XO_LAST_ENTERED		0x38	/* u64 */
#define XO_LAST_EXITED		0x40	/* u64 */
#define XO_ACCUMULATED		0x48	/* u64 */
#define RECORD_LEN		0x50

static void __iomem *regs;
static struct dentry *root;

/* Message RAM is read a 32-bit word at a time, as the glink-rpm driver does. */
static u64 rd64(void __iomem *p)
{
	u64 lo = readl_relaxed(p);
	u64 hi = readl_relaxed(p + 4);

	return lo | (hi << 32);
}

static int stats_show(struct seq_file *s, void *unused)
{
	int i;

	seq_printf(s, "now (arch timer): %llu\n\n", arch_timer_read_counter());
	for (i = 0; i < ARRAY_SIZE(masters); i++) {
		void __iomem *r = regs + i * stride;

		seq_printf(s, "%s\n", masters[i]);
		seq_printf(s, "\tactive_cores: %#x\n", readl_relaxed(r + ACTIVE_CORES));
		seq_printf(s, "\tnumshutdowns: %u\n", readl_relaxed(r + NUMSHUTDOWNS));
		seq_printf(s, "\tshutdown_req: %llu\n", rd64(r + SHUTDOWN_REQ));
		seq_printf(s, "\twakeup_ind: %llu\n", rd64(r + WAKEUP_IND));
		seq_printf(s, "\tbringup_req: %llu\n", rd64(r + BRINGUP_REQ));
		seq_printf(s, "\tbringup_ack: %llu\n", rd64(r + BRINGUP_ACK));
		seq_printf(s, "\twakeup_reason: %u\n", readl_relaxed(r + WAKEUP_REASON));
		seq_printf(s, "\tlast_sleep_transition_duration: %u\n",
			   readl_relaxed(r + LAST_SLEEP_DURATION));
		seq_printf(s, "\tlast_wake_transition_duration: %u\n",
			   readl_relaxed(r + LAST_WAKE_DURATION));
		seq_printf(s, "\txo_count: %u\n", readl_relaxed(r + XO_COUNT));
		seq_printf(s, "\txo_last_entered_at: %llu\n", rd64(r + XO_LAST_ENTERED));
		seq_printf(s, "\txo_last_exited_at: %llu\n", rd64(r + XO_LAST_EXITED));
		seq_printf(s, "\txo_accumulated_duration: %llu\n", rd64(r + XO_ACCUMULATED));
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(stats);

static int raw_show(struct seq_file *s, void *unused)
{
	int i, w;

	for (i = 0; i < ARRAY_SIZE(masters); i++) {
		seq_printf(s, "%s @%#lx:", masters[i], base + i * stride);
		for (w = 0; w < RECORD_LEN; w += 4)
			seq_printf(s, "%s%08x", (w % 32) ? " " : "\n\t",
				   readl_relaxed(regs + i * stride + w));
		seq_puts(s, "\n");
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(raw);

static int __init qcom_rpm_master_stats_init(void)
{
	regs = ioremap(base, (ARRAY_SIZE(masters) - 1) * stride + RECORD_LEN);
	if (!regs)
		return -ENOMEM;

	root = debugfs_create_dir("qcom_rpm_master_stats", NULL);
	debugfs_create_file("stats", 0400, root, NULL, &stats_fops);
	debugfs_create_file("raw", 0400, root, NULL, &raw_fops);

	return 0;
}
module_init(qcom_rpm_master_stats_init);

static void __exit qcom_rpm_master_stats_exit(void)
{
	debugfs_remove_recursive(root);
	iounmap(regs);
}
module_exit(qcom_rpm_master_stats_exit);

MODULE_DESCRIPTION("Qualcomm RPM per-master sleep statistics (debugfs)");
MODULE_LICENSE("GPL");
