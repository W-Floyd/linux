// SPDX-License-Identifier: GPL-2.0-only
/*
 * TCC803x Cortex-M4. The bootloader starts the early rear-camera firmware on
 * it on every normal boot and leaves it running into the kernel, where it
 * keeps rewriting the display WMIX0 and owns I2C3 and the decoder's reset
 * line, all unlocked. Stop it once, the way stock userspace does: a STOP
 * message (the firmware tears its camera path down, hides the overlay and
 * acks), then hold the core in reset.
 *
 * The same core can instead run other firmware (the SoC's secure-processor
 * firmware, loaded before the bootloader, which overwrites it with the
 * camera only on boots that start the camera). Anything not identified as
 * the early camera is left alone.
 *
 * Built in, so it runs before the camera drivers (modules) touch those blocks.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>

/* The A53 side of the A53 <-> M4 mailbox. */
#define MBOX_TX(n)		(0x00 + 4 * (n))
#define MBOX_RX(n)		(0x20 + 4 * (n))
#define MBOX_CTL		0x40
#define MBOX_CTL_OEN		BIT(5)	/* raise: the message is complete */
#define MBOX_STS		0x44
#define MBOX_STS_SCOUNT		GENMASK(23, 20)
#define MBOX_WORDS		8

/*
 * cmd, six data words, preamble. The reply is cmd | ACK; STOP's data[0] is
 * always 0 (the firmware hides the overlay before filling it in).
 */
#define MSG_STOP		0x51
#define MSG_ACK			0x10000000
#define MSG_PREAMBLE		0x0043414d

#define CFG_RESET		0x18
#define CFG_RESET_HOLD		(BIT(1) | BIT(2))	/* POR, SYS; both set = held */

/* A log string of the early-camera firmware, naming the command sent below. */
static const char cm4_camera_marker[] = "MAILBOX_MSG_EARLYCAMERA_STOP";

static bool cm4_is_camera(void __iomem *code, size_t size)
{
	size_t n = sizeof(cm4_camera_marker) - 1, i;
	bool found = false;
	u8 *buf;

	buf = kmalloc(size, GFP_KERNEL);
	if (!buf)
		return false;
	memcpy_fromio(buf, code, size);
	for (i = 0; i + n <= size && !found; i++)
		found = !memcmp(buf + i, cm4_camera_marker, n);
	kfree(buf);

	return found;
}

static bool cm4_rx(void __iomem *mbox, u32 *msg)
{
	int i;

	if (FIELD_GET(MBOX_STS_SCOUNT, readl(mbox + MBOX_STS)) != MBOX_WORDS)
		return false;
	for (i = 0; i < MBOX_WORDS; i++)
		msg[i] = readl(mbox + MBOX_RX(i));
	return true;
}

/* The firmware polls its mailbox from a busy loop, so no interrupt either way. */
static int cm4_stop_camera(struct device *dev, void __iomem *mbox)
{
	unsigned long timeout;
	u32 msg[MBOX_WORDS];
	int i;

	/* Drain a stale message so the reply matched below is to this one. */
	cm4_rx(mbox, msg);

	writel(readl(mbox + MBOX_CTL) & ~MBOX_CTL_OEN, mbox + MBOX_CTL);
	writel(MSG_STOP, mbox + MBOX_TX(0));
	for (i = 1; i < MBOX_WORDS - 1; i++)
		writel(0, mbox + MBOX_TX(i));
	writel(MSG_PREAMBLE, mbox + MBOX_TX(MBOX_WORDS - 1));
	writel(readl(mbox + MBOX_CTL) | MBOX_CTL_OEN, mbox + MBOX_CTL);

	timeout = jiffies + msecs_to_jiffies(1000);
	do {
		if (cm4_rx(mbox, msg) && msg[0] == (MSG_STOP | MSG_ACK)) {
			dev_info(dev, "early camera stopped\n");
			return 0;
		}
		usleep_range(1000, 2000);
	} while (time_before(jiffies, timeout));

	return -ETIMEDOUT;
}

static int cm4_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	void __iomem *mbox, *cfg, *code;
	struct resource *code_res;
	struct clk *clk;
	u32 reset;

	mbox = devm_platform_ioremap_resource_byname(pdev, "mbox");
	if (IS_ERR(mbox))
		return PTR_ERR(mbox);
	cfg = devm_platform_ioremap_resource_byname(pdev, "cfg");
	if (IS_ERR(cfg))
		return PTR_ERR(cfg);
	code_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "code");
	code = devm_ioremap_resource(dev, code_res);
	if (IS_ERR(code))
		return PTR_ERR(code);

	clk = devm_clk_get(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "bus clock\n");

	/*
	 * Stock gates the bus clock after stopping the core, and the registers
	 * behind it can't be read gated. The clock is the monitor's; only ask.
	 */
	if (!__clk_is_enabled(clk)) {
		dev_info(dev, "bus clock off: not running\n");
		return 0;
	}

	reset = readl(cfg + CFG_RESET);
	if ((reset & CFG_RESET_HOLD) == CFG_RESET_HOLD) {
		dev_info(dev, "held in reset: not running\n");
		return 0;
	}

	if (!cm4_is_camera(code, resource_size(code_res))) {
		dev_info(dev, "running firmware that isn't the early camera: left alone\n");
		return 0;
	}

	if (cm4_stop_camera(dev, mbox))
		dev_warn(dev, "no reply to STOP; resetting it, its camera overlay may stay up\n");

	writel(reset | CFG_RESET_HOLD, cfg + CFG_RESET);
	return 0;
}

static const struct of_device_id cm4_of_match[] = {
	{ .compatible = "telechips,tcc803x-cm4" },
	{ }
};

static struct platform_driver cm4_driver = {
	.probe	= cm4_probe,
	.driver	= {
		.name			= "tcc803x-cm4",
		.of_match_table		= cm4_of_match,
		.suppress_bind_attrs	= true,
	},
};
builtin_platform_driver(cm4_driver);
