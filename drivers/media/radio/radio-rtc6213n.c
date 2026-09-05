// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for the Richwave RTC6213N FM radio receiver.
 *
 * The part has no public documentation. Everything here that is not plain
 * V4L2 plumbing comes from the vendor driver Samsung ships in its Android
 * kernels, which is itself a fork of the Si470x driver -- the I2C register
 * window works exactly the same way, so the structure of this driver follows
 * radio-si470x-i2c.c closely.
 *
 * Copyright (c) 2009 Tobias Lorenz <tobias.lorenz@gmx.net>
 * Copyright (c) 2012 Hans de Goede <hdegoede@redhat.com>
 * Copyright (c) 2013 Richwave Technology Co.Ltd
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/videodev2.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>

#define DRIVER_NAME		"rtc6213n"

/* Register numbers */
#define RTC6213N_DEVICEID	0
#define RTC6213N_CHIPID		1
#define RTC6213N_MPXCFG		2
#define RTC6213N_CHANNEL	3
#define RTC6213N_SYSCFG		4
#define RTC6213N_SEEKCFG1	5
#define RTC6213N_POWERCFG	6
#define RTC6213N_PADCFG		7
#define RTC6213N_BANKCFG	8
#define RTC6213N_SEEKCFG2	9
#define RTC6213N_STATUS		10
#define RTC6213N_RSSI		11
#define RTC6213N_BA_DATA	12
#define RTC6213N_BB_DATA	13
#define RTC6213N_BC_DATA	14
#define RTC6213N_BD_DATA	15
#define RTC6213N_NR_REGS	16

/* MPXCFG */
#define MPXCFG_DIS_SMUTE	0x8000
#define MPXCFG_DIS_MUTE		0x4000
#define MPXCFG_MONO		0x2000
#define MPXCFG_DEEM		0x1000
#define MPXCFG_VOLUME		0x000f

/* CHANNEL */
#define CHANNEL_TUNE		0x8000
#define CHANNEL_BAND		0x3000
#define CHANNEL_BAND_SHIFT	12
#define CHANNEL_CHSPACE		0x0c00
#define CHANNEL_CHSPACE_SHIFT	10
#define CHANNEL_CH		0x03ff

/* SYSCFG */
#define SYSCFG_RDSIRQEN		0x8000
#define SYSCFG_STDIRQEN		0x4000
#define SYSCFG_DIS_AGC		0x2000
#define SYSCFG_RDS_EN		0x1000

/* SEEKCFG1 */
#define SEEKCFG1_SEEK		0x8000
#define SEEKCFG1_SEEKUP		0x4000
#define SEEKCFG1_SKMODE		0x2000
#define SEEKCFG1_SEEKRSSITH	0x00ff

/* POWERCFG */
#define POWERCFG_ENABLE		0x8000
#define POWERCFG_DISABLE	0x4000

/* STATUS */
#define STATUS_RDS_RDY		0x8000
#define STATUS_STD		0x4000
#define STATUS_SF		0x2000
#define STATUS_RDS_SYNC		0x0800
#define STATUS_SI		0x0400
#define STATUS_READCH		0x03ff

/* RSSI */
#define RSSI_RDS_BA_ERRS	0xc000
#define RSSI_RDS_BB_ERRS	0x3000
#define RSSI_RDS_BC_ERRS	0x0c00
#define RSSI_RDS_BD_ERRS	0x0300
#define RSSI_RSSI		0x00ff

/*
 * A read starts at the upper byte of STATUS and wraps; a write starts at the
 * upper byte of MPXCFG and wraps. No register address is transferred.
 */
#define READ_INDEX(i)	(((i) + RTC6213N_NR_REGS - RTC6213N_STATUS) % RTC6213N_NR_REGS)
#define WRITE_INDEX(i)	(((i) + RTC6213N_MPXCFG) % RTC6213N_NR_REGS)

/* Frequencies are handled internally in kHz. */
#define FREQ_MUL	16			/* V4L2 works in 62.5 Hz units */
#define SEEK_TIMEOUT_MS	5000
#define TUNE_TIMEOUT_MS	1000

#define RDS_BUFFER_BLOCKS	100
#define RDS_BLOCK_BYTES		3

#define RTC6213N_BANK_WORDS	23

/*
 * Undocumented power-up sequence taken verbatim from the vendor driver. Each
 * array is one register bank; word 6 selects the bank. These are chip
 * constants rather than board tuning: they are byte-identical in the Samsung
 * trees for the Galaxy S7 (Exynos 8890), S8+ (MSM8998) and S9 (SDM845), and
 * the Note 10 (Exynos 9825) differs only in bank 5 word 10. There is no
 * datasheet describing them; do not "clean them up".
 */
static const u16 rtc6213n_banks[][RTC6213N_BANK_WORDS] = {
	{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x7000, 0x4000,
	  0x1A08, 0x0100, 0x0740, 0x0040, 0x005A, 0x02C0, 0x0000,
	  0x1440, 0x0080, 0x0840, 0x0000, 0x4002, 0x805A, 0x0D35,
	  0x7367, 0x0000 },
	{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x7000, 0x8000,
	  0x0000, 0x0000, 0x0333, 0x051C, 0x01EB, 0x01EB, 0x0333,
	  0xF2AB, 0x7F8A, 0x0780, 0x0000, 0x1400, 0x405A, 0x0000,
	  0x3200, 0x0000 },
	{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x7000, 0xC000,
	  0x188F, 0x9628, 0x4040, 0x80FF, 0xCFB0, 0x06F6, 0x0D40,
	  0x0998, 0xC61F, 0x7126, 0x3F4B, 0xEED7, 0xB599, 0x674E,
	  0x3112, 0x0000 },
	{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x7000, 0x2000,
	  0x050F, 0x0E85, 0x5AA6, 0xDC57, 0x8000, 0x00A3, 0x00A3,
	  0xC018, 0x7F80, 0x3C08, 0xB6CF, 0x8100, 0x0000, 0x0140,
	  0x4700, 0x0000 },
	{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x7000, 0x6000,
	  0x3590, 0x6311, 0x3008, 0x0019, 0x0D79, 0x7D2F, 0x8000,
	  0x02A1, 0x771F, 0x323E, 0x262E, 0xA516, 0x8680, 0x0000,
	  0x0000, 0x0000 },
	{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x7000, 0xE000,
	  0x11A2, 0x0F92, 0x0000, 0x0000, 0x0000, 0x0000, 0x801D,
	  0x0000, 0x0000, 0x0072, 0x00FF, 0x001F, 0x03FF, 0x16D1,
	  0x13B7, 0x0000 },
};

/* Seek quality thresholds; the vendor default, which is what ships. */
#define RTC6213N_SEEKCFG2_DEFAULT	0x4050

struct rtc6213n_device {
	struct v4l2_device v4l2_dev;
	struct video_device videodev;
	struct v4l2_ctrl_handler ctrl_handler;
	struct i2c_client *client;

	struct mutex lock;		/* serialises register access */
	u16 registers[RTC6213N_NR_REGS];

	/* RDS receive buffer, three bytes per block */
	wait_queue_head_t read_queue;
	unsigned char *buffer;
	unsigned int buf_size;
	unsigned int rd_index;
	unsigned int wr_index;

	struct completion completion;
	bool stci_enabled;

	struct clk_bulk_data *clks;
	int num_clks;
};

static inline struct rtc6213n_device *to_rtc6213n(struct v4l2_device *v4l2_dev)
{
	return container_of(v4l2_dev, struct rtc6213n_device, v4l2_dev);
}

/*
 * Register access
 */
static int rtc6213n_read_raw(struct rtc6213n_device *radio, u16 *out)
{
	__be16 buf[RTC6213N_NR_REGS];
	struct i2c_msg msg = {
		.addr = radio->client->addr,
		.flags = I2C_M_RD,
		.len = sizeof(buf),
		.buf = (void *)buf,
	};
	int i;

	if (i2c_transfer(radio->client->adapter, &msg, 1) != 1)
		return -EIO;

	for (i = 0; i < RTC6213N_NR_REGS; i++)
		out[i] = be16_to_cpu(buf[READ_INDEX(i)]);

	return 0;
}

/*
 * Refresh the read-only half of the shadow: the ID pair and everything from
 * STATUS up. The configuration registers (MPXCFG..SEEKCFG2) are deliberately
 * left alone -- the driver owns those, every write pushes all 16 out of the
 * shadow, and letting a read overwrite them means a part that answers with
 * all-ones would silently corrupt its own configuration.
 */
static int rtc6213n_get_all_registers(struct rtc6213n_device *radio)
{
	u16 regs[RTC6213N_NR_REGS];
	int i, retval;

	retval = rtc6213n_read_raw(radio, regs);
	if (retval < 0)
		return retval;

	radio->registers[RTC6213N_DEVICEID] = regs[RTC6213N_DEVICEID];
	radio->registers[RTC6213N_CHIPID] = regs[RTC6213N_CHIPID];
	for (i = RTC6213N_STATUS; i < RTC6213N_NR_REGS; i++)
		radio->registers[i] = regs[i];

	return 0;
}

static int rtc6213n_get_register(struct rtc6213n_device *radio, int regnr)
{
	__be16 buf[RTC6213N_NR_REGS];
	struct i2c_msg msg = {
		.addr = radio->client->addr,
		.flags = I2C_M_RD,
		.len = sizeof(buf),
		.buf = (void *)buf,
	};

	if (i2c_transfer(radio->client->adapter, &msg, 1) != 1)
		return -EIO;

	radio->registers[regnr] = be16_to_cpu(buf[READ_INDEX(regnr)]);

	return 0;
}

static int rtc6213n_set_register(struct rtc6213n_device *radio, int regnr)
{
	__be16 buf[RTC6213N_NR_REGS];
	struct i2c_msg msg = {
		.addr = radio->client->addr,
		.flags = 0,
		.len = sizeof(buf),
		.buf = (void *)buf,
	};
	int i;

	for (i = 0; i < RTC6213N_NR_REGS; i++)
		buf[i] = cpu_to_be16(radio->registers[WRITE_INDEX(i)]);

	if (i2c_transfer(radio->client->adapter, &msg, 1) != 1)
		return -EIO;

	return 0;
}

static int rtc6213n_write_bank(struct rtc6213n_device *radio, const u16 *bank)
{
	__be16 buf[RTC6213N_BANK_WORDS];
	struct i2c_msg msg = {
		.addr = radio->client->addr,
		.flags = 0,
		.len = sizeof(buf),
		.buf = (void *)buf,
	};
	int i;

	for (i = 0; i < RTC6213N_BANK_WORDS; i++)
		buf[i] = cpu_to_be16(bank[i]);

	if (i2c_transfer(radio->client->adapter, &msg, 1) != 1)
		return -EIO;

	return 0;
}

/*
 * Tuning
 */
static unsigned int rtc6213n_band_bottom(struct rtc6213n_device *radio)
{
	switch ((radio->registers[RTC6213N_CHANNEL] & CHANNEL_BAND) >>
		CHANNEL_BAND_SHIFT) {
	case 0:
		return 87500;	/* 87.5 - 108 MHz */
	default:
		return 76000;	/* 76 - 108 MHz, and 76 - 90 MHz */
	}
}

static unsigned int rtc6213n_band_top(struct rtc6213n_device *radio)
{
	switch ((radio->registers[RTC6213N_CHANNEL] & CHANNEL_BAND) >>
		CHANNEL_BAND_SHIFT) {
	case 2:
		return 90000;
	default:
		return 108000;
	}
}

static unsigned int rtc6213n_spacing(struct rtc6213n_device *radio)
{
	switch ((radio->registers[RTC6213N_CHANNEL] & CHANNEL_CHSPACE) >>
		CHANNEL_CHSPACE_SHIFT) {
	case 0:
		return 200;
	case 1:
		return 100;
	default:
		return 50;
	}
}

/* Wait for a tune or seek to raise STD, via the interrupt if we have one. */
static int rtc6213n_wait_complete(struct rtc6213n_device *radio,
				  unsigned int timeout_ms)
{
	unsigned long timeout;
	int retval;

	if (radio->stci_enabled) {
		if (!wait_for_completion_timeout(&radio->completion,
						 msecs_to_jiffies(timeout_ms)))
			return -ETIMEDOUT;
		return 0;
	}

	timeout = jiffies + msecs_to_jiffies(timeout_ms);
	do {
		retval = rtc6213n_get_all_registers(radio);
		if (retval < 0)
			return retval;
		if (radio->registers[RTC6213N_STATUS] & STATUS_STD)
			return 0;
		usleep_range(5000, 6000);
	} while (time_before(jiffies, timeout));

	return -ETIMEDOUT;
}

static int rtc6213n_set_chan(struct rtc6213n_device *radio, u16 chan)
{
	int retval;

	reinit_completion(&radio->completion);

	radio->registers[RTC6213N_CHANNEL] &= ~CHANNEL_CH;
	radio->registers[RTC6213N_CHANNEL] |= CHANNEL_TUNE | chan;
	retval = rtc6213n_set_register(radio, RTC6213N_CHANNEL);
	if (retval < 0)
		return retval;

	retval = rtc6213n_wait_complete(radio, TUNE_TIMEOUT_MS);
	if (retval < 0)
		dev_dbg(&radio->client->dev, "tune did not complete\n");

	/* Clear TUNE either way, so the part is left in a sane state. */
	radio->registers[RTC6213N_CHANNEL] &= ~CHANNEL_TUNE;
	if (rtc6213n_set_register(radio, RTC6213N_CHANNEL) < 0)
		return -EIO;

	return retval;
}

static int rtc6213n_set_freq(struct rtc6213n_device *radio, unsigned int freq)
{
	unsigned int bottom = rtc6213n_band_bottom(radio);
	unsigned int top = rtc6213n_band_top(radio);

	freq = clamp(freq, bottom, top);

	return rtc6213n_set_chan(radio,
				 (freq - bottom) / rtc6213n_spacing(radio));
}

static unsigned int rtc6213n_get_freq(struct rtc6213n_device *radio)
{
	u16 chan = radio->registers[RTC6213N_STATUS] & STATUS_READCH;

	return rtc6213n_band_bottom(radio) + chan * rtc6213n_spacing(radio);
}

/*
 * Power up / down
 */
/*
 * BRING-UP DIAGNOSTIC -- remove before submission. Dumps the part's real
 * register contents at each stage of power-up so we can tell a part that is
 * ignoring writes outright from one whose core is simply not clocked.
 */
static void rtc6213n_dump(struct rtc6213n_device *radio, const char *stage)
{
	u16 regs[RTC6213N_NR_REGS];

	if (rtc6213n_read_raw(radio, regs) < 0) {
		dev_info(&radio->client->dev, "%s: read failed\n", stage);
		return;
	}

	dev_info(&radio->client->dev,
		 "%s: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x\n",
		 stage, regs[0], regs[1], regs[2], regs[3], regs[4], regs[5],
		 regs[6], regs[7], regs[8], regs[9], regs[10], regs[11],
		 regs[12], regs[13], regs[14], regs[15]);
}

static int rtc6213n_start(struct rtc6213n_device *radio)
{
	u16 regs[RTC6213N_NR_REGS];
	int retval, i;

	/*
	 * Release the part from any unexpected I2C start condition and unlock
	 * the register banks. Nothing may be read between these two writes.
	 */
	rtc6213n_dump(radio, "power-on  ");

	radio->registers[RTC6213N_DEVICEID] = 0x16AA;
	retval = rtc6213n_set_register(radio, RTC6213N_DEVICEID);
	if (retval < 0)
		return retval;
	msleep(30);

	radio->registers[RTC6213N_DEVICEID] = 0x96AA;
	retval = rtc6213n_set_register(radio, RTC6213N_DEVICEID);
	if (retval < 0)
		return retval;
	msleep(30);

	/*
	 * DEVICEID is writable -- it is the unlock register. If it reads back
	 * as 0x96AA the part is latching writes and only the core is dead; if
	 * it still reads its power-on value the part is ignoring writes
	 * altogether, which would point at power or reset rather than a clock.
	 */
	rtc6213n_dump(radio, "post-unlk ");

	retval = rtc6213n_get_all_registers(radio);
	if (retval < 0)
		return retval;

	for (i = 0; i < ARRAY_SIZE(rtc6213n_banks); i++) {
		retval = rtc6213n_write_bank(radio, rtc6213n_banks[i]);
		if (retval < 0)
			return retval;
	}

	rtc6213n_dump(radio, "post-bank ");

	retval = rtc6213n_get_all_registers(radio);
	if (retval < 0)
		return retval;

	/*
	 * Every write pushes all 16 registers out of the shadow, and until the
	 * part's core comes up the shadow reads back as all-ones. So build the
	 * whole writable set from known values and push it in one transfer --
	 * a series of read-modify-writes would keep feeding those all-ones bits
	 * back into the part (POWERCFG would get ENABLE and DISABLE at once).
	 */
	radio->registers[RTC6213N_MPXCFG] = MPXCFG_DIS_SMUTE | MPXCFG_DIS_MUTE |
					    8;	/* mid volume */
	/* 87.5-108 MHz, 100 kHz spacing, channel 0x1a as the vendor does */
	radio->registers[RTC6213N_CHANNEL] = (1 << CHANNEL_CHSPACE_SHIFT) | 0x1a;
	radio->registers[RTC6213N_SYSCFG] = SYSCFG_RDS_EN;
	if (radio->stci_enabled)
		radio->registers[RTC6213N_SYSCFG] |= SYSCFG_STDIRQEN |
						     SYSCFG_RDSIRQEN;
	/* Seek action bits clear; the low byte is the RSSI threshold. */
	radio->registers[RTC6213N_SEEKCFG1] = 0x0020;
	/* PADCFG[3:2] = 1 routes the seek/tune and RDS interrupt to the pin. */
	radio->registers[RTC6213N_PADCFG] = 1 << 2;
	radio->registers[RTC6213N_SEEKCFG2] = RTC6213N_SEEKCFG2_DEFAULT;
	radio->registers[RTC6213N_POWERCFG] = POWERCFG_ENABLE;

	retval = rtc6213n_set_register(radio, RTC6213N_POWERCFG);
	if (retval < 0)
		return retval;

	/* Give the 32.768 kHz reference time to settle, as si470x does. */
	msleep(110);

	/*
	 * Read the part directly rather than through the shadow, so the dump
	 * shows what the hardware actually latched.
	 */
	retval = rtc6213n_read_raw(radio, regs);
	if (retval < 0)
		return retval;

	dev_info(&radio->client->dev,
		 "after power-up: MPXCFG %04x CHANNEL %04x SYSCFG %04x SEEKCFG1 %04x POWERCFG %04x PADCFG %04x STATUS %04x RSSI %04x\n",
		 regs[RTC6213N_MPXCFG], regs[RTC6213N_CHANNEL],
		 regs[RTC6213N_SYSCFG], regs[RTC6213N_SEEKCFG1],
		 regs[RTC6213N_POWERCFG], regs[RTC6213N_PADCFG],
		 regs[RTC6213N_STATUS], regs[RTC6213N_RSSI]);

	/*
	 * BRING-UP EXPERIMENT -- remove before submission. PADCFG[15:12] is
	 * documented in the vendor header only as "internal crystal RC
	 * enable". If the core is dead because the external 32.768 kHz
	 * reference is not reaching the part, running it from its own
	 * oscillator should bring the registers to life; if nothing changes,
	 * the clock is not the problem.
	 */
	if (regs[RTC6213N_POWERCFG] == 0xffff) {
		dev_warn(&radio->client->dev,
			 "config registers read back as all-ones: the part is not latching writes\n");

		radio->registers[RTC6213N_PADCFG] = 0xf000 | (1 << 2);
		retval = rtc6213n_set_register(radio, RTC6213N_PADCFG);
		if (retval < 0)
			return retval;
		msleep(60);
		rtc6213n_dump(radio, "post-RC   ");

		radio->registers[RTC6213N_POWERCFG] = POWERCFG_ENABLE;
		retval = rtc6213n_set_register(radio, RTC6213N_POWERCFG);
		if (retval < 0)
			return retval;
		msleep(60);
		rtc6213n_dump(radio, "post-RC-en");
	}

	radio->registers[RTC6213N_STATUS] = regs[RTC6213N_STATUS];
	radio->registers[RTC6213N_RSSI] = regs[RTC6213N_RSSI];

	return 0;
}

static int rtc6213n_stop(struct rtc6213n_device *radio)
{
	int retval;

	radio->registers[RTC6213N_SYSCFG] &= ~(SYSCFG_RDS_EN | SYSCFG_STDIRQEN |
					       SYSCFG_RDSIRQEN);
	retval = rtc6213n_set_register(radio, RTC6213N_SYSCFG);
	if (retval < 0)
		return retval;

	radio->registers[RTC6213N_POWERCFG] &= ~POWERCFG_ENABLE;
	radio->registers[RTC6213N_POWERCFG] |= POWERCFG_DISABLE;
	return rtc6213n_set_register(radio, RTC6213N_POWERCFG);
}

/*
 * RDS
 */
static void rtc6213n_rds_on(struct rtc6213n_device *radio)
{
	unsigned char blocknum, tmpbuf[RDS_BLOCK_BYTES];
	u16 rds, bler;

	for (blocknum = 0; blocknum < 4; blocknum++) {
		switch (blocknum) {
		case 0:
			bler = (radio->registers[RTC6213N_RSSI] &
				RSSI_RDS_BA_ERRS) >> 14;
			rds = radio->registers[RTC6213N_BA_DATA];
			break;
		case 1:
			bler = (radio->registers[RTC6213N_RSSI] &
				RSSI_RDS_BB_ERRS) >> 12;
			rds = radio->registers[RTC6213N_BB_DATA];
			break;
		case 2:
			bler = (radio->registers[RTC6213N_RSSI] &
				RSSI_RDS_BC_ERRS) >> 10;
			rds = radio->registers[RTC6213N_BC_DATA];
			break;
		default:
			bler = (radio->registers[RTC6213N_RSSI] &
				RSSI_RDS_BD_ERRS) >> 8;
			rds = radio->registers[RTC6213N_BD_DATA];
			break;
		}

		tmpbuf[0] = rds & 0xff;
		tmpbuf[1] = (rds >> 8) & 0xff;
		/* Low bits name the block, bits 3..5 the offset received. */
		tmpbuf[2] = (blocknum & V4L2_RDS_BLOCK_MSK) | (blocknum << 3);
		/* bler counts the errors needing correction; 3 means it failed. */
		if (bler == 3)
			tmpbuf[2] |= V4L2_RDS_BLOCK_ERROR;
		else if (bler > 0)
			tmpbuf[2] |= V4L2_RDS_BLOCK_CORRECTED;

		/* Drop the oldest group if userspace is not keeping up. */
		if ((radio->wr_index + RDS_BLOCK_BYTES) % radio->buf_size ==
		    radio->rd_index)
			break;

		memcpy(&radio->buffer[radio->wr_index], tmpbuf,
		       RDS_BLOCK_BYTES);
		radio->wr_index += RDS_BLOCK_BYTES;
		if (radio->wr_index >= radio->buf_size)
			radio->wr_index = 0;
	}

	if (radio->wr_index != radio->rd_index)
		wake_up_interruptible(&radio->read_queue);
}

static irqreturn_t rtc6213n_i2c_interrupt(int irq, void *dev_id)
{
	struct rtc6213n_device *radio = dev_id;

	mutex_lock(&radio->lock);

	if (rtc6213n_get_all_registers(radio) < 0)
		goto out;

	if (radio->registers[RTC6213N_STATUS] & STATUS_STD)
		complete(&radio->completion);

	if ((radio->registers[RTC6213N_SYSCFG] & SYSCFG_RDS_EN) &&
	    (radio->registers[RTC6213N_STATUS] & STATUS_RDS_RDY))
		rtc6213n_rds_on(radio);

out:
	mutex_unlock(&radio->lock);
	return IRQ_HANDLED;
}

/*
 * File operations
 */
static ssize_t rtc6213n_fops_read(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct rtc6213n_device *radio = video_drvdata(file);
	unsigned int block_count;
	int retval = 0;

	/* Whole blocks only. */
	count /= RDS_BLOCK_BYTES;
	if (!count)
		return -EINVAL;

	if (mutex_lock_interruptible(&radio->lock))
		return -ERESTARTSYS;

	while (radio->rd_index == radio->wr_index) {
		if (file->f_flags & O_NONBLOCK) {
			retval = -EWOULDBLOCK;
			goto done;
		}
		mutex_unlock(&radio->lock);
		if (wait_event_interruptible(radio->read_queue,
					     radio->rd_index != radio->wr_index))
			return -EINTR;
		if (mutex_lock_interruptible(&radio->lock))
			return -ERESTARTSYS;
	}

	block_count = radio->wr_index - radio->rd_index;
	if (radio->wr_index < radio->rd_index)
		block_count += radio->buf_size;
	block_count /= RDS_BLOCK_BYTES;
	block_count = min(block_count, (unsigned int)count);

	while (block_count--) {
		if (copy_to_user(buf, &radio->buffer[radio->rd_index],
				 RDS_BLOCK_BYTES)) {
			retval = -EFAULT;
			goto done;
		}
		radio->rd_index += RDS_BLOCK_BYTES;
		if (radio->rd_index >= radio->buf_size)
			radio->rd_index = 0;
		buf += RDS_BLOCK_BYTES;
		retval += RDS_BLOCK_BYTES;
	}

done:
	mutex_unlock(&radio->lock);
	return retval;
}

static __poll_t rtc6213n_fops_poll(struct file *file,
				   struct poll_table_struct *pts)
{
	struct rtc6213n_device *radio = video_drvdata(file);
	__poll_t req_events = poll_requested_events(pts);
	__poll_t retval = v4l2_ctrl_poll(file, pts);

	if (!(req_events & (EPOLLIN | EPOLLRDNORM)))
		return retval;

	poll_wait(file, &radio->read_queue, pts);
	if (radio->rd_index != radio->wr_index)
		retval |= EPOLLIN | EPOLLRDNORM;

	return retval;
}

static const struct v4l2_file_operations rtc6213n_fops = {
	.owner		= THIS_MODULE,
	.read		= rtc6213n_fops_read,
	.poll		= rtc6213n_fops_poll,
	.unlocked_ioctl	= video_ioctl2,
	.open		= v4l2_fh_open,
	.release	= v4l2_fh_release,
};

/*
 * Controls
 */
static int rtc6213n_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct rtc6213n_device *radio =
		container_of(ctrl->handler, struct rtc6213n_device,
			     ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_AUDIO_VOLUME:
		radio->registers[RTC6213N_MPXCFG] &= ~MPXCFG_VOLUME;
		radio->registers[RTC6213N_MPXCFG] |= ctrl->val;
		return rtc6213n_set_register(radio, RTC6213N_MPXCFG);
	case V4L2_CID_AUDIO_MUTE:
		if (ctrl->val)
			radio->registers[RTC6213N_MPXCFG] &= ~MPXCFG_DIS_MUTE;
		else
			radio->registers[RTC6213N_MPXCFG] |= MPXCFG_DIS_MUTE;
		return rtc6213n_set_register(radio, RTC6213N_MPXCFG);
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops rtc6213n_ctrl_ops = {
	.s_ctrl = rtc6213n_s_ctrl,
};

/*
 * ioctls
 */
static int rtc6213n_vidioc_querycap(struct file *file, void *priv,
				    struct v4l2_capability *capability)
{
	strscpy(capability->driver, DRIVER_NAME, sizeof(capability->driver));
	strscpy(capability->card, "Richwave RTC6213N FM Receiver",
		sizeof(capability->card));

	return 0;
}

static int rtc6213n_vidioc_g_tuner(struct file *file, void *priv,
				   struct v4l2_tuner *tuner)
{
	struct rtc6213n_device *radio = video_drvdata(file);
	int retval;

	if (tuner->index)
		return -EINVAL;

	retval = rtc6213n_get_all_registers(radio);
	if (retval < 0)
		return retval;

	strscpy(tuner->name, "FM", sizeof(tuner->name));
	tuner->type = V4L2_TUNER_RADIO;
	tuner->capability = V4L2_TUNER_CAP_LOW | V4L2_TUNER_CAP_STEREO |
			    V4L2_TUNER_CAP_RDS | V4L2_TUNER_CAP_RDS_BLOCK_IO |
			    V4L2_TUNER_CAP_HWSEEK_BOUNDED |
			    V4L2_TUNER_CAP_HWSEEK_WRAP;

	tuner->rangelow = rtc6213n_band_bottom(radio) * FREQ_MUL;
	tuner->rangehigh = rtc6213n_band_top(radio) * FREQ_MUL;

	tuner->rxsubchans = V4L2_TUNER_SUB_MONO;
	if (radio->registers[RTC6213N_STATUS] & STATUS_SI)
		tuner->rxsubchans |= V4L2_TUNER_SUB_STEREO;
	if (radio->registers[RTC6213N_STATUS] & STATUS_RDS_SYNC)
		tuner->rxsubchans |= V4L2_TUNER_SUB_RDS;

	tuner->audmode = (radio->registers[RTC6213N_MPXCFG] & MPXCFG_MONO) ?
			 V4L2_TUNER_MODE_MONO : V4L2_TUNER_MODE_STEREO;

	/* The part reports RSSI in dBuV; scale it to the V4L2 0..65535 range. */
	tuner->signal = (radio->registers[RTC6213N_RSSI] & RSSI_RSSI) * 257;
	tuner->afc = 0;

	return 0;
}

static int rtc6213n_vidioc_s_tuner(struct file *file, void *priv,
				   const struct v4l2_tuner *tuner)
{
	struct rtc6213n_device *radio = video_drvdata(file);

	if (tuner->index)
		return -EINVAL;

	if (tuner->audmode == V4L2_TUNER_MODE_MONO)
		radio->registers[RTC6213N_MPXCFG] |= MPXCFG_MONO;
	else
		radio->registers[RTC6213N_MPXCFG] &= ~MPXCFG_MONO;

	return rtc6213n_set_register(radio, RTC6213N_MPXCFG);
}

static int rtc6213n_vidioc_g_frequency(struct file *file, void *priv,
				       struct v4l2_frequency *freq)
{
	struct rtc6213n_device *radio = video_drvdata(file);
	int retval;

	if (freq->tuner)
		return -EINVAL;

	retval = rtc6213n_get_register(radio, RTC6213N_STATUS);
	if (retval < 0)
		return retval;

	freq->type = V4L2_TUNER_RADIO;
	freq->frequency = rtc6213n_get_freq(radio) * FREQ_MUL;

	return 0;
}

static int rtc6213n_vidioc_s_frequency(struct file *file, void *priv,
				       const struct v4l2_frequency *freq)
{
	struct rtc6213n_device *radio = video_drvdata(file);

	if (freq->tuner)
		return -EINVAL;

	return rtc6213n_set_freq(radio, freq->frequency / FREQ_MUL);
}

static int rtc6213n_vidioc_s_hw_freq_seek(struct file *file, void *priv,
					  const struct v4l2_hw_freq_seek *seek)
{
	struct rtc6213n_device *radio = video_drvdata(file);
	int retval;

	if (seek->tuner)
		return -EINVAL;
	if (file->f_flags & O_NONBLOCK)
		return -EWOULDBLOCK;

	reinit_completion(&radio->completion);

	if (seek->seek_upward)
		radio->registers[RTC6213N_SEEKCFG1] |= SEEKCFG1_SEEKUP;
	else
		radio->registers[RTC6213N_SEEKCFG1] &= ~SEEKCFG1_SEEKUP;

	if (seek->wrap_around)
		radio->registers[RTC6213N_SEEKCFG1] &= ~SEEKCFG1_SKMODE;
	else
		radio->registers[RTC6213N_SEEKCFG1] |= SEEKCFG1_SKMODE;

	radio->registers[RTC6213N_SEEKCFG1] |= SEEKCFG1_SEEK;
	retval = rtc6213n_set_register(radio, RTC6213N_SEEKCFG1);
	if (retval < 0)
		return retval;

	retval = rtc6213n_wait_complete(radio, SEEK_TIMEOUT_MS);

	radio->registers[RTC6213N_SEEKCFG1] &= ~SEEKCFG1_SEEK;
	if (rtc6213n_set_register(radio, RTC6213N_SEEKCFG1) < 0)
		return -EIO;

	if (retval < 0)
		return retval;

	/* SF means the seek wrapped without finding anything. */
	if (radio->registers[RTC6213N_STATUS] & STATUS_SF)
		return -ENODATA;

	return 0;
}

static const struct v4l2_ioctl_ops rtc6213n_ioctl_ops = {
	.vidioc_querycap	= rtc6213n_vidioc_querycap,
	.vidioc_g_tuner		= rtc6213n_vidioc_g_tuner,
	.vidioc_s_tuner		= rtc6213n_vidioc_s_tuner,
	.vidioc_g_frequency	= rtc6213n_vidioc_g_frequency,
	.vidioc_s_frequency	= rtc6213n_vidioc_s_frequency,
	.vidioc_s_hw_freq_seek	= rtc6213n_vidioc_s_hw_freq_seek,
	.vidioc_log_status	= v4l2_ctrl_log_status,
	.vidioc_subscribe_event	= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct video_device rtc6213n_viddev_template = {
	.fops		= &rtc6213n_fops,
	.name		= DRIVER_NAME,
	.release	= video_device_release_empty,
	.ioctl_ops	= &rtc6213n_ioctl_ops,
	.device_caps	= V4L2_CAP_RADIO | V4L2_CAP_TUNER |
			  V4L2_CAP_HW_FREQ_SEEK | V4L2_CAP_RDS_CAPTURE |
			  V4L2_CAP_READWRITE,
};

/*
 * Driver
 */
static void rtc6213n_clks_disable(void *data)
{
	struct rtc6213n_device *radio = data;

	clk_bulk_disable_unprepare(radio->num_clks, radio->clks);
}

static int rtc6213n_i2c_probe(struct i2c_client *client)
{
	struct rtc6213n_device *radio;
	struct v4l2_ctrl_handler *hdl;
	int retval;

	radio = devm_kzalloc(&client->dev, sizeof(*radio), GFP_KERNEL);
	if (!radio)
		return -ENOMEM;

	radio->client = client;
	mutex_init(&radio->lock);
	init_completion(&radio->completion);
	init_waitqueue_head(&radio->read_queue);

	radio->buf_size = RDS_BUFFER_BLOCKS * RDS_BLOCK_BYTES;
	radio->buffer = devm_kzalloc(&client->dev, radio->buf_size, GFP_KERNEL);
	if (!radio->buffer)
		return -ENOMEM;

	/*
	 * The part runs off a reference clock generated by the PMIC and routed
	 * out through an alternate function on one of its GPIOs. Without it
	 * the I2C block still answers -- it is clocked by SCL from the host --
	 * but the core never runs and every configuration register reads back
	 * as all-ones.
	 */
	retval = devm_clk_bulk_get_all(&client->dev, &radio->clks);
	if (retval < 0)
		return dev_err_probe(&client->dev, retval,
				     "failed to get reference clocks\n");
	radio->num_clks = retval;

	retval = clk_bulk_prepare_enable(radio->num_clks, radio->clks);
	if (retval < 0)
		return dev_err_probe(&client->dev, retval,
				     "failed to enable reference clocks\n");

	retval = devm_add_action_or_reset(&client->dev, rtc6213n_clks_disable,
					  radio);
	if (retval < 0)
		return retval;

	dev_info(&client->dev, "enabled %d reference clock(s)\n",
		 radio->num_clks);

	retval = v4l2_device_register(&client->dev, &radio->v4l2_dev);
	if (retval < 0)
		return retval;

	hdl = &radio->ctrl_handler;
	v4l2_ctrl_handler_init(hdl, 2);
	v4l2_ctrl_new_std(hdl, &rtc6213n_ctrl_ops, V4L2_CID_AUDIO_VOLUME,
			  0, 15, 1, 15);
	v4l2_ctrl_new_std(hdl, &rtc6213n_ctrl_ops, V4L2_CID_AUDIO_MUTE,
			  0, 1, 1, 1);
	if (hdl->error) {
		retval = hdl->error;
		goto err_ctrl;
	}
	radio->v4l2_dev.ctrl_handler = hdl;

	retval = rtc6213n_get_all_registers(radio);
	if (retval < 0) {
		dev_err(&client->dev, "no response on the I2C bus\n");
		goto err_ctrl;
	}
	dev_info(&client->dev, "DeviceID 0x%04x ChipID 0x%04x\n",
		 radio->registers[RTC6213N_DEVICEID],
		 radio->registers[RTC6213N_CHIPID]);

	if (client->irq) {
		retval = devm_request_threaded_irq(&client->dev, client->irq,
						   NULL,
						   rtc6213n_i2c_interrupt,
						   IRQF_TRIGGER_FALLING |
						   IRQF_ONESHOT,
						   DRIVER_NAME, radio);
		if (retval) {
			dev_err(&client->dev, "cannot request IRQ %d\n",
				client->irq);
			goto err_ctrl;
		}
		radio->stci_enabled = true;
	} else {
		dev_warn(&client->dev,
			 "no interrupt, falling back to polling\n");
	}

	retval = rtc6213n_start(radio);
	if (retval < 0) {
		dev_err(&client->dev, "failed to power up the tuner\n");
		goto err_ctrl;
	}

	radio->videodev = rtc6213n_viddev_template;
	radio->videodev.lock = &radio->lock;
	radio->videodev.v4l2_dev = &radio->v4l2_dev;
	video_set_drvdata(&radio->videodev, radio);

	retval = video_register_device(&radio->videodev, VFL_TYPE_RADIO, -1);
	if (retval) {
		dev_err(&client->dev, "cannot register video device\n");
		goto err_stop;
	}

	i2c_set_clientdata(client, radio);

	return 0;

err_stop:
	rtc6213n_stop(radio);
err_ctrl:
	v4l2_ctrl_handler_free(hdl);
	v4l2_device_unregister(&radio->v4l2_dev);
	return retval;
}

static void rtc6213n_i2c_remove(struct i2c_client *client)
{
	struct rtc6213n_device *radio = i2c_get_clientdata(client);

	video_unregister_device(&radio->videodev);
	rtc6213n_stop(radio);
	v4l2_ctrl_handler_free(&radio->ctrl_handler);
	v4l2_device_unregister(&radio->v4l2_dev);
}

static int rtc6213n_i2c_suspend(struct device *dev)
{
	struct rtc6213n_device *radio = dev_get_drvdata(dev);

	return rtc6213n_stop(radio);
}

static int rtc6213n_i2c_resume(struct device *dev)
{
	struct rtc6213n_device *radio = dev_get_drvdata(dev);

	return rtc6213n_start(radio);
}

static DEFINE_SIMPLE_DEV_PM_OPS(rtc6213n_i2c_pm, rtc6213n_i2c_suspend,
				rtc6213n_i2c_resume);

static const struct of_device_id rtc6213n_of_match[] = {
	{ .compatible = "richwave,rtc6213n" },
	{ }
};
MODULE_DEVICE_TABLE(of, rtc6213n_of_match);

static const struct i2c_device_id rtc6213n_i2c_id[] = {
	{ "rtc6213n" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rtc6213n_i2c_id);

static struct i2c_driver rtc6213n_i2c_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = rtc6213n_of_match,
		.pm = pm_sleep_ptr(&rtc6213n_i2c_pm),
	},
	.probe = rtc6213n_i2c_probe,
	.remove = rtc6213n_i2c_remove,
	.id_table = rtc6213n_i2c_id,
};
module_i2c_driver(rtc6213n_i2c_driver);

MODULE_AUTHOR("Tobias Lorenz <tobias.lorenz@gmx.net>");
MODULE_DESCRIPTION("Richwave RTC6213N FM radio receiver driver");
MODULE_LICENSE("GPL");
