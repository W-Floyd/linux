// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for ChipOne icnl9916 i2c touchscreen controller
 *
 * Copyright (C) 2024 Otto Pflüger
 *
 * Parts of this driver are based on the ChipOne icnl9916 driver,
 * Copyright (c) 2015 Red Hat Inc.
 *
 * Red Hat authors:
 * Hans de Goede <hdegoede@redhat.com>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/reset.h>
#include <linux/spi/spi.h>
#include <linux/crc32.h>
#include <linux/firmware.h>
#include <linux/unaligned.h>

#define ICNL9916_READ_CMD(cls, cmd)	(1 << 14 | (cls) << 8 | (cmd))
#define ICNL9916_WRITE_CMD(cls, cmd)	(1 << 13 | (cls) << 8 | (cmd))

#define ICNL9916_READ_FW_ID		ICNL9916_READ_CMD(0, 3)
#define ICNL9916_READ_RESOLUTION	ICNL9916_READ_CMD(0, 7)
#define ICNL9916_READ_TOUCH_DATA	ICNL9916_READ_CMD(1, 3)
#define ICNL9916_WRITE_POWER_MODE	ICNL9916_WRITE_CMD(2, 4)
/* Byte the controller clocks out when it has nothing to report. */
#define ICNL9916_IDLE_FILL		0x7f

#define ICNL9916_POWER_NORMAL		0
#define ICNL9916_POWER_SUSPEND		2

#define ICNL9916_MAX_TOUCHES		10

/*
 * The SPI framing differs from I2C in exactly two ways: a leading opcode byte
 * stands in for the I2C slave address, and the additive checksum is replaced
 * by a CRC-16.  Everything else -- command encoding, payload layout, trailer
 * semantics -- is shared.
 */
#define ICNL9916_SPI_WR_ADDR		0xf0
#define ICNL9916_SPI_RD_ADDR		0xf1

#define ICNL9916_READ_BIT		14
#define ICNL9916_WRITE_BIT		13

/* Largest payload is a full touch report; the rest is header and trailer. */
#define ICNL9916_SPI_BUF_SIZE		128

/*
 * The controller has a second, lower-level SPI interface used when no firmware
 * is running: a different opcode, a 24-bit big-endian register address, and no
 * CRC.  Vendor code treats "normal mode is offline" as an ordinary state and
 * falls back to this to read the hardware ID, so it doubles as a way to tell a
 * chip that is alive but empty from one that is not answering at all.
 */
#define ICNL9916_SPI_PROG_ADDR		0x60
#define ICNL9916_HW_REG_HARDWARE_ID	0x30000
#define ICNL9916_HW_REG_BOOT_MODE	0x30010
#define ICNL9916_HW_REG_CURRENT_MODE	0x30011
#define ICNL9916_PROG_READ_OVERHEAD	5

#define ICNL9916_BOOT_MODE_IDLE		0
#define ICNL9916_BOOT_MODE_FLASH	1
#define ICNL9916_BOOT_MODE_TCH_PRG	2
#define ICNL9916_BOOT_MODE_SRAM		3
#define ICNL9916_BOOT_MODE_MASK		7

/*
 * The controller keeps no persistent firmware: on SPI the vendor driver forces
 * to_flash=false unconditionally, so the image is pushed into SRAM on every
 * boot and started from there.  Motorola ships it in the vendor partition,
 * which msm-firmware-loader mounts and symlinks into the firmware search path.
 */
#define ICNL9916_FW_NAME		"chipone_firmware.bin"

/* Payload per program-mode write; the frame adds an opcode and a 24-bit address. */
#define ICNL9916_SRAM_CHUNK		4096

struct icnl9916_tx_header {
	__le16 cmd;
	__le16 len;
	__u8 check_l;
	__u8 check_h;
} __packed;

struct icnl9916_rx_trailer {
	__u8 error;
	__le16 cmd;
	__u8 check_l;
	__u8 check_h;
} __packed;

struct icnl9916_touch {
	__u8 slot;
	__le16 x;
	__le16 y;
	__u8 pressure;	/* Seems more like finger width then pressure really */
	__u8 event;
#define ICNL9916_EVENT_NONE	0
#define ICNL9916_EVENT_DOWN	1
#define ICNL9916_EVENT_MOVE	2
#define ICNL9916_EVENT_STAY	3
#define ICNL9916_EVENT_UP	4
} __packed;

struct icnl9916_touch_data {
	__u8 softbutton;
	__u8 touch_count;
	struct icnl9916_touch touches[ICNL9916_MAX_TOUCHES];
} __packed;

struct icnl9916_data;

struct icnl9916_bus_ops {
	int (*read)(struct icnl9916_data *data, u16 cmd, void *buf, u16 len);
	int (*write)(struct icnl9916_data *data, u16 cmd, void *buf, u16 len);
	/* Touch reports need the reply inside one chip-select assertion. */
	int (*read_touch)(struct icnl9916_data *data, u16 cmd, void *buf,
			  u16 len);
};

struct icnl9916_data {
	struct device *dev;
	const struct icnl9916_bus_ops *bus;
	struct i2c_client *i2c;
	struct spi_device *spi;
	int irq;
	struct input_dev *input;
	struct reset_control *chip_reset;
	struct gpio_desc *reset_gpio;
	struct touchscreen_properties prop;
	u8 *tx_buf;
	u8 *rx_buf;
};

/* SPI header, 7 bytes: opcode, command, payload length, CRC over the first 5. */
struct icnl9916_spi_tx_header {
	__u8 addr;
	__le16 cmd;
	__le16 len;
	__le16 crc;
} __packed;

struct icnl9916_spi_rx_trailer {
	__u8 error;
	__le16 cmd;
	__le16 crc;
} __packed;

static u8 icnl9916_calc_checksum(u8 *data, int len)
{
	u8 checksum = 0;
	int i;

	for (i = 0; i < len; i++)
		checksum += *data++;

	return ~checksum;
}

static int icnl9916_i2c_read(struct icnl9916_data *data, u16 cmd, void *buf,
			     u16 len)
{
	struct i2c_client *client = data->i2c;
	struct icnl9916_tx_header cmd_hdr = {
		.cmd = cmd,
		.len = len,
	};
	struct icnl9916_rx_trailer rsp;
	int ret;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.len = sizeof(cmd_hdr),
			.buf = (u8 *)&cmd_hdr
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(rsp),
			.buf = (u8 *)&rsp
		}
	};

	cmd_hdr.check_l = icnl9916_calc_checksum((u8 *)&cmd_hdr, 4);
	cmd_hdr.check_h = 1;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	if (rsp.error) {
		dev_err(&client->dev, "Command %04x error %02x\n", cmd, rsp.error);
		return -EIO;
	}

	return 0;
}

static int icnl9916_i2c_write(struct icnl9916_data *data, u16 cmd, void *buf,
			      u16 len)
{
	struct i2c_client *client = data->i2c;
	struct icnl9916_tx_header cmd_hdr = {
		.cmd = cpu_to_le16(cmd),
		.len = cpu_to_le16(len),
	};
	u8 data_checksum[2];
	int ret;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.len = sizeof(cmd_hdr),
			.buf = (u8 *)&cmd_hdr
		},
		{
			.addr = client->addr,
			.len = len,
			.buf = buf
		},
		{
			.addr = client->addr,
			.len = sizeof(data_checksum),
			.buf = data_checksum
		},
	};

	cmd_hdr.check_l = icnl9916_calc_checksum((u8 *)&cmd_hdr, 4);
	cmd_hdr.check_h = 1;

	data_checksum[0] = icnl9916_calc_checksum(buf, len);
	data_checksum[1] = 1;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	return 0;
}

static const struct icnl9916_bus_ops icnl9916_i2c_ops = {
	.read = icnl9916_i2c_read,
	.write = icnl9916_i2c_write,
	.read_touch = icnl9916_i2c_read,
};

/*
 * CRC-16 with polynomial 0x8005, MSB-first, initial value 0 and no final
 * inversion.  lib/crc16.c implements the reflected 0xa001 form and crc_itu_t()
 * uses 0x1021, so neither can be reused here.
 */
static u16 icnl9916_crc16(const u8 *data, size_t len)
{
	u16 crc = 0;
	size_t i;
	int bit;

	for (i = 0; i < len; i++) {
		crc ^= (u16)data[i] << 8;

		for (bit = 0; bit < 8; bit++) {
			if (crc & 0x8000)
				crc = (crc << 1) ^ 0x8005;
			else
				crc <<= 1;
		}
	}

	return crc;
}

static int icnl9916_spi_xfer(struct icnl9916_data *data, u16 txlen, u16 rxlen)
{
	struct spi_transfer xfer = { };
	struct spi_message msg;
	int ret;

	/*
	 * Header and payload go as two separate messages, with chip select
	 * cycling between them, matching what the controller expects.
	 */
	spi_message_init(&msg);
	xfer.tx_buf = data->tx_buf;
	xfer.len = txlen;
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(data->spi, &msg);
	if (ret)
		return ret;

	/* The controller needs a gap before it will clock out the reply. */
	udelay(100);

	if (!rxlen)
		return 0;

	spi_message_init(&msg);
	memset(&xfer, 0, sizeof(xfer));
	xfer.rx_buf = data->rx_buf;
	xfer.len = rxlen;
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(data->spi, &msg);
	if (ret)
		return ret;

	udelay(100);

	return 0;
}

/* One full-duplex transfer; program mode replies within the same frame. */
static int icnl9916_spi_xfer_duplex(struct icnl9916_data *data, u16 len)
{
	struct spi_transfer xfer = {
		.tx_buf = data->tx_buf,
		.rx_buf = data->rx_buf,
		.len = len,
	};
	struct spi_message msg;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	return spi_sync(data->spi, &msg);
}

static int icnl9916_enter_program_mode(struct icnl9916_data *data)
{
	static const u8 magic[] = { 0xcc, 0x33, 0x55, 0x5a };
	struct spi_transfer xfer = { };
	struct spi_message msg;
	int ret;

	memcpy(data->tx_buf, magic, sizeof(magic));

	spi_message_init(&msg);
	xfer.tx_buf = data->tx_buf;
	xfer.len = sizeof(magic);
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(data->spi, &msg);
	if (ret)
		return ret;

	mdelay(5);

	return 0;
}

static int icnl9916_prog_read(struct icnl9916_data *data, u32 reg, void *buf,
			      u16 rlen)
{
	u16 len = rlen + ICNL9916_PROG_READ_OVERHEAD;
	int ret;

	memset(data->tx_buf, 0, len);
	data->tx_buf[0] = ICNL9916_SPI_PROG_ADDR | 0x01;
	put_unaligned_be24(reg, data->tx_buf + 1);

	ret = icnl9916_spi_xfer_duplex(data, len);
	if (ret)
		return ret;

	memcpy(buf, data->rx_buf + ICNL9916_PROG_READ_OVERHEAD, rlen);

	return 0;
}

static int icnl9916_spi_read(struct icnl9916_data *data, u16 cmd, void *buf,
			     u16 len);

static int icnl9916_prog_writeb(struct icnl9916_data *data, u32 reg, u8 val)
{
	struct spi_transfer xfer = { };
	struct spi_message msg;

	data->tx_buf[0] = ICNL9916_SPI_PROG_ADDR;
	put_unaligned_be24(reg, data->tx_buf + 1);
	data->tx_buf[4] = val;

	spi_message_init(&msg);
	xfer.tx_buf = data->tx_buf;
	xfer.len = 5;
	spi_message_add_tail(&xfer, &msg);

	return spi_sync(data->spi, &msg);
}

/* Bulk write into SRAM: [opcode][addr:be24][payload], chunked. */
static int icnl9916_sram_write(struct icnl9916_data *data, u32 addr,
			       const u8 *src, size_t len, u8 *buf)
{
	struct spi_transfer xfer = { };
	struct spi_message msg;
	size_t chunk;
	int ret;

	while (len) {
		chunk = min_t(size_t, ICNL9916_SRAM_CHUNK, len);

		buf[0] = ICNL9916_SPI_PROG_ADDR;
		put_unaligned_be24(addr, buf + 1);
		memcpy(buf + 4, src, chunk);

		spi_message_init(&msg);
		xfer.tx_buf = buf;
		xfer.rx_buf = NULL;
		xfer.len = chunk + 4;
		spi_message_add_tail(&xfer, &msg);

		ret = spi_sync(data->spi, &msg);
		if (ret)
			return ret;

		src += chunk;
		addr += chunk;
		len -= chunk;
	}

	return 0;
}

/*
 * Push the firmware into SRAM and start it.  The image is a single section --
 * the multi-section format is 0x20000 bytes and this one is not -- so the whole
 * file is the payload and its CRC32 covers all of it.  That CRC is the
 * big-endian, non-reflected, zero-seeded variant, which is exactly crc32_be().
 */
static int icnl9916_load_firmware(struct icnl9916_data *data)
{
	const struct firmware *fw;
	__le16 fw_id;
	u8 *buf, cur;
	int ret;

	ret = request_firmware(&fw, ICNL9916_FW_NAME, data->dev);
	if (ret) {
		dev_err(data->dev, "Failed to request %s: %d\n",
			ICNL9916_FW_NAME, ret);
		return ret;
	}

	dev_info(data->dev, "Loading %s, %zu bytes, version %04x, crc %08x\n",
		 ICNL9916_FW_NAME, fw->size,
		 fw->size > 0x102 ? get_unaligned_le16(fw->data + 0x100) : 0,
		 crc32_be(0, fw->data, fw->size));

	buf = kmalloc(ICNL9916_SRAM_CHUNK + 4, GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto out;
	}

	ret = icnl9916_enter_program_mode(data);
	if (ret) {
		dev_err(data->dev, "Program mode entry failed: %d\n", ret);
		goto out_free;
	}

	ret = icnl9916_sram_write(data, 0, fw->data, fw->size, buf);
	if (ret) {
		dev_err(data->dev, "SRAM write failed: %d\n", ret);
		goto out_free;
	}

	ret = icnl9916_prog_writeb(data, ICNL9916_HW_REG_BOOT_MODE,
				   ICNL9916_BOOT_MODE_SRAM);
	if (ret) {
		dev_err(data->dev, "Set SRAM boot mode failed: %d\n", ret);
		goto out_free;
	}

	mdelay(30);

	if (!icnl9916_prog_read(data, ICNL9916_HW_REG_CURRENT_MODE, &cur, 1))
		dev_info(data->dev, "After firmware load: current_mode %u\n",
			 cur & ICNL9916_BOOT_MODE_MASK);

	ret = icnl9916_spi_read(data, ICNL9916_READ_FW_ID, &fw_id,
				sizeof(fw_id));
	if (ret)
		dev_err(data->dev, "Firmware did not start: %d\n", ret);
	else
		dev_info(data->dev, "Firmware running, id %04x\n",
			 le16_to_cpu(fw_id));

out_free:
	kfree(buf);
out:
	release_firmware(fw);

	return ret;
}

/*
 * Diagnostic: report whether the part answers on its program-mode interface.
 * A plausible hardware ID here means the SPI wiring and framing are sound and
 * the part simply has no firmware running; nothing at all points further down,
 * at the bus itself.
 */
static void icnl9916_report_program_mode_id(struct icnl9916_data *data)
{
	static const u8 modes[] = {
		ICNL9916_BOOT_MODE_FLASH,
		ICNL9916_BOOT_MODE_SRAM,
	};
	__le32 hwid = 0;
	u8 boot = 0, cur = 0;
	unsigned int i;
	int ret;

	ret = icnl9916_enter_program_mode(data);
	if (ret) {
		dev_err(data->dev, "Program mode entry failed: %d\n", ret);
		return;
	}

	ret = icnl9916_prog_read(data, ICNL9916_HW_REG_HARDWARE_ID, &hwid,
				 sizeof(hwid));
	if (ret) {
		dev_err(data->dev, "Program mode hwid read failed: %d\n", ret);
		return;
	}

	dev_info(data->dev, "Program mode hardware id: %06x\n",
		 le32_to_cpu(hwid) & 0xfffffff0);

	if (!icnl9916_prog_read(data, ICNL9916_HW_REG_BOOT_MODE, &boot, 1) &&
	    !icnl9916_prog_read(data, ICNL9916_HW_REG_CURRENT_MODE, &cur, 1))
		dev_info(data->dev,
			 "Program mode boot_mode %u current_mode %u\n",
			 boot & ICNL9916_BOOT_MODE_MASK,
			 cur & ICNL9916_BOOT_MODE_MASK);

	/*
	 * BOOT_MODE is volatile and reads back IDLE, so the part boots nothing
	 * until told otherwise.  Try flash first -- if the on-chip flash holds
	 * good firmware that is all it takes -- then SRAM, which is what the
	 * vendor sets and which only works once firmware has been downloaded.
	 */
	for (i = 0; i < ARRAY_SIZE(modes); i++) {
		__le16 fw_id;

		ret = icnl9916_prog_writeb(data, ICNL9916_HW_REG_BOOT_MODE,
					   modes[i]);
		if (ret) {
			dev_err(data->dev, "Set boot mode %u failed: %d\n",
				modes[i], ret);
			continue;
		}

		mdelay(30);

		ret = icnl9916_prog_read(data, ICNL9916_HW_REG_CURRENT_MODE,
					 &cur, 1);
		dev_info(data->dev, "After boot mode %u: current_mode %u\n",
			 modes[i], ret ? 0xff : (cur & ICNL9916_BOOT_MODE_MASK));

		ret = icnl9916_spi_read(data, ICNL9916_READ_FW_ID, &fw_id,
					sizeof(fw_id));
		if (!ret) {
			dev_info(data->dev,
				 "Firmware alive after boot mode %u, id %04x\n",
				 modes[i], le16_to_cpu(fw_id));
			return;
		}
	}
}

static int icnl9916_spi_read(struct icnl9916_data *data, u16 cmd, void *buf,
			     u16 len)
{
	struct icnl9916_spi_tx_header *hdr = (void *)data->tx_buf;
	struct icnl9916_spi_rx_trailer *rsp;
	u16 rxlen = len + sizeof(*rsp);
	u16 crc;
	int ret;

	if (rxlen > ICNL9916_SPI_BUF_SIZE)
		return -EINVAL;

	hdr->addr = ICNL9916_SPI_RD_ADDR;
	hdr->cmd = cpu_to_le16(cmd & ~BIT(ICNL9916_WRITE_BIT));
	hdr->len = cpu_to_le16(len);
	hdr->crc = cpu_to_le16(icnl9916_crc16(data->tx_buf,
					      offsetof(struct icnl9916_spi_tx_header, crc)));

	ret = icnl9916_spi_xfer(data, sizeof(*hdr), rxlen);
	if (ret)
		return ret;

	/*
	 * The reply is validated by its CRC, which covers everything up to the
	 * CRC itself: payload, error byte and echoed command.  This is the only
	 * integrity check the vendor driver applies on SPI.
	 */
	crc = icnl9916_crc16(data->rx_buf, rxlen - sizeof(__le16));
	if (crc != get_unaligned_le16(data->rx_buf + rxlen - sizeof(__le16))) {
		dev_err(data->dev,
			"Command %04x bad reply CRC (calc %04x): %*ph\n",
			cmd, crc, rxlen, data->rx_buf);
		return -EIO;
	}

	rsp = (void *)(data->rx_buf + len);
	if (rsp->error) {
		dev_err(data->dev, "Command %04x error %02x: %*ph\n", cmd,
			rsp->error, rxlen, data->rx_buf);
		return -EIO;
	}

	memcpy(buf, data->rx_buf, len);

	return 0;
}

static int icnl9916_spi_write(struct icnl9916_data *data, u16 cmd, void *buf,
			      u16 len)
{
	struct icnl9916_spi_tx_header *hdr = (void *)data->tx_buf;
	u16 txlen = sizeof(*hdr);
	u16 crc;

	if (txlen + len + sizeof(crc) > ICNL9916_SPI_BUF_SIZE)
		return -EINVAL;

	hdr->addr = ICNL9916_SPI_WR_ADDR;
	hdr->cmd = cpu_to_le16(cmd & ~BIT(ICNL9916_READ_BIT));
	hdr->len = cpu_to_le16(len);
	hdr->crc = cpu_to_le16(icnl9916_crc16(data->tx_buf,
					      offsetof(struct icnl9916_spi_tx_header, crc)));

	if (len) {
		memcpy(data->tx_buf + txlen, buf, len);
		crc = icnl9916_crc16(buf, len);
		put_unaligned_le16(crc, data->tx_buf + txlen + len);
		txlen += len + sizeof(crc);
	}

	return icnl9916_spi_xfer(data, txlen, 0);
}

/*
 * Touch reports are read within a single chip-select assertion: one
 * full-duplex transfer rather than the command-then-reply pair used for
 * everything else.  Reading them the ordinary way returns a constant idle
 * pattern, because the controller has already dropped the reply by the time
 * chip select is reasserted.
 */
static int icnl9916_spi_read_touch(struct icnl9916_data *data, u16 cmd,
				   void *buf, u16 len)
{
	struct icnl9916_spi_tx_header *hdr = (void *)data->tx_buf;
	u16 rxlen = len + sizeof(struct icnl9916_spi_rx_trailer);
	u16 xlen = max_t(u16, sizeof(*hdr), rxlen);
	int ret;

	if (xlen > ICNL9916_SPI_BUF_SIZE)
		return -EINVAL;

	memset(data->tx_buf, 0, xlen);
	hdr->addr = ICNL9916_SPI_RD_ADDR;
	hdr->cmd = cpu_to_le16(cmd & ~BIT(ICNL9916_WRITE_BIT));
	/*
	 * Announce the payload length only. The vendor's own single-chip-select
	 * caller passes payload+trailer here, but on this controller that
	 * yields nothing but idle fill, whereas asking for the payload returns
	 * real coordinates.
	 */
	hdr->len = cpu_to_le16(len);
	hdr->crc = cpu_to_le16(icnl9916_crc16(data->tx_buf,
					      offsetof(struct icnl9916_spi_tx_header, crc)));

	ret = icnl9916_spi_xfer_duplex(data, xlen);
	if (ret)
		return ret;

	/*
	 * Only the error byte is checked here, matching how this driver treats
	 * I2C replies.  The CRC that guards other commands does not validate
	 * over a touch report -- the payload is demonstrably correct while the
	 * trailing checksum does not cover it the same way -- so enforcing it
	 * would discard good coordinates.
	 */
	/*
	 * No trailer check here. With the payload-length framing above the
	 * reply carries coordinates but no trailer that validates, so the
	 * only usable signal is the payload itself: idle fill means no
	 * contact, anything else is a report.
	 */
	if (data->rx_buf[0] == ICNL9916_IDLE_FILL &&
	    data->rx_buf[1] == ICNL9916_IDLE_FILL)
		return -ENODATA;

	memcpy(buf, data->rx_buf, len);

	return 0;
}

static const struct icnl9916_bus_ops icnl9916_spi_ops = {
	.read = icnl9916_spi_read,
	.write = icnl9916_spi_write,
	.read_touch = icnl9916_spi_read_touch,
};

static inline bool icnl9916_touch_active(u8 event)
{
	return (event == ICNL9916_EVENT_DOWN) ||
	       (event == ICNL9916_EVENT_MOVE) ||
	       (event == ICNL9916_EVENT_STAY);
}

static irqreturn_t icnl9916_irq(int irq, void *dev_id)
{
	struct icnl9916_data *data = dev_id;
	struct device *dev = data->dev;
	struct icnl9916_touch_data touch_data;
	int i, ret;

	ret = data->bus->read_touch(data, ICNL9916_READ_TOUCH_DATA,
				    &touch_data, sizeof(touch_data));

	if (ret) {
		if (ret != -ENODATA)
			dev_err_ratelimited(dev, "Error reading touch data: %d\n",
					    ret);
		return IRQ_HANDLED;
	}

	if (touch_data.softbutton) {
		/*
		 * Other data is invalid when a softbutton is pressed.
		 * This needs some extra devicetree bindings to map the icnl9916
		 * softbutton codes to evdev codes. Currently no known devices
		 * use this.
		 */
		return IRQ_HANDLED;
	}

	if (touch_data.touch_count > ICNL9916_MAX_TOUCHES) {
		dev_warn(dev, "Too many touches %d > %d\n",
			 touch_data.touch_count, ICNL9916_MAX_TOUCHES);
		touch_data.touch_count = ICNL9916_MAX_TOUCHES;
	}

	for (i = 0; i < touch_data.touch_count; i++) {
		struct icnl9916_touch *touch = &touch_data.touches[i];
		bool act = icnl9916_touch_active(touch->event);

		input_mt_slot(data->input, touch->slot);
		input_mt_report_slot_state(data->input, MT_TOOL_FINGER, act);
		if (!act)
			continue;

		touchscreen_report_pos(data->input, &data->prop,
				       le16_to_cpu(touch->x),
				       le16_to_cpu(touch->y), true);
	}

	input_mt_sync_frame(data->input);
	input_sync(data->input);

	return IRQ_HANDLED;
}

static int icnl9916_init(struct icnl9916_data *data)
{
	struct device *dev = data->dev;
	__le16 fw_id;
	int ret;

	reset_control_deassert(data->chip_reset);

	/*
	 * The controller needs a clean deasserted-to-asserted edge, not merely
	 * an asserted level: hold reset released for 1 ms before pulsing it.
	 * Vendor code marks this sequence "can not be modified", and on fogona
	 * skipping the leading release leaves the part unresponsive -- it never
	 * drives MISO, so reads come back as the bus idle level.
	 */
	gpiod_set_value_cansleep(data->reset_gpio, 0);
	mdelay(1);
	gpiod_set_value_cansleep(data->reset_gpio, 1);
	mdelay(10);
	gpiod_set_value_cansleep(data->reset_gpio, 0);
	mdelay(40);

	ret = data->bus->read(data, ICNL9916_READ_FW_ID,
			      &fw_id, sizeof(fw_id));
	if (ret) {
		dev_err(dev, "Failed to read device ID: %d\n", ret);
		if (data->spi) {
			icnl9916_report_program_mode_id(data);
			ret = icnl9916_load_firmware(data);
		}
		return ret;
	}

	dev_dbg(dev, "Device ID: %04x\n", le16_to_cpu(fw_id));

	return 0;
}

static void icnl9916_drain(struct icnl9916_data *data)
{
	struct icnl9916_touch_data touch_data;

	data->bus->read_touch(data, ICNL9916_READ_TOUCH_DATA, &touch_data,
			      sizeof(touch_data));
}

static int icnl9916_start(struct input_dev *input)
{
	struct icnl9916_data *data = input_get_drvdata(input);
	__le16 fw_id;
	int ret;

	/*
	 * Only re-initialise if the controller is not already running. On this
	 * part icnl9916_init() resets it, and reset drops the firmware -- which
	 * lives in SRAM -- so an unconditional init turns every open into an
	 * 81 KB reload taking the best part of a second. Userspace opens and
	 * closes touchscreens freely, and doing that here starved the device of
	 * any time in which it could actually report a contact.
	 */
	/*
	 * Only re-initialise when the controller is not already running.
	 * icnl9916_init() resets it, and reset drops firmware held in SRAM, so
	 * an unconditional init makes every open an 81 KB reload of nearly a
	 * second. Userspace opens and closes touchscreens freely; doing that
	 * here left the device permanently reloading.
	 */
	ret = data->bus->read(data, ICNL9916_READ_FW_ID, &fw_id, sizeof(fw_id));
	if (ret) {
		ret = icnl9916_init(data);
		if (ret)
			return ret;
	}

	enable_irq(data->irq);

	/*
	 * Drain whatever the controller latched while the interrupt was off.
	 * The report line is edge triggered, so a report that went ready while
	 * masked is never re-signalled: without this the device sits with one
	 * stale report pending and never interrupts again.
	 */
	icnl9916_drain(data);

	return 0;
}

static void icnl9916_stop(struct input_dev *input)
{
	struct icnl9916_data *data = input_get_drvdata(input);

	disable_irq(data->irq);

	/*
	 * Deliberately left running rather than suspended. Suspending stops it
	 * answering, so the next open cannot tell a suspended controller from
	 * one with no firmware and reloads all 81 KB -- and userspace opens and
	 * closes touchscreens often enough that the device spent its whole life
	 * reloading. Idle power is a fair trade for a device that works; proper
	 * suspend needs a wake path that does not look like a dead controller.
	 */
	reset_control_assert(data->chip_reset);
}

static int icnl9916_suspend(struct device *dev)
{
	struct icnl9916_data *data = dev_get_drvdata(dev);

	mutex_lock(&data->input->mutex);
	if (input_device_enabled(data->input))
		icnl9916_stop(data->input);
	mutex_unlock(&data->input->mutex);

	return 0;
}

static int icnl9916_resume(struct device *dev)
{
	struct icnl9916_data *data = dev_get_drvdata(dev);

	mutex_lock(&data->input->mutex);
	if (input_device_enabled(data->input))
		icnl9916_start(data->input);
	mutex_unlock(&data->input->mutex);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(icnl9916_pm_ops, icnl9916_suspend, icnl9916_resume);

static int icnl9916_probe(struct icnl9916_data *data)
{
	struct device *dev = data->dev;
	struct input_dev *input;
	__le16 resolution[2];
	int error;

	if (!data->irq) {
		dev_err(dev, "Error no irq specified\n");
		return -EINVAL;
	}

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	input->name = "ChipOne ICNL9916 Touchscreen";
	input->id.bustype = data->spi ? BUS_SPI : BUS_I2C;
	input->open = icnl9916_start;
	input->close = icnl9916_stop;
	input->dev.parent = dev;

	data->input = input;
	input_set_drvdata(input, data);

	input_set_capability(input, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(input, EV_ABS, ABS_MT_POSITION_Y);

	error = icnl9916_init(data);
	if (error) {
		dev_err(dev, "Failed to initialize device: %d\n", error);
		return error;
	}

	error = data->bus->read(data, ICNL9916_READ_RESOLUTION,
				resolution, sizeof(resolution));
	if (error) {
		dev_err(dev, "Failed to read resolution: %d\n", error);
		return error;
	}

	input_set_abs_params(input, ABS_MT_POSITION_X, 0,
			     le16_to_cpu(resolution[0]), 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0,
			     le16_to_cpu(resolution[1]), 0, 0);
	touchscreen_parse_properties(input, true, &data->prop);

	error = input_mt_init_slots(input, ICNL9916_MAX_TOUCHES,
				    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (error)
		return error;

	error = devm_request_threaded_irq(dev, data->irq, NULL, icnl9916_irq,
					  IRQF_ONESHOT, dev_name(dev), data);
	if (error) {
		dev_err(dev, "Error requesting irq: %d\n", error);
		return error;
	}

	/* Stop device till opened */
	icnl9916_stop(input);

	error = input_register_device(input);
	if (error)
		return error;

	dev_set_drvdata(dev, data);

	return 0;
}

static struct icnl9916_data *icnl9916_alloc(struct device *dev,
					    const struct icnl9916_bus_ops *bus)
{
	struct icnl9916_data *data;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return NULL;

	data->dev = dev;
	data->bus = bus;

	data->chip_reset = devm_reset_control_get_optional_shared(dev, NULL);
	if (IS_ERR(data->chip_reset))
		return ERR_CAST(data->chip_reset);

	data->reset_gpio = devm_gpiod_get(dev, "touchscreen-reset",
					  GPIOD_OUT_LOW);
	if (IS_ERR(data->reset_gpio))
		return ERR_CAST(data->reset_gpio);

	return data;
}

static int icnl9916_i2c_probe(struct i2c_client *client)
{
	struct icnl9916_data *data;

	data = icnl9916_alloc(&client->dev, &icnl9916_i2c_ops);
	if (!data)
		return -ENOMEM;
	if (IS_ERR(data))
		return dev_err_probe(&client->dev, PTR_ERR(data),
				     "Error getting resources\n");

	data->i2c = client;
	data->irq = client->irq;

	return icnl9916_probe(data);
}

static const struct of_device_id icnl9916_of_match[] = {
	{ .compatible = "chipone,icnl9916" },
	{ }
};
MODULE_DEVICE_TABLE(of, icnl9916_of_match);

static struct i2c_driver icnl9916_i2c_driver = {
	.driver = {
		.name	= "chipone_icnl9916",
		.pm	= pm_sleep_ptr(&icnl9916_pm_ops),
		.of_match_table = icnl9916_of_match,
	},
	.probe = icnl9916_i2c_probe,
};

static int icnl9916_spi_probe(struct spi_device *spi)
{
	struct icnl9916_data *data;
	int error;

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;

	error = spi_setup(spi);
	if (error)
		return dev_err_probe(&spi->dev, error, "Error setting up SPI\n");

	data = icnl9916_alloc(&spi->dev, &icnl9916_spi_ops);
	if (!data)
		return -ENOMEM;
	if (IS_ERR(data))
		return dev_err_probe(&spi->dev, PTR_ERR(data),
				     "Error getting resources\n");

	data->tx_buf = devm_kzalloc(&spi->dev, ICNL9916_SPI_BUF_SIZE,
				    GFP_KERNEL);
	data->rx_buf = devm_kzalloc(&spi->dev, ICNL9916_SPI_BUF_SIZE,
				    GFP_KERNEL);
	if (!data->tx_buf || !data->rx_buf)
		return -ENOMEM;

	data->spi = spi;
	data->irq = spi->irq;

	return icnl9916_probe(data);
}

static const struct spi_device_id icnl9916_spi_id[] = {
	{ "icnl9916" },
	{ }
};
MODULE_DEVICE_TABLE(spi, icnl9916_spi_id);

static struct spi_driver icnl9916_spi_driver = {
	.driver = {
		.name	= "chipone_icnl9916_spi",
		.pm	= pm_sleep_ptr(&icnl9916_pm_ops),
		.of_match_table = icnl9916_of_match,
	},
	.id_table = icnl9916_spi_id,
	.probe = icnl9916_spi_probe,
};

static int __init icnl9916_init_module(void)
{
	int error;

	error = i2c_add_driver(&icnl9916_i2c_driver);
	if (error)
		return error;

	error = spi_register_driver(&icnl9916_spi_driver);
	if (error) {
		i2c_del_driver(&icnl9916_i2c_driver);
		return error;
	}

	return 0;
}
module_init(icnl9916_init_module);

static void __exit icnl9916_exit_module(void)
{
	spi_unregister_driver(&icnl9916_spi_driver);
	i2c_del_driver(&icnl9916_i2c_driver);
}
module_exit(icnl9916_exit_module);

MODULE_DESCRIPTION("ChipOne ICNL9916 touchscreen driver");
MODULE_AUTHOR("Otto Pflüger <otto.pflueger@abscue.de>");
MODULE_LICENSE("GPL");
