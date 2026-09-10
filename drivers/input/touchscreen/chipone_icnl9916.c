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
#include <linux/unaligned.h>

#define ICNL9916_READ_CMD(cls, cmd)	(1 << 14 | (cls) << 8 | (cmd))
#define ICNL9916_WRITE_CMD(cls, cmd)	(1 << 13 | (cls) << 8 | (cmd))

#define ICNL9916_READ_FW_ID		ICNL9916_READ_CMD(0, 3)
#define ICNL9916_READ_RESOLUTION	ICNL9916_READ_CMD(0, 7)
#define ICNL9916_READ_TOUCH_DATA	ICNL9916_READ_CMD(1, 3)
#define ICNL9916_WRITE_POWER_MODE	ICNL9916_WRITE_CMD(2, 4)

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

static const struct icnl9916_bus_ops icnl9916_spi_ops = {
	.read = icnl9916_spi_read,
	.write = icnl9916_spi_write,
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

	ret = data->bus->read(data, ICNL9916_READ_TOUCH_DATA,
			      &touch_data, sizeof(touch_data));
	if (ret) {
		dev_err(dev, "Error reading touch data: %d\n", ret);
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
		return ret;
	}

	dev_dbg(dev, "Device ID: %04x\n", le16_to_cpu(fw_id));

	return 0;
}

static int icnl9916_start(struct input_dev *input)
{
	struct icnl9916_data *data = input_get_drvdata(input);
	int ret;

	ret = icnl9916_init(data);
	if (ret)
		return ret;

	enable_irq(data->irq);

	return 0;
}

static void icnl9916_stop(struct input_dev *input)
{
	struct icnl9916_data *data = input_get_drvdata(input);
	u8 pwr_mode = ICNL9916_POWER_SUSPEND;

	disable_irq(data->irq);
	data->bus->write(data, ICNL9916_WRITE_POWER_MODE,
			 &pwr_mode, sizeof(pwr_mode));

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

	input->name = dev_name(dev);
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
