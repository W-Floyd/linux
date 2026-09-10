// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Maxim MAX77705 USB Type-C port driver
 *
 * Copyright (C) 2026 William Floyd <git@notmy.space>
 *
 * The MAX77705 runs the Type-C and USB-PD state machines in its own firmware.
 * This driver therefore does not implement PD: it reports the connection the
 * chip has already established, and drives the orientation switch and USB role
 * switch to match. Commands are posted to a 32-byte opcode mailbox and answered
 * through the VDM/PD interrupts.
 */

#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/usb/role.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_mux.h>

#include "max77705-typec.h"

struct max77705_typec {
	struct device *dev;
	struct regmap *regmap;

	struct typec_port *port;
	struct typec_capability cap;
	struct typec_partner *partner;

	struct typec_switch *sw;
	struct typec_mux *mux;
	struct usb_role_switch *role_sw;

	/* the "connector" child, which owns the port graph */
	struct fwnode_handle *fwnode;

	/* one command is in flight at a time, answered by an interrupt */
	struct mutex mailbox_lock;
	struct completion cmd_done;

	enum max77705_cc_state cc_state;
	enum typec_orientation orientation;
};

static const struct regmap_config max77705_typec_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX77705_REG_OPCODE_RES + MAX77705_OPCODE_DATA_LEN,
};

/**
 * max77705_typec_opcode_xfer - run one command through the opcode mailbox
 * @tc: the port
 * @opcode: command selector
 * @tx: payload to send, may be NULL when @tx_len is zero
 * @tx_len: payload length, at most MAX77705_OPCODE_DATA_LEN
 * @rx: where to put the response, may be NULL when @rx_len is zero
 * @rx_len: expected response length, at most MAX77705_OPCODE_DATA_LEN
 *
 * The opcode and its payload go out as one burst, and a command shorter than
 * the full payload is terminated by the end register. The firmware answers by
 * raising its command-response interrupt, and the response begins with the
 * opcode it is answering.
 */
static int max77705_typec_opcode_xfer(struct max77705_typec *tc, u8 opcode,
				      const u8 *tx, size_t tx_len,
				      u8 *rx, size_t rx_len)
{
	u8 buf[MAX77705_OPCODE_DATA_LEN + 1];
	unsigned long left;
	int ret;

	if (tx_len > MAX77705_OPCODE_DATA_LEN ||
	    rx_len > MAX77705_OPCODE_DATA_LEN)
		return -EINVAL;

	guard(mutex)(&tc->mailbox_lock);

	buf[0] = opcode;
	if (tx_len)
		memcpy(&buf[1], tx, tx_len);

	reinit_completion(&tc->cmd_done);

	ret = regmap_bulk_write(tc->regmap, MAX77705_REG_OPCODE, buf,
				tx_len + 1);
	if (ret)
		return ret;

	if (tx_len < MAX77705_OPCODE_DATA_LEN) {
		ret = regmap_write(tc->regmap, MAX77705_REG_OPCODE_END, 0);
		if (ret)
			return ret;
	}

	left = wait_for_completion_timeout(&tc->cmd_done,
					   msecs_to_jiffies(MAX77705_OPCODE_TIMEOUT_MS));
	if (!left)
		return -ETIMEDOUT;

	ret = regmap_bulk_read(tc->regmap, MAX77705_REG_OPCODE_RES, buf,
			       rx_len + 1);
	if (ret)
		return ret;

	if (buf[0] != opcode) {
		dev_err(tc->dev, "answer to 0x%02x carries opcode 0x%02x\n",
			opcode, buf[0]);
		return -EPROTO;
	}

	if (rx_len)
		memcpy(rx, &buf[1], rx_len);

	return 0;
}

static enum typec_orientation
max77705_typec_orientation(unsigned int cc_status0)
{
	switch (FIELD_GET(MAX77705_CC_STATUS0_PINSTAT, cc_status0)) {
	case MAX77705_CC_PIN_CC1:
		return TYPEC_ORIENTATION_NORMAL;
	case MAX77705_CC_PIN_CC2:
		return TYPEC_ORIENTATION_REVERSE;
	default:
		return TYPEC_ORIENTATION_NONE;
	}
}

/*
 * When this port sinks, the source advertises how much it can supply through
 * the Rp it presents. Only a PD contract can describe more than that, and this
 * driver does not report contracts yet.
 */
static enum typec_pwr_opmode
max77705_typec_pwr_opmode(unsigned int cc_status0)
{
	switch (FIELD_GET(MAX77705_CC_STATUS0_ISTAT, cc_status0)) {
	case MAX77705_CC_CURRENT_1_5A:
		return TYPEC_PWR_MODE_1_5A;
	case MAX77705_CC_CURRENT_3_0A:
		return TYPEC_PWR_MODE_3_0A;
	default:
		return TYPEC_PWR_MODE_USB;
	}
}

static void max77705_typec_partner_remove(struct max77705_typec *tc)
{
	if (!tc->partner)
		return;

	typec_unregister_partner(tc->partner);
	tc->partner = NULL;
}

static int max77705_typec_partner_add(struct max77705_typec *tc,
				      enum typec_role role)
{
	struct typec_partner_desc desc = {};
	struct typec_partner *partner;

	desc.usb_pd = false;
	desc.accessory = TYPEC_ACCESSORY_NONE;

	partner = typec_register_partner(tc->port, &desc);
	if (IS_ERR(partner))
		return PTR_ERR(partner);

	tc->partner = partner;
	typec_set_pwr_role(tc->port, role);
	typec_set_data_role(tc->port, role == TYPEC_SOURCE ? TYPEC_HOST :
							     TYPEC_DEVICE);
	return 0;
}

static int max77705_typec_sync_cc(struct max77705_typec *tc)
{
	enum usb_role role = USB_ROLE_NONE;
	enum typec_orientation orientation;
	unsigned int cc_status0;
	enum max77705_cc_state state;
	int ret;

	ret = regmap_read(tc->regmap, MAX77705_REG_CC_STATUS0, &cc_status0);
	if (ret)
		return ret;

	state = FIELD_GET(MAX77705_CC_STATUS0_CCSTAT, cc_status0);

	/*
	 * The CC pin status keeps naming the pin it last saw, so it only
	 * describes an orientation while something is attached.
	 */
	if (state == MAX77705_CC_NO_CONNECTION || state == MAX77705_CC_DISABLED)
		orientation = TYPEC_ORIENTATION_NONE;
	else
		orientation = max77705_typec_orientation(cc_status0);

	if (orientation != tc->orientation) {
		ret = typec_switch_set(tc->sw, orientation);
		if (ret)
			return ret;

		typec_set_orientation(tc->port, orientation);
		tc->orientation = orientation;
	}

	/*
	 * A source settles its advertised current after the attach, so take
	 * the value on every pass rather than once when the partner appears.
	 */
	if (state == MAX77705_CC_SINK)
		typec_set_pwr_opmode(tc->port,
				     max77705_typec_pwr_opmode(cc_status0));

	if (state == tc->cc_state)
		return 0;

	max77705_typec_partner_remove(tc);

	switch (state) {
	case MAX77705_CC_SINK:
		/* The state names the role this port took, not the partner's */
		ret = max77705_typec_partner_add(tc, TYPEC_SINK);
		role = USB_ROLE_DEVICE;
		break;
	case MAX77705_CC_SOURCE:
		ret = max77705_typec_partner_add(tc, TYPEC_SOURCE);
		role = USB_ROLE_HOST;
		break;
	case MAX77705_CC_NO_CONNECTION:
	case MAX77705_CC_DISABLED:
		ret = 0;
		break;
	default:
		dev_dbg(tc->dev, "unhandled CC state %u\n", state);
		ret = 0;
		break;
	}
	if (ret)
		return ret;

	tc->cc_state = state;

	return usb_role_switch_set_role(tc->role_sw, role);
}

static irqreturn_t max77705_typec_irq(int irq, void *data)
{
	struct max77705_typec *tc = data;
	u8 status[MAX77705_INT_COUNT];
	int ret;

	/*
	 * The four interrupt registers are contiguous and clear on read, so
	 * take them in one burst. Leaving any of them unread would strand the
	 * event it describes, since the chip only reports each one once.
	 */
	ret = regmap_bulk_read(tc->regmap, MAX77705_REG_UIC_INT, status,
			       sizeof(status));
	if (ret) {
		dev_err_ratelimited(tc->dev,
				    "failed to read the interrupt status: %d\n",
				    ret);
		return IRQ_NONE;
	}

	if (!status[MAX77705_INT_UIC] && !status[MAX77705_INT_CC] &&
	    !status[MAX77705_INT_PD] && !status[MAX77705_INT_VDM])
		return IRQ_NONE;

	if (status[MAX77705_INT_CC] & (MAX77705_CC_INT_CCSTAT |
				       MAX77705_CC_INT_CCPINSTAT |
				       MAX77705_CC_INT_CCISTAT)) {
		ret = max77705_typec_sync_cc(tc);
		if (ret)
			dev_err_ratelimited(tc->dev,
					    "failed to sync CC state: %d\n", ret);
	}

	if (status[MAX77705_INT_UIC] & MAX77705_UIC_INT_APCMDRES)
		complete(&tc->cmd_done);

	/*
	 * Power delivery and the alternate modes are not driven yet. Report
	 * what arrives so the events are visible while that is built.
	 */
	if (status[MAX77705_INT_UIC] || status[MAX77705_INT_PD] ||
	    status[MAX77705_INT_VDM])
		dev_dbg(tc->dev, "uic 0x%02x pd 0x%02x vdm 0x%02x\n",
			status[MAX77705_INT_UIC], status[MAX77705_INT_PD],
			status[MAX77705_INT_VDM]);

	return IRQ_HANDLED;
}

static int max77705_typec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct i2c_client *parent = to_i2c_client(dev->parent);
	struct max77705_typec *tc;
	struct i2c_client *i2c;
	unsigned int rev;
	u8 ctrl3;
	int irq, ret;

	tc = devm_kzalloc(dev, sizeof(*tc), GFP_KERNEL);
	if (!tc)
		return -ENOMEM;

	tc->dev = dev;
	tc->cc_state = MAX77705_CC_NO_CONNECTION;
	tc->orientation = TYPEC_ORIENTATION_NONE;

	ret = devm_mutex_init(dev, &tc->mailbox_lock);
	if (ret)
		return ret;

	init_completion(&tc->cmd_done);

	/*
	 * The Type-C block answers on its own address rather than the one the
	 * MFD parent claimed, so take a second client on the same adapter.
	 */
	i2c = devm_i2c_new_dummy_device(dev, parent->adapter,
					MAX77705_TYPEC_I2C_ADDR);
	if (IS_ERR(i2c))
		return dev_err_probe(dev, PTR_ERR(i2c),
				     "failed to claim the Type-C I2C address\n");

	tc->regmap = devm_regmap_init_i2c(i2c, &max77705_typec_regmap_config);
	if (IS_ERR(tc->regmap))
		return dev_err_probe(dev, PTR_ERR(tc->regmap),
				     "failed to initialise regmap\n");

	ret = regmap_read(tc->regmap, MAX77705_REG_UIC_FW_REV, &rev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to read the firmware revision\n");
	dev_dbg(dev, "MAX77705 Type-C firmware revision 0x%02x\n", rev);

	/*
	 * The connector child owns the port graph, so the switches and the
	 * role switch are all described relative to it rather than to the
	 * Type-C block itself.
	 */
	tc->fwnode = device_get_named_child_node(dev, "connector");
	if (!tc->fwnode)
		return dev_err_probe(dev, -EINVAL, "no connector node\n");

	tc->sw = fwnode_typec_switch_get(tc->fwnode);
	if (IS_ERR(tc->sw)) {
		ret = dev_err_probe(dev, PTR_ERR(tc->sw),
				    "failed to acquire orientation switch\n");
		goto err_fwnode_put;
	}

	tc->mux = fwnode_typec_mux_get(tc->fwnode);
	if (IS_ERR(tc->mux)) {
		ret = dev_err_probe(dev, PTR_ERR(tc->mux),
				    "failed to acquire mode mux\n");
		goto err_switch_put;
	}

	tc->role_sw = fwnode_usb_role_switch_get(tc->fwnode);
	if (IS_ERR(tc->role_sw)) {
		ret = dev_err_probe(dev, PTR_ERR(tc->role_sw),
				    "failed to acquire USB role switch\n");
		goto err_mux_put;
	}

	tc->cap.type = TYPEC_PORT_DRP;
	tc->cap.data = TYPEC_PORT_DRD;
	tc->cap.revision = USB_TYPEC_REV_1_2;
	/* The CC pin status names the orientation whether or not a mux exists */
	tc->cap.orientation_aware = true;
	tc->cap.fwnode = tc->fwnode;
	tc->cap.driver_data = tc;

	tc->port = typec_register_port(dev, &tc->cap);
	if (IS_ERR(tc->port)) {
		ret = dev_err_probe(dev, PTR_ERR(tc->port),
				    "failed to register the Type-C port\n");
		goto err_role_put;
	}

	platform_set_drvdata(pdev, tc);

	/* Reflect whatever is already attached before interrupts are enabled */
	ret = max77705_typec_sync_cc(tc);
	if (ret) {
		dev_err_probe(dev, ret, "failed to read the initial CC state\n");
		goto err_port_unregister;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = irq;
		goto err_port_unregister;
	}

	ret = devm_request_threaded_irq(dev, irq, NULL, max77705_typec_irq,
					IRQF_ONESHOT, dev_name(dev), tc);
	if (ret) {
		dev_err_probe(dev, ret, "failed to request the CC interrupt\n");
		goto err_port_unregister;
	}

	/* Unmask the connection state interrupts we act on */
	ret = regmap_write(tc->regmap, MAX77705_REG_CC_INT_M,
			   (u8)~(MAX77705_CC_INT_CCSTAT |
				 MAX77705_CC_INT_CCPINSTAT |
				 MAX77705_CC_INT_CCISTAT));
	if (ret) {
		dev_err_probe(dev, ret, "failed to unmask CC interrupts\n");
		goto err_port_unregister;
	}

	/* The mailbox is answered by the command-response interrupt */
	ret = regmap_write(tc->regmap, MAX77705_REG_UIC_INT_M,
			   (u8)~MAX77705_UIC_INT_APCMDRES);
	if (ret) {
		dev_err_probe(dev, ret, "failed to unmask the command response\n");
		goto err_port_unregister;
	}

	/*
	 * Exercise the mailbox once, so that a firmware which does not answer
	 * is apparent here rather than when an alternate mode depends on it.
	 */
	ret = max77705_typec_opcode_xfer(tc, MAX77705_OPCODE_CTRL3_R,
					 NULL, 0, &ctrl3, sizeof(ctrl3));
	if (ret)
		dev_warn(dev, "the opcode mailbox did not answer: %d\n", ret);
	else
		dev_dbg(dev, "opcode mailbox ready, ctrl3 0x%02x\n", ctrl3);

	return 0;

err_port_unregister:
	max77705_typec_partner_remove(tc);
	typec_unregister_port(tc->port);
err_role_put:
	usb_role_switch_put(tc->role_sw);
err_mux_put:
	typec_mux_put(tc->mux);
err_switch_put:
	typec_switch_put(tc->sw);
err_fwnode_put:
	fwnode_handle_put(tc->fwnode);

	return ret;
}

static void max77705_typec_remove(struct platform_device *pdev)
{
	struct max77705_typec *tc = platform_get_drvdata(pdev);

	regmap_write(tc->regmap, MAX77705_REG_CC_INT_M, 0xff);

	max77705_typec_partner_remove(tc);
	typec_unregister_port(tc->port);
	usb_role_switch_put(tc->role_sw);
	typec_mux_put(tc->mux);
	typec_switch_put(tc->sw);
	fwnode_handle_put(tc->fwnode);
}

static const struct platform_device_id max77705_typec_id[] = {
	{ "max77705-typec" },
	{ }
};
MODULE_DEVICE_TABLE(platform, max77705_typec_id);

static struct platform_driver max77705_typec_driver = {
	.driver = {
		.name = "max77705-typec",
	},
	.probe = max77705_typec_probe,
	.remove = max77705_typec_remove,
	.id_table = max77705_typec_id,
};
module_platform_driver(max77705_typec_driver);

MODULE_AUTHOR("William Floyd <git@notmy.space>");
MODULE_DESCRIPTION("Maxim MAX77705 USB Type-C port driver");
MODULE_LICENSE("GPL");
