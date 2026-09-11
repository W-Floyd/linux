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

#include <drm/bridge/aux-bridge.h>
#include <linux/auxiliary_bus.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>
#include <linux/usb/pd_vdo.h>
#include <linux/usb/role.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_mux.h>
#include <linux/workqueue.h>

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

	/* carries HPD to whichever display controller the graph names */
	struct auxiliary_device *hpd_bridge;

	/* the board's own copy of the sink's hot plug detect, if wired */
	struct gpio_desc *hpd_gpio;

	/* one command is in flight at a time, answered by an interrupt */
	struct mutex mailbox_lock;
	struct completion cmd_done;

	/*
	 * The mailbox is answered by the same threaded interrupt that reports
	 * an alternate mode event, so a command cannot be posted from the
	 * handler: it would wait for a completion only it could deliver.
	 * Events are collected here and answered from a work item instead.
	 */
	struct delayed_work altmode_work;
	spinlock_t event_lock;	/* protects the three fields below */
	u8 vdm_events;
	u8 pd_events;
	bool altmode_enable;

	/*
	 * Set once the interrupt is live. Until then nothing may be queued,
	 * because the work item would wait for a command response that has
	 * nothing to deliver it.
	 */
	bool irq_ready;

	/* only touched by the work item, which is never concurrent with itself */
	u8 dp_pin_assign;
	u32 dp_status;
	u32 dp_conf;
	unsigned int dp_tries;
	bool hpd;

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

/**
 * max77705_typec_vdm_write - send one VDM to the partner
 * @tc: the port
 * @header: VDM header to send
 * @vdo: the VDOs to append, may be NULL when @nr_vdo is zero
 * @nr_vdo: how many VDOs to append
 *
 * Returns zero once the partner has ACKed the command, or a negative errno.
 * The firmware answers this synchronously with whatever the partner replied,
 * so a NAK is reported here rather than through an interrupt.
 */
static int max77705_typec_vdm_write(struct max77705_typec *tc, u32 header,
				    const u32 *vdo, unsigned int nr_vdo)
{
	u8 tx[1 + 4 * MAX77705_VDM_REQ_MAX_OBJ];
	u8 rx[1 + 4 * MAX77705_VDM_REQ_MAX_OBJ];
	unsigned int nr_obj = nr_vdo + 1;
	u32 answer;
	int ret, i;

	if (nr_obj > MAX77705_VDM_REQ_MAX_OBJ)
		return -EINVAL;

	tx[0] = FIELD_PREP(MAX77705_VDM_REQ_NR_OBJ, nr_obj) |
		FIELD_PREP(MAX77705_VDM_REQ_CMD_TYPE, CMDT_RSP_ACK);
	put_unaligned_le32(header, &tx[1]);
	for (i = 0; i < nr_vdo; i++)
		put_unaligned_le32(vdo[i], &tx[5 + i * 4]);

	ret = max77705_typec_opcode_xfer(tc, MAX77705_OPCODE_VDM_REQ,
					 tx, 1 + 4 * nr_obj,
					 rx, 1 + 4 * nr_obj);
	if (ret)
		return ret;

	if (rx[0] == MAX77705_VDM_NO_RESPONSE) {
		dev_dbg(tc->dev, "VDM 0x%08x went unanswered\n", header);
		return -ENODATA;
	}

	answer = get_unaligned_le32(&rx[1]);
	if (PD_VDO_CMDT(answer) != CMDT_RSP_ACK) {
		dev_dbg(tc->dev, "VDM 0x%08x answered 0x%08x\n", header, answer);
		return -EPROTO;
	}

	return 0;
}

/**
 * max77705_typec_vdm_read - read back a VDM the firmware has stored
 * @tc: the port
 * @id: which VDM to read
 * @header: where to put the VDM header the partner sent
 * @vdo: where to put the VDOs, may be NULL when @nr_vdo is zero
 * @nr_vdo: how many VDOs to take
 */
static int max77705_typec_vdm_read(struct max77705_typec *tc,
				   enum max77705_vdm id, u32 *header,
				   u32 *vdo, unsigned int nr_vdo)
{
	u8 rx[MAX77705_VDM_RESP_VDO + 4 * MAX77705_VDM_RESP_NR_VDO];
	u8 sel = id;
	int ret, i;

	if (nr_vdo > MAX77705_VDM_RESP_NR_VDO)
		return -EINVAL;

	ret = max77705_typec_opcode_xfer(tc, MAX77705_OPCODE_VDM_RESP, &sel, 1,
					 rx, MAX77705_VDM_RESP_VDO + 4 * nr_vdo);
	if (ret)
		return ret;

	/*
	 * The response names the VDM it belongs to, which is the only way to
	 * tell a fresh answer from the one the previous read left behind.
	 */
	if (rx[MAX77705_VDM_RESP_ID] != id) {
		dev_dbg(tc->dev, "asked for VDM 0x%02x, got 0x%02x\n", id,
			rx[MAX77705_VDM_RESP_ID]);
		return -ENODATA;
	}

	*header = get_unaligned_le32(&rx[MAX77705_VDM_RESP_VDM_HDR]);

	/*
	 * Holding nothing is answered by echoing the requested VDM back with
	 * the rest of the response zeroed, so the name matching above says
	 * only that the firmware understood the question. A real VDM header
	 * always carries an SVID and a command, so an empty one is the way to
	 * tell "there is no answer yet" from "here is the answer" -- and
	 * mistaking the two reads a fresh connection as one that has already
	 * been discovered and found to offer nothing.
	 */
	if (!*header)
		return -ENODATA;

	for (i = 0; i < nr_vdo; i++)
		vdo[i] = get_unaligned_le32(&rx[MAX77705_VDM_RESP_VDO + i * 4]);

	return 0;
}

/*
 * How long to wait for the firmware to produce a result of its own. Discovery
 * takes a few tens of milliseconds in practice, and there is nothing to lose
 * by allowing well over that.
 */
#define MAX77705_VDM_WAIT_TRIES		25
#define MAX77705_VDM_WAIT_POLL_MS	20

/**
 * max77705_typec_vdm_present - wait until the firmware has a VDM stored
 * @tc: the port
 * @id: which VDM to wait for
 *
 * The firmware clears the later VDMs on a detach, so one being readable again
 * means this connection produced it.
 */
static bool max77705_typec_vdm_present(struct max77705_typec *tc,
				       enum max77705_vdm id)
{
	u32 header, vdo;
	int i;

	for (i = 0; i < MAX77705_VDM_WAIT_TRIES; i++) {
		if (!max77705_typec_vdm_read(tc, id, &header, &vdo, 1))
			return true;

		msleep(MAX77705_VDM_WAIT_POLL_MS);
	}

	return false;
}

/*
 * Pin assignment preference, taken from the vendor driver because it is what
 * this firmware has been tested against. A partner that wants to keep USB
 * alongside DisplayPort gets a two lane assignment, and anything else gets
 * four lanes so the link is as fast as the cable allows.
 */
static const u8 max77705_dp_pins_multi_func[] = {
	DP_PIN_ASSIGN_D, DP_PIN_ASSIGN_B, DP_PIN_ASSIGN_F,
};

static const u8 max77705_dp_pins_dp_only[] = {
	DP_PIN_ASSIGN_C, DP_PIN_ASSIGN_E, DP_PIN_ASSIGN_A,
	DP_PIN_ASSIGN_D, DP_PIN_ASSIGN_B, DP_PIN_ASSIGN_F,
};

static int max77705_typec_dp_pick_pin(struct max77705_typec *tc)
{
	const u8 *order;
	size_t nr, i;

	if (tc->dp_status & DP_STATUS_PREFER_MULTI_FUNC) {
		order = max77705_dp_pins_multi_func;
		nr = ARRAY_SIZE(max77705_dp_pins_multi_func);
	} else {
		order = max77705_dp_pins_dp_only;
		nr = ARRAY_SIZE(max77705_dp_pins_dp_only);
	}

	for (i = 0; i < nr; i++)
		if (tc->dp_pin_assign & BIT(order[i]))
			return order[i];

	return -EOPNOTSUPP;
}

/*
 * The mux carries both the pin assignment, as a connector state, and the DP
 * specific VDOs, which is where a consumer finds HPD.
 */
static int max77705_typec_dp_mux_set(struct max77705_typec *tc, int pin)
{
	struct typec_displayport_data dp = {
		.status = tc->dp_status,
		.conf = tc->dp_conf,
	};
	struct typec_mux_state state = {
		.mode = TYPEC_DP_STATE_A + pin,
		.data = &dp,
	};

	return typec_mux_set(tc->mux, &state);
}

/*
 * HPD is the sink saying a display is there and the link may be trained. The
 * DisplayPort controller is reached through the connector graph rather than
 * directly, so it is told over the HPD bridge.
 */
static void max77705_typec_dp_hpd(struct max77705_typec *tc, bool hpd)
{
	/*
	 * Only on a change. The state is re-read on every interrupt, every hot
	 * plug edge and every retry, and reporting it each time buries the
	 * display driver in plug events it has already acted on -- hundreds a
	 * second, measured, with neither the link training nor the sink's EDID
	 * ever completing in between, leaving a connector that calls itself
	 * connected and offers no modes. A notification describes an edge.
	 */
	if (hpd == tc->hpd)
		return;

	tc->hpd = hpd;

	drm_aux_hpd_bridge_notify(&tc->hpd_bridge->dev,
				  hpd ? connector_status_connected :
					connector_status_disconnected);
}

static int max77705_typec_dp_configure(struct max77705_typec *tc)
{
	u32 header = VDO(USB_TYPEC_DP_SID, 1, 0,
			 VDO_OPOS(USB_TYPEC_DP_MODE) | DP_CMD_CONFIGURE);
	int pin, ret;
	u32 conf;

	pin = max77705_typec_dp_pick_pin(tc);
	if (pin < 0) {
		dev_warn(tc->dev,
			 "no pin assignment in common, partner offers 0x%02x\n",
			 tc->dp_pin_assign);
		return pin;
	}

	/*
	 * This port drives the display, so it takes the DisplayPort source
	 * role and the partner remains the sink.
	 */
	conf = DP_CONF_UFP_U_AS_UFP_D |
	       FIELD_PREP(DP_CONF_SIGNALLING_MASK, DP_CONF_SIGNALLING_HBR3) |
	       DP_CONF_SET_PIN_ASSIGN(BIT(pin));

	ret = max77705_typec_vdm_write(tc, header, &conf, 1);
	if (ret) {
		dev_warn(tc->dev, "DP Configure failed: %d\n", ret);
		return ret;
	}

	tc->dp_conf = conf;

	ret = max77705_typec_dp_mux_set(tc, pin);
	if (ret) {
		dev_warn(tc->dev, "failed to switch the mux to DP: %d\n", ret);
		return ret;
	}

	dev_dbg(tc->dev, "DisplayPort configured, pin assignment %c\n",
		'A' + pin);

	return 0;
}

static void max77705_typec_dp_status(struct max77705_typec *tc,
				     enum max77705_vdm id);

/*
 * Discover Modes is the point the firmware stops on its own, because entering
 * a mode is the AP's decision.
 */
static void max77705_typec_dp_discover_modes(struct max77705_typec *tc)
{
	u32 header, cap;
	int ret;

	ret = max77705_typec_vdm_read(tc, MAX77705_VDM_DISCOVER_MODES,
				      &header, &cap, 1);
	if (ret)
		return;

	/* The partner may also carry modes of its own, which are not ours */
	if (PD_VDO_VID(header) != USB_TYPEC_DP_SID)
		return;

	/*
	 * This port is the DisplayPort source, so what matters is the pin
	 * assignments the partner can take as the sink. Which field holds
	 * them depends on whether the partner is a plug or a receptacle.
	 */
	tc->dp_pin_assign = DP_CAP_PIN_ASSIGN_UFP_D(cap);
	if (!tc->dp_pin_assign) {
		dev_warn(tc->dev, "partner offers no DP pin assignment (0x%08x)\n",
			 cap);
		return;
	}

	header = VDO(USB_TYPEC_DP_SID, 1, 0,
		     VDO_OPOS(USB_TYPEC_DP_MODE) | CMD_ENTER_MODE);
	ret = max77705_typec_vdm_write(tc, header, NULL, 0);
	if (ret) {
		dev_warn_ratelimited(tc->dev, "DP Enter Mode failed: %d\n", ret);
		return;
	}

	/*
	 * Entering the mode makes the partner report its DisplayPort status.
	 * That arrives as an interrupt too, but for the same reasons as the
	 * discovery above it is better read than waited for; handling it twice
	 * is harmless, since it only ever restates the status.
	 */
	if (max77705_typec_vdm_present(tc, MAX77705_VDM_DP_STATUS))
		max77705_typec_dp_status(tc, MAX77705_VDM_DP_STATUS);
}

/*
 * Both a Status Update and an Attention deliver a DP status VDO, and the only
 * difference is that the first is the answer to entering the mode and so is
 * the one that has to be configured.
 */
static void max77705_typec_dp_status(struct max77705_typec *tc,
				     enum max77705_vdm id)
{
	u32 header, status;
	int pin, ret;

	ret = max77705_typec_vdm_read(tc, id, &header, &status, 1);
	if (ret)
		return;

	if (PD_VDO_VID(header) != USB_TYPEC_DP_SID)
		return;

	tc->dp_status = status;

	dev_dbg(tc->dev, "DP status 0x%08x: %s%s hpd %d\n", status,
		DP_STATUS_CONNECTION(status) ? "connected" : "disconnected",
		status & DP_STATUS_ENABLED ? ", enabled" : "",
		!!(status & DP_STATUS_HPD_STATE));

	if (DP_STATUS_CONNECTION(status) == DP_STATUS_CON_DISABLED)
		return;

	if (!tc->dp_conf) {
		if (max77705_typec_dp_configure(tc))
			return;
	} else {
		/* Already configured, so this only carries a new HPD state */
		pin = max77705_typec_dp_pick_pin(tc);
		if (pin < 0)
			return;

		ret = max77705_typec_dp_mux_set(tc, pin);
		if (ret) {
			dev_warn(tc->dev, "failed to update the mux: %d\n", ret);
			return;
		}
	}

	/* Only once the lanes are configured is HPD worth acting on */
	max77705_typec_dp_hpd(tc, status & DP_STATUS_HPD_STATE);
}

static bool max77705_typec_dp_attached(struct max77705_typec *tc)
{
	return tc->cc_state == MAX77705_CC_SINK ||
	       tc->cc_state == MAX77705_CC_SOURCE;
}

/**
 * max77705_typec_altmode_enable - turn the alternate modes on for a connection
 * @tc: the port
 *
 * Turning them off first is what makes this work on a reattach. The setting
 * survives a detach, so asking for something the chip believes it is already
 * doing achieves nothing at all -- and with it already on from the previous
 * cable, the firmware runs no discovery for the new one, which leaves the
 * driver waiting for a result that is never going to come. Measured on
 * hardware: a bare enable left the progress register on the data role swap it
 * had reached, while turning it off and on again walked it all the way to a
 * configured DisplayPort link.
 */
static int max77705_typec_altmode_enable(struct max77705_typec *tc)
{
	u8 off = 0;
	u8 on = MAX77705_ALTMODE_SRCCAP | MAX77705_ALTMODE_VDM;
	int ret;

	ret = max77705_typec_opcode_xfer(tc, MAX77705_OPCODE_SET_ALTMODE,
					 &off, sizeof(off), NULL, 0);
	if (ret)
		return ret;

	return max77705_typec_opcode_xfer(tc, MAX77705_OPCODE_SET_ALTMODE,
					  &on, sizeof(on), NULL, 0);
}

/**
 * max77705_typec_dp_supported - does this partner offer DisplayPort
 * @tc: the port
 *
 * Returns 1 when the partner's SVIDs include DisplayPort, 0 when discovery
 * has run and they do not, and a negative errno when there is no answer to
 * read yet. The firmware clears this on a detach, so an answer being there at
 * all is this connection's.
 *
 * The SVIDs come packed two to a VDO. Which half holds which is a question
 * not worth answering, since every half is searched anyway and the unused
 * ones read as zero.
 */
static int max77705_typec_dp_supported(struct max77705_typec *tc)
{
	u32 header, vdo[MAX77705_VDM_RESP_NR_VDO];
	int ret, i;

	ret = max77705_typec_vdm_read(tc, MAX77705_VDM_DISCOVER_SVIDS, &header,
				      vdo, ARRAY_SIZE(vdo));
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(vdo) * 2; i++)
		if ((u16)(vdo[i / 2] >> (16 * (i % 2))) == USB_TYPEC_DP_SID)
			return 1;

	return 0;
}

/*
 * How long to keep trying to get DisplayPort going once something is attached,
 * which has to cover a whole connection coming up: the power contract, a data
 * role swap and then the firmware's own mode discovery.
 */
/*
 * How long to keep trying to get DisplayPort going once something is attached,
 * which has to cover a whole connection coming up: the power contract, a data
 * role swap and then the firmware's own mode discovery.
 *
 * Ten seconds is known to be too short for a replug, where the data role swap
 * has been measured taking the best part of a minute, and DisplayPort cannot
 * start until the port is the DFP. Extending it is not simply a matter of a
 * bigger number, though: this work blocks for up to a mailbox timeout at a
 * time and runs on the shared system workqueue, so retrying for much longer
 * needs a workqueue of its own first. Tried without one, the phone reached the
 * network and never finished booting.
 */
#define MAX77705_DP_TRIES		40
#define MAX77705_DP_RETRY_MS		250
/* Retries between repeats of the enable, so it is asked again every 2s */
#define MAX77705_DP_REENABLE_EVERY	8

/*
 * Work out where the negotiation has got to and push it along, rather than
 * acting on whichever interrupt happened to arrive.
 *
 * The events this chip reports are not reliably delivered. Its interrupt
 * registers clear on read and the PMIC summarises them in one latched source
 * bit, so two events close together collapse into a single interrupt and the
 * later one is left sitting in a register nobody comes back to read. That has
 * been seen at every step: a reattach where Discover Modes never arrived and
 * the chip sat there with nothing further happening, and a connection that
 * negotiated perfectly and then stranded the Attention carrying HPD, so the
 * display was ready and nothing was told about it.
 *
 * None of that matters if the state is read rather than awaited, because
 * everything needed is readable at any time. So this is called from every
 * interrupt and from a retry, and each time it takes the negotiation as far as
 * it can. Enter Mode may be sent to a partner that has already entered the
 * mode, and a status may be read twice; both are harmless.
 */
static void max77705_typec_dp_sync(struct max77705_typec *tc)
{
	u32 header, vdo;

	if (!max77705_typec_dp_attached(tc))
		return;

	if (!tc->dp_conf) {
		int supported = max77705_typec_dp_supported(tc);

		/*
		 * Discovery has run and this partner has nothing to do with
		 * DisplayPort, which is the common case: a charger. Stop, so
		 * that a charge does not carry ten seconds of pointless
		 * retrying behind it.
		 */
		if (supported == 0)
			return;

		if (supported > 0) {
			if (!max77705_typec_vdm_read(tc,
						     MAX77705_VDM_DISCOVER_MODES,
						     &header, &vdo, 1))
				max77705_typec_dp_discover_modes(tc);
		} else if (tc->dp_tries &&
			   tc->dp_tries % MAX77705_DP_REENABLE_EVERY == 0) {
			/*
			 * Still nothing discovered at all. Enabling the
			 * alternate modes can land before the connection is
			 * ready to act on it, and then the firmware simply
			 * never begins discovery; nothing reports that, and
			 * re-reading a result cannot recover an enable that
			 * did not take. So ask again, spaced well apart, since
			 * this must also not interrupt a discovery already
			 * under way.
			 */
			max77705_typec_altmode_enable(tc);
		}

		/*
		 * A connection takes a while to reach mode discovery, so keep
		 * looking rather than deciding on the one glance taken when it
		 * attached.
		 */
		if (!tc->dp_conf && tc->dp_tries++ < MAX77705_DP_TRIES) {
			queue_delayed_work(system_wq, &tc->altmode_work,
					   msecs_to_jiffies(MAX77705_DP_RETRY_MS));
			return;
		}
	}

	if (!tc->dp_conf)
		return;

	/*
	 * Configured, so the only thing left to follow is HPD. An Attention is
	 * the newest word on it, and the status update stands in until one
	 * arrives.
	 */
	if (!max77705_typec_vdm_read(tc, MAX77705_VDM_ATTENTION, &header, &vdo, 1))
		max77705_typec_dp_status(tc, MAX77705_VDM_ATTENTION);
	else
		max77705_typec_dp_status(tc, MAX77705_VDM_DP_STATUS);
}

static void max77705_typec_altmode_work(struct work_struct *work)
{
	struct max77705_typec *tc = container_of(to_delayed_work(work),
						 struct max77705_typec,
						 altmode_work);
	bool enable;
	int ret;

	/*
	 * Which events arrived does not matter, only that something did: the
	 * state is read rather than inferred from the bits.
	 */
	scoped_guard(spinlock_irq, &tc->event_lock) {
		enable = tc->altmode_enable;
		tc->altmode_enable = false;
		tc->vdm_events = 0;
		tc->pd_events = 0;
	}

	if (enable) {
		ret = max77705_typec_altmode_enable(tc);
		if (ret) {
			dev_warn(tc->dev, "failed to enable alternate mode: %d\n",
				 ret);
			return;
		}
	}

	max77705_typec_dp_sync(tc);
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

	/*
	 * Nothing survives a detach: the firmware forgets the stored VDMs, and
	 * leaving the SBU switch on would hold AUX against the next cable.
	 */
	max77705_typec_dp_hpd(tc, false);

	tc->dp_pin_assign = 0;
	tc->dp_status = 0;
	tc->dp_conf = 0;
	tc->dp_tries = 0;

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

	if (state == MAX77705_CC_SINK || state == MAX77705_CC_SOURCE) {
		/*
		 * Ask for the alternate modes again on every attach. The chip
		 * does not report being in one, so there is nothing to check
		 * first, and asking twice is harmless.
		 */
		scoped_guard(spinlock_irq, &tc->event_lock)
			tc->altmode_enable = true;
		if (tc->irq_ready)
			queue_delayed_work(system_wq, &tc->altmode_work, 0);
	} else {
		struct typec_mux_state safe = { .mode = TYPEC_STATE_SAFE };

		ret = typec_mux_set(tc->mux, &safe);
		if (ret)
			dev_warn(tc->dev, "failed to park the mux: %d\n", ret);
	}

	return usb_role_switch_set_role(tc->role_sw, role);
}

static irqreturn_t max77705_typec_irq(int irq, void *data)
{
	struct max77705_typec *tc = data;
	u8 status[MAX77705_INT_COUNT];
	u8 news;
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
	 * The alternate mode steps need the mailbox to answer them, which this
	 * handler cannot use: it is the one thread that delivers the command
	 * response. Hand them to the work item.
	 *
	 * Any interrupt at all is worth handing over, not only the ones naming
	 * a DisplayPort event. Two events arriving together are reported once,
	 * so a bit that matters can be missing from the very interrupt it was
	 * meant to travel in, and looking at the state on every interrupt is
	 * what makes that survivable.
	 *
	 * Queueing must not disturb a run that is already scheduled. Bringing
	 * one forward instead would let a burst of interrupts cancel the retry
	 * backoff over and over, which turns the retry into a tight loop
	 * hammering the chip.
	 *
	 * Only the bits actually asked for count as news, and the command
	 * response is not among them however it arrives: it is this driver's
	 * own doing, it is answered above, and it says nothing about the
	 * connection. Treating it as a reason to go and look closes a loop --
	 * reading the state posts a command, the command answers with this
	 * interrupt, and that sends us back to read the state. Measured, that
	 * ran about once every fifteen milliseconds for as long as a display
	 * was attached.
	 *
	 * Masking each register down to those bits also matters, because a
	 * masked bit still latches here even though it raised nothing. The
	 * chip sets a spurious VBUS bit constantly, so anything looking at
	 * whole registers sees news on every single interrupt.
	 */
	news = (status[MAX77705_INT_CC] & (MAX77705_CC_INT_CCSTAT |
					   MAX77705_CC_INT_CCPINSTAT |
					   MAX77705_CC_INT_CCISTAT)) |
	       (status[MAX77705_INT_PD] & (MAX77705_PD_INT_ATTENTION |
					   MAX77705_PD_INT_DP_CONFIGURE |
					   MAX77705_PD_INT_DP_STATUS)) |
	       (status[MAX77705_INT_VDM] & (MAX77705_VDM_INT_DISCOVER_ID |
					    MAX77705_VDM_INT_DISCOVER_SVIDS |
					    MAX77705_VDM_INT_DISCOVER_MODES |
					    MAX77705_VDM_INT_ENTER_MODE));

	if (news && max77705_typec_dp_attached(tc)) {
		scoped_guard(spinlock, &tc->event_lock) {
			tc->vdm_events |= status[MAX77705_INT_VDM];
			tc->pd_events |= status[MAX77705_INT_PD];
		}
		queue_delayed_work(system_wq, &tc->altmode_work, 0);
	}

	dev_dbg(tc->dev, "uic 0x%02x cc 0x%02x pd 0x%02x vdm 0x%02x\n",
		status[MAX77705_INT_UIC], status[MAX77705_INT_CC],
		status[MAX77705_INT_PD], status[MAX77705_INT_VDM]);

	return IRQ_HANDLED;
}

/*
 * An edge on the board's hot plug line is only used as a prompt to go and look
 * at the chip. The DisplayPort status the firmware holds stays the authority
 * on what actually happened, and this line's value is not read at all -- what
 * matters is that it fires when a display is switched on behind an adapter
 * that has already been negotiated, which is the one thing a single Attention
 * VDM is relied on for and the one thing this chip loses most readily.
 */
static irqreturn_t max77705_typec_hpd_irq(int irq, void *data)
{
	struct max77705_typec *tc = data;

	queue_delayed_work(system_wq, &tc->altmode_work, 0);

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
	spin_lock_init(&tc->event_lock);
	INIT_DELAYED_WORK(&tc->altmode_work, max77705_typec_altmode_work);

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

	/*
	 * Optional, and only a prompt to re-read the chip rather than the
	 * source of truth, so a board without it loses nothing but the
	 * promptness.
	 */
	tc->hpd_gpio = devm_gpiod_get_optional(dev, "hpd", GPIOD_IN);
	if (IS_ERR(tc->hpd_gpio))
		return dev_err_probe(dev, PTR_ERR(tc->hpd_gpio),
				     "failed to acquire the HPD gpio\n");

	/*
	 * Allocated before anything can report HPD, and only published once
	 * the port is up, so a display controller never sees a half built
	 * connector.
	 */
	tc->hpd_bridge = devm_drm_dp_hpd_bridge_alloc(dev,
						      to_of_node(tc->fwnode));
	if (IS_ERR(tc->hpd_bridge)) {
		ret = dev_err_probe(dev, PTR_ERR(tc->hpd_bridge),
				    "failed to allocate the HPD bridge\n");
		goto err_fwnode_put;
	}

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

	if (tc->hpd_gpio) {
		ret = devm_request_threaded_irq(dev, gpiod_to_irq(tc->hpd_gpio),
						NULL, max77705_typec_hpd_irq,
						IRQF_ONESHOT |
						IRQF_TRIGGER_RISING |
						IRQF_TRIGGER_FALLING,
						"max77705-typec-hpd", tc);
		if (ret) {
			dev_err_probe(dev, ret,
				      "failed to request the HPD interrupt\n");
			goto err_port_unregister;
		}
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

	/* The four discovery steps, which are all this register carries */
	ret = regmap_write(tc->regmap, MAX77705_REG_VDM_INT_M,
			   (u8)~(MAX77705_VDM_INT_DISCOVER_ID |
				 MAX77705_VDM_INT_DISCOVER_SVIDS |
				 MAX77705_VDM_INT_DISCOVER_MODES |
				 MAX77705_VDM_INT_ENTER_MODE));
	if (ret) {
		dev_err_probe(dev, ret, "failed to unmask VDM interrupts\n");
		goto err_port_unregister;
	}

	/* Where the DisplayPort specific events arrive */
	ret = regmap_write(tc->regmap, MAX77705_REG_PD_INT_M,
			   (u8)~(MAX77705_PD_INT_ATTENTION |
				 MAX77705_PD_INT_DP_CONFIGURE |
				 MAX77705_PD_INT_DP_STATUS));
	if (ret) {
		dev_err_probe(dev, ret, "failed to unmask PD interrupts\n");
		goto err_port_unregister;
	}

	ret = devm_drm_dp_hpd_bridge_add(dev, tc->hpd_bridge);
	if (ret) {
		dev_err_probe(dev, ret, "failed to add the HPD bridge\n");
		goto err_port_unregister;
	}

	tc->irq_ready = true;

	/*
	 * Exercise the mailbox once, so that a firmware which does not answer
	 * is apparent here rather than when an alternate mode depends on it.
	 */
	ret = max77705_typec_opcode_xfer(tc, MAX77705_OPCODE_CTRL3_R,
					 NULL, 0, &ctrl3, sizeof(ctrl3));
	if (ret) {
		dev_warn(dev, "the opcode mailbox did not answer: %d\n", ret);
		return 0;
	}
	dev_dbg(dev, "opcode mailbox ready, ctrl3 0x%02x\n", ctrl3);

	/*
	 * Turn the alternate modes on for whatever the initial sync found
	 * already attached, since that attach raised no interrupt.
	 */
	if (tc->cc_state == MAX77705_CC_SINK ||
	    tc->cc_state == MAX77705_CC_SOURCE) {
		scoped_guard(spinlock_irq, &tc->event_lock)
			tc->altmode_enable = true;
		queue_delayed_work(system_wq, &tc->altmode_work, 0);
	}

	return 0;

err_port_unregister:
	tc->irq_ready = false;
	cancel_delayed_work_sync(&tc->altmode_work);
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
	regmap_write(tc->regmap, MAX77705_REG_PD_INT_M, 0xff);
	regmap_write(tc->regmap, MAX77705_REG_VDM_INT_M, 0xff);

	/* Masked above, so the interrupt can no longer queue this again */
	tc->irq_ready = false;
	cancel_delayed_work_sync(&tc->altmode_work);

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
