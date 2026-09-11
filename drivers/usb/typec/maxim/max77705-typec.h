/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Register map for the USB Type-C block of the Maxim MAX77705.
 *
 * The block sits at its own I2C address, separate from the PMIC address the
 * MFD parent is bound to, and runs the USB-PD state machine in firmware. The
 * AP does not implement PD: it posts commands to a mailbox and is told the
 * result, so this is a firmware PD controller in the same family as tps6598x
 * rather than anything TCPCI can describe.
 */

#ifndef __MAX77705_TYPEC_H
#define __MAX77705_TYPEC_H

#include <linux/bits.h>

/* 7-bit address of the USBC block (0x4A in the vendor 8-bit notation) */
#define MAX77705_TYPEC_I2C_ADDR		0x25

#define MAX77705_REG_UIC_HW_REV		0x00
#define MAX77705_REG_UIC_FW_REV		0x01

/* The four interrupt registers are contiguous, so they can be read at once */
#define MAX77705_REG_UIC_INT		0x02
#define MAX77705_REG_CC_INT		0x03
#define MAX77705_REG_PD_INT		0x04
#define MAX77705_REG_VDM_INT		0x05

enum max77705_int_index {
	MAX77705_INT_UIC = 0,
	MAX77705_INT_CC,
	MAX77705_INT_PD,
	MAX77705_INT_VDM,
	MAX77705_INT_COUNT,
};

#define MAX77705_REG_USBC_STATUS1	0x06
#define MAX77705_REG_USBC_STATUS2	0x07
#define MAX77705_REG_BC_STATUS		0x08

#define MAX77705_REG_CC_STATUS0		0x0a
#define MAX77705_REG_CC_STATUS1		0x0b
#define MAX77705_REG_PD_STATUS0		0x0c
#define MAX77705_REG_PD_STATUS1		0x0d

#define MAX77705_REG_UIC_INT_M		0x0e
#define MAX77705_REG_CC_INT_M		0x0f
#define MAX77705_REG_PD_INT_M		0x10
#define MAX77705_REG_VDM_INT_M		0x11

/*
 * Opcode mailbox.
 *
 * A command is one burst write starting at MAX77705_REG_OPCODE: the opcode
 * byte followed by up to MAX77705_OPCODE_DATA_LEN payload bytes. A command
 * shorter than the full payload is terminated by writing zero to
 * MAX77705_REG_OPCODE_END. The firmware answers by raising
 * MAX77705_UIC_INT_APCMDRES, after which a burst read from
 * MAX77705_REG_OPCODE_RES returns the echoed opcode followed by the response.
 */
#define MAX77705_REG_OPCODE		0x21
#define MAX77705_REG_OPCODE_END		0x41
#define MAX77705_REG_OPCODE_RES		0x51

#define MAX77705_OPCODE_DATA_LEN	32
#define MAX77705_OPCODE_TIMEOUT_MS	3000

/* Reads the third control register, and takes no payload */
#define MAX77705_OPCODE_CTRL3_R		0x09

/*
 * Sends one VDM to the partner. The answer carries the VDM header the partner
 * replied with, so it says whether the command was ACKed.
 */
#define MAX77705_OPCODE_VDM_REQ		0x48

/*
 * Reads back the VDM named by the single payload byte, an enum max77705_vdm.
 * The firmware keeps the last of each, so a result stays readable until the
 * same VDM arrives again.
 */
#define MAX77705_OPCODE_VDM_RESP	0x4b

/*
 * Enables the alternate modes, and the reason the chip otherwise looks like it
 * never begins discovery: until this is posted it answers nothing. Once it is
 * enabled the firmware runs Discover Identity, Discover SVIDs and Discover
 * Modes on its own, and the AP only has to answer with Enter Mode and DP
 * Configure.
 */
#define MAX77705_OPCODE_SET_ALTMODE	0x55
#define MAX77705_ALTMODE_SRCCAP		BIT(0)
#define MAX77705_ALTMODE_VDM		BIT(1)

/*
 * Payload of MAX77705_OPCODE_VDM_REQ: a descriptor byte, then the VDM header,
 * then the VDOs. The object count includes the header itself, and the
 * firmware wants the command type to read as an ACK even though what is being
 * sent is a request.
 */
#define MAX77705_VDM_REQ_NR_OBJ		GENMASK(2, 0)
#define MAX77705_VDM_REQ_CMD_TYPE	GENMASK(4, 3)
#define MAX77705_VDM_REQ_MAX_OBJ	2

/*
 * A MAX77705_OPCODE_VDM_RESP answer, with the echoed opcode already taken off
 * the front: which VDM this is, the PD message header, the VDM header, and
 * then the VDOs the partner sent.
 */
#define MAX77705_VDM_RESP_ID		0
#define MAX77705_VDM_RESP_MSG_HDR	1
#define MAX77705_VDM_RESP_VDM_HDR	3
#define MAX77705_VDM_RESP_VDO		7
#define MAX77705_VDM_RESP_NR_VDO	6

/* A MAX77705_OPCODE_VDM_REQ answer reports this when it holds nothing */
#define MAX77705_VDM_NO_RESPONSE	0xff

/* Selects one stored VDM for MAX77705_OPCODE_VDM_RESP */
enum max77705_vdm {
	MAX77705_VDM_DISCOVER_ID = 0x01,
	MAX77705_VDM_DISCOVER_SVIDS,
	MAX77705_VDM_DISCOVER_MODES,
	MAX77705_VDM_ENTER_MODE,
	MAX77705_VDM_EXIT_MODE,
	MAX77705_VDM_ATTENTION,
	MAX77705_VDM_DP_STATUS = 0x10,
	MAX77705_VDM_DP_CONFIGURE,
};

/* MAX77705_REG_UIC_INT */
#define MAX77705_UIC_INT_APCMDRES	BIT(7)
#define MAX77705_UIC_INT_SYSMSG		BIT(6)
#define MAX77705_UIC_INT_VBUSDET	BIT(5)
#define MAX77705_UIC_INT_VBADC		BIT(4)
#define MAX77705_UIC_INT_DCDTMO		BIT(3)
#define MAX77705_UIC_INT_CHGTYP		BIT(1)
#define MAX77705_UIC_INT_UIDADC		BIT(0)

/* MAX77705_REG_CC_INT */
#define MAX77705_CC_INT_VCONNOCP	BIT(7)
#define MAX77705_CC_INT_VSAFE0V		BIT(6)
#define MAX77705_CC_INT_ATTACH_SRC_ERR	BIT(5)
#define MAX77705_CC_INT_VCONNSC		BIT(4)
#define MAX77705_CC_INT_CCPINSTAT	BIT(3)
#define MAX77705_CC_INT_CCISTAT		BIT(2)
#define MAX77705_CC_INT_CCVCNSTAT	BIT(1)
#define MAX77705_CC_INT_CCSTAT		BIT(0)

/*
 * MAX77705_REG_PD_INT
 *
 * The DisplayPort events arrive here rather than in the VDM register, which
 * only carries the four discovery steps.
 */
#define MAX77705_PD_INT_PDMSG		BIT(7)
#define MAX77705_PD_INT_PS_RDY		BIT(6)
#define MAX77705_PD_INT_DATAROLE	BIT(5)
#define MAX77705_PD_INT_ATTENTION	BIT(4)
#define MAX77705_PD_INT_DP_CONFIGURE	BIT(3)
#define MAX77705_PD_INT_DP_STATUS	BIT(2)
#define MAX77705_PD_INT_SSACC		BIT(1)
#define MAX77705_PD_INT_FCTID		BIT(0)

/* MAX77705_REG_VDM_INT: the upper four bits are unused */
#define MAX77705_VDM_INT_ENTER_MODE	BIT(3)
#define MAX77705_VDM_INT_DISCOVER_MODES	BIT(2)
#define MAX77705_VDM_INT_DISCOVER_SVIDS	BIT(1)
#define MAX77705_VDM_INT_DISCOVER_ID	BIT(0)

/*
 * MAX77705_REG_PD_STATUS1
 *
 * The data role is the one that matters for DisplayPort: this port drives the
 * display, so it has to be the DFP, and a UFP may not begin the mode discovery
 * the firmware runs.
 */
#define MAX77705_PD_STATUS1_DATAROLE	BIT(7)	/* set while this port is DFP */
#define MAX77705_PD_STATUS1_PSRDY	BIT(4)

/*
 * Swaps a role. The payload selects which, and it is a toggle rather than a
 * request for a particular role: asking while already the DFP gives the role
 * away again.
 */
#define MAX77705_OPCODE_SWAP_REQUEST	0x37
#define MAX77705_SWAP_DATA_ROLE		0x01
#define MAX77705_SWAP_POWER_ROLE	0x02

/* MAX77705_REG_CC_STATUS0 */
#define MAX77705_CC_STATUS0_PINSTAT	GENMASK(7, 6)
#define MAX77705_CC_STATUS0_ISTAT	GENMASK(5, 4)
#define MAX77705_CC_STATUS0_VCNSTAT	BIT(3)
#define MAX77705_CC_STATUS0_CCSTAT	GENMASK(2, 0)

/* MAX77705_CC_STATUS0_PINSTAT values */
enum max77705_cc_pin {
	MAX77705_CC_PIN_UNDETERMINED = 0,
	MAX77705_CC_PIN_CC1,
	MAX77705_CC_PIN_CC2,
};

/* MAX77705_CC_STATUS0_ISTAT values: advertised Rp current when sinking */
enum max77705_cc_current {
	MAX77705_CC_CURRENT_NONE = 0,
	MAX77705_CC_CURRENT_DEFAULT,
	MAX77705_CC_CURRENT_1_5A,
	MAX77705_CC_CURRENT_3_0A,
};

/* MAX77705_CC_STATUS0_CCSTAT values */
enum max77705_cc_state {
	MAX77705_CC_NO_CONNECTION = 0,
	MAX77705_CC_SINK,
	MAX77705_CC_SOURCE,
	MAX77705_CC_AUDIO_ACCESSORY,
	MAX77705_CC_DEBUG_ACCESSORY,
	MAX77705_CC_ERROR,
	MAX77705_CC_DISABLED,
	MAX77705_CC_RFU,
};

#endif /* __MAX77705_TYPEC_H */
