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
