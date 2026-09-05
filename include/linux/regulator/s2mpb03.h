/* SPDX-License-Identifier: GPL-2.0+ */
// s2mpb03.h
//
// Copyright (c) 2017 Samsung Electronics Co., Ltd
//              http://www.samsung.com
// Copyright (C) 2026 William Floyd <git@notmy.space>

#ifndef __LINUX_S2MPB03_H
#define __LINUX_S2MPB03_H

// S2MPB03 registers
// Slave Addr : 0xAC
enum S2MPB03_reg {
	S2MPB03_REG_PMIC_ID,
	S2MPB03_REG_STATUS,
	S2MPB03_REG_CTRL,
	S2MPB03_REG_LDO1_CTRL,
	S2MPB03_REG_LDO2_CTRL,
	S2MPB03_REG_LDO3_CTRL,
	S2MPB03_REG_LDO4_CTRL,
	S2MPB03_REG_LDO5_CTRL,
	S2MPB03_REG_LDO6_CTRL,
	S2MPB03_REG_LDO7_CTRL,
	S2MPB03_REG_LDO_SLEW1,
	S2MPB03_REG_LDO_SLEW2,
};

// S2MPB03 regulator ids
enum S2MPB03_regulators {
	S2MPB03_LDO1,
	S2MPB03_LDO2,
	S2MPB03_LDO3,
	S2MPB03_LDO4,
	S2MPB03_LDO5,
	S2MPB03_LDO6,
	S2MPB03_LDO7,
	S2MPB03_REG_MAX,
};

/*
 * Each LDO holds both its enable bit and its voltage selector in a single
 * LDOn_CTRL register: bit 7 enables, bits 5:0 select the voltage.
 */
#define S2MPB03_LDO_MIN1	700000
#define S2MPB03_LDO_MIN2	1800000
#define S2MPB03_LDO_STEP1	25000
#define S2MPB03_LDO_STEP2	12500
#define S2MPB03_LDO_VSEL_MASK	0x3F
#define S2MPB03_LDO_ENABLE_MASK	0x80

/*
 * LDO_SLEW1 and LDO_SLEW2 hold the per-LDO slew rate and discharge controls:
 *
 *	LDO_SLEW1  bit 0     LDO2 remote sense, active high
 *	LDO_SLEW2  bit 0     LDO1 discharge
 *	           bit 1     LDO2 discharge
 *	           bits 3:2  LDO7 slew rate
 *	           bits 5:4  LDO6 slew rate
 *	           bits 7:6  LDO5 slew rate
 *
 * The slew rate fields select one of four rates. Only the zero encoding is
 * known -- 10mV/us -- so this driver can ask for that rate and no other.
 */
#define S2MPB03_LDO2_REMOTE_SENSE_MASK	0x01

#define S2MPB03_LDO1_DISCHARGE_MASK	0x01
#define S2MPB03_LDO2_DISCHARGE_MASK	0x02
#define S2MPB03_LDO7_SOFT_START_MASK	0x0c
#define S2MPB03_LDO6_SOFT_START_MASK	0x30
#define S2MPB03_LDO5_SOFT_START_MASK	0xc0

#define S2MPB03_RAMP_DELAY	12000

#define S2MPB03_ENABLE_TIME_LDO	150

#define S2MPB03_LDO_N_VOLTAGES	(S2MPB03_LDO_VSEL_MASK + 1)

#define S2MPB03_REGULATOR_MAX	(S2MPB03_REG_MAX)

#endif // __LINUX_S2MPB03_H
