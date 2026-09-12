/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __IRIS_PLATFORM_SM6225_H__
#define __IRIS_PLATFORM_SM6225_H__

/*
 * SM6225 ("khaje") carries the AR50_LITE core, the same VPU 2.0 generation the
 * rest of this file covers. Bandwidth figures are khaje's, taken from the venus
 * driver's qcm2290_bw_table_dec; bw_ddr is that table's average column.
 */
static const struct bw_info sm6225_bw_table_dec[] = {
	{ ((1920 * 1088) / 256) * 30 + ((1280 *  736) / 256) * 30, 597000 },
	{ ((1920 * 1088) / 256) * 30,                              413000 },
	{ ((1280 *  736) / 256) * 60,                              364000 },
	{ ((1280 *  736) / 256) * 30,                              182000 },
};

static const char * const sm6225_opp_pd_table[] = { "cx" };

/*
 * The DT node names the per-core clocks vcodec0_*, not vcodec_*, because it is
 * shared with the venus binding (qcom,qcm2290-venus).
 */
static const struct platform_clk_data sm6225_clk_table[] = {
	{IRIS_CTRL_CLK,     "core"          },
	{IRIS_AXI_CLK,      "iface"         },
	{IRIS_AHB_CLK,      "bus"           },
	{IRIS_THROTTLE_CLK, "throttle"      },
	{IRIS_HW_CLK,       "vcodec0_core"  },
	{IRIS_HW_AHB_CLK,   "vcodec0_bus"   },
};

static const char * const sm6225_opp_clk_table[] = {
	"vcodec0_core",
	NULL,
};

/*
 * khaje's content-protection layout differs from the shared vpu2 one: cp_size
 * is 0x70800000, matching the venus_ns virtual-addr-pool base in Qualcomm's
 * khaje-vidc.dtsi (and qcm2290_res.cp_size). The non-pixel pair matches the
 * secure_non_pixel context bank, 0x1000000 / 0x24800000.
 */
static const struct tz_cp_config tz_cp_config_sm6225[] = {
	{
		.cp_start = 0,
		.cp_size = 0x70800000,
		.cp_nonpixel_start = 0x01000000,
		.cp_nonpixel_size = 0x24800000,
	},
};

#endif
