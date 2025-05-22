// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025, The Linux Foundation. All rights reserved.
 */

#include <linux/device.h>
#include <linux/interconnect-provider.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <dt-bindings/interconnect/qcom,sm6225.h>

#include "icc-rpm.h"
#include "sm6225.h"

static const u16 apps_proc_links[] = {
	SM6225_MASTER_AMPSS_M0,
	SM6225_BIMC_SNOC_SLV,
	SM6225_SLAVE_EBI_CH0
};

static struct qcom_icc_node apps_proc = {
	.name = "apps_proc",
	.id = SM6225_MASTER_AMPSS_M0,
	.buswidth = 16,
	.mas_rpm_id = 0,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 0,
	.num_links = ARRAY_SIZE(apps_proc_links),
	.links = apps_proc_links
};

static const u16 master_snoc_bimc_rt_links[] = {
	SM6225_MASTER_SNOC_BIMC_RT,
	SM6225_SLAVE_EBI_CH0
};

static struct qcom_icc_node master_snoc_bimc_rt = {
	.name = "master_snoc_bimc_rt",
	.id = SM6225_MASTER_SNOC_BIMC_RT,
	.buswidth = 16,
	.mas_rpm_id = 163,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_BYPASS,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 2,
	.num_links = ARRAY_SIZE(master_snoc_bimc_rt_links),
	.links = master_snoc_bimc_rt_links
};

static const u16 master_snoc_bimc_nrt_links[] = {
	SM6225_MASTER_SNOC_BIMC_NRT,
	SM6225_SLAVE_EBI_CH0
};

static struct qcom_icc_node master_snoc_bimc_nrt = {
	.name = "master_snoc_bimc_nrt",
	.id = SM6225_MASTER_SNOC_BIMC_NRT,
	.buswidth = 16,
	.mas_rpm_id = 164,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_BYPASS,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 3,
	.num_links = ARRAY_SIZE(master_snoc_bimc_nrt_links),
	.links = master_snoc_bimc_nrt_links
};

static const u16 snoc_bimc_links_mas[] = {
	SM6225_SLAVE_EBI_CH0,
	SM6225_SNOC_BIMC_MAS
};

static struct qcom_icc_node snoc_bimc_mas = {
	.name = "snoc_bimc_mas",
	.id = SM6225_SNOC_BIMC_MAS,
	.buswidth = 16,
	.mas_rpm_id = 3,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_BYPASS,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 6,
	.num_links = ARRAY_SIZE(snoc_bimc_links_mas),
	.links = snoc_bimc_links_mas
};

static const u16 master_gpu_cdsp_bimc_links[] = {
	SM6225_MASTER_GPU_CDSP_PROC,
	SM6225_BIMC_SNOC_SLV,
	SM6225_SLAVE_EBI_CH0
};

static struct qcom_icc_node master_gpu_cdsp_bimc = {
	.name = "master_gpu_cdsp_bimc",
	.id = SM6225_MASTER_GPU_CDSP_PROC,
	.buswidth = 32,
	.mas_rpm_id = 165,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_BYPASS,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 1,
	.num_links = ARRAY_SIZE(master_gpu_cdsp_bimc_links),
	.links = master_gpu_cdsp_bimc_links
};

static const u16 tcu_0_links[] = {
	SM6225_MASTER_TCU_0,
	SM6225_BIMC_SNOC_SLV,
	SM6225_SLAVE_EBI_CH0
};

static struct qcom_icc_node tcu_0 = {
	.name = "tcu_0",
	.id = SM6225_MASTER_TCU_0,
	.buswidth = 8,
	.mas_rpm_id = 102,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 4,
	.num_links = ARRAY_SIZE(tcu_0_links),
	.links = tcu_0_links
};

static const u16 snoc_cnoc_links_mas[] = {
	SM6225_SLAVE_PIMEM_CFG,
	SM6225_SLAVE_QUP_0,
	SM6225_SLAVE_SDCC_2,
	SM6225_SLAVE_SDCC_1,
	SM6225_SLAVE_QDSS_CFG,
	SM6225_SLAVE_VSENSE_CTRL_CFG,
	SM6225_SLAVE_LPASS,
	SM6225_SLAVE_UFS_MEM_CFG,
	SM6225_SLAVE_TLMM_EAST,
	SM6225_SLAVE_PMIC_ARB,
	SM6225_SLAVE_IMEM_CFG,
	SM6225_SLAVE_VENUS_CFG,
	SM6225_SLAVE_DISPLAY_THROTTLE_CFG,
	SM6225_SLAVE_IPA_CFG,
	SM6225_SLAVE_QM_MPU_CFG,
	SM6225_SLAVE_CLK_CTL,
	SM6225_SLAVE_CAMERA_NRT_THROTTLE_CFG,
	SM6225_SLAVE_VENUS_THROTTLE_CFG,
	SM6225_SLAVE_CDSP_THROTTLE_CFG,
	SM6225_SLAVE_BIMC_CFG,
	SM6225_SLAVE_GPU_CFG,
	SM6225_SLAVE_TCSR,
	SM6225_SLAVE_USB3,
	SM6225_SLAVE_CAMERA_RT_THROTTLE_CFG,
	SM6225_SLAVE_PRNG,
	SM6225_SLAVE_CRYPTO_0_CFG,
	SM6225_SLAVE_SERVICE_CNOC,
	SM6225_SLAVE_MESSAGE_RAM,
	SM6225_SNOC_CNOC_MAS,
	SM6225_SLAVE_TLMM_SOUTH,
	SM6225_SLAVE_DISPLAY_CFG,
	SM6225_SLAVE_TLMM_WEST,
	SM6225_SLAVE_CAMERA_CFG,
	SM6225_SLAVE_PDM,
	SM6225_SLAVE_QM_CFG,
	SM6225_SLAVE_SNOC_CFG
};

static struct qcom_icc_node snoc_cnoc_mas = {
	.name = "snoc_cnoc_mas",
	.id = SM6225_SNOC_CNOC_MAS,
	.buswidth = 8,
	.mas_rpm_id = 52,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(snoc_cnoc_links_mas),
	.links = snoc_cnoc_links_mas
};

static const u16 xm_dap_links[] = {
	SM6225_SLAVE_PIMEM_CFG,
	SM6225_SLAVE_QUP_0,
	SM6225_SLAVE_SDCC_2,
	SM6225_SLAVE_SDCC_1,
	SM6225_SLAVE_QDSS_CFG,
	SM6225_SLAVE_VSENSE_CTRL_CFG,
	SM6225_SLAVE_LPASS,
	SM6225_SLAVE_UFS_MEM_CFG,
	SM6225_SLAVE_TLMM_EAST,
	SM6225_SLAVE_PMIC_ARB,
	SM6225_SLAVE_IMEM_CFG,
	SM6225_SLAVE_VENUS_CFG,
	SM6225_MASTER_QDSS_DAP,
	SM6225_SLAVE_DISPLAY_THROTTLE_CFG,
	SM6225_SLAVE_IPA_CFG,
	SM6225_SLAVE_QM_MPU_CFG,
	SM6225_SLAVE_CLK_CTL,
	SM6225_SLAVE_CAMERA_NRT_THROTTLE_CFG,
	SM6225_SLAVE_VENUS_THROTTLE_CFG,
	SM6225_SLAVE_CDSP_THROTTLE_CFG,
	SM6225_SLAVE_BIMC_CFG,
	SM6225_SLAVE_GPU_CFG,
	SM6225_SLAVE_TCSR,
	SM6225_SLAVE_USB3,
	SM6225_SLAVE_CAMERA_RT_THROTTLE_CFG,
	SM6225_SLAVE_PRNG,
	SM6225_SLAVE_CRYPTO_0_CFG,
	SM6225_SLAVE_SERVICE_CNOC,
	SM6225_SLAVE_MESSAGE_RAM,
	SM6225_SLAVE_TLMM_SOUTH,
	SM6225_SLAVE_DISPLAY_CFG,
	SM6225_SLAVE_TLMM_WEST,
	SM6225_SLAVE_CAMERA_CFG,
	SM6225_SLAVE_PDM,
	SM6225_SLAVE_QM_CFG,
	SM6225_SLAVE_SNOC_CFG
};

static struct qcom_icc_node xm_dap = {
	.name = "xm_dap",
	.id = SM6225_MASTER_QDSS_DAP,
	.buswidth = 8,
	.mas_rpm_id = 49,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(xm_dap_links),
	.links = xm_dap_links
};

static const u16 crypto_c0_links[] = {
	SM6225_MASTER_CRYPTO_CORE0,
	SM6225_SLAVE_ANOC_SNOC
};

static struct qcom_icc_node crypto_c0 = {
	.name = "crypto_c0",
	.id = SM6225_MASTER_CRYPTO_CORE0,
	.buswidth = 8,
	.mas_rpm_id = 23,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 22,
	.num_links = ARRAY_SIZE(crypto_c0_links),
	.links = crypto_c0_links
};

static const u16 qup_core_master_0_links[] = {
	SM6225_MASTER_QUP_CORE_0,
	SM6225_SLAVE_QUP_CORE_0
};

static struct qcom_icc_node qup_core_master_0 = {
	.name = "qup_core_master_0",
	.id = SM6225_MASTER_QUP_CORE_0,
	.buswidth = 4,
	.mas_rpm_id = 170,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qup_core_master_0_links),
	.links = qup_core_master_0_links
};

static const u16 master_snoc_cfg_links[] = {
	SM6225_SLAVE_SERVICE_SNOC,
	SM6225_MASTER_SNOC_CFG
};

static struct qcom_icc_node master_snoc_cfg = {
	.name = "master_snoc_cfg",
	.id = SM6225_MASTER_SNOC_CFG,
	.buswidth = 4,
	.mas_rpm_id = 20,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(master_snoc_cfg_links),
	.links = master_snoc_cfg_links
};

static const u16 qhm_tic_links[] = {
	SM6225_SLAVE_QDSS_STM,
	SM6225_SLAVE_APPSS,
	SM6225_SNOC_BIMC_SLV,
	SM6225_SLAVE_OCIMEM,
	SM6225_SLAVE_PIMEM,
	SM6225_MASTER_TIC,
	SM6225_SLAVE_TCU,
	SM6225_SNOC_CNOC_SLV
};

static struct qcom_icc_node qhm_tic = {
	.name = "qhm_tic",
	.id = SM6225_MASTER_TIC,
	.buswidth = 4,
	.mas_rpm_id = 51,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhm_tic_links),
	.links = qhm_tic_links
};

static const u16 master_anoc_snoc_links[] = {
	SM6225_SLAVE_QDSS_STM,
	SM6225_SLAVE_APPSS,
	SM6225_SNOC_BIMC_SLV,
	SM6225_MASTER_ANOC_SNOC,
	SM6225_SLAVE_OCIMEM,
	SM6225_SLAVE_PIMEM,
	SM6225_SLAVE_TCU,
	SM6225_SNOC_CNOC_SLV
};

static struct qcom_icc_node master_anoc_snoc = {
	.name = "master_anoc_snoc",
	.id = SM6225_MASTER_ANOC_SNOC,
	.buswidth = 16,
	.mas_rpm_id = 110,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(master_anoc_snoc_links),
	.links = master_anoc_snoc_links
};

static const u16 qnm_camera_nrt_links[] = {
	SM6225_SLAVE_SNOC_BIMC_NRT,
	SM6225_MASTER_CAMNOC_SF
};

static struct qcom_icc_node qnm_camera_nrt = {
	.name = "qnm_camera_nrt",
	.id = SM6225_MASTER_CAMNOC_SF,
	.buswidth = 32,
	.mas_rpm_id = 172,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 4,
	.num_links = ARRAY_SIZE(qnm_camera_nrt_links),
	.links = qnm_camera_nrt_links
};

static const u16 qnm_camera_rt_links[] = {
	SM6225_SLAVE_SNOC_BIMC_RT,
	SM6225_MASTER_CAMNOC_HF
};

static struct qcom_icc_node qnm_camera_rt = {
	.name = "qnm_camera_rt",
	.id = SM6225_MASTER_CAMNOC_HF,
	.buswidth = 32,
	.mas_rpm_id = 173,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 10,
	.num_links = ARRAY_SIZE(qnm_camera_rt_links),
	.links = qnm_camera_rt_links
};

static const u16 bimc_snoc_links_mas[] = {
	SM6225_SLAVE_QDSS_STM,
	SM6225_SLAVE_APPSS,
	SM6225_SLAVE_OCIMEM,
	SM6225_SLAVE_PIMEM,
	SM6225_SLAVE_TCU,
	SM6225_SNOC_CNOC_SLV,
	SM6225_BIMC_SNOC_MAS
};

static struct qcom_icc_node bimc_snoc_mas = {
	.name = "bimc_snoc_mas",
	.id = SM6225_BIMC_SNOC_MAS,
	.buswidth = 8,
	.mas_rpm_id = 21,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(bimc_snoc_links_mas),
	.links = bimc_snoc_links_mas
};

static const u16 qxm_mdp0_links[] = {
	SM6225_MASTER_MDP_PORT0,
	SM6225_SLAVE_SNOC_BIMC_RT
};

static struct qcom_icc_node qxm_mdp0 = {
	.name = "qxm_mdp0",
	.id = SM6225_MASTER_MDP_PORT0,
	.buswidth = 16,
	.mas_rpm_id = 8,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 5,
	.num_links = ARRAY_SIZE(qxm_mdp0_links),
	.links = qxm_mdp0_links
};

static const u16 qxm_pimem_links[] = {
	SM6225_SNOC_BIMC_SLV,
	SM6225_SLAVE_OCIMEM,
	SM6225_MASTER_PIMEM
};

static struct qcom_icc_node qxm_pimem = {
	.name = "qxm_pimem",
	.id = SM6225_MASTER_PIMEM,
	.buswidth = 8,
	.mas_rpm_id = 113,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 20,
	.num_links = ARRAY_SIZE(qxm_pimem_links),
	.links = qxm_pimem_links
};

static const u16 qxm_venus0_links[] = {
	SM6225_MASTER_VIDEO_P0,
	SM6225_SLAVE_SNOC_BIMC_NRT
};

static struct qcom_icc_node qxm_venus0 = {
	.name = "qxm_venus0",
	.id = SM6225_MASTER_VIDEO_P0,
	.buswidth = 16,
	.mas_rpm_id = 9,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 9,
	.num_links = ARRAY_SIZE(qxm_venus0_links),
	.links = qxm_venus0_links
};

static const u16 qxm_venus_cpu_links[] = {
	SM6225_MASTER_VIDEO_PROC,
	SM6225_SLAVE_SNOC_BIMC_NRT
};

static struct qcom_icc_node qxm_venus_cpu = {
	.name = "qxm_venus_cpu",
	.id = SM6225_MASTER_VIDEO_PROC,
	.buswidth = 8,
	.mas_rpm_id = 168,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 13,
	.num_links = ARRAY_SIZE(qxm_venus_cpu_links),
	.links = qxm_venus_cpu_links
};

static const u16 qhm_qdss_bam_links[] = {
	SM6225_MASTER_QDSS_BAM,
	SM6225_SLAVE_ANOC_SNOC
};

static struct qcom_icc_node qhm_qdss_bam = {
	.name = "qhm_qdss_bam",
	.id = SM6225_MASTER_QDSS_BAM,
	.buswidth = 4,
	.mas_rpm_id = 19,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 2,
	.num_links = ARRAY_SIZE(qhm_qdss_bam_links),
	.links = qhm_qdss_bam_links
};

static const u16 qhm_qup0_links[] = {
	SM6225_MASTER_QUP_0,
	SM6225_SLAVE_ANOC_SNOC
};

static struct qcom_icc_node qhm_qup0 = {
	.name = "qhm_qup0",
	.id = SM6225_MASTER_QUP_0,
	.buswidth = 4,
	.mas_rpm_id = 166,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 0,
	.num_links = ARRAY_SIZE(qhm_qup0_links),
	.links = qhm_qup0_links
};

static const u16 qxm_ipa_links[] = {
	SM6225_MASTER_IPA,
	SM6225_SLAVE_ANOC_SNOC
};

static struct qcom_icc_node qxm_ipa = {
	.name = "qxm_ipa",
	.id = SM6225_MASTER_IPA,
	.buswidth = 8,
	.mas_rpm_id = 59,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 3,
	.num_links = ARRAY_SIZE(qxm_ipa_links),
	.links = qxm_ipa_links
};

static const u16 xm_qdss_etr_links[] = {
	SM6225_SLAVE_ANOC_SNOC,
	SM6225_MASTER_QDSS_ETR
};

static struct qcom_icc_node xm_qdss_etr = {
	.name = "xm_qdss_etr",
	.id = SM6225_MASTER_QDSS_ETR,
	.buswidth = 8,
	.mas_rpm_id = 31,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 12,
	.num_links = ARRAY_SIZE(xm_qdss_etr_links),
	.links = xm_qdss_etr_links
};

static const u16 xm_sdc1_links[] = {
	SM6225_SLAVE_ANOC_SNOC,
	SM6225_MASTER_SDCC_1
};

static struct qcom_icc_node xm_sdc1 = {
	.name = "xm_sdc1",
	.id = SM6225_MASTER_SDCC_1,
	.buswidth = 8,
	.mas_rpm_id = 33,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 17,
	.num_links = ARRAY_SIZE(xm_sdc1_links),
	.links = xm_sdc1_links
};

static const u16 xm_sdc2_links[] = {
	SM6225_SLAVE_ANOC_SNOC,
	SM6225_MASTER_SDCC_2
};

static struct qcom_icc_node xm_sdc2 = {
	.name = "xm_sdc2",
	.id = SM6225_MASTER_SDCC_2,
	.buswidth = 8,
	.mas_rpm_id = 35,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 23,
	.num_links = ARRAY_SIZE(xm_sdc2_links),
	.links = xm_sdc2_links
};

static const u16 xm_ufs_mem_links[] = {
	SM6225_MASTER_UFS_MEM,
	SM6225_SLAVE_ANOC_SNOC
};

static struct qcom_icc_node xm_ufs_mem = {
	.name = "xm_ufs_mem",
	.id = SM6225_MASTER_UFS_MEM,
	.buswidth = 8,
	.mas_rpm_id = 167,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 25,
	.num_links = ARRAY_SIZE(xm_ufs_mem_links),
	.links = xm_ufs_mem_links
};

static const u16 xm_usb3_0_links[] = {
	SM6225_SLAVE_ANOC_SNOC,
	SM6225_MASTER_USB3
};

static struct qcom_icc_node xm_usb3_0 = {
	.name = "xm_usb3_0",
	.id = SM6225_MASTER_USB3,
	.buswidth = 8,
	.mas_rpm_id = 32,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 24,
	.num_links = ARRAY_SIZE(xm_usb3_0_links),
	.links = xm_usb3_0_links
};

static const u16 qnm_gpu_qos_links[] = {
	SM6225_MASTER_GRAPHICS_3D_PORT1
};

static struct qcom_icc_node qnm_gpu_qos = {
	.name = "qnm_gpu_qos",
	.id = SM6225_MASTER_GRAPHICS_3D_PORT1,
	.buswidth = 32,
	.mas_rpm_id = 6,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 16,
	.num_links = ARRAY_SIZE(qnm_gpu_qos_links),
	.links = qnm_gpu_qos_links
};

static const u16 qnm_gpu_links[] = {
	SM6225_SLAVE_GPU_CDSP_BIMC,
	SM6225_MASTER_GRAPHICS_3D
};

static struct qcom_icc_node qnm_gpu = {
	.name = "qnm_gpu",
	.id = SM6225_MASTER_GRAPHICS_3D,
	.buswidth = 32,
	.mas_rpm_id = 6,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qnm_gpu_links),
	.links = qnm_gpu_links
};

static const u16 ebi_links[] = {
	SM6225_SLAVE_EBI_CH0
};

static struct qcom_icc_node ebi = {
	.name = "ebi",
	.id = SM6225_SLAVE_EBI_CH0,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 0,
	.qos.ap_owned = false,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(ebi_links),
	.links = ebi_links
};

static const u16 bimc_snoc_links_slv[] = {
	SM6225_BIMC_SNOC_SLV,
	SM6225_BIMC_SNOC_MAS
};

static struct qcom_icc_node bimc_snoc_slv = {
	.name = "bimc_snoc_slv",
	.id = SM6225_BIMC_SNOC_SLV,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 2,
	.qos.ap_owned = false,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(bimc_snoc_links_slv),
	.links = bimc_snoc_links_slv
};

static const u16 qhs_bimc_cfg_links[] = {
	SM6225_SLAVE_BIMC_CFG
};

static struct qcom_icc_node qhs_bimc_cfg = {
	.name = "qhs_bimc_cfg",
	.id = SM6225_SLAVE_BIMC_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 56,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_bimc_cfg_links),
	.links = qhs_bimc_cfg_links
};

static const u16 qhs_camera_nrt_throtle_cfg_links[] = {
	SM6225_SLAVE_CAMERA_NRT_THROTTLE_CFG
};

static struct qcom_icc_node qhs_camera_nrt_throtle_cfg = {
	.name = "qhs_camera_nrt_throtle_cfg",
	.id = SM6225_SLAVE_CAMERA_NRT_THROTTLE_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 271,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_camera_nrt_throtle_cfg_links),
	.links = qhs_camera_nrt_throtle_cfg_links
};

static const u16 qhs_camera_rt_throttle_cfg_links[] = {
	SM6225_SLAVE_CAMERA_RT_THROTTLE_CFG
};

static struct qcom_icc_node qhs_camera_rt_throttle_cfg = {
	.name = "qhs_camera_rt_throttle_cfg",
	.id = SM6225_SLAVE_CAMERA_RT_THROTTLE_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 279,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_camera_rt_throttle_cfg_links),
	.links = qhs_camera_rt_throttle_cfg_links
};

static const u16 qhs_camera_ss_cfg_links[] = {
	SM6225_SLAVE_CAMERA_CFG
};

static struct qcom_icc_node qhs_camera_ss_cfg = {
	.name = "qhs_camera_ss_cfg",
	.id = SM6225_SLAVE_CAMERA_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 3,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_camera_ss_cfg_links),
	.links = qhs_camera_ss_cfg_links
};

static const u16 qhs_cdsp_throttle_cfg_links[] = {
	SM6225_SLAVE_CDSP_THROTTLE_CFG
};

static struct qcom_icc_node qhs_cdsp_throttle_cfg = {
	.name = "qhs_cdsp_throttle_cfg",
	.id = SM6225_SLAVE_CDSP_THROTTLE_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 272,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_cdsp_throttle_cfg_links),
	.links = qhs_cdsp_throttle_cfg_links
};

static const u16 qhs_clk_ctl_links[] = {
	SM6225_SLAVE_CLK_CTL
};

static struct qcom_icc_node qhs_clk_ctl = {
	.name = "qhs_clk_ctl",
	.id = SM6225_SLAVE_CLK_CTL,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 47,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_clk_ctl_links),
	.links = qhs_clk_ctl_links
};

static const u16 qhs_crypto0_cfg_links[] = {
	SM6225_SLAVE_CRYPTO_0_CFG
};

static struct qcom_icc_node qhs_crypto0_cfg = {
	.name = "qhs_crypto0_cfg",
	.id = SM6225_SLAVE_CRYPTO_0_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 52,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_crypto0_cfg_links),
	.links = qhs_crypto0_cfg_links
};

static const u16 qhs_disp_ss_cfg_links[] = {
	SM6225_SLAVE_DISPLAY_CFG
};

static struct qcom_icc_node qhs_disp_ss_cfg = {
	.name = "qhs_disp_ss_cfg",
	.id = SM6225_SLAVE_DISPLAY_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 4,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_disp_ss_cfg_links),
	.links = qhs_disp_ss_cfg_links
};

static const u16 qhs_display_throttle_cfg_links[] = {
	SM6225_SLAVE_DISPLAY_THROTTLE_CFG
};

static struct qcom_icc_node qhs_display_throttle_cfg = {
	.name = "qhs_display_throttle_cfg",
	.id = SM6225_SLAVE_DISPLAY_THROTTLE_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 156,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_display_throttle_cfg_links),
	.links = qhs_display_throttle_cfg_links
};

static const u16 qhs_gpu_cfg_links[] = {
	SM6225_SLAVE_GPU_CFG
};

static struct qcom_icc_node qhs_gpu_cfg = {
	.name = "qhs_gpu_cfg",
	.id = SM6225_SLAVE_GPU_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 275,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_gpu_cfg_links),
	.links = qhs_gpu_cfg_links
};

static const u16 qhs_imem_cfg_links[] = {
	SM6225_SLAVE_IMEM_CFG
};

static struct qcom_icc_node qhs_imem_cfg = {
	.name = "qhs_imem_cfg",
	.id = SM6225_SLAVE_IMEM_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 54,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_imem_cfg_links),
	.links = qhs_imem_cfg_links
};

static const u16 qhs_ipa_cfg_links[] = {
	SM6225_SLAVE_IPA_CFG
};

static struct qcom_icc_node qhs_ipa_cfg = {
	.name = "qhs_ipa_cfg",
	.id = SM6225_SLAVE_IPA_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 183,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_ipa_cfg_links),
	.links = qhs_ipa_cfg_links
};

static const u16 qhs_lpass_links[] = {
	SM6225_SLAVE_LPASS
};

static struct qcom_icc_node qhs_lpass = {
	.name = "qhs_lpass",
	.id = SM6225_SLAVE_LPASS,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 21,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_lpass_links),
	.links = qhs_lpass_links
};

static const u16 qhs_mesg_ram_links[] = {
	SM6225_SLAVE_MESSAGE_RAM
};

static struct qcom_icc_node qhs_mesg_ram = {
	.name = "qhs_mesg_ram",
	.id = SM6225_SLAVE_MESSAGE_RAM,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 55,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_mesg_ram_links),
	.links = qhs_mesg_ram_links
};

static const u16 qhs_pdm_links[] = {
	SM6225_SLAVE_PDM
};

static struct qcom_icc_node qhs_pdm = {
	.name = "qhs_pdm",
	.id = SM6225_SLAVE_PDM,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 41,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_pdm_links),
	.links = qhs_pdm_links
};

static const u16 qhs_pimem_cfg_links[] = {
	SM6225_SLAVE_PIMEM_CFG
};

static struct qcom_icc_node qhs_pimem_cfg = {
	.name = "qhs_pimem_cfg",
	.id = SM6225_SLAVE_PIMEM_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 167,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_pimem_cfg_links),
	.links = qhs_pimem_cfg_links
};

static const u16 qhs_pmic_arb_links[] = {
	SM6225_SLAVE_PMIC_ARB
};

static struct qcom_icc_node qhs_pmic_arb = {
	.name = "qhs_pmic_arb",
	.id = SM6225_SLAVE_PMIC_ARB,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 59,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_pmic_arb_links),
	.links = qhs_pmic_arb_links
};

static const u16 qhs_prng_links[] = {
	SM6225_SLAVE_PRNG
};

static struct qcom_icc_node qhs_prng = {
	.name = "qhs_prng",
	.id = SM6225_SLAVE_PRNG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 44,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_prng_links),
	.links = qhs_prng_links
};

static const u16 qhs_qdss_cfg_links[] = {
	SM6225_SLAVE_QDSS_CFG
};

static struct qcom_icc_node qhs_qdss_cfg = {
	.name = "qhs_qdss_cfg",
	.id = SM6225_SLAVE_QDSS_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 63,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_qdss_cfg_links),
	.links = qhs_qdss_cfg_links
};

static const u16 qhs_qm_cfg_links[] = {
	SM6225_SLAVE_QM_CFG
};

static struct qcom_icc_node qhs_qm_cfg = {
	.name = "qhs_qm_cfg",
	.id = SM6225_SLAVE_QM_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 212,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_qm_cfg_links),
	.links = qhs_qm_cfg_links
};

static const u16 qhs_qm_mpu_cfg_links[] = {
	SM6225_SLAVE_QM_MPU_CFG
};

static struct qcom_icc_node qhs_qm_mpu_cfg = {
	.name = "qhs_qm_mpu_cfg",
	.id = SM6225_SLAVE_QM_MPU_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 231,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_qm_mpu_cfg_links),
	.links = qhs_qm_mpu_cfg_links
};

static const u16 qhs_qup0_links[] = {
	SM6225_SLAVE_QUP_0
};

static struct qcom_icc_node qhs_qup0 = {
	.name = "qhs_qup0",
	.id = SM6225_SLAVE_QUP_0,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 261,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_qup0_links),
	.links = qhs_qup0_links
};

static const u16 qhs_sdc1_links[] = {
	SM6225_SLAVE_SDCC_1
};

static struct qcom_icc_node qhs_sdc1 = {
	.name = "qhs_sdc1",
	.id = SM6225_SLAVE_SDCC_1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 31,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_sdc1_links),
	.links = qhs_sdc1_links
};

static const u16 qhs_sdc2_links[] = {
	SM6225_SLAVE_SDCC_2
};

static struct qcom_icc_node qhs_sdc2 = {
	.name = "qhs_sdc2",
	.id = SM6225_SLAVE_SDCC_2,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 33,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_sdc2_links),
	.links = qhs_sdc2_links
};

static const u16 slave_snoc_cfg_links[] = {
	SM6225_MASTER_SNOC_CFG,
	SM6225_SLAVE_SNOC_CFG
};

static struct qcom_icc_node slave_snoc_cfg = {
	.name = "slave_snoc_cfg",
	.id = SM6225_SLAVE_SNOC_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 70,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slave_snoc_cfg_links),
	.links = slave_snoc_cfg_links
};

static const u16 qhs_tcsr_links[] = {
	SM6225_SLAVE_TCSR
};

static struct qcom_icc_node qhs_tcsr = {
	.name = "qhs_tcsr",
	.id = SM6225_SLAVE_TCSR,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 50,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_tcsr_links),
	.links = qhs_tcsr_links
};

static const u16 qhs_tlmm_east_links[] = {
	SM6225_SLAVE_TLMM_EAST
};

static struct qcom_icc_node qhs_tlmm_east = {
	.name = "qhs_tlmm_east",
	.id = SM6225_SLAVE_TLMM_EAST,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 213,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_tlmm_east_links),
	.links = qhs_tlmm_east_links
};

static const u16 qhs_tlmm_south_links[] = {
	SM6225_SLAVE_TLMM_SOUTH
};

static struct qcom_icc_node qhs_tlmm_south = {
	.name = "qhs_tlmm_south",
	.id = SM6225_SLAVE_TLMM_SOUTH,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 216,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_tlmm_south_links),
	.links = qhs_tlmm_south_links
};

static const u16 qhs_tlmm_west_links[] = {
	SM6225_SLAVE_TLMM_WEST
};

static struct qcom_icc_node qhs_tlmm_west = {
	.name = "qhs_tlmm_west",
	.id = SM6225_SLAVE_TLMM_WEST,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 215,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_tlmm_west_links),
	.links = qhs_tlmm_west_links
};

static const u16 qhs_ufs_mem_cfg_links[] = {
	SM6225_SLAVE_UFS_MEM_CFG
};

static struct qcom_icc_node qhs_ufs_mem_cfg = {
	.name = "qhs_ufs_mem_cfg",
	.id = SM6225_SLAVE_UFS_MEM_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 262,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_ufs_mem_cfg_links),
	.links = qhs_ufs_mem_cfg_links
};

static const u16 qhs_usb3_links[] = {
	SM6225_SLAVE_USB3
};

static struct qcom_icc_node qhs_usb3 = {
	.name = "qhs_usb3",
	.id = SM6225_SLAVE_USB3,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 22,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_usb3_links),
	.links = qhs_usb3_links
};

static const u16 qhs_venus_cfg_links[] = {
	SM6225_SLAVE_VENUS_CFG
};

static struct qcom_icc_node qhs_venus_cfg = {
	.name = "qhs_venus_cfg",
	.id = SM6225_SLAVE_VENUS_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 10,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_venus_cfg_links),
	.links = qhs_venus_cfg_links
};

static const u16 qhs_venus_throttle_cfg_links[] = {
	SM6225_SLAVE_VENUS_THROTTLE_CFG
};

static struct qcom_icc_node qhs_venus_throttle_cfg = {
	.name = "qhs_venus_throttle_cfg",
	.id = SM6225_SLAVE_VENUS_THROTTLE_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 178,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_venus_throttle_cfg_links),
	.links = qhs_venus_throttle_cfg_links
};

static const u16 qhs_vsense_ctrl_cfg_links[] = {
	SM6225_SLAVE_VSENSE_CTRL_CFG
};

static struct qcom_icc_node qhs_vsense_ctrl_cfg = {
	.name = "qhs_vsense_ctrl_cfg",
	.id = SM6225_SLAVE_VSENSE_CTRL_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 263,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_vsense_ctrl_cfg_links),
	.links = qhs_vsense_ctrl_cfg_links
};

static const u16 srvc_cnoc_links[] = {
	SM6225_SLAVE_SERVICE_CNOC
};

static struct qcom_icc_node srvc_cnoc = {
	.name = "srvc_cnoc",
	.id = SM6225_SLAVE_SERVICE_CNOC,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 76,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(srvc_cnoc_links),
	.links = srvc_cnoc_links
};

static const u16 qup_core_slave_0_links[] = {
	SM6225_SLAVE_QUP_CORE_0
};

static struct qcom_icc_node qup_core_slave_0 = {
	.name = "qup_core_slave_0",
	.id = SM6225_SLAVE_QUP_CORE_0,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 264,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qup_core_slave_0_links),
	.links = qup_core_slave_0_links
};

static const u16 qhs_apss_links[] = {
	SM6225_SLAVE_APPSS
};

static struct qcom_icc_node qhs_apss = {
	.name = "qhs_apss",
	.id = SM6225_SLAVE_APPSS,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 20,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qhs_apss_links),
	.links = qhs_apss_links
};

static const u16 slave_snoc_bimc_nrt_links[] = {
	SM6225_SLAVE_SNOC_BIMC_NRT,
	SM6225_MASTER_SNOC_BIMC_NRT
};

static struct qcom_icc_node slave_snoc_bimc_nrt = {
	.name = "slave_snoc_bimc_nrt",
	.id = SM6225_SLAVE_SNOC_BIMC_NRT,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 259,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slave_snoc_bimc_nrt_links),
	.links = slave_snoc_bimc_nrt_links
};

static const u16 slave_snoc_bimc_rt_links[] = {
	SM6225_SLAVE_SNOC_BIMC_RT,
	SM6225_MASTER_SNOC_BIMC_RT
};

static struct qcom_icc_node slave_snoc_bimc_rt = {
	.name = "slave_snoc_bimc_rt",
	.id = SM6225_SLAVE_SNOC_BIMC_RT,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 260,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slave_snoc_bimc_rt_links),
	.links = slave_snoc_bimc_rt_links
};

static const u16 snoc_cnoc_links_slv[] = {
	SM6225_SNOC_CNOC_SLV,
	SM6225_SNOC_CNOC_MAS
};

static struct qcom_icc_node snoc_cnoc_slv = {
	.name = "snoc_cnoc_slv",
	.id = SM6225_SNOC_CNOC_SLV,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 25,
	.qos.ap_owned = false,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 8,
	.num_links = ARRAY_SIZE(snoc_cnoc_links_slv),
	.links = snoc_cnoc_links_slv
};

static const u16 qxs_imem_links[] = {
	SM6225_SLAVE_OCIMEM
};

static struct qcom_icc_node qxs_imem = {
	.name = "qxs_imem",
	.id = SM6225_SLAVE_OCIMEM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 26,
	.qos.ap_owned = false,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qxs_imem_links),
	.links = qxs_imem_links
};

static const u16 qxs_pimem_links[] = {
	SM6225_SLAVE_PIMEM
};

static struct qcom_icc_node qxs_pimem = {
	.name = "qxs_pimem",
	.id = SM6225_SLAVE_PIMEM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 166,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(qxs_pimem_links),
	.links = qxs_pimem_links
};

static const u16 snoc_bimc_links_slv[] = {
	SM6225_SNOC_BIMC_SLV,
	SM6225_SNOC_BIMC_MAS
};

static struct qcom_icc_node snoc_bimc_slv = {
	.name = "snoc_bimc_slv",
	.id = SM6225_SNOC_BIMC_SLV,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 24,
	.qos.ap_owned = false,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(snoc_bimc_links_slv),
	.links = snoc_bimc_links_slv
};

static const u16 srvc_snoc_links[] = {
	SM6225_SLAVE_SERVICE_SNOC
};

static struct qcom_icc_node srvc_snoc = {
	.name = "srvc_snoc",
	.id = SM6225_SLAVE_SERVICE_SNOC,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 29,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(srvc_snoc_links),
	.links = srvc_snoc_links
};

static const u16 xs_qdss_stm_links[] = {
	SM6225_SLAVE_QDSS_STM
};

static struct qcom_icc_node xs_qdss_stm = {
	.name = "xs_qdss_stm",
	.id = SM6225_SLAVE_QDSS_STM,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 30,
	.qos.ap_owned = false,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(xs_qdss_stm_links),
	.links = xs_qdss_stm_links
};

static const u16 xs_sys_tcu_cfg_links[] = {
	SM6225_SLAVE_TCU
};

static struct qcom_icc_node xs_sys_tcu_cfg = {
	.name = "xs_sys_tcu_cfg",
	.id = SM6225_SLAVE_TCU,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 133,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(xs_sys_tcu_cfg_links),
	.links = xs_sys_tcu_cfg_links
};

static const u16 slave_anoc_snoc_links[] = {
	SM6225_MASTER_ANOC_SNOC,
	SM6225_SLAVE_ANOC_SNOC
};

static struct qcom_icc_node slave_anoc_snoc = {
	.name = "slave_anoc_snoc",
	.id = SM6225_SLAVE_ANOC_SNOC,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 141,
	.qos.ap_owned = false,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slave_anoc_snoc_links),
	.links = slave_anoc_snoc_links
};

static const u16 slave_gpu_cdsp_bimc_links[] = {
	SM6225_MASTER_GPU_CDSP_PROC,
	SM6225_SLAVE_GPU_CDSP_BIMC
};

static struct qcom_icc_node slave_gpu_cdsp_bimc = {
	.name = "slave_gpu_cdsp_bimc",
	.id = SM6225_SLAVE_GPU_CDSP_BIMC,
	.buswidth = 32,
	.mas_rpm_id = -1,
	.slv_rpm_id = 266,
	.qos.ap_owned = false,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slave_gpu_cdsp_bimc_links),
	.links = slave_gpu_cdsp_bimc_links
};

static struct qcom_icc_node * const bimc_nodes[] = {
	[MASTER_AMPSS_M0] = &apps_proc,
	[MASTER_SNOC_BIMC_RT] = &master_snoc_bimc_rt,
	[MASTER_SNOC_BIMC_NRT] = &master_snoc_bimc_nrt,
	[SNOC_BIMC_MAS] = &snoc_bimc_mas,
	[MASTER_GPU_CDSP_PROC] = &master_gpu_cdsp_bimc,
	[MASTER_TCU_0] = &tcu_0,
	[SLAVE_EBI_CH0] = &ebi,
	[BIMC_SNOC_SLV] = &bimc_snoc_slv,
};

static const struct qcom_icc_desc sm6225_bimc = {
	.nodes = bimc_nodes,
	.num_nodes = ARRAY_SIZE(bimc_nodes),
	.type = QCOM_ICC_BIMC,
};

static struct qcom_icc_node * const config_noc_nodes[] = {
	[SNOC_CNOC_MAS] = &snoc_cnoc_mas,
	[MASTER_QDSS_DAP] = &xm_dap,
	[SLAVE_BIMC_CFG] = &qhs_bimc_cfg,
	[SLAVE_CAMERA_NRT_THROTTLE_CFG] = &qhs_camera_nrt_throtle_cfg,
	[SLAVE_CAMERA_RT_THROTTLE_CFG] = &qhs_camera_rt_throttle_cfg,
	[SLAVE_CAMERA_CFG] = &qhs_camera_ss_cfg,
	[SLAVE_CDSP_THROTTLE_CFG] = &qhs_cdsp_throttle_cfg,
	[SLAVE_CLK_CTL] = &qhs_clk_ctl,
	[SLAVE_CRYPTO_0_CFG] = &qhs_crypto0_cfg,
	[SLAVE_DISPLAY_CFG] = &qhs_disp_ss_cfg,
	[SLAVE_DISPLAY_THROTTLE_CFG] = &qhs_display_throttle_cfg,
	[SLAVE_GPU_CFG] = &qhs_gpu_cfg,
	[SLAVE_IMEM_CFG] = &qhs_imem_cfg,
	[SLAVE_IPA_CFG] = &qhs_ipa_cfg,
	[SLAVE_LPASS] = &qhs_lpass,
	[SLAVE_MESSAGE_RAM] = &qhs_mesg_ram,
	[SLAVE_PDM] = &qhs_pdm,
	[SLAVE_PIMEM_CFG] = &qhs_pimem_cfg,
	[SLAVE_PMIC_ARB] = &qhs_pmic_arb,
	[SLAVE_PRNG] = &qhs_prng,
	[SLAVE_QDSS_CFG] = &qhs_qdss_cfg,
	[SLAVE_QM_CFG] = &qhs_qm_cfg,
	[SLAVE_QM_MPU_CFG] = &qhs_qm_mpu_cfg,
	[SLAVE_QUP_0] = &qhs_qup0,
	[SLAVE_SDCC_1] = &qhs_sdc1,
	[SLAVE_SDCC_2] = &qhs_sdc2,
	[SLAVE_SNOC_CFG] = &slave_snoc_cfg,
	[SLAVE_TCSR] = &qhs_tcsr,
	[SLAVE_TLMM_EAST] = &qhs_tlmm_east,
	[SLAVE_TLMM_SOUTH] = &qhs_tlmm_south,
	[SLAVE_TLMM_WEST] = &qhs_tlmm_west,
	[SLAVE_UFS_MEM_CFG] = &qhs_ufs_mem_cfg,
	[SLAVE_USB3] = &qhs_usb3,
	[SLAVE_VENUS_CFG] = &qhs_venus_cfg,
	[SLAVE_VENUS_THROTTLE_CFG] = &qhs_venus_throttle_cfg,
	[SLAVE_VSENSE_CTRL_CFG] = &qhs_vsense_ctrl_cfg,
	[SLAVE_SERVICE_CNOC] = &srvc_cnoc,
};

static const struct qcom_icc_desc sm6225_config_noc = {
	.nodes = config_noc_nodes,
	.num_nodes = ARRAY_SIZE(config_noc_nodes),
	.type = QCOM_ICC_NOC,
};

static struct qcom_icc_node * const qup_virt_nodes[] = {
	[MASTER_QUP_CORE_0] = &qup_core_master_0,
	[SLAVE_QUP_CORE_0] = &qup_core_slave_0,
};

static const struct qcom_icc_desc sm6225_qup_virt = {
	.nodes = qup_virt_nodes,
	.num_nodes = ARRAY_SIZE(qup_virt_nodes),
	.type = QCOM_ICC_NOC,
};

static struct qcom_icc_node * const sys_noc_nodes[] = {
	[MASTER_CRYPTO_CORE0] = &crypto_c0,
	[MASTER_SNOC_CFG] = &master_snoc_cfg,
	[MASTER_TIC] = &qhm_tic,
	[MASTER_ANOC_SNOC] = &master_anoc_snoc,
	[BIMC_SNOC_MAS] = &bimc_snoc_mas,
	[MASTER_PIMEM] = &qxm_pimem,
	[MASTER_QDSS_BAM] = &qhm_qdss_bam,
	[MASTER_QUP_0] = &qhm_qup0,
	[MASTER_IPA] = &qxm_ipa,
	[MASTER_QDSS_ETR] = &xm_qdss_etr,
	[MASTER_SDCC_1] = &xm_sdc1,
	[MASTER_SDCC_2] = &xm_sdc2,
	[MASTER_UFS_MEM] = &xm_ufs_mem,
	[MASTER_USB3] = &xm_usb3_0,
	[MASTER_GRAPHICS_3D_PORT1] = &qnm_gpu_qos,
	[SLAVE_APPSS] = &qhs_apss,
	[SNOC_CNOC_SLV] = &snoc_cnoc_slv,
	[SLAVE_OCIMEM] = &qxs_imem,
	[SLAVE_PIMEM] = &qxs_pimem,
	[SNOC_BIMC_SLV] = &snoc_bimc_slv,
	[SLAVE_SERVICE_SNOC] = &srvc_snoc,
	[SLAVE_QDSS_STM] = &xs_qdss_stm,
	[SLAVE_TCU] = &xs_sys_tcu_cfg,
	[SLAVE_ANOC_SNOC] = &slave_anoc_snoc,
};

static const struct qcom_icc_desc sm6225_sys_noc = {
	.nodes = sys_noc_nodes,
	.num_nodes = ARRAY_SIZE(sys_noc_nodes),
	.type = QCOM_ICC_QNOC,
	.qos_offset = 0x15000,
};

static struct qcom_icc_node * const gpu_vert_nodes[] = {
	[MASTER_GRAPHICS_3D] = &qnm_gpu,
	[SLAVE_GPU_CDSP_BIMC] = &slave_gpu_cdsp_bimc,
};

static const struct qcom_icc_desc sm6225_gpu_vert = {
	.nodes = gpu_vert_nodes,
	.num_nodes = ARRAY_SIZE(gpu_vert_nodes),
	.type = QCOM_ICC_NOC,
};

static struct qcom_icc_node * const mmnrt_virt_nodes[] = {
	[MASTER_CAMNOC_SF] = &qnm_camera_nrt,
	[MASTER_VIDEO_P0] = &qxm_venus0,
	[MASTER_VIDEO_PROC] = &qxm_venus_cpu,
	[SLAVE_SNOC_BIMC_NRT] = &slave_snoc_bimc_nrt,
};

static const struct qcom_icc_desc sm6225_mmnrt_virt = {
	.nodes = mmnrt_virt_nodes,
	.num_nodes = ARRAY_SIZE(mmnrt_virt_nodes),
	.type = QCOM_ICC_QNOC,
	.qos_offset = 0x15000,
};

static struct qcom_icc_node * const mmrt_virt_nodes[] = {
	[MASTER_CAMNOC_HF] = &qnm_camera_rt,
	[MASTER_MDP_PORT0] = &qxm_mdp0,
	[SLAVE_SNOC_BIMC_RT] = &slave_snoc_bimc_rt,
};

static const struct qcom_icc_desc sm6225_mmrt_virt = {
	.nodes = mmrt_virt_nodes,
	.num_nodes = ARRAY_SIZE(mmrt_virt_nodes),
	.type = QCOM_ICC_QNOC,
	.qos_offset = 0x15000,
};

static const struct of_device_id qnoc_of_match[] = {
	{ .compatible = "qcom,sm6225-bimc", .data = &sm6225_bimc},
	{ .compatible = "qcom,sm6225-config-noc", .data = &sm6225_config_noc},
	{ .compatible = "qcom,sm6225-gpu-vert", .data = &sm6225_gpu_vert},
	{ .compatible = "qcom,sm6225-mmnrt-virt", .data = &sm6225_mmnrt_virt},
	{ .compatible = "qcom,sm6225-mmrt-virt", .data = &sm6225_mmrt_virt},
	{ .compatible = "qcom,sm6225-qup-virt", .data = &sm6225_qup_virt},
	{ .compatible = "qcom,sm6225-sys-noc", .data = &sm6225_sys_noc},
	{ }
};
MODULE_DEVICE_TABLE(of, qnoc_of_match);

static struct platform_driver qnoc_driver = {
	.probe = qnoc_probe,
	.remove = qnoc_remove,
	.driver = {
		.name = "qnoc-sm6225",
		.of_match_table = qnoc_of_match,
		.sync_state = icc_sync_state,
	},
};

static int __init qnoc_driver_init(void)
{
	return platform_driver_register(&qnoc_driver);
}
core_initcall(qnoc_driver_init);

static void __exit qnoc_driver_exit(void)
{
	platform_driver_unregister(&qnoc_driver);
}
module_exit(qnoc_driver_exit);

MODULE_DESCRIPTION("Qualcomm SM6225 NoC driver");
MODULE_LICENSE("GPL v2");
