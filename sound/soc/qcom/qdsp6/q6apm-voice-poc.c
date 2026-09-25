// SPDX-License-Identifier: GPL-2.0
/*
 * EXPERIMENT: voice call proof of concept, as a module of its own so it can
 * be reloaded without rebooting (it reaches the APM through hooks exported by
 * snd-q6apm). Opens stock's voice graphs on the ADSP by hand:
 *
 *   echo start > /sys/kernel/debug/q6apm-voice/ctl   (during a call)
 *   echo stop  > /sys/kernel/debug/q6apm-voice/ctl
 *   echo mic   > /sys/kernel/debug/q6apm-voice/ctl   (mic sub-graph alone)
 *
 * qcom/sm6225/voice-rx4-open.bin is the merged GRAPH_OPEN of stock's four
 * voice RX sub-graphs (tools/acdb/merge-graph.py), with stock's edges into
 * its earpiece device sub-graph remapped onto the topology's backend: voice
 * and DTMF into the backend SAL (0x6004), the mailbox's timing link to the
 * I2S sink (0x6003). The backend must therefore be running (a playback
 * stream on MultiMedia1) before start; it also powers the amplifiers and
 * owns the I2S clock. voice-rx4-cfg.bin / voice-tx-* as before: the VCPM
 * voice config and calibration sent after each open. The rest follows
 * stock's call-start sequence (investigations/call-audio.md).
 */
#include <dt-bindings/sound/qcom,q6dsp-lpass-ports.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/sizes.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_clk.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/soc/qcom/apr.h>
#include "audioreach.h"
#include "q6apm.h"

static const u32 voice_poc_rx_sgs[] = {
	4, 0xb0000045, 0xb0000044, 0xb0000041, 0xb0000040,
};
/*
 * voice_poc_rx_direct: voice RX without b0000045, its output (0x41dc) linked
 * straight into the backend SAL (voice-rx5-*.bin). b0000045's container never
 * got a frame length on a call, so the downlink stopped there.
 */
static const u32 voice_poc_rx3_sgs[] = {
	3, 0xb0000044, 0xb0000041, 0xb0000040,
};
static bool voice_poc_rx_direct;
module_param(voice_poc_rx_direct, bool, 0644);
MODULE_PARM_DESC(voice_poc_rx_direct, "voice PoC: voice RX without b0000045");
#define VOICE_POC_RX(op) (voice_poc_rx_direct ? \
	voice_poc_sg_cmd(apm, op, voice_poc_rx3_sgs, sizeof(voice_poc_rx3_sgs)) : \
	voice_poc_sg_cmd(apm, op, voice_poc_rx_sgs, sizeof(voice_poc_rx_sgs)))
/*
 * Voice TX without stock's mic device sub-graph (0xb0000039): the mics come
 * from the topology's TX_CODEC_DMA_TX_3 backend, which ALSA keeps running
 * (arecord on MultiMedia2), and voice-tx-open.bin links its splitter's
 * output 15 (0x6091, stock's 0x4167) into voice TX.
 */
static const u32 voice_poc_tx_sgs[] = {
	2, 0xb000003f, 0xb00000b1,
};
/* TX codec core and NPL clocks (mic); the backend owns the MI2S bit clock */
static const struct { u32 id; unsigned long rate; } voice_poc_clk_ids[] = {
	{ LPASS_CLK_ID_TX_CORE_MCLK, 19200000 },
	{ LPASS_CLK_ID_TX_CORE_NPL_MCLK, 19200000 },
};
static struct clk *voice_poc_clks[ARRAY_SIZE(voice_poc_clk_ids)];
static struct dentry *voice_poc_dir;
static bool voice_poc_rx_open, voice_poc_tx_open;

static int voice_poc_send(struct q6apm *apm, u32 opcode, const void *data, size_t len)
{
	struct gpr_pkt *pkt;
	int rc;

	pkt = audioreach_alloc_apm_cmd_pkt(len, opcode, 0);
	if (IS_ERR(pkt))
		return PTR_ERR(pkt);
	memcpy((void *)pkt + GPR_HDR_SIZE + APM_CMD_HDR_SIZE, data, len);
	rc = q6apm_send_cmd_sync(apm, pkt, 0);
	kfree(pkt);
	dev_info(apm->dev, "voice-poc: opcode %#x, %zu bytes -> %d\n", opcode, len, rc);
	return rc;
}

#ifndef APM_CMD_REGISTER_CFG
#define APM_CMD_REGISTER_CFG	0x01001008
#endif
#define APM_CMD_DEREGISTER_CFG_POC	0x01001009

/*
 * Persistent calibration must stay in DSP-visible memory while registered,
 * so it goes out of band: one DMA buffer on the q6apm-dais device (the one
 * with the ADSP IOMMU stream), mapped with APM_CMD_SHARED_MEM_MAP_REGIONS.
 */
static bool voice_poc_pcal_enable;
module_param_named(voice_poc_pcal, voice_poc_pcal_enable, bool, 0644);
MODULE_PARM_DESC(voice_poc_pcal, "voice PoC: register persistent calibration out of band (hard-reset the SoC once)");
#define VOICE_POC_PCAL_SZ	SZ_64K
static struct device *voice_poc_dma_dev;
static void *voice_poc_pcal;
static dma_addr_t voice_poc_pcal_iova;
static size_t voice_poc_pcal_used;
/* registered blocks, de-registered (while their modules exist) before close */
static struct { size_t off, len; } voice_poc_reg[4];
static int voice_poc_nreg;

static int voice_poc_oob_cmd(struct q6apm *apm, u32 opcode, size_t off, size_t len)
{
	struct apm_cmd_header *hdr;
	struct gpr_pkt *pkt;
	int rc;

	pkt = audioreach_alloc_apm_cmd_pkt(0, opcode, 0);
	if (IS_ERR(pkt))
		return PTR_ERR(pkt);
	hdr = (void *)pkt + GPR_HDR_SIZE;
	hdr->payload_address_lsw = lower_32_bits(voice_poc_pcal_iova + off);
	hdr->payload_address_msw = upper_32_bits(voice_poc_pcal_iova + off);
	hdr->mem_map_handle = q6apm_poc_map_handle;
	hdr->payload_size = len;
	rc = q6apm_send_cmd_sync(apm, pkt, 0);
	kfree(pkt);
	dev_info(apm->dev, "voice-poc: oob opcode %#x, %zu bytes -> %d\n", opcode, len, rc);
	return rc;
}

static void voice_poc_deregister_all(struct q6apm *apm)
{
	while (voice_poc_nreg > 0) {
		voice_poc_nreg--;
		voice_poc_oob_cmd(apm, APM_CMD_DEREGISTER_CFG_POC,
				  voice_poc_reg[voice_poc_nreg].off,
				  voice_poc_reg[voice_poc_nreg].len);
	}
}

static int voice_poc_map_pcal(struct q6apm *apm)
{
	struct apm_shared_map_region_payload *mregion;
	struct apm_cmd_shared_mem_map_regions *cmd;
	struct device_node *np;
	struct platform_device *pdev;
	struct gpr_pkt *pkt;
	int rc;

	np = of_find_compatible_node(NULL, NULL, "qcom,q6apm-dais");
	pdev = np ? of_find_device_by_node(np) : NULL;
	of_node_put(np);
	if (!pdev)
		return -ENODEV;
	voice_poc_dma_dev = &pdev->dev;
	voice_poc_pcal = dma_alloc_coherent(voice_poc_dma_dev, VOICE_POC_PCAL_SZ,
					    &voice_poc_pcal_iova, GFP_KERNEL);
	if (!voice_poc_pcal)
		return -ENOMEM;
	voice_poc_pcal_used = 0;

	pkt = audioreach_alloc_apm_cmd_pkt(sizeof(*cmd) + sizeof(*mregion),
					   APM_CMD_SHARED_MEM_MAP_REGIONS, Q6APM_POC_MAP_TOKEN);
	if (IS_ERR(pkt))
		return PTR_ERR(pkt);
	/* alloc_apm_cmd_pkt adds an apm_cmd_header; a map command has none */
	cmd = (void *)pkt + GPR_HDR_SIZE;
	pkt->hdr.pkt_size = GPR_HDR_SIZE + sizeof(*cmd) + sizeof(*mregion);
	cmd->mem_pool_id = APM_MEMORY_MAP_SHMEM8_4K_POOL;
	cmd->num_regions = 1;
	cmd->property_flag = 0;
	mregion = (void *)cmd + sizeof(*cmd);
	mregion->shm_addr_lsw = lower_32_bits(voice_poc_pcal_iova);
	mregion->shm_addr_msw = upper_32_bits(voice_poc_pcal_iova);
	mregion->mem_size_bytes = VOICE_POC_PCAL_SZ;
	q6apm_poc_map_handle = 0;
	rc = q6apm_send_cmd_sync(apm, pkt, APM_CMD_RSP_SHARED_MEM_MAP_REGIONS);
	kfree(pkt);
	dev_info(apm->dev, "voice-poc: pcal map %pad -> %d, handle %#x\n",
		 &voice_poc_pcal_iova, rc, q6apm_poc_map_handle);
	return rc ? rc : (q6apm_poc_map_handle ? 0 : -EIO);
}

static void voice_poc_unmap_pcal(struct q6apm *apm)
{
	struct apm_cmd_shared_mem_unmap_regions *cmd;
	struct gpr_pkt *pkt;

	if (q6apm_poc_map_handle) {
		pkt = audioreach_alloc_apm_cmd_pkt(sizeof(*cmd),
						   APM_CMD_SHARED_MEM_UNMAP_REGIONS, 0);
		if (!IS_ERR(pkt)) {
			cmd = (void *)pkt + GPR_HDR_SIZE;
			pkt->hdr.pkt_size = GPR_HDR_SIZE + sizeof(*cmd);
			cmd->mem_map_handle = q6apm_poc_map_handle;
			q6apm_send_cmd_sync(apm, pkt, 0);
			kfree(pkt);
		}
		q6apm_poc_map_handle = 0;
	}
	if (voice_poc_pcal) {
		dma_free_coherent(voice_poc_dma_dev, VOICE_POC_PCAL_SZ,
				  voice_poc_pcal, voice_poc_pcal_iova);
		voice_poc_pcal = NULL;
	}
}

/* Copy a file of persistent cal into the mapped buffer and register it. */
static int voice_poc_register_pcal(struct q6apm *apm, const char *name)
{
	const struct firmware *fw;
	size_t off = ALIGN(voice_poc_pcal_used, 64), len;
	int rc;

	if (voice_poc_nreg >= ARRAY_SIZE(voice_poc_reg))
		return -ENOSPC;
	rc = request_firmware(&fw, name, apm->dev);
	if (rc)
		return rc;
	len = fw->size;
	if (off + len > VOICE_POC_PCAL_SZ) {
		release_firmware(fw);
		return -ENOSPC;
	}
	memcpy(voice_poc_pcal + off, fw->data, len);
	release_firmware(fw);
	voice_poc_pcal_used = off + len;
	rc = voice_poc_oob_cmd(apm, APM_CMD_REGISTER_CFG, off, len);
	if (!rc) {
		voice_poc_reg[voice_poc_nreg].off = off;
		voice_poc_reg[voice_poc_nreg].len = len;
		voice_poc_nreg++;
	}
	return rc;
}

/*
 * Send a firmware file of param records as several commands, each at most
 * ~3.5 KB, split at record boundaries (records are independent). Stock sends
 * the persistent voice calibration out of band; in band it has to be split.
 */
static int voice_poc_send_fw_chunked(struct q6apm *apm, u32 opcode, const char *name)
{
	const struct firmware *fw;
	size_t off = 0, start = 0;
	int rc = 0;

	rc = request_firmware(&fw, name, apm->dev);
	if (rc)
		return rc;
	while (off + sizeof(struct apm_module_param_data) <= fw->size) {
		const struct apm_module_param_data *p = (const void *)(fw->data + off);
		size_t len = ALIGN(sizeof(*p) + p->param_size, 8);

		if (off > start && off + len - start > 3584) {
			rc = voice_poc_send(apm, opcode, fw->data + start, off - start);
			if (rc)
				goto out;
			start = off;
		}
		off += len;
	}
	if (off > start)
		rc = voice_poc_send(apm, opcode, fw->data + start, off - start);
out:
	release_firmware(fw);
	return rc;
}

static int voice_poc_send_fw(struct q6apm *apm, u32 opcode, const char *name)
{
	const struct firmware *fw;
	int rc;

	rc = request_firmware(&fw, name, apm->dev);
	if (rc)
		return rc;
	rc = voice_poc_send(apm, opcode, fw->data, fw->size);
	release_firmware(fw);
	return rc;
}

/* One param record: header, payload, padded to 8 bytes. */
static size_t voice_poc_rec(u8 *buf, u32 miid, u32 pid, const void *pl, u32 size)
{
	struct apm_module_param_data *p = (void *)buf;

	p->module_instance_id = miid;
	p->param_id = pid;
	p->param_size = size;
	p->error_code = 0;
	memcpy(buf + sizeof(*p), pl, size);
	return ALIGN(sizeof(*p) + size, 8);
}

static int voice_poc_sg_cmd(struct q6apm *apm, u32 opcode, const u32 *list, size_t len)
{
	u8 buf[64] = { 0 };
	size_t n;

	n = voice_poc_rec(buf, APM_MODULE_INSTANCE_ID, APM_PARAM_ID_SUB_GRAPH_LIST,
			  list, len);
	return voice_poc_send(apm, opcode, buf, n);
}

#define VOICE_POC_SG(op, sgs) voice_poc_sg_cmd(apm, op, sgs, sizeof(sgs))

static void voice_poc_put_clks(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(voice_poc_clks); i++) {
		if (!voice_poc_clks[i])
			continue;
		clk_disable_unprepare(voice_poc_clks[i]);
		clk_put(voice_poc_clks[i]);
		voice_poc_clks[i] = NULL;
	}
}

static int voice_poc_get_clks(struct q6apm *apm)
{
	struct of_phandle_args args = { .args_count = 2 };
	struct clk *c;
	int i, rc;

	args.np = of_find_compatible_node(NULL, NULL, "qcom,q6prm-lpass-clocks");
	for (i = 0; i < ARRAY_SIZE(voice_poc_clk_ids); i++) {
		args.args[0] = voice_poc_clk_ids[i].id;
		args.args[1] = LPASS_CLK_ATTRIBUTE_COUPLE_NO;
		c = of_clk_get_from_provider(&args);
		if (IS_ERR(c)) {
			rc = PTR_ERR(c);
			goto err;
		}
		clk_set_rate(c, voice_poc_clk_ids[i].rate);
		rc = clk_prepare_enable(c);
		dev_info(apm->dev, "voice-poc: clk %u -> %d\n", voice_poc_clk_ids[i].id, rc);
		if (rc) {
			clk_put(c);
			goto err;
		}
		voice_poc_clks[i] = c;
	}
	of_node_put(args.np);
	return 0;
err:
	of_node_put(args.np);
	voice_poc_put_clks();
	return rc;
}

/* Stock's call-start order: open RX, open TX, VCPM params, RX up, TX up. */
static int voice_poc_start(struct q6apm *apm)
{
	static const u32 vsid[] = { 0x11c05000 };
	static const u32 tx_ch[] = { 0x11c05000, 2 };
	static const u32 cal_keys[] = { 0x11c05000, 3, 0x08001166, 0,
					0x080011b6, 0, 0x08001167, 0 };
	/*
	 * MFC output: 48 kHz, 16 bit, 2 channels, channel map FL FR (12 bytes),
	 * to match the backend SAL's operating format. Stock sends 32 bit mono,
	 * its earpiece device's format.
	 */
	static const u32 mfc[] = { 48000, 0x00020010, 0x00020001 };
	u8 buf[256];
	size_t n = 0;
	int rc;

	rc = voice_poc_get_clks(apm);
	if (rc)
		return rc;

	/* Marked open first: an open that times out here may still complete on the DSP. */
	voice_poc_rx_open = true;
	rc = voice_poc_send_fw(apm, APM_CMD_GRAPH_OPEN, voice_poc_rx_direct ?
			       "qcom/sm6225/voice-rx5-open.bin" :
			       "qcom/sm6225/voice-rx4-open.bin");
	if (rc)
		return rc;
	rc = voice_poc_send_fw(apm, APM_CMD_SET_CFG, voice_poc_rx_direct ?
			       "qcom/sm6225/voice-rx5-cfg.bin" :
			       "qcom/sm6225/voice-rx4-cfg.bin");
	if (rc)
		return rc;
	/*
	 * persistent calibration, as stock's REGISTER_CFGs (b0000044, 41, 40).
	 * Off by default: an out-of-band attempt hard-reset the SoC.
	 */
	if (voice_poc_pcal_enable) {
		rc = voice_poc_map_pcal(apm);
		if (rc)
			return rc;
		rc = voice_poc_register_pcal(apm, "qcom/sm6225/voice-rx-pcal.bin");
		if (rc)
			return rc;
	}
	voice_poc_tx_open = true;
	rc = voice_poc_send_fw(apm, APM_CMD_GRAPH_OPEN, "qcom/sm6225/voice-tx-open.bin");
	if (rc)
		return rc;
	/* ECNS's tuning makes this ~10 KiB: send it in several SET_CFGs */
	rc = voice_poc_send_fw_chunked(apm, APM_CMD_SET_CFG, "qcom/sm6225/voice-tx-cfg.bin");
	if (rc)
		return rc;
	/* persistent calibration, as stock's REGISTER_CFGs (b000003f, b1) */
	if (voice_poc_pcal_enable) {
		rc = voice_poc_register_pcal(apm, "qcom/sm6225/voice-tx-pcal.bin");
		if (rc)
			return rc;
	}

	n += voice_poc_rec(buf + n, VCPM_MODULE_INSTANCE_ID, 0x080011bc, vsid, sizeof(vsid));
	n += voice_poc_rec(buf + n, VCPM_MODULE_INSTANCE_ID, 0x08001310, tx_ch, sizeof(tx_ch));
	n += voice_poc_rec(buf + n, VCPM_MODULE_INSTANCE_ID, 0x0800116b, cal_keys, sizeof(cal_keys));
	rc = voice_poc_send(apm, APM_CMD_SET_CFG, buf, n);
	if (rc)
		return rc;

	n = 0;
	if (!voice_poc_rx_direct)
		n += voice_poc_rec(buf + n, 0x465b, 0x08001024, mfc, 12);
	n += voice_poc_rec(buf + n, 0x41dd, 0x08001024, mfc, 12);
	rc = voice_poc_send(apm, APM_CMD_SET_CFG, buf, n);
	if (rc)
		return rc;
	rc = VOICE_POC_RX(APM_CMD_GRAPH_PREPARE);
	if (rc)
		return rc;
	rc = VOICE_POC_RX(APM_CMD_GRAPH_START);
	if (rc)
		return rc;

	rc = VOICE_POC_SG(APM_CMD_GRAPH_PREPARE, voice_poc_tx_sgs);
	if (rc)
		return rc;
	return VOICE_POC_SG(APM_CMD_GRAPH_START, voice_poc_tx_sgs);
}

static void voice_poc_stop(struct q6apm *apm)
{
	/* while the modules still exist, before the calibration memory goes */
	voice_poc_deregister_all(apm);
	if (voice_poc_tx_open) {
		VOICE_POC_SG(APM_CMD_GRAPH_STOP, voice_poc_tx_sgs);
		VOICE_POC_SG(APM_CMD_GRAPH_CLOSE, voice_poc_tx_sgs);
		voice_poc_tx_open = false;
	}
	if (voice_poc_rx_open) {
		VOICE_POC_RX(APM_CMD_GRAPH_STOP);
		VOICE_POC_RX(APM_CMD_GRAPH_CLOSE);
		voice_poc_rx_open = false;
	}
	voice_poc_unmap_pcal(apm);
	voice_poc_put_clks();
}

static ssize_t voice_poc_write(struct file *file, const char __user *ubuf,
			       size_t count, loff_t *ppos)
{
	struct q6apm *apm = file->private_data;
	char cmd[8] = { 0 };
	int rc = 0;

	if (copy_from_user(cmd, ubuf, min(count, sizeof(cmd) - 1)))
		return -EFAULT;
	if (!strncmp(cmd, "start", 5)) {
		rc = voice_poc_start(apm);
		if (rc)
			voice_poc_stop(apm);
	} else if (!strncmp(cmd, "stop", 4)) {
		voice_poc_stop(apm);
	} else {
		return -EINVAL;
	}
	return rc ? rc : count;
}

static const struct file_operations voice_poc_fops = {
	.open = simple_open,
	.write = voice_poc_write,
};


static int __init voice_poc_init(void)
{
	struct q6apm *apm = q6apm_poc_get();

	if (!apm)
		return -EPROBE_DEFER;
	voice_poc_dir = debugfs_create_dir("q6apm-voice", NULL);
	debugfs_create_file("ctl", 0200, voice_poc_dir, apm, &voice_poc_fops);
	return 0;
}
module_init(voice_poc_init);

static void __exit voice_poc_exit(void)
{
	voice_poc_stop(q6apm_poc_get());
	debugfs_remove_recursive(voice_poc_dir);
}
module_exit(voice_poc_exit);

MODULE_DESCRIPTION("EXPERIMENT: AudioReach voice call proof of concept");
MODULE_LICENSE("GPL");
