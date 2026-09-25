// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2020, Linaro Limited

#include <dt-bindings/soc/qcom,gpr.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/soc/qcom/apr.h>
#include <linux/wait.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include "audioreach.h"
#include "q6apm.h"

/* Graph Management */
struct apm_graph_mgmt_cmd {
	struct apm_module_param_data param_data;
	uint32_t num_sub_graphs;
	uint32_t sub_graph_id_list[];
} __packed;

#define APM_GRAPH_MGMT_PSIZE(p, n) ALIGN(struct_size(p, sub_graph_id_list, n), 8)

static struct q6apm *g_apm;

int q6apm_send_cmd_sync(struct q6apm *apm, const struct gpr_pkt *pkt,
			uint32_t rsp_opcode)
{
	gpr_device_t *gdev = apm->gdev;

	return audioreach_send_cmd_sync(&gdev->dev, gdev, &apm->result, &apm->lock,
					NULL, &apm->wait, pkt, rsp_opcode);
}

static struct audioreach_graph *q6apm_get_audioreach_graph(struct q6apm *apm, uint32_t graph_id)
{
	struct audioreach_graph_info *info;
	struct audioreach_graph *graph;
	int id;

	mutex_lock(&apm->lock);
	graph = idr_find(&apm->graph_idr, graph_id);
	mutex_unlock(&apm->lock);

	if (graph) {
		kref_get(&graph->refcount);
		return graph;
	}

	info = idr_find(&apm->graph_info_idr, graph_id);

	if (!info)
		return ERR_PTR(-ENODEV);

	graph = kzalloc_obj(*graph);
	if (!graph)
		return ERR_PTR(-ENOMEM);

	graph->apm = apm;
	graph->info = info;
	graph->id = graph_id;

	graph->graph = audioreach_alloc_graph_pkt(apm, info);
	if (IS_ERR(graph->graph)) {
		void *err = graph->graph;

		kfree(graph);
		return ERR_CAST(err);
	}

	mutex_lock(&apm->lock);
	id = idr_alloc(&apm->graph_idr, graph, graph_id, graph_id + 1, GFP_KERNEL);
	if (id < 0) {
		dev_err(apm->dev, "Unable to allocate graph id (%d)\n", graph_id);
		kfree(graph->graph);
		kfree(graph);
		mutex_unlock(&apm->lock);
		return ERR_PTR(id);
	}
	mutex_unlock(&apm->lock);

	kref_init(&graph->refcount);

	q6apm_send_cmd_sync(apm, graph->graph, 0);

	return graph;
}

static int audioreach_graph_mgmt_cmd(struct audioreach_graph *graph, uint32_t opcode)
{
	struct audioreach_graph_info *info = graph->info;
	int num_sub_graphs = info->num_sub_graphs;
	struct apm_module_param_data *param_data;
	struct apm_graph_mgmt_cmd *mgmt_cmd;
	struct audioreach_sub_graph *sg;
	struct q6apm *apm = graph->apm;
	int i = 0, payload_size = APM_GRAPH_MGMT_PSIZE(mgmt_cmd, num_sub_graphs);

	struct gpr_pkt *pkt __free(kfree) = audioreach_alloc_apm_cmd_pkt(payload_size, opcode, 0);
	if (IS_ERR(pkt))
		return PTR_ERR(pkt);

	mgmt_cmd = (void *)pkt + GPR_HDR_SIZE + APM_CMD_HDR_SIZE;

	mgmt_cmd->num_sub_graphs = num_sub_graphs;

	param_data = &mgmt_cmd->param_data;
	param_data->module_instance_id = APM_MODULE_INSTANCE_ID;
	param_data->param_id = APM_PARAM_ID_SUB_GRAPH_LIST;
	param_data->param_size = payload_size - APM_MODULE_PARAM_DATA_SIZE;

	list_for_each_entry(sg, &info->sg_list, node)
		mgmt_cmd->sub_graph_id_list[i++] = sg->sub_graph_id;

	return q6apm_send_cmd_sync(apm, pkt, 0);
}

static void q6apm_put_audioreach_graph(struct kref *ref)
{
	struct audioreach_graph *graph;
	struct q6apm *apm;

	graph = container_of(ref, struct audioreach_graph, refcount);
	apm = graph->apm;

	audioreach_graph_mgmt_cmd(graph, APM_CMD_GRAPH_CLOSE);

	mutex_lock(&apm->lock);
	graph = idr_remove(&apm->graph_idr, graph->id);
	mutex_unlock(&apm->lock);

	kfree(graph->graph);
	kfree(graph);
}


static int q6apm_get_apm_state(struct q6apm *apm)
{
	struct gpr_pkt *pkt __free(kfree) = audioreach_alloc_apm_cmd_pkt(0,
								APM_CMD_GET_SPF_STATE, 0);
	if (IS_ERR(pkt))
		return PTR_ERR(pkt);

	q6apm_send_cmd_sync(apm, pkt, APM_CMD_RSP_GET_SPF_STATE);

	return apm->state;
}

bool q6apm_is_adsp_ready(void)
{
	if (g_apm)
		return q6apm_get_apm_state(g_apm);

	return false;
}
EXPORT_SYMBOL_GPL(q6apm_is_adsp_ready);

static struct audioreach_module *__q6apm_find_module_by_mid(struct q6apm *apm,
						    struct audioreach_graph_info *info,
						    uint32_t mid)
{
	struct audioreach_container *container;
	struct audioreach_sub_graph *sgs;
	struct audioreach_module *module;

	list_for_each_entry(sgs, &info->sg_list, node) {
		list_for_each_entry(container, &sgs->container_list, node) {
			list_for_each_entry(module, &container->modules_list, node) {
				if (mid == module->module_id)
					return module;
			}
		}
	}

	return NULL;
}

int q6apm_graph_media_format_shmem(struct q6apm_graph *graph,
				   struct audioreach_module_config *cfg)
{
	struct audioreach_module *module;

	if (cfg->direction == SNDRV_PCM_STREAM_CAPTURE) {
		module = q6apm_find_module_by_mid(graph, MODULE_ID_SH_MEM_PUSH_MODE);
		if (!module)
			module = q6apm_find_module_by_mid(graph, MODULE_ID_RD_SHARED_MEM_EP);
	} else {
		module = q6apm_find_module_by_mid(graph, MODULE_ID_SH_MEM_PULL_MODE);
		if (!module)
			module = q6apm_find_module_by_mid(graph, MODULE_ID_WR_SHARED_MEM_EP);
	}

	if (!module) {
		dev_err(graph->dev, "No SHMEM module found in graph\n");
		return -ENODEV;
	}

	return audioreach_set_media_format(graph, module, cfg);
}
EXPORT_SYMBOL_GPL(q6apm_graph_media_format_shmem);

static int __q6apm_map_memory_fixed_region(struct device *dev, unsigned int graph_id,
					   phys_addr_t phys, size_t sz, bool is_pos_buf)
{
	struct audioreach_graph_info *info;
	struct q6apm *apm = dev_get_drvdata(dev->parent);
	struct apm_shared_map_region_payload *mregions;
	struct apm_cmd_shared_mem_map_regions *cmd;
	int payload_size = sizeof(*cmd) + (sizeof(*mregions));
	uint32_t buf_sz;
	void *p;
	uint32_t pos_mask = is_pos_buf ? APM_MMAP_TOKEN_MAP_TYPE_POS_BUF : 0;
	struct gpr_pkt *pkt __free(kfree) = audioreach_alloc_apm_cmd_pkt(payload_size,
					APM_CMD_SHARED_MEM_MAP_REGIONS, (graph_id | pos_mask));

	if (IS_ERR(pkt))
		return PTR_ERR(pkt);

	info = idr_find(&apm->graph_info_idr, graph_id);
	if (!info)
		return -ENODEV;

	if (is_pos_buf) {
		if (info->pos_buf_mem_map_handle)
			return 0;
	} else {
		if (info->mem_map_handle)
			return 0;
	}

	/* DSP expects size should be aligned to 4K */
	buf_sz = ALIGN(sz, 4096);

	p = (void *)pkt + GPR_HDR_SIZE;
	cmd = p;
	cmd->mem_pool_id = APM_MEMORY_MAP_SHMEM8_4K_POOL;
	cmd->num_regions = 1;
	if (is_pos_buf)
		cmd->property_flag = 0x2;
	else
		cmd->property_flag = 0x0;

	mregions = p + sizeof(*cmd);

	mregions->shm_addr_lsw = lower_32_bits(phys);
	mregions->shm_addr_msw = upper_32_bits(phys);
	mregions->mem_size_bytes = buf_sz;

	return q6apm_send_cmd_sync(apm, pkt, APM_CMD_RSP_SHARED_MEM_MAP_REGIONS);
}

int q6apm_map_pos_buffer(struct device *dev, unsigned int graph_id, phys_addr_t phys, size_t sz)
{
	return __q6apm_map_memory_fixed_region(dev, graph_id, phys, sz, true);
}
EXPORT_SYMBOL_GPL(q6apm_map_pos_buffer);

int q6apm_map_memory_fixed_region(struct device *dev, unsigned int graph_id,
				  phys_addr_t phys, size_t sz)
{
	return __q6apm_map_memory_fixed_region(dev, graph_id, phys, sz, false);
}
EXPORT_SYMBOL_GPL(q6apm_map_memory_fixed_region);

int q6apm_alloc_fragments(struct q6apm_graph *graph, unsigned int dir, phys_addr_t phys,
				size_t period_sz, unsigned int periods)
{
	struct audioreach_graph_data *data;
	struct audio_buffer *buf;
	int cnt;

	if (dir == SNDRV_PCM_STREAM_PLAYBACK)
		data = &graph->rx_data;
	else
		data = &graph->tx_data;

	mutex_lock(&graph->lock);

	data->dsp_buf = 0;

	if (data->buf) {
		mutex_unlock(&graph->lock);
		return 0;
	}

	buf = kzalloc_objs(struct audio_buffer, periods);
	if (!buf) {
		mutex_unlock(&graph->lock);
		return -ENOMEM;
	}

	if (dir == SNDRV_PCM_STREAM_PLAYBACK)
		data = &graph->rx_data;
	else
		data = &graph->tx_data;

	data->buf = buf;

	buf[0].phys = phys;
	buf[0].size = period_sz;

	for (cnt = 1; cnt < periods; cnt++) {
		if (period_sz > 0) {
			buf[cnt].phys = buf[0].phys + (cnt * period_sz);
			buf[cnt].size = period_sz;
		}
	}
	data->num_periods = periods;

	mutex_unlock(&graph->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(q6apm_alloc_fragments);

static int __q6apm_unmap_memory_fixed_region(struct device *dev, unsigned int graph_id,
					     bool is_pos_buf)
{
	struct apm_cmd_shared_mem_unmap_regions *cmd;
	struct q6apm *apm = dev_get_drvdata(dev->parent);
	struct audioreach_graph_info *info;
	uint32_t mem_map_handle;
	struct gpr_pkt *pkt __free(kfree) = audioreach_alloc_apm_cmd_pkt(sizeof(*cmd),
						APM_CMD_SHARED_MEM_UNMAP_REGIONS, graph_id);
	if (IS_ERR(pkt))
		return PTR_ERR(pkt);

	info = idr_find(&apm->graph_info_idr, graph_id);
	if (!info)
		return -ENODEV;

	if (is_pos_buf) {
		if (!info->pos_buf_mem_map_handle)
			return 0;
		mem_map_handle = info->pos_buf_mem_map_handle;
	} else {

		if (!info->mem_map_handle)
			return 0;
		mem_map_handle = info->mem_map_handle;
	}

	cmd = (void *)pkt + GPR_HDR_SIZE;
	cmd->mem_map_handle = mem_map_handle;

	return q6apm_send_cmd_sync(apm, pkt, APM_CMD_SHARED_MEM_UNMAP_REGIONS);
}

int q6apm_unmap_memory_fixed_region(struct device *dev, unsigned int graph_id)
{
	return __q6apm_unmap_memory_fixed_region(dev, graph_id, false);
}
EXPORT_SYMBOL_GPL(q6apm_unmap_memory_fixed_region);

int q6apm_unmap_pos_buffer(struct device *dev, unsigned int graph_id)
{
	return __q6apm_unmap_memory_fixed_region(dev, graph_id, true);
}
EXPORT_SYMBOL_GPL(q6apm_unmap_pos_buffer);

int q6apm_free_fragments(struct q6apm_graph *graph, unsigned int dir)
{
	audioreach_graph_free_buf(graph);

	return 0;
}
EXPORT_SYMBOL_GPL(q6apm_free_fragments);

int q6apm_remove_initial_silence(struct device *dev, struct q6apm_graph *graph, uint32_t samples)
{
	struct audioreach_module *module;

	module = q6apm_find_module_by_mid(graph, MODULE_ID_PLACEHOLDER_DECODER);
	if (!module)
		return -ENODEV;

	return audioreach_send_u32_param(graph, module, PARAM_ID_REMOVE_INITIAL_SILENCE, samples);
}
EXPORT_SYMBOL_GPL(q6apm_remove_initial_silence);

int q6apm_remove_trailing_silence(struct device *dev, struct q6apm_graph *graph, uint32_t samples)
{
	struct audioreach_module *module;

	module = q6apm_find_module_by_mid(graph, MODULE_ID_PLACEHOLDER_DECODER);
	if (!module)
		return -ENODEV;

	return audioreach_send_u32_param(graph, module, PARAM_ID_REMOVE_TRAILING_SILENCE, samples);
}
EXPORT_SYMBOL_GPL(q6apm_remove_trailing_silence);

int q6apm_enable_compress_module(struct device *dev, struct q6apm_graph *graph, bool en)
{
	struct audioreach_module *module;

	module = q6apm_find_module_by_mid(graph, MODULE_ID_PLACEHOLDER_DECODER);
	if (!module)
		return -ENODEV;

	return audioreach_send_u32_param(graph, module, PARAM_ID_MODULE_ENABLE, en);
}
EXPORT_SYMBOL_GPL(q6apm_enable_compress_module);

int q6apm_set_real_module_id(struct device *dev, struct q6apm_graph *graph,
			     uint32_t codec_id)
{
	struct audioreach_module *module;
	uint32_t module_id;

	module = q6apm_find_module_by_mid(graph, MODULE_ID_PLACEHOLDER_DECODER);
	if (!module)
		return -ENODEV;

	switch (codec_id) {
	case SND_AUDIOCODEC_MP3:
		module_id = MODULE_ID_MP3_DECODE;
		break;
	case SND_AUDIOCODEC_AAC:
		module_id = MODULE_ID_AAC_DEC;
		break;
	case SND_AUDIOCODEC_FLAC:
		module_id = MODULE_ID_FLAC_DEC;
		break;
	case SND_AUDIOCODEC_OPUS_RAW:
		module_id = MODULE_ID_OPUS_DEC;
		break;
	default:
		return -EINVAL;
	}

	return audioreach_send_u32_param(graph, module, PARAM_ID_REAL_MODULE_ID,
					 module_id);
}
EXPORT_SYMBOL_GPL(q6apm_set_real_module_id);

int q6apm_graph_media_format_pcm(struct q6apm_graph *graph, struct audioreach_module_config *cfg)
{
	struct audioreach_graph_info *info = graph->info;
	struct audioreach_sub_graph *sgs;
	struct audioreach_container *container;
	struct audioreach_module *module;
	int ret;

	list_for_each_entry(sgs, &info->sg_list, node) {
		list_for_each_entry(container, &sgs->container_list, node) {
			list_for_each_entry(module, &container->modules_list, node) {
				if ((module->module_id == MODULE_ID_WR_SHARED_MEM_EP) ||
					(module->module_id == MODULE_ID_RD_SHARED_MEM_EP) ||
					(module->module_id == MODULE_ID_SH_MEM_PULL_MODE) ||
					(module->module_id == MODULE_ID_SH_MEM_PUSH_MODE))
					continue;

				ret = audioreach_set_media_format(graph, module, cfg);
				if (ret)
					return ret;
			}
		}
	}

	return 0;

}
EXPORT_SYMBOL_GPL(q6apm_graph_media_format_pcm);

int q6apm_write_async(struct q6apm_graph *graph, uint32_t len, uint32_t msw_ts,
		      uint32_t lsw_ts, uint32_t wflags)
{
	struct apm_data_cmd_wr_sh_mem_ep_data_buffer_v2 *write_buffer;
	struct audio_buffer *ab;

	struct gpr_pkt *pkt __free(kfree) = audioreach_alloc_pkt(sizeof(*write_buffer),
					DATA_CMD_WR_SH_MEM_EP_DATA_BUFFER_V2,
					graph->rx_data.dsp_buf | (len << APM_WRITE_TOKEN_LEN_SHIFT),
					graph->port->id, graph->shm_iid);
	if (IS_ERR(pkt))
		return PTR_ERR(pkt);

	write_buffer = (void *)pkt + GPR_HDR_SIZE;

	mutex_lock(&graph->lock);
	ab = &graph->rx_data.buf[graph->rx_data.dsp_buf];

	write_buffer->buf_addr_lsw = lower_32_bits(ab->phys);
	write_buffer->buf_addr_msw = upper_32_bits(ab->phys);
	write_buffer->buf_size = len;
	write_buffer->timestamp_lsw = lsw_ts;
	write_buffer->timestamp_msw = msw_ts;
	write_buffer->mem_map_handle = graph->info->mem_map_handle;
	write_buffer->flags = wflags;

	graph->rx_data.dsp_buf++;

	if (graph->rx_data.dsp_buf >= graph->rx_data.num_periods)
		graph->rx_data.dsp_buf = 0;

	mutex_unlock(&graph->lock);

	return gpr_send_port_pkt(graph->port, pkt);
}
EXPORT_SYMBOL_GPL(q6apm_write_async);

int q6apm_read(struct q6apm_graph *graph)
{
	struct data_cmd_rd_sh_mem_ep_data_buffer_v2 *read_buffer;
	struct audioreach_graph_data *port;
	struct audio_buffer *ab;

	struct gpr_pkt *pkt __free(kfree) = audioreach_alloc_pkt(sizeof(*read_buffer),
					DATA_CMD_RD_SH_MEM_EP_DATA_BUFFER_V2,
					graph->tx_data.dsp_buf, graph->port->id, graph->shm_iid);
	if (IS_ERR(pkt))
		return PTR_ERR(pkt);

	read_buffer = (void *)pkt + GPR_HDR_SIZE;

	mutex_lock(&graph->lock);
	port = &graph->tx_data;
	ab = &port->buf[port->dsp_buf];

	read_buffer->buf_addr_lsw = lower_32_bits(ab->phys);
	read_buffer->buf_addr_msw = upper_32_bits(ab->phys);
	read_buffer->mem_map_handle = graph->info->mem_map_handle;
	read_buffer->buf_size = ab->size;

	port->dsp_buf++;

	if (port->dsp_buf >= port->num_periods)
		port->dsp_buf = 0;

	mutex_unlock(&graph->lock);

	return gpr_send_port_pkt(graph->port, pkt);
}
EXPORT_SYMBOL_GPL(q6apm_read);

int q6apm_get_hw_pointer(struct q6apm_graph *graph, int dir)
{
	struct audioreach_graph_data *data;

	if (dir == SNDRV_PCM_STREAM_PLAYBACK)
		data = &graph->rx_data;
	else
		data = &graph->tx_data;

	return (int)atomic_read(&data->hw_ptr);
}
EXPORT_SYMBOL_GPL(q6apm_get_hw_pointer);

static int graph_callback(const struct gpr_resp_pkt *data, void *priv, int op)
{
	struct data_cmd_rsp_rd_sh_mem_ep_data_buffer_done_v2 *rd_done;
	struct data_cmd_rsp_wr_sh_mem_ep_data_buffer_done_v2 *done;
	struct apm_module_event *event;
	const struct gpr_ibasic_rsp_result_t *result;
	struct q6apm_graph *graph = priv;
	const struct gpr_hdr *hdr = &data->hdr;
	struct device *dev = graph->dev;
	uint32_t client_event;
	phys_addr_t phys;
	int token;

	result = data->payload;

	switch (hdr->opcode) {
	case APM_EVENT_MODULE_TO_CLIENT:
		event = data->payload;
		switch (event->event_id) {
		case EVENT_ID_SH_MEM_PULL_PUSH_MODE_WATERMARK:
			client_event = APM_CLIENT_EVENT_WATERMARK_EVENT;
			graph->cb(client_event, hdr->token, data->payload, graph->priv);
			break;
		}

		break;
	case DATA_CMD_RSP_WR_SH_MEM_EP_DATA_BUFFER_DONE_V2:
		if (!graph->ar_graph)
			break;
		client_event = APM_CLIENT_EVENT_DATA_WRITE_DONE;
		mutex_lock(&graph->lock);
		token = hdr->token & APM_WRITE_TOKEN_MASK;

		done = data->payload;
		if (!graph->rx_data.buf) {
			mutex_unlock(&graph->lock);
			break;
		}
		phys = graph->rx_data.buf[token].phys;
		mutex_unlock(&graph->lock);
		/* token numbering starts at 0 */
		atomic_set(&graph->rx_data.hw_ptr, token + 1);
		if (lower_32_bits(phys) == done->buf_addr_lsw &&
		    upper_32_bits(phys) == done->buf_addr_msw) {
			graph->result.opcode = hdr->opcode;
			graph->result.status = done->status;
			if (graph->cb)
				graph->cb(client_event, hdr->token, data->payload, graph->priv);
		} else {
			dev_err(dev, "WR BUFF Unexpected addr %08x-%08x\n", done->buf_addr_lsw,
				done->buf_addr_msw);
		}

		break;
	case DATA_CMD_RSP_RD_SH_MEM_EP_DATA_BUFFER_V2:
		if (!graph->ar_graph)
			break;
		client_event = APM_CLIENT_EVENT_DATA_READ_DONE;
		mutex_lock(&graph->lock);
		rd_done = data->payload;
		if (!graph->tx_data.buf) {
			mutex_unlock(&graph->lock);
			break;
		}
		phys = graph->tx_data.buf[hdr->token].phys;
		mutex_unlock(&graph->lock);
		/* token numbering starts at 0 */
		atomic_set(&graph->tx_data.hw_ptr, hdr->token + 1);

		if (upper_32_bits(phys) == rd_done->buf_addr_msw &&
		    lower_32_bits(phys) == rd_done->buf_addr_lsw) {
			graph->result.opcode = hdr->opcode;
			graph->result.status = rd_done->status;
			if (graph->cb)
				graph->cb(client_event, hdr->token, data->payload, graph->priv);
		} else {
			dev_err(dev, "RD BUFF Unexpected addr %08x-%08x\n", rd_done->buf_addr_lsw,
				rd_done->buf_addr_msw);
		}
		break;
	case DATA_CMD_WR_SH_MEM_EP_EOS_RENDERED:
		client_event = APM_CLIENT_EVENT_CMD_EOS_DONE;
		if (graph->cb)
			graph->cb(client_event, hdr->token, data->payload, graph->priv);
		break;
	case GPR_BASIC_RSP_RESULT:
		switch (result->opcode) {
		case APM_CMD_SHARED_MEM_MAP_REGIONS:
		case DATA_CMD_WR_SH_MEM_EP_MEDIA_FORMAT:
		case APM_CMD_REGISTER_MODULE_EVENTS:
		case APM_CMD_SET_CFG:
			graph->result.opcode = result->opcode;
			graph->result.status = result->status;
			if (result->status)
				dev_err(dev, "Error (%d) Processing 0x%08x cmd\n",
					result->status, result->opcode);
			wake_up(&graph->cmd_wait);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
	return 0;
}

int q6apm_register_watermark_event(struct q6apm_graph *graph, int water_mark_level_bytes,
				   int num_levels)
{
	return audioreach_shmem_register_event(graph, water_mark_level_bytes, num_levels);
}
EXPORT_SYMBOL_GPL(q6apm_register_watermark_event);

int q6apm_push_pull_config(struct q6apm_graph *graph, phys_addr_t bphys,
			   phys_addr_t pphys, uint32_t size)
{
	struct audioreach_graph_info *info = graph->info;

	return audioreach_setup_push_pull(graph, bphys, pphys, info->mem_map_handle,
					  info->pos_buf_mem_map_handle, size);
}
EXPORT_SYMBOL_GPL(q6apm_push_pull_config);

bool q6apm_is_graph_in_push_pull_mode_from_id(struct device *dev, unsigned int graph_id, int dir)
{
	struct audioreach_graph_info *info;
	struct q6apm *apm = dev_get_drvdata(dev->parent);
	struct audioreach_module *module;

	info = idr_find(&apm->graph_info_idr, graph_id);
	if (!info)
		return false;

	if (dir == SNDRV_PCM_STREAM_PLAYBACK)
		module = __q6apm_find_module_by_mid(apm, info, MODULE_ID_SH_MEM_PULL_MODE);
	else
		module = __q6apm_find_module_by_mid(apm, info, MODULE_ID_SH_MEM_PUSH_MODE);

	return !!module;

}
EXPORT_SYMBOL_GPL(q6apm_is_graph_in_push_pull_mode_from_id);

bool q6apm_is_graph_in_push_pull_mode(struct q6apm_graph *graph)
{
	return graph->info->is_push_pull_mode;
}
EXPORT_SYMBOL_GPL(q6apm_is_graph_in_push_pull_mode);

static int q6apm_graph_get_module_iid(struct q6apm_graph *graph, uint32_t mid)
{
	struct audioreach_module *module;

	module = q6apm_find_module_by_mid(graph, mid);
	if (!module)
		return -ENODEV;

	return module->instance_id;
}

struct q6apm_graph *q6apm_graph_open(struct device *dev, q6apm_cb cb,
				     void *priv, int graph_id, int dir)
{
	struct q6apm *apm = dev_get_drvdata(dev->parent);
	struct audioreach_graph *ar_graph;
	struct q6apm_graph *graph;
	int ret, iid = 0;

	ar_graph = q6apm_get_audioreach_graph(apm, graph_id);
	if (IS_ERR(ar_graph)) {
		dev_err(dev, "No graph found with id %d\n", graph_id);
		return ERR_CAST(ar_graph);
	}

	graph = kzalloc_obj(*graph);
	if (!graph) {
		ret = -ENOMEM;
		goto put_ar_graph;
	}

	graph->apm = apm;
	graph->priv = priv;
	graph->cb = cb;
	graph->info = ar_graph->info;
	graph->ar_graph = ar_graph;
	graph->id = ar_graph->id;
	graph->dev = dev;

	if (dir == SNDRV_PCM_STREAM_PLAYBACK) {
		iid = q6apm_graph_get_module_iid(graph, MODULE_ID_SH_MEM_PULL_MODE);
		if (iid < 0)
			iid = q6apm_graph_get_module_iid(graph, MODULE_ID_WR_SHARED_MEM_EP);
		else
			graph->info->is_push_pull_mode = true;

	} else {
		iid = q6apm_graph_get_module_iid(graph, MODULE_ID_SH_MEM_PUSH_MODE);
		if (iid < 0)
			iid = q6apm_graph_get_module_iid(graph, MODULE_ID_RD_SHARED_MEM_EP);
		else
			graph->info->is_push_pull_mode = true;
	}

	if (iid > 0)
		graph->shm_iid = iid;

	mutex_init(&graph->lock);
	init_waitqueue_head(&graph->cmd_wait);

	graph->port = gpr_alloc_port(apm->gdev, dev, graph_callback, graph);
	if (IS_ERR(graph->port)) {
		ret = PTR_ERR(graph->port);
		goto free_graph;
	}

	return graph;

free_graph:
	kfree(graph);
put_ar_graph:
	kref_put(&ar_graph->refcount, q6apm_put_audioreach_graph);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(q6apm_graph_open);

int q6apm_graph_close(struct q6apm_graph *graph)
{
	struct audioreach_graph *ar_graph = graph->ar_graph;

	graph->ar_graph = NULL;
	kref_put(&ar_graph->refcount, q6apm_put_audioreach_graph);
	gpr_free_port(graph->port);
	kfree(graph);

	return 0;
}
EXPORT_SYMBOL_GPL(q6apm_graph_close);

int q6apm_graph_prepare(struct q6apm_graph *graph)
{
	return audioreach_graph_mgmt_cmd(graph->ar_graph, APM_CMD_GRAPH_PREPARE);
}
EXPORT_SYMBOL_GPL(q6apm_graph_prepare);

int q6apm_graph_start(struct q6apm_graph *graph)
{
	struct audioreach_graph *ar_graph = graph->ar_graph;
	int ret;

	if (ar_graph->start_count == 0) {
		ret = audioreach_graph_mgmt_cmd(ar_graph, APM_CMD_GRAPH_START);
		if (ret)
			return ret;
	}

	ar_graph->start_count++;

	return 0;
}
EXPORT_SYMBOL_GPL(q6apm_graph_start);

int q6apm_graph_stop(struct q6apm_graph *graph)
{
	struct audioreach_graph *ar_graph = graph->ar_graph;

	if (ar_graph->start_count == 0)
		return 0;

	if (--ar_graph->start_count > 0)
		return 0;

	return audioreach_graph_mgmt_cmd(ar_graph, APM_CMD_GRAPH_STOP);
}
EXPORT_SYMBOL_GPL(q6apm_graph_stop);

int q6apm_graph_flush(struct q6apm_graph *graph)
{
	return audioreach_graph_mgmt_cmd(graph->ar_graph, APM_CMD_GRAPH_FLUSH);
}
EXPORT_SYMBOL_GPL(q6apm_graph_flush);

static int q6apm_audio_probe(struct snd_soc_component *component)
{
	return audioreach_tplg_init(component);
}

static void q6apm_audio_remove(struct snd_soc_component *component)
{
	/* remove topology */
	snd_soc_tplg_component_remove(component);
}

#define APM_AUDIO_DRV_NAME "q6apm-audio"

static const struct snd_soc_component_driver q6apm_audio_component = {
	.name		= APM_AUDIO_DRV_NAME,
	.probe		= q6apm_audio_probe,
	.remove		= q6apm_audio_remove,
	.remove_order   = SND_SOC_COMP_ORDER_LAST,
};

/*
 * EXPERIMENT: voice call proof of concept. Opens stock's voice RX graph on
 * the ADSP by hand, to see whether the modem's downlink reaches VCPM:
 *
 *   echo start > /sys/kernel/debug/q6apm-voice/ctl   (during a call)
 *   echo stop  > /sys/kernel/debug/q6apm-voice/ctl
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
#include <linux/firmware.h>
#include <linux/of_clk.h>

static const u32 voice_poc_rx_sgs[] = {
	4, 0xb0000045, 0xb0000044, 0xb0000041, 0xb0000040,
};
static const u32 voice_poc_tx_sgs[] = {
	3, 0xb000003f, 0xb00000b1, 0xb0000039,
};
/*
 * Started without the mic device sub-graph (0xb0000039): its CODEC_DMA
 * source fails GRAPH_START with no TX macro described in the DT.
 */
static const u32 voice_poc_tx_start_sgs[] = {
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
	/* mics: 48 kHz, 16 bit, 2 channels; LPAIF_RXTX, TX codec DMA 3, mask 5 */
	static const u32 mic_mf[] = { 48000, 0x00020010, 1 };
	static const u32 mic_dma[] = { 1, 4, 5 };
	u8 buf[256];
	size_t n = 0;
	int rc;

	rc = voice_poc_get_clks(apm);
	if (rc)
		return rc;

	/* Marked open first: an open that times out here may still complete on the DSP. */
	voice_poc_rx_open = true;
	rc = voice_poc_send_fw(apm, APM_CMD_GRAPH_OPEN, "qcom/sm6225/voice-rx4-open.bin");
	if (rc)
		return rc;
	rc = voice_poc_send_fw(apm, APM_CMD_SET_CFG, "qcom/sm6225/voice-rx4-cfg.bin");
	if (rc)
		return rc;
	voice_poc_tx_open = true;
	rc = voice_poc_send_fw(apm, APM_CMD_GRAPH_OPEN, "qcom/sm6225/voice-tx-open.bin");
	if (rc)
		return rc;
	rc = voice_poc_send_fw(apm, APM_CMD_SET_CFG, "qcom/sm6225/voice-tx-cfg.bin");
	if (rc)
		return rc;

	n += voice_poc_rec(buf + n, VCPM_MODULE_INSTANCE_ID, 0x080011bc, vsid, sizeof(vsid));
	n += voice_poc_rec(buf + n, VCPM_MODULE_INSTANCE_ID, 0x08001310, tx_ch, sizeof(tx_ch));
	n += voice_poc_rec(buf + n, VCPM_MODULE_INSTANCE_ID, 0x0800116b, cal_keys, sizeof(cal_keys));
	rc = voice_poc_send(apm, APM_CMD_SET_CFG, buf, n);
	if (rc)
		return rc;

	n = 0;
	n += voice_poc_rec(buf + n, 0x465b, 0x08001024, mfc, 12);
	n += voice_poc_rec(buf + n, 0x41dd, 0x08001024, mfc, 12);
	rc = voice_poc_send(apm, APM_CMD_SET_CFG, buf, n);
	if (rc)
		return rc;
	rc = VOICE_POC_SG(APM_CMD_GRAPH_PREPARE, voice_poc_rx_sgs);
	if (rc)
		return rc;
	rc = VOICE_POC_SG(APM_CMD_GRAPH_START, voice_poc_rx_sgs);
	if (rc)
		return rc;

	n = 0;
	n += voice_poc_rec(buf + n, 0x43af, 0x08001017, mic_mf, sizeof(mic_mf));
	n += voice_poc_rec(buf + n, 0x43af, 0x08001063, mic_dma, sizeof(mic_dma));
	rc = voice_poc_send(apm, APM_CMD_SET_CFG, buf, n);
	if (rc)
		return rc;
	rc = VOICE_POC_SG(APM_CMD_GRAPH_PREPARE, voice_poc_tx_sgs);
	if (rc)
		return rc;
	rc = VOICE_POC_SG(APM_CMD_GRAPH_START, voice_poc_tx_start_sgs);
	if (rc)
		return rc;
	/* the mic sub-graph separately, so a failure there keeps the rest */
	{
		static const u32 mic_sg[] = { 1, 0xb0000039 };

		if (VOICE_POC_SG(APM_CMD_GRAPH_START, mic_sg))
			dev_warn(apm->dev, "voice-poc: mic sub-graph did not start\n");
	}
	return 0;
}

static void voice_poc_stop(struct q6apm *apm)
{
	if (voice_poc_tx_open) {
		VOICE_POC_SG(APM_CMD_GRAPH_STOP, voice_poc_tx_sgs);
		VOICE_POC_SG(APM_CMD_GRAPH_CLOSE, voice_poc_tx_sgs);
		voice_poc_tx_open = false;
	}
	if (voice_poc_rx_open) {
		VOICE_POC_SG(APM_CMD_GRAPH_STOP, voice_poc_rx_sgs);
		VOICE_POC_SG(APM_CMD_GRAPH_CLOSE, voice_poc_rx_sgs);
		voice_poc_rx_open = false;
	}
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
	} else if (!strncmp(cmd, "mic", 3)) {
		/* start the mic device sub-graph on a running session, alone */
		static const u32 mic_sg[] = { 1, 0xb0000039 };

		rc = VOICE_POC_SG(APM_CMD_GRAPH_START, mic_sg);
	} else {
		return -EINVAL;
	}
	return rc ? rc : count;
}

static const struct file_operations voice_poc_fops = {
	.open = simple_open,
	.write = voice_poc_write,
};

static int apm_probe(gpr_device_t *gdev)
{
	struct device *dev = &gdev->dev;
	struct q6apm *apm;
	int ret;

	apm = devm_kzalloc(dev, sizeof(*apm), GFP_KERNEL);
	if (!apm)
		return -ENOMEM;

	dev_set_drvdata(dev, apm);

	mutex_init(&apm->lock);
	apm->dev = dev;
	apm->gdev = gdev;
	init_waitqueue_head(&apm->wait);

	INIT_LIST_HEAD(&apm->widget_list);
	idr_init(&apm->graph_idr);
	idr_init(&apm->graph_info_idr);
	idr_init(&apm->sub_graphs_idr);
	idr_init(&apm->containers_idr);

	idr_init(&apm->modules_idr);

	g_apm = apm;

	q6apm_get_apm_state(apm);

	ret = snd_soc_register_component(dev, &q6apm_audio_component, NULL, 0);
	if (ret < 0) {
		dev_err(dev, "failed to register q6apm: %d\n", ret);
		return ret;
	}

	ret = of_platform_populate(dev->of_node, NULL, NULL, dev);
	if (ret)
		snd_soc_unregister_component(dev);

	voice_poc_dir = debugfs_create_dir("q6apm-voice", NULL);
	debugfs_create_file("ctl", 0200, voice_poc_dir, apm, &voice_poc_fops);

	return ret;
}

static void apm_remove(gpr_device_t *gdev)
{
	voice_poc_stop(dev_get_drvdata(&gdev->dev));
	debugfs_remove_recursive(voice_poc_dir);
	of_platform_depopulate(&gdev->dev);
	snd_soc_unregister_component(&gdev->dev);
}

struct audioreach_module *q6apm_find_module_by_mid(struct q6apm_graph *graph, uint32_t mid)
{
	struct audioreach_graph_info *info = graph->info;
	struct q6apm *apm = graph->apm;

	return __q6apm_find_module_by_mid(apm, info, mid);

}

static int apm_callback(const struct gpr_resp_pkt *data, void *priv, int op)
{
	gpr_device_t *gdev = priv;
	struct audioreach_graph_info *info;
	struct q6apm *apm = dev_get_drvdata(&gdev->dev);
	struct apm_cmd_rsp_shared_mem_map_regions *rsp;
	struct device *dev = &gdev->dev;
	struct gpr_ibasic_rsp_result_t *result;
	const struct gpr_hdr *hdr = &data->hdr;
	int graph_id, is_pos_buf;

	result = data->payload;

	switch (hdr->opcode) {
	case APM_CMD_RSP_GET_SPF_STATE:
		apm->result.opcode = hdr->opcode;
		apm->result.status = 0;
		/* First word of result it state */
		apm->state = result->opcode;
		wake_up(&apm->wait);
		break;
	case GPR_BASIC_RSP_RESULT:
		switch (result->opcode) {
		case APM_CMD_SHARED_MEM_MAP_REGIONS:
		case APM_CMD_GRAPH_START:
		case APM_CMD_GRAPH_OPEN:
		case APM_CMD_GRAPH_PREPARE:
		case APM_CMD_GRAPH_CLOSE:
		case APM_CMD_GRAPH_FLUSH:
		case APM_CMD_GRAPH_STOP:
		case APM_CMD_SET_CFG:
			apm->result.opcode = result->opcode;
			apm->result.status = result->status;
			if (result->status)
				dev_err(dev, "Error (%d) Processing 0x%08x cmd\n", result->status,
					result->opcode);
			wake_up(&apm->wait);
			break;
		case APM_CMD_SHARED_MEM_UNMAP_REGIONS:
			apm->result.opcode = hdr->opcode;
			apm->result.status = 0;
			rsp = data->payload;

			info = idr_find(&apm->graph_info_idr, hdr->token);
			if (info)
				info->mem_map_handle = 0;
			else
				dev_err(dev, "Error (%d) Processing 0x%08x cmd\n", result->status,
					result->opcode);

			wake_up(&apm->wait);
			break;
		default:
			break;
		}
		break;
	case APM_CMD_RSP_SHARED_MEM_MAP_REGIONS:
		apm->result.opcode = hdr->opcode;
		apm->result.status = 0;
		rsp = data->payload;
		graph_id = hdr->token & APM_MMAP_TOKEN_GID_MASK;
		is_pos_buf = hdr->token & APM_MMAP_TOKEN_MAP_TYPE_POS_BUF;

		info = idr_find(&apm->graph_info_idr, graph_id);
		if (info) {
			if (is_pos_buf)
				info->pos_buf_mem_map_handle = rsp->mem_map_handle;
			else
				info->mem_map_handle = rsp->mem_map_handle;
		} else {
			dev_err(dev, "Error (%d) Processing 0x%08x cmd\n", result->status,
				result->opcode);
		}

		wake_up(&apm->wait);
		break;
	default:
		break;
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id apm_device_id[]  = {
	{ .compatible = "qcom,q6apm" },
	{},
};
MODULE_DEVICE_TABLE(of, apm_device_id);
#endif

static gpr_driver_t apm_driver = {
	.probe = apm_probe,
	.remove = apm_remove,
	.gpr_callback = apm_callback,
	.driver = {
		.name = "qcom-apm",
		.of_match_table = of_match_ptr(apm_device_id),
	},
};

module_gpr_driver(apm_driver);
MODULE_DESCRIPTION("Audio Process Manager");
MODULE_LICENSE("GPL");
