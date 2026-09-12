// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/pm_runtime.h>

#include "iris_core.h"
#include "iris_firmware.h"
#include "iris_state.h"
#include "iris_vpu_common.h"

void iris_core_deinit(struct iris_core *core)
{
	int ret;

	ret = pm_runtime_resume_and_get(core->dev);

	mutex_lock(&core->lock);
	if (core->state != IRIS_CORE_DEINIT) {
		iris_fw_unload(core);

		if (!ret)
			iris_vpu_power_off(core);

		iris_hfi_queues_deinit(core);
		core->state = IRIS_CORE_DEINIT;
		/*
		 * Cleared only when this actually tore the hardware down. A core
		 * that failed init already cleaned up and left the state at
		 * IRIS_CORE_DEINIT, so this block is skipped and the latch
		 * survives -- otherwise the sys_error handler's deinit + init
		 * would walk straight back into the init that failed.
		 */
		core->init_failed = false;
	}
	mutex_unlock(&core->lock);

	if (!ret)
		pm_runtime_put_sync(core->dev);
}

static int iris_wait_for_system_response(struct iris_core *core)
{
	int ret;

	if (core->state == IRIS_CORE_ERROR)
		return -EIO;

	ret = wait_for_completion_timeout(&core->core_init_done,
					  msecs_to_jiffies(HW_RESPONSE_TIMEOUT_VALUE));
	if (!ret) {
		core->state = IRIS_CORE_ERROR;
		return -ETIMEDOUT;
	}

	return 0;
}

int iris_core_init(struct iris_core *core)
{
	int ret;

	mutex_lock(&core->lock);
	if (core->state == IRIS_CORE_INIT) {
		ret = 0;
		goto exit;
	} else if (core->state == IRIS_CORE_ERROR) {
		ret = -EINVAL;
		goto exit;
	}

	/*
	 * A previous init failed and cleaned the hardware back up, so the state
	 * is an honest IRIS_CORE_DEINIT -- but the core is known bad. Re-running
	 * the power-on and firmware-boot sequence against it is not recoverable
	 * on every part, and iris registers two video nodes, so udev opening the
	 * second one walks straight back in here. Refuse until an explicit
	 * deinit clears the latch.
	 */
	if (core->init_failed) {
		ret = -EIO;
		goto exit;
	}

	core->state = IRIS_CORE_INIT;

	ret = iris_hfi_queues_init(core);
	if (ret)
		goto error;

	ret = iris_vpu_power_on(core);
	if (ret)
		goto error_queue_deinit;

	ret = iris_fw_load(core);
	if (ret)
		goto error_power_off;

	ret = iris_vpu_boot_firmware(core);
	if (ret)
		goto error_unload_fw;

	ret = iris_vpu_switch_to_hwmode(core);
	if (ret)
		goto error_unload_fw;

	core->iris_firmware_data->init_hfi_ops(core);

	ret = iris_hfi_core_init(core);
	if (ret)
		goto error_unload_fw;

	mutex_unlock(&core->lock);

	return iris_wait_for_system_response(core);

error_unload_fw:
	iris_fw_unload(core);
error_power_off:
	iris_vpu_power_off(core);
error_queue_deinit:
	iris_hfi_queues_deinit(core);
error:
	core->init_failed = true;
	core->state = IRIS_CORE_DEINIT;
exit:
	mutex_unlock(&core->lock);

	return ret;
}
