// SPDX-License-Identifier: GPL-2.0
/*
 * Telechips TCC803x reboot mode, recorded through the secure monitor.
 *
 * The reset-reason register is secure; the monitor keeps it across the reset
 * and the bootloader reads it back to pick fastboot, recovery or a normal boot.
 */

#include <linux/arm-smccc.h>
#include <linux/device.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reboot-mode.h>
#include <linux/types.h>

/* SiP device 3 (reset), SET_RESET_REASON; x1 = magic. */
#define TCC803X_SIP_SET_RESET_REASON	0x82003002

static int tcc803x_reboot_mode_write(struct reboot_mode_driver *reboot,
				     unsigned int magic)
{
	struct arm_smccc_res res;

	arm_smccc_smc(TCC803X_SIP_SET_RESET_REASON, magic, 0, 0, 0, 0, 0, 0,
		      &res);

	/* SMC32 call: only w0 is defined, the upper half of x0 is not. */
	if ((s32)res.a0 == SMCCC_RET_NOT_SUPPORTED)
		return -EOPNOTSUPP;

	return 0;
}

static int tcc803x_reboot_mode_probe(struct platform_device *pdev)
{
	struct reboot_mode_driver *reboot;
	int ret;

	reboot = devm_kzalloc(&pdev->dev, sizeof(*reboot), GFP_KERNEL);
	if (!reboot)
		return -ENOMEM;

	reboot->dev = &pdev->dev;
	reboot->write = tcc803x_reboot_mode_write;

	ret = devm_reboot_mode_register(&pdev->dev, reboot);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to register reboot mode\n");

	return 0;
}

static const struct of_device_id tcc803x_reboot_mode_of_match[] = {
	{ .compatible = "telechips,tcc803x-reboot-mode" },
	{ }
};
MODULE_DEVICE_TABLE(of, tcc803x_reboot_mode_of_match);

static struct platform_driver tcc803x_reboot_mode_driver = {
	.probe = tcc803x_reboot_mode_probe,
	.driver = {
		.name = "tcc803x-reboot-mode",
		.of_match_table = tcc803x_reboot_mode_of_match,
	},
};
module_platform_driver(tcc803x_reboot_mode_driver);

MODULE_DESCRIPTION("Telechips TCC803x reboot mode driver");
MODULE_LICENSE("GPL");
