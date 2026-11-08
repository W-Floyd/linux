// SPDX-License-Identifier: GPL-2.0-only
/*
 * Chips&Media Wave410 HEVC decoder: platform glue
 *
 * Found in the Telechips TCC803x (Mobis Gen5W head units) next to a CODA960.
 * The firmware is not distributed: stock ships it as the bit_code array of
 * its hevc_lib.ko, 63308 bytes, loaded as-is at the start of a 1 MiB code
 * region.
 */
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "wave4.h"

#define WAVE4_DEFAULT_FW	"vpu/tcc803x/hevc_bit_code"

static int wave4_boot(struct wave4_dev *dev)
{
	const struct firmware *fw;
	const char *name = WAVE4_DEFAULT_FW;
	int ret;

	of_property_read_string(dev->dev->of_node, "firmware-name", &name);
	ret = request_firmware(&fw, name, dev->dev);
	if (ret)
		return dev_err_probe(dev->dev, ret, "firmware %s\n", name);

	if (!fw->size || fw->size > WAVE4_CODE_SIZE || fw->size & 1) {
		dev_err(dev->dev, "firmware %s: bad size %zu\n", name, fw->size);
		ret = -EINVAL;
		goto out;
	}

	/* The block must start cold: a boot over a running VCPU fails on CODA. */
	ret = reset_control_reset(dev->rst);
	if (ret)
		goto out;

	mutex_lock(&dev->hw_lock);
	ret = wave4_hw_boot(dev, fw->data, fw->size);
	mutex_unlock(&dev->hw_lock);
	if (ret)
		dev_err(dev->dev, "firmware boot failed: %d\n", ret);
	else
		dev_info(dev->dev, "firmware %s (%zu bytes) up, version %#x\n",
			 name, fw->size, dev->fw_version);
out:
	release_firmware(fw);
	return ret;
}

static int wave4_probe(struct platform_device *pdev)
{
	struct device *d = &pdev->dev;
	struct wave4_dev *dev;
	int ret;

	dev = devm_kzalloc(d, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	dev->dev = d;
	mutex_init(&dev->hw_lock);
	mutex_init(&dev->dev_lock);
	init_completion(&dev->cmd_done);
	ida_init(&dev->inst_ida);
	platform_set_drvdata(pdev, dev);

	dev->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dev->base))
		return PTR_ERR(dev->base);

	dev->num_clks = devm_clk_bulk_get_all_enabled(d, &dev->clks);
	if (dev->num_clks < 0)
		return dev_err_probe(d, dev->num_clks, "clocks\n");

	dev->rst = devm_reset_control_get_optional_exclusive(d, NULL);
	if (IS_ERR(dev->rst))
		return dev_err_probe(d, PTR_ERR(dev->rst), "reset\n");

	ret = dma_set_mask_and_coherent(d, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	dev->common_vaddr = dmam_alloc_coherent(d, WAVE4_COMMON_SIZE,
						&dev->common_daddr, GFP_KERNEL);
	if (!dev->common_vaddr)
		return -ENOMEM;

	dev->irq = platform_get_irq(pdev, 0);
	if (dev->irq < 0)
		return dev->irq;
	ret = devm_request_irq(d, dev->irq, wave4_irq, 0, dev_name(d), dev);
	if (ret)
		return ret;

	ret = wave4_boot(dev);
	if (ret)
		return ret;

	dev->wq = alloc_ordered_workqueue("wave4", 0);
	if (!dev->wq) {
		ret = -ENOMEM;
		goto err_halt;
	}

	ret = v4l2_device_register(d, &dev->v4l2_dev);
	if (ret)
		goto err_wq;

	ret = wave4_dec_register(dev);
	if (ret)
		goto err_v4l2;

	return 0;

err_v4l2:
	v4l2_device_unregister(&dev->v4l2_dev);
err_wq:
	destroy_workqueue(dev->wq);
err_halt:
	wave4_hw_halt(dev);
	return ret;
}

static void wave4_remove(struct platform_device *pdev)
{
	struct wave4_dev *dev = platform_get_drvdata(pdev);

	wave4_dec_unregister(dev);
	v4l2_device_unregister(&dev->v4l2_dev);
	destroy_workqueue(dev->wq);
	wave4_hw_halt(dev);
	ida_destroy(&dev->inst_ida);
}

/* Stop the VCPU and its DMA before a kexec'd kernel owns the memory. */
static void wave4_shutdown(struct platform_device *pdev)
{
	wave4_hw_halt(platform_get_drvdata(pdev));
}

static const struct of_device_id wave4_of_match[] = {
	{ .compatible = "telechips,tcc803x-wave410" },
	{ }
};
MODULE_DEVICE_TABLE(of, wave4_of_match);

static struct platform_driver wave4_driver = {
	.probe = wave4_probe,
	.remove = wave4_remove,
	.shutdown = wave4_shutdown,
	.driver = {
		.name = "wave4",
		.of_match_table = wave4_of_match,
	},
};
module_platform_driver(wave4_driver);

MODULE_FIRMWARE(WAVE4_DEFAULT_FW);
MODULE_DESCRIPTION("Chips&Media Wave410 HEVC decoder");
MODULE_LICENSE("GPL");
