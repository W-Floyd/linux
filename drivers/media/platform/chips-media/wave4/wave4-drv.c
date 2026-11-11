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
#include <linux/slab.h>

#include "wave4.h"

#define WAVE4_FW	"vpu/tcc803x/hevc_bit_code"

/* Kept until the device is released: a firmware restart loads it again. */
static int wave4_load_firmware(struct wave4_dev *dev)
{
	int ret;

	ret = request_firmware(&dev->fw, WAVE4_FW, dev->dev);
	if (ret)
		return dev_err_probe(dev->dev, ret, "firmware %s\n", WAVE4_FW);

	if (!dev->fw->size || dev->fw->size > WAVE4_CODE_SIZE ||
	    dev->fw->size & 1) {
		dev_err(dev->dev, "firmware %s: bad size %zu\n", WAVE4_FW,
			dev->fw->size);
		release_firmware(dev->fw);
		dev->fw = NULL;
		return -EINVAL;
	}
	return 0;
}

/* Reset and boot: at probe, and from w4_recover(). */
int wave4_restart(struct wave4_dev *dev)
{
	int ret;

	lockdep_assert_held(&dev->hw_lock);
	/* The block must start cold: a boot over a running VCPU fails on CODA. */
	ret = reset_control_reset(dev->rst);
	if (ret)
		return ret;
	return wave4_hw_boot(dev, dev->fw->data, dev->fw->size);
}

/* The last reference: remove() has run and every file is closed. */
static void wave4_v4l2_release(struct v4l2_device *v4l2_dev)
{
	struct wave4_dev *dev = container_of(v4l2_dev, struct wave4_dev,
					     v4l2_dev);

	if (dev->m2m_dev)
		v4l2_m2m_release(dev->m2m_dev);
	if (dev->wq)
		destroy_workqueue(dev->wq);
	release_firmware(dev->fw);
	ida_destroy(&dev->inst_ida);
	mutex_destroy(&dev->dev_lock);
	mutex_destroy(&dev->hw_lock);
	kfree(dev);
}

/* Registered first, so it runs after the rest of devres, irq included. */
static void wave4_put(void *data)
{
	struct wave4_dev *dev = data;

	v4l2_device_unregister(&dev->v4l2_dev);
	v4l2_device_put(&dev->v4l2_dev);
}

static int wave4_probe(struct platform_device *pdev)
{
	struct device *d = &pdev->dev;
	struct wave4_dev *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	dev->dev = d;
	mutex_init(&dev->hw_lock);
	mutex_init(&dev->dev_lock);
	spin_lock_init(&dev->irq_lock);
	init_completion(&dev->cmd_done);
	ida_init(&dev->inst_ida);

	dev->v4l2_dev.release = wave4_v4l2_release;
	ret = v4l2_device_register(d, &dev->v4l2_dev);
	if (ret) {
		wave4_v4l2_release(&dev->v4l2_dev);
		return ret;
	}
	ret = devm_add_action_or_reset(d, wave4_put, dev);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, dev);

	dev->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dev->base))
		return PTR_ERR(dev->base);

	dev->num_clks = devm_clk_bulk_get_all_enabled(d, &dev->clks);
	if (dev->num_clks < 0)
		return dev_err_probe(d, dev->num_clks, "clocks\n");

	/* Required: a firmware restart starts from a reset. */
	dev->rst = devm_reset_control_get_exclusive(d, NULL);
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

	ret = wave4_load_firmware(dev);
	if (ret)
		return ret;

	mutex_lock(&dev->hw_lock);
	ret = wave4_restart(dev);
	mutex_unlock(&dev->hw_lock);
	if (ret) {
		/* Leave it held in reset: devres is about to free the DMA
		 * memory a half-booted VCPU could still be holding.
		 */
		wave4_hw_halt(dev);
		return dev_err_probe(d, ret, "firmware boot\n");
	}
	dev_info(d, "firmware %s (%zu bytes) up, version %#x\n", WAVE4_FW,
		 dev->fw->size, dev->fw_version);

	/* Both freed by wave4_v4l2_release(). */
	dev->wq = alloc_ordered_workqueue("wave4", 0);
	if (!dev->wq) {
		ret = -ENOMEM;
		goto err_halt;
	}

	ret = wave4_dec_register(dev);
	if (ret)
		goto err_halt;

	return 0;

err_halt:
	wave4_hw_halt(dev);
	return ret;
}

/*
 * dev itself stays until the last file is closed (wave4_v4l2_release()).
 * The halt waits for a command in progress and keeps every later one off
 * the hardware, whose registers and clocks devres is about to drop.
 */
static void wave4_remove(struct platform_device *pdev)
{
	struct wave4_dev *dev = platform_get_drvdata(pdev);

	wave4_dec_unregister(dev);
	wave4_hw_halt(dev);
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

MODULE_FIRMWARE(WAVE4_FW);
MODULE_DESCRIPTION("Chips&Media Wave410 HEVC decoder");
MODULE_LICENSE("GPL");
