// SPDX-License-Identifier: GPL-2.0-only
/*
 * Motorola unit info for the modem
 *
 * Motorola's modem firmware takes its RF hardware id at runtime from a
 * "unit info" block the application processor publishes in SMEM item 134
 * (SMEM_ID_VENDOR0) before the modem boots. Motorola's downstream kernels
 * fill it from the bootloader's androidboot.* arguments. Without it the
 * modem reads RF hardware id 0, fails to create its RF card and never
 * brings RF up, while otherwise booting normally.
 *
 * The modem only needs the device name and the radio SKU. The device name
 * is described in the devicetree. The radio SKU is detected per unit by the
 * bootloader (on fogona from a hardware id, ATT or RET) and passed only to
 * its own kernels, so it comes either from the devicetree, where a board
 * has a single SKU, or from userspace through the "radio" attribute, which
 * publishes the block when written. The modem must not boot before then.
 */

#include <linux/cleanup.h>
#include <linux/err.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/soc/qcom/smem.h>
#include <linux/string.h>

#define MMI_UNIT_INFO_SMEM_ID	134
#define MMI_UNIT_INFO_SIZE	1024
#define MMI_UNIT_INFO_VERSION	3

/* Layout as Motorola's modem firmware reads it; do not reorder. */
struct mmi_unit_info {
	__le32 version;
	__le32 system_rev;
	__le32 system_serial_low;
	__le32 system_serial_high;
	char machine[33];
	char barcode[65];
	char carrier[65];
	char baseband[97];
	char device[33];
	__le32 radio;
	__le32 powerup_reason;
	char radio_str[33];
};

struct mmi_unit_info_priv {
	const char *device;
	const char *carrier;
	char radio[33];
	struct mutex lock;
};

static int mmi_unit_info_publish(struct device *dev,
				 struct mmi_unit_info_priv *priv)
{
	struct mmi_unit_info *mui;
	size_t size;
	int ret;

	ret = qcom_smem_alloc(QCOM_SMEM_HOST_ANY, MMI_UNIT_INFO_SMEM_ID,
			      MMI_UNIT_INFO_SIZE);
	if (ret && ret != -EEXIST)
		return dev_err_probe(dev, ret, "failed to allocate SMEM item\n");

	mui = qcom_smem_get(QCOM_SMEM_HOST_ANY, MMI_UNIT_INFO_SMEM_ID, &size);
	if (IS_ERR(mui))
		return dev_err_probe(dev, PTR_ERR(mui), "failed to get SMEM item\n");
	if (size < sizeof(*mui))
		return dev_err_probe(dev, -EINVAL, "SMEM item too small: %zu\n",
				     size);

	memset(mui, 0, size);
	mui->version = cpu_to_le32(MMI_UNIT_INFO_VERSION);
	strscpy(mui->device, priv->device, sizeof(mui->device));
	strscpy(mui->radio_str, priv->radio, sizeof(mui->radio_str));
	strscpy(mui->carrier, priv->carrier, sizeof(mui->carrier));

	dev_info(dev, "published unit info: device '%s', radio '%s'\n",
		 priv->device, priv->radio);

	return 0;
}

static ssize_t radio_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct mmi_unit_info_priv *priv = dev_get_drvdata(dev);

	guard(mutex)(&priv->lock);
	return sysfs_emit(buf, "%s\n", priv->radio);
}

static ssize_t radio_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct mmi_unit_info_priv *priv = dev_get_drvdata(dev);
	size_t len = strcspn(buf, "\n");
	int ret;

	if (!len || len >= sizeof(priv->radio))
		return -EINVAL;

	guard(mutex)(&priv->lock);
	memcpy(priv->radio, buf, len);
	priv->radio[len] = '\0';

	ret = mmi_unit_info_publish(dev, priv);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(radio);

static struct attribute *mmi_unit_info_attrs[] = {
	&dev_attr_radio.attr,
	NULL
};
ATTRIBUTE_GROUPS(mmi_unit_info);

static int mmi_unit_info_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mmi_unit_info_priv *priv;
	const char *radio;
	int ret;

	BUILD_BUG_ON(sizeof(struct mmi_unit_info) > MMI_UNIT_INFO_SIZE);

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ret = devm_mutex_init(dev, &priv->lock);
	if (ret)
		return ret;

	ret = of_property_read_string(dev->of_node, "motorola,device",
				      &priv->device);
	if (ret)
		return dev_err_probe(dev, ret, "missing motorola,device\n");

	priv->carrier = "";
	of_property_read_string(dev->of_node, "motorola,carrier",
				&priv->carrier);

	platform_set_drvdata(pdev, priv);

	/* A board with a single radio SKU can describe it; otherwise wait. */
	if (of_property_read_string(dev->of_node, "motorola,radio", &radio))
		return 0;

	strscpy(priv->radio, radio, sizeof(priv->radio));
	return mmi_unit_info_publish(dev, priv);
}

static const struct of_device_id mmi_unit_info_of_match[] = {
	{ .compatible = "motorola,mmi-unit-info" },
	{ }
};
MODULE_DEVICE_TABLE(of, mmi_unit_info_of_match);

static struct platform_driver mmi_unit_info_driver = {
	.probe = mmi_unit_info_probe,
	.driver = {
		.name = "mmi-unit-info",
		.of_match_table = mmi_unit_info_of_match,
		.dev_groups = mmi_unit_info_groups,
	},
};
module_platform_driver(mmi_unit_info_driver);

MODULE_DESCRIPTION("Motorola unit info for the modem");
MODULE_LICENSE("GPL");
