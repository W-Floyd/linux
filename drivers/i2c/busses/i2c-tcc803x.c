// SPDX-License-Identifier: GPL-2.0
/*
 * Telechips TCC803x I2C master (an OpenCores-style core with 32-bit registers).
 *
 * Polled only. Clocks, the prescaler and the pad mux are left as the previous
 * stage (the vendor kernel, across a kexec) configured them; DT only has to name
 * the register window. Register and bit meanings follow the vendor driver.
 */
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define TCC_I2C_PRES	0x00
#define TCC_I2C_CTRL	0x04
#define TCC_I2C_TXR	0x08
#define TCC_I2C_CMD	0x0c
#define TCC_I2C_RXR	0x10
#define TCC_I2C_SR	0x14

#define TCC_I2C_CTRL_EN	BIT(7)

#define TCC_I2C_CMD_IACK	BIT(0)
#define TCC_I2C_CMD_ACK		BIT(3)	/* sent as the ACK bit: 1 = NACK, for the last read byte */
#define TCC_I2C_CMD_WR		BIT(4)
#define TCC_I2C_CMD_RD		BIT(5)
#define TCC_I2C_CMD_STO		BIT(6)
#define TCC_I2C_CMD_STA		BIT(7)
#define TCC_I2C_CMD_BUSY_MASK	GENMASK(7, 4)

#define TCC_I2C_SR_TIP		BIT(1)
#define TCC_I2C_SR_BUSY		BIT(6)
#define TCC_I2C_SR_NACK		BIT(7)

#define TCC_I2C_TIMEOUT_US	500000

struct tcc_i2c {
	void __iomem *base;
	struct i2c_adapter adap;
};

/* Wait for the command to finish and acknowledge the interrupt flag. */
static int tcc_i2c_wait(struct tcc_i2c *i2c)
{
	u32 v;
	int ret;

	ret = readl_poll_timeout(i2c->base + TCC_I2C_CMD, v,
				 !(v & TCC_I2C_CMD_BUSY_MASK), 5, TCC_I2C_TIMEOUT_US);
	if (ret)
		return ret;
	ret = readl_poll_timeout(i2c->base + TCC_I2C_SR, v,
				 !(v & TCC_I2C_SR_TIP), 5, TCC_I2C_TIMEOUT_US);
	if (ret)
		return ret;
	writel(readl(i2c->base + TCC_I2C_CMD) | TCC_I2C_CMD_IACK, i2c->base + TCC_I2C_CMD);
	return 0;
}

static int tcc_i2c_cmd(struct tcc_i2c *i2c, u32 cmd)
{
	writel(cmd, i2c->base + TCC_I2C_CMD);
	return tcc_i2c_wait(i2c);
}

static int tcc_i2c_do_msg(struct tcc_i2c *i2c, struct i2c_msg *m)
{
	u8 addr = (m->addr & 0x7f) << 1;
	int i, ret;

	if (m->flags & I2C_M_RD)
		addr |= 1;
	if (m->flags & I2C_M_REV_DIR_ADDR)
		addr ^= 1;

	writel(addr, i2c->base + TCC_I2C_TXR);
	ret = tcc_i2c_cmd(i2c, TCC_I2C_CMD_STA | TCC_I2C_CMD_WR);
	if (ret)
		return ret;
	if (readl(i2c->base + TCC_I2C_SR) & TCC_I2C_SR_NACK)
		return -ENXIO;

	for (i = 0; i < m->len; i++) {
		if (m->flags & I2C_M_RD) {
			ret = tcc_i2c_cmd(i2c, TCC_I2C_CMD_RD |
					  (i == m->len - 1 ? TCC_I2C_CMD_ACK : 0));
			if (ret)
				return ret;
			m->buf[i] = readl(i2c->base + TCC_I2C_RXR);
		} else {
			writel(m->buf[i], i2c->base + TCC_I2C_TXR);
			ret = tcc_i2c_cmd(i2c, TCC_I2C_CMD_WR);
			if (ret)
				return ret;
			if (readl(i2c->base + TCC_I2C_SR) & TCC_I2C_SR_NACK)
				return -EIO;
		}
	}
	return 0;
}

static int tcc_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct tcc_i2c *i2c = i2c_get_adapdata(adap);
	int i, ret = 0;
	u32 v;

	for (i = 0; i < num; i++) {
		ret = tcc_i2c_do_msg(i2c, &msgs[i]);
		if (ret)
			break;
	}

	/* Always end with a STOP, even after an error, and wait for an idle bus. */
	tcc_i2c_cmd(i2c, TCC_I2C_CMD_STO);
	readl_poll_timeout(i2c->base + TCC_I2C_SR, v, !(v & TCC_I2C_SR_BUSY), 5,
			   TCC_I2C_TIMEOUT_US);
	return ret ? ret : num;
}

static u32 tcc_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm tcc_i2c_algo = {
	.master_xfer	= tcc_i2c_xfer,
	.functionality	= tcc_i2c_func,
};

static int tcc_i2c_probe(struct platform_device *pdev)
{
	struct tcc_i2c *i2c;

	i2c = devm_kzalloc(&pdev->dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	i2c->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(i2c->base))
		return PTR_ERR(i2c->base);

	/* Make sure the core is enabled; keep whatever prescale was set up. */
	writel(readl(i2c->base + TCC_I2C_CTRL) | TCC_I2C_CTRL_EN, i2c->base + TCC_I2C_CTRL);
	dev_info(&pdev->dev, "prescale %#x ctrl %#x sr %#x\n",
		 readl(i2c->base + TCC_I2C_PRES), readl(i2c->base + TCC_I2C_CTRL),
		 readl(i2c->base + TCC_I2C_SR));

	i2c->adap.owner = THIS_MODULE;
	i2c->adap.algo = &tcc_i2c_algo;
	i2c->adap.dev.parent = &pdev->dev;
	i2c->adap.dev.of_node = pdev->dev.of_node;
	i2c->adap.retries = 2;
	strscpy(i2c->adap.name, "tcc803x-i2c", sizeof(i2c->adap.name));
	i2c_set_adapdata(&i2c->adap, i2c);
	platform_set_drvdata(pdev, i2c);

	return i2c_add_adapter(&i2c->adap);
}

static void tcc_i2c_remove(struct platform_device *pdev)
{
	struct tcc_i2c *i2c = platform_get_drvdata(pdev);

	i2c_del_adapter(&i2c->adap);
}

static const struct of_device_id tcc_i2c_of_match[] = {
	{ .compatible = "telechips,tcc803x-i2c" },
	{ }
};
MODULE_DEVICE_TABLE(of, tcc_i2c_of_match);

static struct platform_driver tcc_i2c_driver = {
	.probe	= tcc_i2c_probe,
	.remove	= tcc_i2c_remove,
	.driver	= {
		.name		= "tcc803x-i2c",
		.of_match_table	= tcc_i2c_of_match,
	},
};
module_platform_driver(tcc_i2c_driver);

MODULE_DESCRIPTION("Telechips TCC803x I2C master driver");
MODULE_LICENSE("GPL");
