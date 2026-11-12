// SPDX-License-Identifier: GPL-2.0
/*
 * Telechips TCC803x OTP (one-time-programmable fuse) nvmem provider.
 *
 * The OTP controller (0x19101000, cmd shadow 0xe0004000) is secure I/O: only
 * the EL3 monitor touches it, and the normal world reaches it through a SiP
 * SMC, SIP_OPTEE_READ_OTP (0x82008000, device 8). ABI verified by
 * disassembling this unit's monitor: x1 = byte offset (4-byte aligned), the
 * 32-bit word comes back in res.a1, status in res.a0 (0 = ok). The call is
 * undocumented -- in no vendor header (which name only the device-5
 * write/verify/jtag calls); the read routine it reaches drives the same OTP
 * controller OP-TEE's plat-telechips driver does. Not yet exercised on
 * hardware: validate by reading the chip revision (monitor also serves it from
 * OTP offset 0x800 via id 0x82007000) and cross-checking, before trusting
 * arbitrary offsets. read_fid overrides the id.
 *
 * Read-only: the write path is a signed/CMAC-authenticated fuse package
 * (device-5 OTP_WRITE), needs the vendor key, and burns irreversibly -- not
 * implemented.
 */

#include <linux/arm-smccc.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/nvmem-provider.h>
#include <linux/platform_device.h>
#include <linux/types.h>

/* OTPROM_MAX in OP-TEE plat-telechips otprom.h. */
#define TCC803X_OTP_SIZE	0x4000
#define TCC803X_OTP_WORD	4

/* Device-8 OTP read-word SiP call; see the file comment on the ABI. */
#define SIP_OPTEE_READ_OTP	0x82008000

static u32 read_fid = SIP_OPTEE_READ_OTP;
module_param(read_fid, uint, 0444);
MODULE_PARM_DESC(read_fid,
	"Telechips SiP SMC id for a 32-bit OTP read (default 0x82008000; 0 disables reads)");

struct tcc803x_otp {
	struct device *dev;
	u32 read_fid;
};

/* One 32-bit word at a 4-byte-aligned byte offset; word returned in res.a1. */
static int tcc803x_otp_smc_read(struct tcc803x_otp *otp, u32 word_off, u32 *word)
{
	struct arm_smccc_res res;

	arm_smccc_smc(otp->read_fid, word_off, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0)
		return -EIO;

	*word = res.a1;
	return 0;
}

static int tcc803x_otp_reg_read(void *priv, unsigned int offset,
				void *val, size_t bytes)
{
	struct tcc803x_otp *otp = priv;
	u8 *dst = val;

	if (!otp->read_fid) {
		dev_warn_once(otp->dev, "OTP read id unset (read_fid=0)\n");
		return -EOPNOTSUPP;
	}

	while (bytes) {
		unsigned int head = offset % TCC803X_OTP_WORD;
		unsigned int n = min_t(size_t, bytes, TCC803X_OTP_WORD - head);
		u32 word;
		int ret;

		ret = tcc803x_otp_smc_read(otp, offset - head, &word);
		if (ret)
			return ret;

		memcpy(dst, (u8 *)&word + head, n);
		dst += n;
		offset += n;
		bytes -= n;
	}

	return 0;
}

static int tcc803x_otp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvmem_config config = {
		.name = "tcc803x-otp",
		.type = NVMEM_TYPE_OTP,
		.read_only = true,
		.add_legacy_fixed_of_cells = true,
		.stride = TCC803X_OTP_WORD,
		.word_size = TCC803X_OTP_WORD,
		.size = TCC803X_OTP_SIZE,
		.reg_read = tcc803x_otp_reg_read,
		.dev = dev,
		.of_node = dev->of_node,
	};
	struct tcc803x_otp *otp;

	otp = devm_kzalloc(dev, sizeof(*otp), GFP_KERNEL);
	if (!otp)
		return -ENOMEM;

	otp->dev = dev;
	otp->read_fid = read_fid;
	config.priv = otp;

	return PTR_ERR_OR_ZERO(devm_nvmem_register(dev, &config));
}

static const struct of_device_id tcc803x_otp_of_match[] = {
	{ .compatible = "telechips,tcc803x-otp" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tcc803x_otp_of_match);

static struct platform_driver tcc803x_otp_driver = {
	.probe = tcc803x_otp_probe,
	.driver = {
		.name = "tcc803x-otp",
		.of_match_table = tcc803x_otp_of_match,
	},
};
module_platform_driver(tcc803x_otp_driver);

MODULE_DESCRIPTION("Telechips TCC803x OTP nvmem provider");
MODULE_LICENSE("GPL");
