// SPDX-License-Identifier: GPL-2.0-only
/*
 * Telechips TCC803x I2S transmit: a DAI block fed by its ADMA, as one ASoC
 * component. Playback only: 2 channels, S16_LE, I2S, SoC as bit/frame clock
 * provider (mclk = 1024 fs, bit clock 64 fs).
 *
 * Every register and field below is from the vendor kernel
 * (AVNT5W_TCC803XKK_kernel, GPLv2), sound/soc/tcc_tcc803x/:
 *   tcc_audio_hw.h  offsets and fields
 *   tcc_adma.h      ADMA programming (tcc_adma_set_dai_tx_dma_buffer(), :757)
 *   tcc_dai.h       DAI programming (tcc_dai_set_master_mode(), :482)
 *   tcc_i2s.c       the sequence (i2s_initialize(), hw_params(), trigger())
 *   tcc_adma_pcm.c  the ADMA PCM ops
 * Left out because the vendor's own stereo/master/S16 path does not use
 * them: TDM, DSP/PCM modes, justified formats, 24-bit, mono, capture, and
 * the engineering-sample (system_rev == 0) workarounds.
 *
 * The probe touches no register. The first one is written at the first
 * startup, and the clocks (sipclk) may be read-only.
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>

#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

/* ADMA block, tcc_audio_hw.h:38-96 */
#define ADMA_TRANSCTRL		0x038
#define  TRANSCTRL_TX_WSIZE_32		GENMASK(1, 0)
#define  TRANSCTRL_TX_BURST_8		GENMASK(9, 8)
#define  TRANSCTRL_TX_REPEAT		BIT(16)
#define  TRANSCTRL_TX_CONT_CUR_ADDR	BIT(28)
#define  TRANSCTRL_HOP_CLEAR		BIT(30)
#define ADMA_RPTCTRL		0x03c
#define  RPTCTRL_DAI_THRESHOLD		GENMASK(27, 24)
#define ADMA_TXDASAR		0x040
#define ADMA_TXDAPARAM		0x044
#define ADMA_TXDATCNT		0x048
#define  TXDATCNT_CUR			GENMASK(31, 16)
#define ADMA_TXDACSAR		0x04c
#define ADMA_TXDASARL		0x068
#define ADMA_CHCTRL		0x078
#define  CHCTRL_TX_IRQ			BIT(0)
#define  CHCTRL_TXD_WIDTH_16		BIT(12)
#define  CHCTRL_TX_LR			BIT(16)
#define  CHCTRL_TX_MULTI_SEL		GENMASK(21, 20)
#define  CHCTRL_TX_MULTI_EN		BIT(24)
#define  CHCTRL_TX_DMA			BIT(28)
#define ADMA_INTSTATUS		0x07c
#define  INTSTATUS_TX			(BIT(0) | BIT(4))
#define ADMA_TXDAADRCNT		0x088
#define  ADRCNT_MODE			BIT(31)
#define  ADRCNT_COUNT			GENMASK(30, 0)
#define ADMA_RESET		0x180
#define  RESET_DAI_TX			BIT(2)

/* DAI block, tcc_audio_hw.h:111-137, 640-910 */
#define DAI_DAMR		0x040
#define  DAMR_POLARITY_NEG		BIT(3)
#define  DAMR_FRAME_MASTER_SYS		BIT(9)
#define  DAMR_BIT_MASTER_SYS		BIT(10)
#define  DAMR_SYS_MASTER_SYS		BIT(11)
#define  DAMR_SYNC_JUSTIFIED		BIT(12)
#define  DAMR_RX_EN			BIT(13)
#define  DAMR_TX_EN			BIT(14)
#define  DAMR_EN			BIT(15)
#define  DAMR_TX_SHIFT			GENMASK(19, 18)
#define  DAMR_TX_SHIFT_LSB_16		3
#define  DAMR_TX_JUSTIFIED_RIGHT	BIT(23)
#define  DAMR_RX_JUSTIFIED_RIGHT	BIT(24)
#define  DAMR_DSP_TDM			BIT(25)
#define  DAMR_FILTER			BIT(27)
#define  DAMR_MULTIPORT			BIT(28)
#define  DAMR_THRESHOLD			BIT(29)
#define  DAMR_LRCK_DIRECT		BIT(30)
#define  DAMR_BCLK_DIRECT		BIT(31)
#define DAI_DAVC		0x044
#define  DAVC_TX_VOLUME			GENMASK(4, 0)
#define DAI_MCCR0		0x048
#define  MCCR0_TDM_BIT_CLK_DIV		GENMASK(13, 12)
#define  MCCR0_DAO_MASK			GENMASK(31, 27)
#define DAI_MCCR1		0x04c
#define  MCCR1_DAO_PATH_ASRC		GENMASK(12, 9)
#define DAI_DRMR		0x050
#define  DRMR_TX_FIFO_CLEAR		BIT(20)
#define DAI_DCLKDIV		0x054
#define  DCLKDIV_FRAME_64		2
#define  DCLKDIV_BIT_16			(3 << 8)

#define MCLK_FS			1024
#define MCLK_DEFAULT_RATE	(44100 * MCLK_FS)
#define MCLK_MAX_RATE		100000000
#define FILTER_CLK_RATE		300000000
#define BUFFER_BYTES_MAX	(64 * 1024 * 4)
#define PERIOD_BYTES_MAX	(64 * 1024)

/*
 * DAI chmux, tcc_audio_chmux.h:96-124: DAIs 0-2 in 0x5c (4-bit fields 8 bits
 * apart), DAIs 3-6 in 0x60 (6-bit fields).
 */
#define CHMUX_DAI_71CH		0x5c
#define CHMUX_DAI_SRCH		0x60

struct tcc803x_i2s {
	struct device *dev;
	void __iomem *adma;
	void __iomem *dai;
	struct clk *pclk, *hclk, *filter, *adma_clk;
	struct regmap *chmux;
	unsigned int chmux_reg, chmux_mask, chmux_val;
	spinlock_t lock;
	struct snd_pcm_substream *substream;
	bool pclk_on;
};

static void reg_update(void __iomem *base, unsigned int off, u32 mask, u32 val)
{
	u32 v = readl(base + off);

	writel((v & ~mask) | (val & mask), base + off);
}

static irqreturn_t tcc803x_i2s_irq(int irq, void *data)
{
	struct tcc803x_i2s *i2s = data;
	struct snd_pcm_substream *ss;
	unsigned long flags;

	if (!(readl(i2s->adma + ADMA_INTSTATUS) & INTSTATUS_TX))
		return IRQ_NONE;

	writel(INTSTATUS_TX, i2s->adma + ADMA_INTSTATUS);

	spin_lock_irqsave(&i2s->lock, flags);
	ss = i2s->substream;
	spin_unlock_irqrestore(&i2s->lock, flags);
	if (ss)
		snd_pcm_period_elapsed(ss);

	return IRQ_HANDLED;
}

/* DAI side */

static int tcc803x_i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S ||
	    (fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF ||
	    (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_CBP_CFP)
		return -EINVAL;

	return 0;
}

/* The vendor enables the clocks before touching a register (i2s_initialize()). */
static int tcc803x_i2s_startup(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct tcc803x_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	int ret;

	ret = clk_prepare_enable(i2s->hclk);
	if (ret)
		return ret;
	ret = clk_prepare_enable(i2s->adma_clk);
	if (ret)
		goto err_hclk;
	ret = clk_set_rate(i2s->pclk, MCLK_DEFAULT_RATE);
	if (ret)
		goto err_adma;
	ret = clk_prepare_enable(i2s->pclk);
	if (ret)
		goto err_adma;
	i2s->pclk_on = true;
	ret = clk_set_rate(i2s->filter, FILTER_CLK_RATE);
	if (ret)
		goto err_pclk;
	ret = clk_prepare_enable(i2s->filter);
	if (ret)
		goto err_pclk;

	/* Volume 0 dB, DAI off, every data-out lane masked. */
	reg_update(i2s->dai, DAI_DAVC, DAVC_TX_VOLUME, 0);
	reg_update(i2s->dai, DAI_DAMR, DAMR_EN | DAMR_TX_EN | DAMR_RX_EN, 0);
	reg_update(i2s->dai, DAI_MCCR0, MCCR0_DAO_MASK, MCCR0_DAO_MASK);

	/* tcc_adma_pcm_open(): take the DAI TX channel out of reset. */
	reg_update(i2s->adma, ADMA_RESET, RESET_DAI_TX, 0);

	return 0;

err_pclk:
	clk_disable_unprepare(i2s->pclk);
	i2s->pclk_on = false;
err_adma:
	clk_disable_unprepare(i2s->adma_clk);
err_hclk:
	clk_disable_unprepare(i2s->hclk);
	return ret;
}

static void tcc803x_i2s_shutdown(struct snd_pcm_substream *substream,
				 struct snd_soc_dai *dai)
{
	struct tcc803x_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	reg_update(i2s->dai, DAI_DAMR, DAMR_EN, 0);

	if (i2s->pclk_on) {
		clk_disable_unprepare(i2s->pclk);
		i2s->pclk_on = false;
	}
	clk_disable_unprepare(i2s->filter);
	clk_disable_unprepare(i2s->adma_clk);
	clk_disable_unprepare(i2s->hclk);
}

static int tcc803x_i2s_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct tcc803x_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	unsigned long mclk = (unsigned long)params_rate(params) * MCLK_FS;
	int ret;

	if (mclk > MCLK_MAX_RATE)
		return -EINVAL;

	/* A rate change goes through a disabled clock (tcc_i2s.c:803-809). */
	if (i2s->pclk_on) {
		clk_disable_unprepare(i2s->pclk);
		i2s->pclk_on = false;
	}
	ret = clk_set_rate(i2s->pclk, mclk);
	if (ret)
		return ret;
	ret = clk_prepare_enable(i2s->pclk);
	if (ret)
		return ret;
	i2s->pclk_on = true;

	ret = regmap_update_bits(i2s->chmux, i2s->chmux_reg, i2s->chmux_mask,
				 i2s->chmux_val);
	if (ret)
		return ret;

	/*
	 * Bit clock = mclk/16, frame clock = bit clock/64. The stereo block
	 * cannot read DCLKDIV back (tcc_dai.h:526), so write the whole word.
	 */
	writel(DCLKDIV_FRAME_64 | DCLKDIV_BIT_16, i2s->dai + DAI_DCLKDIV);
	reg_update(i2s->dai, DAI_MCCR0, MCCR0_TDM_BIT_CLK_DIV, 0);

	/* I2S, 16-bit LSB-aligned in the FIFO word, ADMA-fed, no multiport. */
	reg_update(i2s->dai, DAI_DAMR,
		   DAMR_POLARITY_NEG | DAMR_SYNC_JUSTIFIED |
		   DAMR_TX_JUSTIFIED_RIGHT | DAMR_RX_JUSTIFIED_RIGHT |
		   DAMR_DSP_TDM | DAMR_MULTIPORT | DAMR_TX_SHIFT |
		   DAMR_THRESHOLD | DAMR_FILTER,
		   FIELD_PREP(DAMR_TX_SHIFT, DAMR_TX_SHIFT_LSB_16) |
		   DAMR_THRESHOLD | DAMR_FILTER);
	reg_update(i2s->dai, DAI_MCCR1, MCCR1_DAO_PATH_ASRC, 0);

	/* Provider of mclk, bit clock and frame clock. */
	reg_update(i2s->dai, DAI_DAMR,
		   DAMR_BCLK_DIRECT | DAMR_LRCK_DIRECT | DAMR_SYS_MASTER_SYS |
		   DAMR_BIT_MASTER_SYS | DAMR_FRAME_MASTER_SYS,
		   DAMR_BCLK_DIRECT | DAMR_LRCK_DIRECT | DAMR_SYS_MASTER_SYS |
		   DAMR_BIT_MASTER_SYS | DAMR_FRAME_MASTER_SYS);

	reg_update(i2s->dai, DAI_DAMR, DAMR_EN, DAMR_EN);

	return 0;
}

static int tcc803x_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	struct tcc803x_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/* tcc_i2s_trigger(): clear the FIFO, enable, unmask, release. */
		reg_update(i2s->dai, DAI_DRMR, DRMR_TX_FIFO_CLEAR,
			   DRMR_TX_FIFO_CLEAR);
		reg_update(i2s->dai, DAI_DAMR, DAMR_TX_EN, DAMR_TX_EN);
		reg_update(i2s->dai, DAI_MCCR0, MCCR0_DAO_MASK, 0);
		mdelay(1);
		reg_update(i2s->dai, DAI_DRMR, DRMR_TX_FIFO_CLEAR, 0);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		reg_update(i2s->dai, DAI_DAMR, DAMR_TX_EN, 0);
		reg_update(i2s->dai, DAI_MCCR0, MCCR0_DAO_MASK, MCCR0_DAO_MASK);
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct snd_soc_dai_ops tcc803x_i2s_dai_ops = {
	.set_fmt = tcc803x_i2s_set_fmt,
	.startup = tcc803x_i2s_startup,
	.shutdown = tcc803x_i2s_shutdown,
	.hw_params = tcc803x_i2s_hw_params,
	.trigger = tcc803x_i2s_trigger,
};

static struct snd_soc_dai_driver tcc803x_i2s_dai = {
	.name = "tcc803x-i2s",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
			 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
			 SNDRV_PCM_RATE_96000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &tcc803x_i2s_dai_ops,
};

/* ADMA side */

static const struct snd_pcm_hardware tcc803x_i2s_hw = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
	.period_bytes_min = 256,
	.period_bytes_max = PERIOD_BYTES_MAX,
	.periods_min = 2,
	.periods_max = BUFFER_BYTES_MAX / 256,
	.buffer_bytes_max = BUFFER_BYTES_MAX,
	.fifo_size = 16,
};

static int tcc803x_i2s_pcm_open(struct snd_soc_component *component,
				struct snd_pcm_substream *substream)
{
	struct tcc803x_i2s *i2s = snd_soc_component_get_drvdata(component);
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long flags;
	int ret;

	runtime->hw = tcc803x_i2s_hw;

	/* Address-count mode wants whole 8-word groups (tcc_adma_pcm.c:301-306). */
	ret = snd_pcm_hw_constraint_step(runtime, 0,
					 SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 32);
	if (ret)
		return ret;
	ret = snd_pcm_hw_constraint_step(runtime, 0,
					 SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 32);
	if (ret)
		return ret;

	spin_lock_irqsave(&i2s->lock, flags);
	i2s->substream = substream;
	spin_unlock_irqrestore(&i2s->lock, flags);

	return 0;
}

static int tcc803x_i2s_pcm_close(struct snd_soc_component *component,
				 struct snd_pcm_substream *substream)
{
	struct tcc803x_i2s *i2s = snd_soc_component_get_drvdata(component);
	unsigned long flags;

	spin_lock_irqsave(&i2s->lock, flags);
	i2s->substream = NULL;
	spin_unlock_irqrestore(&i2s->lock, flags);

	return 0;
}

static int tcc803x_i2s_pcm_hw_params(struct snd_soc_component *component,
				     struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct tcc803x_i2s *i2s = snd_soc_component_get_drvdata(component);
	u32 buffer_bytes = params_buffer_bytes(params);
	u32 period_bytes = params_period_bytes(params);

	/* tcc_adma_set_dai_tx_dma_buffer(): 32-bit words, burst of 8, address-count mode. */
	reg_update(i2s->adma, ADMA_CHCTRL, CHCTRL_TX_DMA, 0);

	writel(ADRCNT_MODE | FIELD_PREP(ADRCNT_COUNT, buffer_bytes / 4 - 1),
	       i2s->adma + ADMA_TXDAADRCNT);
	writel(4, i2s->adma + ADMA_TXDAPARAM);
	writel((period_bytes >> 5) - 1, i2s->adma + ADMA_TXDATCNT);
	writel(lower_32_bits(substream->runtime->dma_addr),
	       i2s->adma + ADMA_TXDASAR);
	writel(0, i2s->adma + ADMA_TXDASARL);

	reg_update(i2s->adma, ADMA_CHCTRL, CHCTRL_TX_LR, 0);
	reg_update(i2s->adma, ADMA_TRANSCTRL,
		   TRANSCTRL_TX_WSIZE_32 | TRANSCTRL_TX_BURST_8 |
		   TRANSCTRL_TX_REPEAT | TRANSCTRL_TX_CONT_CUR_ADDR,
		   TRANSCTRL_TX_WSIZE_32 | TRANSCTRL_TX_BURST_8 |
		   TRANSCTRL_TX_REPEAT | TRANSCTRL_TX_CONT_CUR_ADDR);
	reg_update(i2s->adma, ADMA_CHCTRL, CHCTRL_TXD_WIDTH_16,
		   CHCTRL_TXD_WIDTH_16);

	/* Single-port: the vendor leaves 7.1 selected and multi-channel off. */
	reg_update(i2s->adma, ADMA_CHCTRL,
		   CHCTRL_TX_MULTI_EN | CHCTRL_TX_MULTI_SEL,
		   FIELD_PREP(CHCTRL_TX_MULTI_SEL, 3));
	reg_update(i2s->adma, ADMA_RPTCTRL, RPTCTRL_DAI_THRESHOLD,
		   FIELD_PREP(RPTCTRL_DAI_THRESHOLD, 1));

	return 0;
}

static int tcc803x_i2s_pcm_hw_free(struct snd_soc_component *component,
				   struct snd_pcm_substream *substream)
{
	struct tcc803x_i2s *i2s = snd_soc_component_get_drvdata(component);
	u32 v;

	reg_update(i2s->adma, ADMA_CHCTRL, CHCTRL_TX_IRQ | CHCTRL_TX_DMA, 0);
	readl_poll_timeout(i2s->adma + ADMA_CHCTRL, v, !(v & CHCTRL_TX_DMA),
			   10, 10000);

	return 0;
}

/* tcc_adma_dai_tx_hopcnt_clear(): clear the hop counter, then back to counting. */
static int tcc803x_i2s_pcm_prepare(struct snd_soc_component *component,
				   struct snd_pcm_substream *substream)
{
	struct tcc803x_i2s *i2s = snd_soc_component_get_drvdata(component);
	u32 addr = readl(i2s->adma + ADMA_TXDASAR);
	u32 v;
	int ret;

	reg_update(i2s->adma, ADMA_TRANSCTRL, TRANSCTRL_HOP_CLEAR,
		   TRANSCTRL_HOP_CLEAR);
	writel(addr, i2s->adma + ADMA_TXDASAR);
	ret = readl_poll_timeout(i2s->adma + ADMA_TXDATCNT, v,
				 !FIELD_GET(TXDATCNT_CUR, v), 10, 10000);
	reg_update(i2s->adma, ADMA_TRANSCTRL, TRANSCTRL_HOP_CLEAR, 0);

	return ret;
}

static int tcc803x_i2s_pcm_trigger(struct snd_soc_component *component,
				   struct snd_pcm_substream *substream, int cmd)
{
	struct tcc803x_i2s *i2s = snd_soc_component_get_drvdata(component);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		reg_update(i2s->adma, ADMA_CHCTRL, CHCTRL_TX_IRQ | CHCTRL_TX_DMA,
			   CHCTRL_TX_IRQ | CHCTRL_TX_DMA);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/* As the vendor: the DMA runs on until hw_free, the DAI is masked. */
		return 0;
	default:
		return -EINVAL;
	}
}

static snd_pcm_uframes_t
tcc803x_i2s_pcm_pointer(struct snd_soc_component *component,
			struct snd_pcm_substream *substream)
{
	struct tcc803x_i2s *i2s = snd_soc_component_get_drvdata(component);
	struct snd_pcm_runtime *runtime = substream->runtime;
	u32 cur = readl(i2s->adma + ADMA_TXDACSAR);

	return bytes_to_frames(runtime, cur - lower_32_bits(runtime->dma_addr));
}

static int tcc803x_i2s_pcm_new(struct snd_soc_component *component,
			       struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	int ret;

	/* The ADMA addresses are 32-bit. */
	ret = dma_coerce_mask_and_coherent(card->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_DEV, card->dev,
				       BUFFER_BYTES_MAX, BUFFER_BYTES_MAX);
	return 0;
}

static const struct snd_soc_component_driver tcc803x_i2s_component = {
	.name = "tcc803x-i2s",
	.open = tcc803x_i2s_pcm_open,
	.close = tcc803x_i2s_pcm_close,
	.hw_params = tcc803x_i2s_pcm_hw_params,
	.hw_free = tcc803x_i2s_pcm_hw_free,
	.prepare = tcc803x_i2s_pcm_prepare,
	.trigger = tcc803x_i2s_pcm_trigger,
	.pointer = tcc803x_i2s_pcm_pointer,
	.pcm_new = tcc803x_i2s_pcm_new,
	.legacy_dai_naming = 1,
};

static int tcc803x_i2s_parse_chmux(struct tcc803x_i2s *i2s)
{
	struct of_phandle_args args;
	u32 idx, port, width, pos;
	int ret;

	ret = of_parse_phandle_with_fixed_args(i2s->dev->of_node,
					       "telechips,chmux", 2, 0, &args);
	if (ret)
		return dev_err_probe(i2s->dev, ret, "telechips,chmux\n");

	idx = args.args[0];
	port = args.args[1];
	i2s->chmux = syscon_node_to_regmap(args.np);
	of_node_put(args.np);
	if (IS_ERR(i2s->chmux))
		return PTR_ERR(i2s->chmux);

	if (idx < 3) {		/* 7.1ch blocks: 4-bit fields, ports 0-3 */
		i2s->chmux_reg = CHMUX_DAI_71CH;
		pos = 8 * idx;
		width = 4;
	} else if (idx < 7) {	/* stereo blocks: 6-bit fields, ports 0-5 */
		i2s->chmux_reg = CHMUX_DAI_SRCH;
		pos = 8 * (idx - 3);
		width = 6;
	} else {
		return -EINVAL;
	}

	if (port >= width)
		return -EINVAL;

	i2s->chmux_mask = GENMASK(width - 1, 0) << pos;
	i2s->chmux_val = BIT(port) << pos;
	return 0;
}

static int tcc803x_i2s_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tcc803x_i2s *i2s;
	int irq, ret;

	i2s = devm_kzalloc(dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;

	i2s->dev = dev;
	spin_lock_init(&i2s->lock);
	platform_set_drvdata(pdev, i2s);

	i2s->adma = devm_platform_ioremap_resource_byname(pdev, "adma");
	if (IS_ERR(i2s->adma))
		return PTR_ERR(i2s->adma);
	i2s->dai = devm_platform_ioremap_resource_byname(pdev, "dai");
	if (IS_ERR(i2s->dai))
		return PTR_ERR(i2s->dai);

	i2s->pclk = devm_clk_get(dev, "pclk");
	if (IS_ERR(i2s->pclk))
		return PTR_ERR(i2s->pclk);
	i2s->hclk = devm_clk_get(dev, "hclk");
	if (IS_ERR(i2s->hclk))
		return PTR_ERR(i2s->hclk);
	i2s->filter = devm_clk_get(dev, "filter");
	if (IS_ERR(i2s->filter))
		return PTR_ERR(i2s->filter);
	i2s->adma_clk = devm_clk_get(dev, "adma");
	if (IS_ERR(i2s->adma_clk))
		return PTR_ERR(i2s->adma_clk);

	ret = tcc803x_i2s_parse_chmux(i2s);
	if (ret)
		return ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ret = devm_request_irq(dev, irq, tcc803x_i2s_irq, 0, dev_name(dev), i2s);
	if (ret)
		return ret;

	return devm_snd_soc_register_component(dev, &tcc803x_i2s_component,
					       &tcc803x_i2s_dai, 1);
}

/*
 * Reboot and kexec call .shutdown: a stream still open has the ADMA reading its
 * buffer, which the next kernel will own. Nothing is touched when none is open
 * (the clocks are off then, and the DMA idle).
 */
static void tcc803x_i2s_platform_shutdown(struct platform_device *pdev)
{
	struct tcc803x_i2s *i2s = platform_get_drvdata(pdev);
	unsigned long flags;
	bool open;
	u32 v;

	spin_lock_irqsave(&i2s->lock, flags);
	open = i2s->substream;
	spin_unlock_irqrestore(&i2s->lock, flags);
	if (!open)
		return;

	reg_update(i2s->adma, ADMA_CHCTRL, CHCTRL_TX_IRQ | CHCTRL_TX_DMA, 0);
	readl_poll_timeout(i2s->adma + ADMA_CHCTRL, v, !(v & CHCTRL_TX_DMA),
			   10, 10000);
}

static const struct of_device_id tcc803x_i2s_of_match[] = {
	{ .compatible = "telechips,tcc803x-i2s" },
	{ }
};
MODULE_DEVICE_TABLE(of, tcc803x_i2s_of_match);

static struct platform_driver tcc803x_i2s_driver = {
	.probe = tcc803x_i2s_probe,
	.shutdown = tcc803x_i2s_platform_shutdown,
	.driver = {
		.name = "tcc803x-i2s",
		.of_match_table = tcc803x_i2s_of_match,
	},
};
module_platform_driver(tcc803x_i2s_driver);

MODULE_DESCRIPTION("Telechips TCC803x I2S playback through the ADMA");
MODULE_LICENSE("GPL");
