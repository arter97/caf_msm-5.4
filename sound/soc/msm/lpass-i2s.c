/* Copyright (c) 2010-2011, 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/dai.h>

#define MSM_MI2S_RX		0
#define SECONDARY_I2S_RX	3
#define PRIMARY_I2S_TX		5

static int msm_cpu_dai_startup(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	uint32_t dma_ch = dai->id;
	int ret = 0;

	pr_debug("%s\n", __func__);
	ret = dai_open(dma_ch);
	return ret;

}

static void msm_cpu_dai_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	uint32_t dma_ch = dai->id;

	pr_debug("%s\n", __func__);
	dai_close(dma_ch);
}

static int msm_cpu_dai_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *dai)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int msm_cpu_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	uint32_t dma_ch = dai->id;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		dai_set_master_mode(dma_ch, 1); /* CPU is master */
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		dai_set_master_mode(dma_ch, 0); /* CPU is slave */
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static struct snd_soc_dai_ops msm_cpu_dai_ops = {
	.startup	= msm_cpu_dai_startup,
	.shutdown	= msm_cpu_dai_shutdown,
	.trigger	= msm_cpu_dai_trigger,
	.set_fmt	= msm_cpu_dai_fmt,

};

#define MSM_DAI_SPEAKER_BUILDER(link_id)			\
{								\
	.name = "msm-speaker-dai-"#link_id,			\
	.id = (link_id),					\
	.playback = {						\
		.stream_name = "auto-playback-"#link_id,	\
		.rates = SNDRV_PCM_RATE_8000_96000,		\
		.formats = SNDRV_PCM_FMTBIT_S16_LE,		\
		.channels_min = 1,				\
		.channels_max = 2,				\
		.rate_max =	96000,				\
		.rate_min =	8000,				\
	},							\
	.ops = &msm_cpu_dai_ops,				\
}

#define MSM_DAI_SEC_SPEAKER_BUILDER(link_id)			\
{								\
	.name = "msm-sec-speaker-dai-"#link_id,			\
	.id = (link_id),					\
	.playback = {						\
		.stream_name = "auto-sec-playback-"#link_id,	\
		.rates = SNDRV_PCM_RATE_8000_96000,		\
		.formats = SNDRV_PCM_FMTBIT_S16_LE,		\
		.channels_min = 1,				\
		.channels_max = 2,				\
		.rate_max =	96000,				\
		.rate_min =	8000,				\
	},							\
	.ops = &msm_cpu_dai_ops,				\
}

#define MSM_DAI_MIC_BUILDER(link_id)				\
{								\
	.name = "msm-mic-dai-"#link_id,				\
	.id = (link_id),					\
	.capture = {						\
		.stream_name = "auto-capture-"#link_id,		\
		.rates = SNDRV_PCM_RATE_8000_96000,		\
		.formats = SNDRV_PCM_FMTBIT_S16_LE,		\
		.rate_min =	8000,				\
		.rate_max =	96000,				\
		.channels_min = 1,				\
		.channels_max = 2,				\
	},							\
	.ops = &msm_cpu_dai_ops,				\
}

static struct snd_soc_dai_driver msm_cpu_mi2s_dai =
	MSM_DAI_SPEAKER_BUILDER(0);
static struct snd_soc_dai_driver msm_cpu_sec_i2s_dai =
	MSM_DAI_SEC_SPEAKER_BUILDER(3);
static struct snd_soc_dai_driver msm_cpu_pri_mic_dai =
	MSM_DAI_MIC_BUILDER(5);

static __devinit int msm_cpu_dai_dev_probe(struct platform_device *pdev)
{
	int rc = 0;
	dev_dbg(&pdev->dev, "%s: dev name %s\n", __func__,
			dev_name(&pdev->dev));
	switch (pdev->id) {
	case MSM_MI2S_RX:
		snd_soc_register_dai(&pdev->dev, &msm_cpu_mi2s_dai);
		break;
	case SECONDARY_I2S_RX:
		snd_soc_register_dai(&pdev->dev, &msm_cpu_sec_i2s_dai);
		break;
	case PRIMARY_I2S_TX:
		snd_soc_register_dai(&pdev->dev, &msm_cpu_pri_mic_dai);
		break;
	default:
		rc = -ENODEV;
		break;
	}
	return rc;
}

static __devexit int msm_cpu_dai_dev_remove(struct platform_device *pdev)
{
	snd_soc_unregister_dai(&pdev->dev);
	return 0;
}

static struct platform_driver msm_cpu_dai_driver = {
		.probe	= msm_cpu_dai_dev_probe,
		.remove	= msm_cpu_dai_dev_remove,
		.driver	= {
			.name = "apq8064_cpudai_lpa",
			.owner = THIS_MODULE,
		}
};

static int __init msm_cpu_dai_init(void)
{
	return platform_driver_register(&msm_cpu_dai_driver);
}
module_init(msm_cpu_dai_init);

static void __exit msm_cpu_dai_exit(void)
{
	return;
}
module_exit(msm_cpu_dai_exit);

/* Module information */
MODULE_DESCRIPTION("MSM CPU DAI driver");
MODULE_LICENSE("GPL v2");
