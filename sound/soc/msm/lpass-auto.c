/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
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


#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm_params.h>
#include <sound/pcm.h>
#include <sound/dai.h>

/* MI2S Configuration */
#define GPIO_MI2S_WS	27
#define GPIO_MI2S_SCK	28
#define GPIO_MI2S_SD1	31
#define GPIO_MI2S_SD0	32

/* Secondary I2S Configuration */
#define GPIO_SEC_I2S_RX_SCK  47
#define GPIO_SEC_I2S_RX_WS   48
#define GPIO_SEC_I2S_RX_DOUT 49
#define GPIO_SEC_I2S_RX_MCLK 50
#define I2S_MCLK_RATE 1536000

/* MIC I2S Configuration */
#define GPIO_MIC_I2S_SCK	35
#define GPIO_MIC_I2S_WS		36
#define GPIO_MIC_I2S_DIN0	37

struct request_gpio {
	unsigned gpio_no;
	char *gpio_name;
};

static struct request_gpio mi2s_gpio[] = {
	{
		.gpio_no = GPIO_MI2S_WS,
		.gpio_name = "MI2S_WS",
	},
	{
		.gpio_no = GPIO_MI2S_SCK,
		.gpio_name = "MI2S_SCK",
	},
	{
		.gpio_no = GPIO_MI2S_SD1,
		.gpio_name = "MI2S_SD1",
	},
	{
		.gpio_no = GPIO_MI2S_SD0,
		.gpio_name = "MI2S_SD0",
	}
};

static struct request_gpio sec_i2s_rx_gpio[] = {
	{
		.gpio_no = GPIO_SEC_I2S_RX_MCLK,
		.gpio_name = "SEC_I2S_RX_MCLK",
	},
	{
		.gpio_no = GPIO_SEC_I2S_RX_SCK,
		.gpio_name = "SEC_I2S_RX_SCK",
	},
	{
		.gpio_no = GPIO_SEC_I2S_RX_WS,
		.gpio_name = "SEC_I2S_RX_WS",
	},
	{
		.gpio_no = GPIO_SEC_I2S_RX_DOUT,
		.gpio_name = "SEC_I2S_RX_DOUT",
	},
};


static struct request_gpio mic_i2s_gpio[] = {
	{
		.gpio_no = GPIO_MIC_I2S_WS,
		.gpio_name = "MIC_I2S_WS",
	},
	{
		.gpio_no = GPIO_MIC_I2S_SCK,
		.gpio_name = "MIC_I2S_SCk",
	},
	{
		.gpio_no = GPIO_MIC_I2S_DIN0,
		.gpio_name = "MIC_I2S_DIN0",
	},
};

static struct clk *mi2s_osr_clk;
static struct clk *mi2s_bit_clk;
static struct clk *sec_i2s_rx_osr_clk;
static struct clk *sec_i2s_rx_bit_clk;
static struct clk *i2s_tx_bit_clk;
static atomic_t mi2s_rsc_ref;

static int msm_mi2s_free_gpios(void)
{
	int	i;
	for (i = 0; i < ARRAY_SIZE(mi2s_gpio); i++)
		gpio_free(mi2s_gpio[i].gpio_no);
	return 0;
}

static int msm_i2s_tx_free_gpios(void)
{
	int	i;
	for (i = 0; i < ARRAY_SIZE(mic_i2s_gpio); i++)
		gpio_free(mic_i2s_gpio[i].gpio_no);
	return 0;
}

static int msm_configure_mi2s_gpio(void)
{
	int	rtn;
	int	i;
	int	j;
	for (i = 0; i < ARRAY_SIZE(mi2s_gpio); i++) {
		rtn = gpio_request(mi2s_gpio[i].gpio_no,
						   mi2s_gpio[i].gpio_name);
		pr_debug("%s: gpio = %d, gpio name = %s, rtn = %d\n",
				 __func__,
				 mi2s_gpio[i].gpio_no,
				 mi2s_gpio[i].gpio_name,
				 rtn);
		if (rtn) {
			pr_err("%s: Failed to request gpio %d\n",
				   __func__,
				   mi2s_gpio[i].gpio_no);
			for (j = i; j >= 0; j--)
				gpio_free(mi2s_gpio[j].gpio_no);
			goto err;
		}
	}
err:
	return rtn;
}

static int msm_configure_i2s_tx_gpio(void)
{
	int	rtn;
	int	i;
	int	j;
	for (i = 0; i < ARRAY_SIZE(mic_i2s_gpio); i++) {
		rtn = gpio_request(mic_i2s_gpio[i].gpio_no,
						   mic_i2s_gpio[i].gpio_name);
		pr_debug("%s: gpio = %d, gpio name = %s, rtn = %d\n",
				 __func__,
				 mic_i2s_gpio[i].gpio_no,
				 mic_i2s_gpio[i].gpio_name,
				 rtn);
		if (rtn) {
			pr_err("%s: Failed to request gpio %d\n",
				   __func__,
				   mic_i2s_gpio[i].gpio_no);
			for (j = i; j >= 0; j--)
				gpio_free(mic_i2s_gpio[j].gpio_no);
			goto err;
		}
	}
err:
	return rtn;
}

static int apq8064_mi2s_startup(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	pr_debug("%s: dai name %s %p\n", __func__, cpu_dai->name, cpu_dai->dev);

	if (atomic_inc_return(&mi2s_rsc_ref) == 1) {
		pr_debug("%s: acquire mi2s resources\n", __func__);
		msm_configure_mi2s_gpio();
		pr_debug("%s: APQ is MI2S master\n", __func__);

		mi2s_osr_clk = clk_get(cpu_dai->dev, "osr_clk");
		if (IS_ERR(mi2s_osr_clk)) {
			pr_debug("%s: fail to get mi2s_osr_clk\n", __func__);
			return PTR_ERR(mi2s_osr_clk);
		}
		clk_set_rate(mi2s_osr_clk, 48000 * 256);
		ret = clk_prepare_enable(mi2s_osr_clk);
		if (IS_ERR_VALUE(ret)) {
			pr_err("Unable to enable mi2s_osr_clk\n");
			clk_put(mi2s_osr_clk);
			return ret;
		}
		mi2s_bit_clk = clk_get(cpu_dai->dev, "bit_clk");
		if (IS_ERR(mi2s_bit_clk)) {
			pr_err("Unable to get mi2s_bit_clk\n");
			clk_disable_unprepare(mi2s_osr_clk);
			clk_put(mi2s_osr_clk);
			return PTR_ERR(mi2s_bit_clk);
		}
		clk_set_rate(mi2s_bit_clk, 8);
		ret = clk_prepare_enable(mi2s_bit_clk);
		if (IS_ERR_VALUE(ret)) {
			pr_err("Unable to enable mi2s_bit_clk\n");
			clk_disable_unprepare(mi2s_osr_clk);
			clk_put(mi2s_osr_clk);
			clk_put(mi2s_bit_clk);
			return ret;
		}
		ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_CBS_CFS);
		if (IS_ERR_VALUE(ret))
			pr_err("set format for CPU dai failed\n");
	}
	return ret;
}

static void apq8064_mi2s_shutdown(struct snd_pcm_substream *substream)
{
	if (atomic_dec_return(&mi2s_rsc_ref) == 0) {
		pr_debug("%s: free mi2s resources\n", __func__);
		if (mi2s_bit_clk) {
			clk_disable_unprepare(mi2s_bit_clk);
			clk_put(mi2s_bit_clk);
			mi2s_bit_clk = NULL;
		}

		if (mi2s_osr_clk) {
			clk_disable_unprepare(mi2s_osr_clk);
			clk_put(mi2s_osr_clk);
			mi2s_osr_clk = NULL;
		}
		msm_mi2s_free_gpios();
	}
}

static void apq8064_i2s_shutdown(struct snd_pcm_substream *substream)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		pr_debug("%s: nothing for playback", __func__);
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		pr_debug("%s: free i2s tx resources\n", __func__);
		if (i2s_tx_bit_clk) {
			clk_disable_unprepare(i2s_tx_bit_clk);
			clk_put(i2s_tx_bit_clk);
			i2s_tx_bit_clk = NULL;
		}
		msm_i2s_tx_free_gpios();
	}
}

static int apq8064_i2s_startup(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		pr_debug("%s: nothing for playback", __func__);
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		msm_configure_i2s_tx_gpio();
		i2s_tx_bit_clk = clk_get(cpu_dai->dev, "bit_clk");
		if (IS_ERR(i2s_tx_bit_clk)) {
			pr_err("Failed to get i2s_tx_bit_clk\n");
			return PTR_ERR(i2s_tx_bit_clk);
		}
		clk_set_rate(i2s_tx_bit_clk, 0);
		ret = clk_prepare_enable(i2s_tx_bit_clk);
		if (ret != 0) {
			pr_err("Unable to enable i2s_tx_bit_clk\n");
			clk_put(i2s_tx_bit_clk);
			return ret;
		}
		ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_CBM_CFM);
		if (IS_ERR_VALUE(ret))
			pr_err("set format for CPU dai failed\n");
	}

	pr_debug("%s: ret = %d\n", __func__, ret);
	pr_debug("%s(): substream = %s  stream = %d\n", __func__,
			substream->name, substream->stream);
	return ret;
}

static int apq8064_sec_i2s_rx_free_gpios(void)
{
	int	i;
	for (i = 0; i < ARRAY_SIZE(sec_i2s_rx_gpio); i++)
		gpio_free(sec_i2s_rx_gpio[i].gpio_no);
	return 0;
}

static int apq8064_sec_i2s_rx_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{

	int rate = params_rate(params);
	int bit_clk_set = 0;
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S16_LE:
			bit_clk_set = I2S_MCLK_RATE/(rate * 2 * 16);
			clk_set_rate(sec_i2s_rx_bit_clk, bit_clk_set);
			break;
		case SNDRV_PCM_FORMAT_S24_LE:
			bit_clk_set = I2S_MCLK_RATE/(rate * 2 * 24);
			clk_set_rate(sec_i2s_rx_bit_clk, bit_clk_set);
			break;
		default:
			pr_err("wrong format\n");
			break;
		}
	}
	return 0;
}

static void apq8064_sec_i2s_rx_shutdown(struct snd_pcm_substream *substream)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (sec_i2s_rx_bit_clk) {
			clk_disable_unprepare(sec_i2s_rx_bit_clk);
			clk_put(sec_i2s_rx_bit_clk);
			sec_i2s_rx_bit_clk = NULL;
		}
		if (sec_i2s_rx_osr_clk) {
			clk_disable_unprepare(sec_i2s_rx_osr_clk);
			clk_put(sec_i2s_rx_osr_clk);
			sec_i2s_rx_osr_clk = NULL;
		}
		apq8064_sec_i2s_rx_free_gpios();
	}
	pr_debug("%s(): substream = %s  stream = %d\n", __func__,
		 substream->name, substream->stream);
}

static int configure_sec_i2s_rx_gpio(void)
{
	int rtn;
	int i;
	int j;
	for (i = 0; i < ARRAY_SIZE(sec_i2s_rx_gpio); i++) {
		rtn = gpio_request(sec_i2s_rx_gpio[i].gpio_no,
				sec_i2s_rx_gpio[i].gpio_name);
		pr_debug("%s: gpio = %d, gpio name = %s, rtn = %d\n",
				__func__,
				sec_i2s_rx_gpio[i].gpio_no,
				sec_i2s_rx_gpio[i].gpio_name,
					rtn);
		if (rtn) {
			pr_err("%s: Failed to request gpio %d\n",
				__func__,
			sec_i2s_rx_gpio[i].gpio_no);
			for (j = i; j >= 0; j--)
				gpio_free(sec_i2s_rx_gpio[j].gpio_no);

			goto err;
		}
	}
err:
	return rtn;
}

static int apq8064_sec_i2s_rx_startup(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		configure_sec_i2s_rx_gpio();
		sec_i2s_rx_osr_clk = clk_get(cpu_dai->dev, "osr_clk");
		if (IS_ERR(sec_i2s_rx_osr_clk)) {
			pr_err("Failed to get sec_i2s_rx_osr_clk\n");
			return PTR_ERR(sec_i2s_rx_osr_clk);
		}
		clk_set_rate(sec_i2s_rx_osr_clk, I2S_MCLK_RATE);
		clk_prepare_enable(sec_i2s_rx_osr_clk);
		sec_i2s_rx_bit_clk = clk_get(cpu_dai->dev, "bit_clk");
		if (IS_ERR(sec_i2s_rx_bit_clk)) {
			pr_err("Failed to get sec i2s osr_clk\n");
			clk_disable_unprepare(sec_i2s_rx_osr_clk);
			clk_put(sec_i2s_rx_osr_clk);
			return PTR_ERR(sec_i2s_rx_bit_clk);
		}
		clk_set_rate(sec_i2s_rx_bit_clk, 1);
		ret = clk_prepare_enable(sec_i2s_rx_bit_clk);
		if (ret != 0) {
			pr_err("Unable to enable sec i2s rx_bit_clk\n");
			clk_put(sec_i2s_rx_bit_clk);
			clk_disable_unprepare(sec_i2s_rx_osr_clk);
			clk_put(sec_i2s_rx_osr_clk);
			return ret;
		}
		ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_CBS_CFS);
		if (ret < 0)
			pr_err("set format for codec dai failed\n");
	}
	pr_debug("%s: ret = %d\n", __func__, ret);
	pr_debug("%s(): substream = %s  stream = %d\n", __func__,
		 substream->name, substream->stream);
	return ret;
}

static struct snd_soc_ops sec_i2s_rx_ops = {
	.startup = apq8064_sec_i2s_rx_startup,
	.shutdown = apq8064_sec_i2s_rx_shutdown,
	.hw_params = apq8064_sec_i2s_rx_hw_params,
};

static struct snd_soc_ops mi2s_machine_ops = {
	.startup	= apq8064_mi2s_startup,
	.shutdown	= apq8064_mi2s_shutdown,
};

static struct snd_soc_ops i2s_machine_ops = {
	.startup	= apq8064_i2s_startup,
	.shutdown	= apq8064_i2s_shutdown,
};

static struct snd_soc_dai_link apq8064_dai[] = {
	{
		.name		= "Audio MI2S Rx",
		.stream_name	= "Audio MI2S Rx output",
		.cpu_dai_name   = "apq8064_cpudai_lpa.0",
		.platform_name	= "apq8064_pcm_lpa",
		.codec_dai_name	= "msm-stub-rx",
		.codec_name	= "msm-stub-codec.1",
		.ops		= &mi2s_machine_ops,
		.ignore_pmdown_time	= 1,
	},
	{
		.name		= "Audio Sec Rx",
		.stream_name	= "Audio Sec Rx output",
		.cpu_dai_name	= "apq8064_cpudai_lpa.3",
		.platform_name	= "apq8064_pcm_lpa",
		.codec_dai_name	= "msm-stub-rx",
		.codec_name	= "msm-stub-codec.1",
		.ops		= &sec_i2s_rx_ops,
		.ignore_pmdown_time	= 1,
	},
	{
		.name		= "Audio Tx",
		.stream_name	= "Audio Tx input",
		.cpu_dai_name	= "apq8064_cpudai_lpa.5",
		.platform_name	= "apq8064_pcm_lpa",
		.codec_dai_name	= "msm-stub-tx",
		.codec_name	= "msm-stub-codec.1",
		.ops		= &i2s_machine_ops,
	},
};

static struct snd_soc_card snd_soc_card_apq8064 = {
	.name		= "apq8064-tabla-snd-card",
	.dai_link	= apq8064_dai,
	.num_links	= ARRAY_SIZE(apq8064_dai),
};

static struct platform_device *apq8064_snd_device;

static int __init msm_audio_init(void)
{
	int ret;

	apq8064_snd_device = platform_device_alloc("soc-audio", 0);
	if (!apq8064_snd_device) {
		pr_err("Platform device allocation fialed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(apq8064_snd_device, &snd_soc_card_apq8064);
	ret = platform_device_add(apq8064_snd_device);
	if (ret) {
		platform_device_put(apq8064_snd_device);
		return ret;
	}
	atomic_set(&mi2s_rsc_ref, 0);
	return ret;
}
module_init(msm_audio_init);

static void __exit msm_audio_exit(void)
{
	platform_device_unregister(apq8064_snd_device);
}
module_exit(msm_audio_exit);

MODULE_DESCRIPTION("ALSA Soc APQ8064");
MODULE_LICENSE("GPL v2");
