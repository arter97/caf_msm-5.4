/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
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
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/slimbus/slimbus.h>
#include <linux/input.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/jack.h>
#include <asm/mach-types.h>
#include <mach/socinfo.h>
#include "msm-pcm-routing.h"
#include "../codecs/wcd9310.h"


/* 8064 machine driver */
#define PM8921_GPIO_BASE		NR_GPIO_IRQS
#define PM8921_MPP_BASE			(PM8921_GPIO_BASE + PM8921_NR_GPIOS)
#define PM8921_GPIO_PM_TO_SYS(pm_gpio)  (pm_gpio - 1 + PM8921_GPIO_BASE)
#define PM8921_MPP_PM_TO_SYS(pm_mpp)	(pm_mpp - 1 + PM8921_MPP_BASE)
#define OSR_CLK_RATE 12288000

/* SPKR I2S Configuration */
#define GPIO_SPKR_I2S_SCK   40
#define GPIO_SPKR_I2S_DOUT  41
#define GPIO_SPKR_I2S_WS    42

/* MIC I2S Configuration */
#define GPIO_MIC_I2S_SCK	35
#define GPIO_MIC_I2S_WS		36
#define GPIO_MIC_I2S_DIN0	37
#define GPIO_MIC_I2S_DIN1	38

/* MI2S Configuration */
#define GPIO_MI2S_WS    27
#define GPIO_MI2S_SCK   28
#define GPIO_MI2S_SD2   30
#define GPIO_MI2S_SD1   31
#define GPIO_MI2S_SD0   32

struct request_gpio {
	unsigned gpio_no;
	char *gpio_name;
};

/* SD0 - SD2 as RX. SD3 are unused */
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
		.gpio_no = GPIO_MI2S_SD2,
		.gpio_name = "MI2S_SD2",
	},
	{
		.gpio_no = GPIO_MI2S_SD1,
		.gpio_name = "MI2S_SD1",
	},
	{
		.gpio_no = GPIO_MI2S_SD0,
		.gpio_name = "MI2S_SD0",
	},
};

/* I2S RX is slave so MCLK is not needed */
static struct request_gpio spkr_i2s_gpio[] = {
#ifdef NO_CONFLICT_WITH_SLIM_BUS
	{
		.gpio_no = GPIO_SPKR_I2S_WS,
		.gpio_name = "SPKR_I2S_WS",
	},
	{
		.gpio_no = GPIO_SPKR_I2S_SCK,
		.gpio_name = "SPKR_I2S_SCK",
	},
	{
		.gpio_no = GPIO_SPKR_I2S_DOUT,
		.gpio_name = "SPKR_I2S_DOUT",
	},
#endif
};


/* I2S TX is slave so MCLK is not needed. DIN1 is not used */
static struct request_gpio mic_i2s_gpio[] = {
	{
		.gpio_no = GPIO_MIC_I2S_WS,
		.gpio_name = "MIC_I2S_WS",
	},
	{
		.gpio_no = GPIO_MIC_I2S_SCK,
		.gpio_name = "MIC_I2S_SCK",
	},
	{
		.gpio_no = GPIO_MIC_I2S_DIN1,
		.gpio_name = "MIC_I2S_DIN",
	},
	{
		.gpio_no = GPIO_MIC_I2S_DIN0,
		.gpio_name = "MIC_I2S_DIN",
	},
};


static struct clk *i2s_rx_bit_clk;
static struct clk *i2s_tx_bit_clk;
static struct clk *mi2s_osr_clk;
static struct clk *mi2s_bit_clk;

static u32 reset_n = PM8921_MPP_PM_TO_SYS(7);

static int msm_i2s_rx_ch = 1;
static int msm_i2s_tx_ch = 1;
static int msm_mi2s_rx_ch = 1;
/* Only needed if MI2S used as both RX and TX */
static atomic_t mi2s_rsc_ref;


static const char * const two_ch_text[] = {"One", "Two"};
static const char * const four_ch_text[] = {"One", "Two", "Three", "Four"};
static const char * const six_ch_text[] = {"One", "Two", "Three", "Four",
					   "Five", "Six"};
static const struct soc_enum msm_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, two_ch_text),
	SOC_ENUM_SINGLE_EXT(4, four_ch_text),
	SOC_ENUM_SINGLE_EXT(6, six_ch_text),
};

static int msm_mi2s_rx_ch_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: msm_mi2s_rx_ch  = %d\n", __func__,
			msm_mi2s_rx_ch);
	ucontrol->value.integer.value[0] = msm_mi2s_rx_ch - 1;
	return 0;
}

static int codec_reset(void)
{
	int ret;
	struct pm8xxx_mpp_config_data param = {
		.type		= PM8XXX_MPP_TYPE_D_OUTPUT,
		.level		= PM8038_MPP_DIG_LEVEL_VPH,
		.control	= PM8XXX_MPP_DOUT_CTRL_HIGH,
	};

	pr_debug("%s\n", __func__);

	ret = pm8xxx_mpp_config(reset_n, &param);
	if (ret)
		pr_err("%s: Failed to configure reset_n gpio %u\n",
			__func__, reset_n);
	else {
		pr_debug("%s: enable reset_n\n", __func__);
	}
	return ret;
}


static int msm_mi2s_rx_ch_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	msm_mi2s_rx_ch = ucontrol->value.integer.value[0] + 1;

	pr_debug("%s: msm_mi2s_rx_ch = %d\n", __func__,
			msm_mi2s_rx_ch);
	return 0;
}

static int msm_mi2s_rx_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
	SNDRV_PCM_HW_PARAM_RATE);

	struct snd_interval *channels = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("%s()\n", __func__);
	rate->min = rate->max = 48000;
	channels->min = channels->max = msm_mi2s_rx_ch;

	return 0;
}

static int msm_i2s_rx_ch_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: msm_i2s_rx_ch  = %d\n", __func__,
			msm_i2s_rx_ch);
	ucontrol->value.integer.value[0] = msm_i2s_rx_ch - 1;
	return 0;
}

static int msm_i2s_rx_ch_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	msm_i2s_rx_ch = ucontrol->value.integer.value[0] + 1;

	pr_debug("%s: msm_i2s_rx_ch = %d\n", __func__,
			msm_i2s_rx_ch);
	return 0;
}

static int msm_i2s_rx_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
	SNDRV_PCM_HW_PARAM_RATE);

	struct snd_interval *channels = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("%s()\n", __func__);
	rate->min = rate->max = 48000;
	channels->min = channels->max = msm_i2s_rx_ch;

	return 0;
}

static int msm_i2s_tx_ch_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: msm_i2s_tx_ch  = %d\n", __func__,
			msm_i2s_tx_ch);
	ucontrol->value.integer.value[0] = msm_i2s_tx_ch - 1;
	return 0;
}

static int msm_i2s_tx_ch_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	msm_i2s_tx_ch = ucontrol->value.integer.value[0] + 1;

	pr_debug("%s: msm_i2s_tx_ch = %d\n", __func__,
			msm_i2s_tx_ch);
	return 0;
}

static int msm_i2s_tx_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
	SNDRV_PCM_HW_PARAM_RATE);

	struct snd_interval *channels = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("%s()\n", __func__);
	rate->min = rate->max = 48000;

	channels->min = channels->max = msm_i2s_tx_ch;

	return 0;
}

static int msm_mi2s_audrx_init(struct snd_soc_pcm_runtime *rtd)
{
	pr_debug("%s(), dev_name(%s)\n", __func__, dev_name(rtd->cpu_dai->dev));
	codec_reset();
	return 0;
}

static int msm_mi2s_free_gpios(void)
{
	int	i;
	for (i = 0; i < ARRAY_SIZE(mi2s_gpio); i++)
		gpio_free(mi2s_gpio[i].gpio_no);
	return 0;
}

static void msm_mi2s_shutdown(struct snd_pcm_substream *substream)
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

static int msm_mi2s_startup(struct snd_pcm_substream *substream)
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
		if (IS_ERR(mi2s_osr_clk))
			return PTR_ERR(mi2s_osr_clk);
		clk_set_rate(mi2s_osr_clk, OSR_CLK_RATE);
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

static int msm_i2s_rx_free_gpios(void)
{
	int	i;
	for (i = 0; i < ARRAY_SIZE(spkr_i2s_gpio); i++)
		gpio_free(spkr_i2s_gpio[i].gpio_no);
	return 0;
}


static int msm_i2s_tx_free_gpios(void)
{
	int	i;
	for (i = 0; i < ARRAY_SIZE(mic_i2s_gpio); i++)
		gpio_free(mic_i2s_gpio[i].gpio_no);
	return 0;
}

static void msm_i2s_shutdown(struct snd_pcm_substream *substream)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		pr_debug("%s: free i2s rx resources\n", __func__);
		if (i2s_rx_bit_clk) {
			clk_disable_unprepare(i2s_rx_bit_clk);
			clk_put(i2s_rx_bit_clk);
			i2s_rx_bit_clk = NULL;
		}
		msm_i2s_rx_free_gpios();
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

static int msm_configure_i2s_rx_gpio(void)
{
	int	rtn = 0;
	int	i;
	int	j;
	for (i = 0; i < ARRAY_SIZE(spkr_i2s_gpio); i++) {
		rtn = gpio_request(spkr_i2s_gpio[i].gpio_no,
						   spkr_i2s_gpio[i].gpio_name);
		pr_debug("%s: gpio = %d, gpio name = %s, rtn = %d\n",
				 __func__,
				 spkr_i2s_gpio[i].gpio_no,
				 spkr_i2s_gpio[i].gpio_name,
				 rtn);
		if (rtn) {
			pr_err("%s: Failed to request gpio %d\n",
				   __func__,
				   spkr_i2s_gpio[i].gpio_no);
			for (j = i; j >= 0; j--)
				gpio_free(spkr_i2s_gpio[j].gpio_no);
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

static int msm_i2s_startup(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	pr_debug("%s: APQ is I2S slave\n", __func__);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		msm_configure_i2s_rx_gpio();
		i2s_rx_bit_clk = clk_get(cpu_dai->dev, "bit_clk");
		if (IS_ERR(i2s_rx_bit_clk)) {
			pr_err("Failed to get i2s bit_clk\n");
			return PTR_ERR(i2s_rx_bit_clk);
		}
		clk_set_rate(i2s_rx_bit_clk, 0);
		ret = clk_prepare_enable(i2s_rx_bit_clk);
		if (IS_ERR_VALUE(ret)) {
			pr_err("Unable to enable i2s_rx_bit_clk\n");
			clk_put(i2s_rx_bit_clk);
			return ret;
		}
		ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_CBM_CFM);
		if (IS_ERR_VALUE(ret))
			pr_err("set format for CPU dai failed\n");
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


static struct snd_soc_ops msm_mi2s_be_ops = {
	.startup = msm_mi2s_startup,
	.shutdown = msm_mi2s_shutdown,
};

static struct snd_soc_ops msm_i2s_be_ops = {
	.startup = msm_i2s_startup,
	.shutdown = msm_i2s_shutdown,
};

static const struct snd_kcontrol_new msm_controls[] = {
	SOC_ENUM_EXT("MI2S_RX Channels", msm_enum[2],
		msm_mi2s_rx_ch_get, msm_mi2s_rx_ch_put),
	SOC_ENUM_EXT("I2S_RX Channels", msm_enum[0],
		msm_i2s_rx_ch_get, msm_i2s_rx_ch_put),
	SOC_ENUM_EXT("I2S_TX Channels", msm_enum[1],
			msm_i2s_tx_ch_get, msm_i2s_tx_ch_put),
};

/* Digital audio interface glue - connects codec <---> CPU */
static struct snd_soc_dai_link msm_dai[] = {
	/* FrontEnd DAI Links */
	{
		.name = "Media1",
		.stream_name = "MultiMedia1",
		.cpu_dai_name	= "MultiMedia1",
		.platform_name  = "msm-pcm-dsp",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1, /* playback support */
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA1
	},
	{
		.name = "Media2",
		.stream_name = "MultiMedia2",
		.cpu_dai_name	= "MultiMedia2",
		.platform_name  = "msm-multi-ch-pcm-dsp",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
					SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1, /* playback support */
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA2,
	},
	{
		.name = "Circuit-Switch Voice",
		.stream_name = "CS-Voice",
		.cpu_dai_name   = "CS-VOICE",
		.platform_name  = "msm-pcm-voice",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1, /* playback support */
		.be_id = MSM_FRONTEND_DAI_CS_VOICE,
	},
	{
		.name = "LPA",
		.stream_name = "LPA",
		.cpu_dai_name	= "MultiMedia3",
		.platform_name  = "msm-pcm-lpa",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1, /* playback support */
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA3,
	},
	{
		.name = "Compr",
		.stream_name = "COMPR",
		.cpu_dai_name	= "MultiMedia4",
		.platform_name  = "msm-compr-dsp",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1, /* playback support */
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA4,
	},
	{
		.name = "LowLatency",
		.stream_name = "MultiMedia5",
		.cpu_dai_name   = "MultiMedia5",
		.platform_name  = "msm-lowlatency-pcm-dsp",
		.dynamic = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		/* this dainlink has playback support */
		.ignore_pmdown_time = 1,
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA5,
	},
	/* Backend DAI Links */
	{
		.name = LPASS_BE_MI2S_RX,
		.stream_name = "MI2S Playback",
		.cpu_dai_name = "msm-dai-q6-mi2s",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "msm-stub-codec.1",
		.codec_dai_name	= "msm-stub-rx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_MI2S_RX,
		.init = &msm_mi2s_audrx_init,
		.be_hw_params_fixup = msm_mi2s_rx_be_hw_params_fixup,
		.ops = &msm_mi2s_be_ops,
		.ignore_pmdown_time = 1, /* Playback support */
	},

	{
		.name = LPASS_BE_PRI_I2S_RX,
		.stream_name = "Primary I2S Playback",
		.cpu_dai_name = "msm-dai-q6.0",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-rx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_PRI_I2S_RX,
		.be_hw_params_fixup = msm_i2s_rx_be_hw_params_fixup,
		.ops = &msm_i2s_be_ops,
		.ignore_pmdown_time = 1, /* playback support */
	},
	{
		.name = LPASS_BE_PRI_I2S_TX,
		.stream_name = "Primary I2S Capture",
		.cpu_dai_name = "msm-dai-q6.1",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-tx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_PRI_I2S_TX,
		.be_hw_params_fixup = msm_i2s_tx_be_hw_params_fixup,
		.ops = &msm_i2s_be_ops,
	},
};

static struct snd_soc_card snd_soc_card_msm = {
	.name		= "apq8064-auto-snd-card",
	.dai_link	= msm_dai,
	.num_links	= ARRAY_SIZE(msm_dai),
	.controls = msm_controls,
	.num_controls = ARRAY_SIZE(msm_controls),
};

static struct platform_device *msm_snd_device;

static int __init msm_audio_init(void)
{
	int ret;
	u32	version = socinfo_get_platform_version();
	if ((!machine_is_apq8064_adp_2() ||
	    (SOCINFO_VERSION_MAJOR(version) != 2))) {
		pr_err("%s: Not APQ8064-auto machine type\n", __func__);
		ret = -ENODEV;
		goto err;
	}

	pr_info("%s: apq8064-auto platform\n", __func__);

	msm_snd_device = platform_device_alloc("soc-audio", 0);
	if (!msm_snd_device) {
		pr_err("Platform device allocation failed\n");
		ret = -ENOMEM;
		goto err;
	}

	platform_set_drvdata(msm_snd_device, &snd_soc_card_msm);
	ret = platform_device_add(msm_snd_device);
	if (ret) {
		platform_device_put(msm_snd_device);
		pr_err("%s: platform_device_add failed\n", __func__);
		goto err;
	}

	atomic_set(&mi2s_rsc_ref, 0);
err:
	return ret;

}
module_init(msm_audio_init);

static void __exit msm_audio_exit(void)
{
	u32	version = socinfo_get_platform_version();

	if ((!machine_is_apq8064_adp_2() ||
	    (SOCINFO_VERSION_MAJOR(version) != 2))) {
		pr_err("%s: Not APQ8064-auto machine type\n", __func__);
		return ;
	}

	pr_info("%s: apq8064-auto.c exit\n", __func__);
	platform_device_unregister(msm_snd_device);
}
module_exit(msm_audio_exit);

MODULE_DESCRIPTION("ALSA SoC msm");
MODULE_LICENSE("GPL v2");
