/* Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
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
#include <linux/module.h>
#include <linux/input.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <asm/mach-types.h>
#include <mach/socinfo.h>
#include "msm-pcm-routing.h"

#define SAMPLE_RATE_8KHZ 8000
#define SAMPLE_RATE_16KHZ 16000
#define SAMPLE_RATE_48KHZ 48000

/* 8064 machine driver */
#define PM8921_GPIO_BASE		NR_GPIO_IRQS
#define PM8921_MPP_BASE			(PM8921_GPIO_BASE + PM8921_NR_GPIOS)
#define PM8921_GPIO_PM_TO_SYS(pm_gpio)  (pm_gpio - 1 + PM8921_GPIO_BASE)
#define PM8921_MPP_PM_TO_SYS(pm_mpp)	(pm_mpp - 1 + PM8921_MPP_BASE)
#define OSR_CLK_RATE 12288000
#define OSR_CLK_DOUBLE_RATE 24576000


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

/* Secondary I2S Configuration */
#define GPIO_SEC_I2S_RX_SCK  47
#define GPIO_SEC_I2S_RX_WS   48
#define GPIO_SEC_I2S_RX_DOUT 49
#define GPIO_SEC_I2S_RX_MCLK 50
#define I2S_MCLK_RATE 1536000
/* AUX PCM */
#define GPIO_AUX_PCM_DOUT 43
#define GPIO_AUX_PCM_DIN 44
#define GPIO_AUX_PCM_SYNC 45
#define GPIO_AUX_PCM_CLK 46

#define REPLACEMENT_FRONTEND_DAI_INDEX	11

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

static struct request_gpio mi2s_gpio_mplatform[] = {
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


static struct request_gpio mic_i2s_gpio_mplatform[] = {
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

static struct clk *sec_i2s_rx_osr_clk;
static struct clk *sec_i2s_rx_bit_clk;
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
static int msm_btsco_rate = SAMPLE_RATE_8KHZ;
static int msm_auxpcm_rate = SAMPLE_RATE_8KHZ;
static atomic_t auxpcm_rsc_ref;
static int is_mplatform;
static int is_mi2s_master = 1;
static int msm_mi2s_rx_bit_format = SNDRV_PCM_FORMAT_S24_LE;
static int msm_i2s_rx_bit_format = SNDRV_PCM_FORMAT_S24_LE;
static int msm_i2s_tx_bit_format = SNDRV_PCM_FORMAT_S16_LE;

static const char * const two_ch_text[] = {"One", "Two"};
static const char * const four_ch_text[] = {"One", "Two", "Three", "Four"};
static const char * const six_ch_text[] = {"One", "Two", "Three", "Four",
					   "Five", "Six"};
static const char * const btsco_rate_text[] = {"8000", "16000"};
static const struct soc_enum msm_btsco_enum[] = {
		SOC_ENUM_SINGLE_EXT(2, btsco_rate_text),
};

static const char * const auxpcm_rate_text[] = {"rate_8000", "rate_16000"};
static const struct soc_enum msm_auxpcm_enum[] = {
		SOC_ENUM_SINGLE_EXT(2, auxpcm_rate_text),
};

static char const *format_text[] = {"S16_LE", "S24_LE"};

static const struct soc_enum msm_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, two_ch_text),
	SOC_ENUM_SINGLE_EXT(4, four_ch_text),
	SOC_ENUM_SINGLE_EXT(6, six_ch_text),
	SOC_ENUM_SINGLE_EXT(2, format_text),
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

static int msm_btsco_rate_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: msm_btsco_rate  = %d", __func__, msm_btsco_rate);
	ucontrol->value.integer.value[0] = msm_btsco_rate;
	return 0;
}

static int msm_btsco_rate_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 0:
		msm_btsco_rate = SAMPLE_RATE_8KHZ;
		break;
	case 1:
		msm_btsco_rate = SAMPLE_RATE_16KHZ;
		break;
	default:
		msm_btsco_rate = SAMPLE_RATE_8KHZ;
		break;
	}
	pr_debug("%s: msm_btsco_rate = %d\n", __func__, msm_btsco_rate);
	return 0;
}

static int msm_auxpcm_rate_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: msm_auxpcm_rate  = %d", __func__,
		msm_auxpcm_rate);
	ucontrol->value.integer.value[0] = msm_auxpcm_rate;
	return 0;
}

static int msm_auxpcm_rate_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 0:
		msm_auxpcm_rate = SAMPLE_RATE_8KHZ;
		break;
	case 1:
		msm_auxpcm_rate = SAMPLE_RATE_16KHZ;
		break;
	default:
		msm_auxpcm_rate = SAMPLE_RATE_8KHZ;
		break;
	}
	pr_debug("%s: msm_auxpcm_rate = %d, ucontrol->value.integer.value[0] = %d\n",
		 __func__, msm_auxpcm_rate,
		 (int)ucontrol->value.integer.value[0]);
	return 0;
}

static int msm_mi2s_rx_bit_format_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	switch (msm_mi2s_rx_bit_format) {
	case SNDRV_PCM_FORMAT_S24_LE:
		ucontrol->value.integer.value[0] = 1;
		break;

	case SNDRV_PCM_FORMAT_S16_LE:
	default:
		ucontrol->value.integer.value[0] = 0;
		break;
	}

	pr_debug("%s: msm_mi2s_rx_bit_format = %d, ucontrol value = %ld\n",
			 __func__, msm_mi2s_rx_bit_format,
			ucontrol->value.integer.value[0]);

	return 0;
}

static int msm_mi2s_rx_bit_format_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 1:
		msm_mi2s_rx_bit_format = SNDRV_PCM_FORMAT_S24_LE;
		break;
	case 0:
	default:
		msm_mi2s_rx_bit_format = SNDRV_PCM_FORMAT_S16_LE;
		break;
	}
	pr_debug("%s: msm_mi2s_rx_bit_format = %d, ucontrol value = %ld\n",
		 __func__, msm_mi2s_rx_bit_format,
		 ucontrol->value.integer.value[0]);
	return 0;
}

static int msm_proxy_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
	SNDRV_PCM_HW_PARAM_RATE);

	rate->min = rate->max = SAMPLE_RATE_48KHZ;

	return 0;
}

static int msm_auxpcm_be_params_fixup(struct snd_soc_pcm_runtime *rtd,
					struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);

	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);

	rate->min = rate->max = msm_auxpcm_rate;
	/* PCM only supports mono output */
	channels->min = channels->max = 1;

	return 0;
}

static int msm_btsco_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
					struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
						      SNDRV_PCM_HW_PARAM_RATE);

	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);
	rate->min = rate->max = msm_btsco_rate;
	channels->min = channels->max = 1;

	return 0;
}
static inline int param_is_mask(int p)
{
	return ((p >= SNDRV_PCM_HW_PARAM_FIRST_MASK) &&
			(p <= SNDRV_PCM_HW_PARAM_LAST_MASK));
}

static inline struct snd_mask *param_to_mask(struct snd_pcm_hw_params *p, int n)
{
	return &(p->masks[n - SNDRV_PCM_HW_PARAM_FIRST_MASK]);
}
static void param_set_mask(struct snd_pcm_hw_params *p, int n, unsigned bit)
{
	if (bit >= SNDRV_MASK_MAX)
		return;
	if (param_is_mask(n)) {
		struct snd_mask *m = param_to_mask(p, n);
		m->bits[0] = 0;
		m->bits[1] = 0;
		m->bits[bit >> 5] |= (1 << (bit & 31));
	}
}

static int msm_mi2s_rx_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
	SNDRV_PCM_HW_PARAM_RATE);

	struct snd_interval *channels = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("%s()\n", __func__);
	rate->min = rate->max = SAMPLE_RATE_48KHZ;
	channels->min = channels->max = msm_mi2s_rx_ch;

	param_set_mask(params, SNDRV_PCM_HW_PARAM_FORMAT,
				   msm_mi2s_rx_bit_format);
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

static int msm_i2s_rx_bit_format_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	switch (msm_i2s_rx_bit_format) {
	case SNDRV_PCM_FORMAT_S24_LE:
		ucontrol->value.integer.value[0] = 1;
		break;

	case SNDRV_PCM_FORMAT_S16_LE:
	default:
		ucontrol->value.integer.value[0] = 0;
		break;
	}

	pr_debug("%s: msm_i2s_rx_bit_format = %d, ucontrol value = %ld\n",
			 __func__, msm_i2s_rx_bit_format,
			ucontrol->value.integer.value[0]);

	return 0;
}

static int msm_i2s_rx_bit_format_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 1:
		msm_i2s_rx_bit_format = SNDRV_PCM_FORMAT_S24_LE;
		break;
	case 0:
	default:
		msm_i2s_rx_bit_format = SNDRV_PCM_FORMAT_S16_LE;
		break;
	}
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
	rate->min = rate->max = SAMPLE_RATE_48KHZ;
	channels->min = channels->max = msm_i2s_rx_ch;

	param_set_mask(params, SNDRV_PCM_HW_PARAM_FORMAT,
				   msm_i2s_rx_bit_format);

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

static int msm_i2s_tx_bit_format_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	switch (msm_i2s_tx_bit_format) {
	case SNDRV_PCM_FORMAT_S24_LE:
		ucontrol->value.integer.value[0] = 1;
		break;

	case SNDRV_PCM_FORMAT_S16_LE:
	default:
		ucontrol->value.integer.value[0] = 0;
		break;
	}

	pr_debug("%s: msm_i2s_tx_bit_format = %d, ucontrol value = %ld\n",
			 __func__, msm_i2s_tx_bit_format,
			ucontrol->value.integer.value[0]);

	return 0;
}

static int msm_i2s_tx_bit_format_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 1:
		msm_i2s_tx_bit_format = SNDRV_PCM_FORMAT_S24_LE;
		break;
	case 0:
	default:
		msm_i2s_tx_bit_format = SNDRV_PCM_FORMAT_S16_LE;
		break;
	}
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
	rate->min = rate->max = SAMPLE_RATE_48KHZ;

	channels->min = channels->max = msm_i2s_tx_ch;

	param_set_mask(params, SNDRV_PCM_HW_PARAM_FORMAT,
				   msm_i2s_tx_bit_format);

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
	struct request_gpio *gpio_list = mi2s_gpio;
	int gpio_list_size = ARRAY_SIZE(mi2s_gpio);

	if (is_mplatform) {
		gpio_list = mi2s_gpio_mplatform;
		gpio_list_size = ARRAY_SIZE(mi2s_gpio_mplatform);
	}

	for (i = 0; i < gpio_list_size; i++)
		gpio_free(gpio_list[i].gpio_no);
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
	struct request_gpio *gpio_list = mi2s_gpio;
	int gpio_list_size = ARRAY_SIZE(mi2s_gpio);

	if (is_mplatform) {
		gpio_list = mi2s_gpio_mplatform;
		gpio_list_size = ARRAY_SIZE(mi2s_gpio_mplatform);
	}

	for (i = 0; i < gpio_list_size; i++) {
		rtn = gpio_request(gpio_list[i].gpio_no,
				   gpio_list[i].gpio_name);
		pr_debug("%s: gpio = %d, gpio name = %s, rtn = %d\n",
				 __func__,
				 gpio_list[i].gpio_no,
				 gpio_list[i].gpio_name,
				 rtn);
		if (rtn) {
			pr_err("%s: Failed to request gpio %d\n",
				   __func__,
				   gpio_list[i].gpio_no);
			for (j = i; j >= 0; j--)
				gpio_free(gpio_list[j].gpio_no);
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
	unsigned long rate = OSR_CLK_DOUBLE_RATE;

	pr_debug("%s: dai name %s %p\n", __func__, cpu_dai->name, cpu_dai->dev);

	if (atomic_inc_return(&mi2s_rsc_ref) == 1) {
		pr_debug("%s: acquire mi2s resources\n", __func__);
		msm_configure_mi2s_gpio();
		if (is_mi2s_master) {
			pr_debug("%s: APQ is MI2S master\n", __func__);
			mi2s_osr_clk = clk_get(cpu_dai->dev, "osr_clk");
			if (IS_ERR(mi2s_osr_clk)) {
				pr_err("Unable to get mi2s_osr_clk\n");
				return PTR_ERR(mi2s_osr_clk);
			}
			if (msm_mi2s_rx_bit_format == SNDRV_PCM_FORMAT_S16_LE)
				rate = OSR_CLK_RATE;

			pr_debug("%s: mi2s_osr_clk rate is %lu\n", __func__,
				 rate);
			clk_set_rate(mi2s_osr_clk, rate);

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
			ret = snd_soc_dai_set_fmt(cpu_dai,
						  SND_SOC_DAIFMT_CBS_CFS);
			if (IS_ERR_VALUE(ret))
				pr_err("set format for CPU dai failed\n");
		} else {
			pr_debug("%s: APQ is MI2S slave.\n", __func__);

			mi2s_bit_clk = clk_get(cpu_dai->dev, "bit_clk");
			if (IS_ERR(mi2s_bit_clk)) {
				pr_err("Unable to get mi2s_bit_clk\n");
				return PTR_ERR(mi2s_bit_clk);
			}
			clk_set_rate(mi2s_bit_clk, 0);
			ret = clk_prepare_enable(mi2s_bit_clk);
			if (IS_ERR_VALUE(ret)) {
				pr_err("Unable to enable mi2s_bit_clk\n");
				clk_put(mi2s_bit_clk);
				return ret;
			}
			ret = snd_soc_dai_set_fmt(cpu_dai,
						  SND_SOC_DAIFMT_CBM_CFM);
			if (IS_ERR_VALUE(ret))
				pr_err("set format for CPU dai failed\n");

		}
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
	struct request_gpio *gpio_list = mic_i2s_gpio;
	int gpio_list_size = ARRAY_SIZE(mic_i2s_gpio);

	if (is_mplatform) {
		gpio_list = mic_i2s_gpio_mplatform;
		gpio_list_size = ARRAY_SIZE(mic_i2s_gpio_mplatform);
	}

	for (i = 0; i < gpio_list_size; i++)
		gpio_free(gpio_list[i].gpio_no);
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
	struct request_gpio *gpio_list = mic_i2s_gpio;
	int gpio_list_size = ARRAY_SIZE(mic_i2s_gpio);

	if (is_mplatform) {
		gpio_list = mic_i2s_gpio_mplatform;
		gpio_list_size = ARRAY_SIZE(mic_i2s_gpio_mplatform);
	}

	for (i = 0; i < gpio_list_size; i++) {
		rtn = gpio_request(gpio_list[i].gpio_no,
				   gpio_list[i].gpio_name);
		pr_debug("%s: gpio = %d, gpio name = %s, rtn = %d\n",
				 __func__,
				 gpio_list[i].gpio_no,
				 gpio_list[i].gpio_name,
				 rtn);
		if (rtn) {
			pr_err("%s: Failed to request gpio %d\n",
				   __func__,
				   gpio_list[i].gpio_no);
			for (j = i; j >= 0; j--)
				gpio_free(gpio_list[j].gpio_no);
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

static int msm_aux_pcm_get_gpios(void)
{
	int ret = 0;

	ret = gpio_request(GPIO_AUX_PCM_DOUT, "AUX PCM DOUT");
	if (ret < 0) {
		pr_err("%s: Failed to request gpio(%d): AUX PCM DOUT",
				__func__, GPIO_AUX_PCM_DOUT);
		goto fail_dout;
	}

	ret = gpio_request(GPIO_AUX_PCM_DIN, "AUX PCM DIN");
	if (ret < 0) {
		pr_err("%s: Failed to request gpio(%d): AUX PCM DIN",
				__func__, GPIO_AUX_PCM_DIN);
		goto fail_din;
	}

	ret = gpio_request(GPIO_AUX_PCM_SYNC, "AUX PCM SYNC");
	if (ret < 0) {
		pr_err("%s: Failed to request gpio(%d): AUX PCM SYNC",
				__func__, GPIO_AUX_PCM_SYNC);
		goto fail_sync;
	}
	ret = gpio_request(GPIO_AUX_PCM_CLK, "AUX PCM CLK");
	if (ret < 0) {
		pr_err("%s: Failed to request gpio(%d): AUX PCM CLK",
				__func__, GPIO_AUX_PCM_CLK);
		goto fail_clk;
	}

	return 0;

fail_clk:
	gpio_free(GPIO_AUX_PCM_SYNC);
fail_sync:
	gpio_free(GPIO_AUX_PCM_DIN);
fail_din:
	gpio_free(GPIO_AUX_PCM_DOUT);
fail_dout:

	return ret;
}

static int msm_aux_pcm_free_gpios(void)
{
	gpio_free(GPIO_AUX_PCM_DIN);
	gpio_free(GPIO_AUX_PCM_DOUT);
	gpio_free(GPIO_AUX_PCM_SYNC);
	gpio_free(GPIO_AUX_PCM_CLK);

	return 0;
}

static int msm_auxpcm_startup(struct snd_pcm_substream *substream)
{
	int ret = 0;

	pr_debug("%s(): substream = %s, auxpcm_rsc_ref counter = %d\n",
		__func__, substream->name, atomic_read(&auxpcm_rsc_ref));
	if (atomic_inc_return(&auxpcm_rsc_ref) == 1)
		ret = msm_aux_pcm_get_gpios();

	if (ret < 0) {
		pr_err("%s: Aux PCM GPIO request failed\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static void msm_auxpcm_shutdown(struct snd_pcm_substream *substream)
{
	pr_debug("%s(): substream = %s, auxpcm_rsc_ref counter = %d\n",
		__func__, substream->name, atomic_read(&auxpcm_rsc_ref));
	if (atomic_dec_return(&auxpcm_rsc_ref) == 0)
		msm_aux_pcm_free_gpios();
}

static int apq8064_sec_i2s_rx_free_gpios(void)
{
	int	i;
	for (i = 0; i < ARRAY_SIZE(sec_i2s_rx_gpio); i++)
		gpio_free(sec_i2s_rx_gpio[i].gpio_no);
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

static int msm_hdmi_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
					struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);

	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("%s channels->min %u channels->max %u ()\n", __func__,
			channels->min, channels->max);

	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;
	return 0;
}

static struct snd_soc_ops msm_mi2s_be_ops = {
	.startup = msm_mi2s_startup,
	.shutdown = msm_mi2s_shutdown,
};

static struct snd_soc_ops sec_i2s_rx_ops = {
	.startup = apq8064_sec_i2s_rx_startup,
	.shutdown = apq8064_sec_i2s_rx_shutdown,
};

static struct snd_soc_ops msm_i2s_be_ops = {
	.startup = msm_i2s_startup,
	.shutdown = msm_i2s_shutdown,
};

static struct snd_soc_ops msm_auxpcm_be_ops = {
	.startup = msm_auxpcm_startup,
	.shutdown = msm_auxpcm_shutdown,
};

static const struct snd_kcontrol_new msm_controls[] = {
	SOC_ENUM_EXT("MI2S_RX Channels", msm_enum[2],
		msm_mi2s_rx_ch_get, msm_mi2s_rx_ch_put),
	SOC_ENUM_EXT("I2S_RX Channels", msm_enum[0],
		msm_i2s_rx_ch_get, msm_i2s_rx_ch_put),
	SOC_ENUM_EXT("I2S_TX Channels", msm_enum[1],
			msm_i2s_tx_ch_get, msm_i2s_tx_ch_put),
	SOC_ENUM_EXT("Internal BTSCO SampleRate", msm_btsco_enum[0],
		msm_btsco_rate_get, msm_btsco_rate_put),
	SOC_ENUM_EXT("AUX PCM SampleRate", msm_auxpcm_enum[0],
		msm_auxpcm_rate_get, msm_auxpcm_rate_put),
	SOC_ENUM_EXT("MI2S_0_RX Format", msm_enum[3],
			msm_mi2s_rx_bit_format_get, msm_mi2s_rx_bit_format_put),
	SOC_ENUM_EXT("I2S_0_RX Format", msm_enum[3],
			msm_i2s_rx_bit_format_get, msm_i2s_rx_bit_format_put),
	SOC_ENUM_EXT("I2S_0_TX Format", msm_enum[3],
			msm_i2s_tx_bit_format_get, msm_i2s_tx_bit_format_put),
};

/* Digital audio interface glue - connects codec <---> CPU */
static struct snd_soc_dai_link msm_dai[] = {
	/* FrontEnd DAI Links */
	{
		.name = "Media1",
		.stream_name = "MultiMedia1", /* hw:0,0 */
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
		.stream_name = "MultiMedia2", /* hw:0,1 */
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
		.stream_name = "CS-Voice", /* hw:0,2 */
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
		.name = "MSM VoIP",
		.stream_name = "VoIP", /* hw:0,3 */
		.cpu_dai_name	= "VoIP",
		.platform_name	= "msm-voip-dsp",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1, /* playback support */
		.be_id = MSM_FRONTEND_DAI_VOIP,
	},
	{
		.name = "Driver Side",
		.stream_name = "MultiMedia3",
		.cpu_dai_name	= "MultiMedia3", /* hw:0,4 */
		.platform_name  = "msm-lowlatency-pcm-dsp",
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
		.name = "MSM AFE-PCM RX",
		.stream_name = "AFE-PROXY RX", /* hw:0,5 */
		.cpu_dai_name = "msm-dai-q6.241",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-rx",
		.platform_name	= "msm-pcm-afe",
		.ignore_suspend = 1,
		/* this dainlink has playback support */
		.ignore_pmdown_time = 1,
	},
	{
		.name = "MSM AFE-PCM TX",
		.stream_name = "AFE-PROXY TX",
		.cpu_dai_name = "msm-dai-q6.240", /* hw:0,6 */
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-tx",
		.platform_name	= "msm-pcm-afe",
		.ignore_suspend = 1,
	},
	{
		.name = "Compr",
		.stream_name = "COMPR",
		.cpu_dai_name	= "MultiMedia4", /* hw:0,7 */
		.platform_name  = "msm-compress-dsp",
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
		.stream_name = "MultiMedia5", /* hw:0,8 */
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
	/* Hostless from BT HFP */
	{
		.name = "INT_HFP_BT Hostless",
		.stream_name = "INT_HFP_BT Hostless", /* hw:0,9 */
		.cpu_dai_name	= "INT_HFP_BT_HOSTLESS",
		.platform_name	= "msm-pcm-hostless",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		/* this dai link has playback support */
		.ignore_pmdown_time = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
	/* Dai for ASM loopback */
	{
		.name = "HFP TX",
		.stream_name = "MultiMedia6", /* hw:0,10 */
		.cpu_dai_name = "MultiMedia6",
		.platform_name	= "msm-pcm-loopback",
		.dynamic = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		/* this dainlink has playback support */
		.ignore_pmdown_time = 1,
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA6,
	},
	/* Primary I2S RX and TX Hybrid Hostless*/
	{
		/* hw:0,11, must match to REPLACEMENT_FRONTEND_DAI_INDEX */
		.name = "I2S Hybrid Hostless", /* hw:0,11 */
		.stream_name = "I2S Hybrid Hostless",
		.cpu_dai_name	= "I2S_HYBRID_HOSTLESS",
		.platform_name	= "msm-pcm-hostless",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1, /* playback support */
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		/* .be_id = do not care */
	},
	{
		.name		= "Audio MI2S Rx", /* hw:0,12 */
		.stream_name	= "Audio MI2S Rx output",
		.cpu_dai_name	= "apq8064_cpudai_lpa.0",
		.platform_name	= "apq8064_pcm_lpa",
		.codec_dai_name = "msm-stub-rx",
		.codec_name	= "msm-stub-codec.1",
		.ops		= &msm_mi2s_be_ops,
		.ignore_pmdown_time	= 1,
	},
	{
		.name = "RES Playback",
		.stream_name = "MultiMedia7",
		.cpu_dai_name	= "MultiMedia7", /* hw:0,13 */
		.platform_name  = "msm-pcm-dsp",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1, /* playback support */
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA7,
	},
	{
		.name = "RES Compr",
		.stream_name = "MultiMedia8",
		.cpu_dai_name	= "MultiMedia8", /* hw:0,14 */
		.platform_name  = "msm-compress-dsp",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1, /* playback support */
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA8,
	},
	/* Any new frondend DAIs have to be inserted after this point */
	/* Backend DAI Links */
	{
		.name = LPASS_BE_INT_BT_SCO_RX,
		.stream_name = "Internal BT-SCO Playback",
		.cpu_dai_name = "msm-dai-q6.12288",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-rx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_INT_BT_SCO_RX,
		.be_hw_params_fixup = msm_btsco_be_hw_params_fixup,
		.ignore_pmdown_time = 1, /* playback support */
	},
	{
		.name = LPASS_BE_INT_BT_SCO_TX,
		.stream_name = "Internal BT-SCO Capture",
		.cpu_dai_name = "msm-dai-q6.12289",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-tx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_INT_BT_SCO_TX,
		.be_hw_params_fixup = msm_btsco_be_hw_params_fixup,
	},
	/* Backend AFE DAI Links */
	{
		.name = LPASS_BE_AFE_PCM_RX,
		.stream_name = "AFE Playback",
		.cpu_dai_name = "msm-dai-q6.224",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-rx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_AFE_PCM_RX,
		.be_hw_params_fixup = msm_proxy_be_hw_params_fixup,
		/* this dainlink has playback support */
		.ignore_pmdown_time = 1,
	},
	{
		.name = LPASS_BE_AFE_PCM_TX,
		.stream_name = "AFE Capture",
		.cpu_dai_name = "msm-dai-q6.225",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-tx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_AFE_PCM_TX,
		.be_hw_params_fixup = msm_proxy_be_hw_params_fixup,
	},
	/* AUX PCM Backend DAI Links */
	{
		.name = LPASS_BE_AUXPCM_RX,
		.stream_name = "AUX PCM Playback",
		.cpu_dai_name = "msm-dai-q6.2",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-rx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_AUXPCM_RX,
		.be_hw_params_fixup = msm_auxpcm_be_params_fixup,
		.ops = &msm_auxpcm_be_ops,
		.ignore_pmdown_time = 1,
	},
	{
		.name = LPASS_BE_AUXPCM_TX,
		.stream_name = "AUX PCM Capture",
		.cpu_dai_name = "msm-dai-q6.3",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-tx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_AUXPCM_TX,
		.be_hw_params_fixup = msm_auxpcm_be_params_fixup,
		.ops = &msm_auxpcm_be_ops,
	},
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
		.name = LPASS_BE_MI2S_GROUP_RX_0,
		.stream_name = "MI2S_GROUP_0 Playback",
		.cpu_dai_name = "msm-dai-q6-mi2s-group.6",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "msm-stub-codec.1",
		.codec_dai_name	= "msm-stub-rx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_MI2S_GROUP_RX_0,
		.be_hw_params_fixup = msm_mi2s_rx_be_hw_params_fixup,
		.ops = &msm_mi2s_be_ops,
		.ignore_pmdown_time = 1, /* Playback support */
	},
	{
		.name = LPASS_BE_MI2S_GROUP_RX_1,
		.stream_name = "MI2S_GROUP_1 Playback",
		.cpu_dai_name = "msm-dai-q6-mi2s-group.14",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "msm-stub-codec.1",
		.codec_dai_name	= "msm-stub-rx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_MI2S_GROUP_RX_1,
		.be_hw_params_fixup = msm_mi2s_rx_be_hw_params_fixup,
		.ops = &msm_mi2s_be_ops,
		.ignore_pmdown_time = 1, /* Playback support */
	},
	{
		.name = LPASS_BE_MI2S_GROUP_RX_2,
		.stream_name = "MI2S_GROUP_2 Playback",
		.cpu_dai_name = "msm-dai-q6-mi2s-group.16",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "msm-stub-codec.1",
		.codec_dai_name	= "msm-stub-rx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_MI2S_GROUP_RX_2,
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
	{
		.name = LPASS_BE_SEC_I2S_RX,
		.stream_name = "Secondary I2S Playback",
		.cpu_dai_name = "msm-dai-q6.4",
		.platform_name = "msm-pcm-routing",
		.codec_name	= "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-rx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SEC_I2S_RX,
		.be_hw_params_fixup = msm_i2s_rx_be_hw_params_fixup,
		.ops = &sec_i2s_rx_ops,
		.ignore_pmdown_time = 1, /* playback support */
	},
	/* HDMI BACK END DAI Link */
	{
		.name = LPASS_BE_HDMI,
		.stream_name = "HDMI Playback",
		.cpu_dai_name = "msm-dai-q6-hdmi.8",
		.platform_name = "msm-pcm-routing",
		.codec_name	= "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-rx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_HDMI_RX,
		.be_hw_params_fixup = msm_hdmi_be_hw_params_fixup,
		.ignore_pmdown_time = 1, /* playback support */
	},
};

/* It is the replacement for msm_dai[REPLACEMENT_FRONTEND_DAI_INDEX] */
static struct snd_soc_dai_link msm_mplatform_dai[] = {
/* MI2S RX and Primary I2S TX Hybrid Hostless*/
	/* MI2S RX and Primary I2S TX Hybrid Hostless*/
	{
		.name = "MI2S_I2S Hybrid Hostless", /* hw:0,11 */
		.stream_name = "MI2S_I2S Hostless",
		.cpu_dai_name	= "MI2S_I2S_HYBRID_HOSTLESS",
		.platform_name	= "msm-pcm-hostless",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1, /* playback support */
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		/* .be_id = do not care */
	},
};

static struct snd_soc_card snd_soc_card_msm_mplatform = {
	.name		= "apq8064-mplatform-snd-card",
	.dai_link	= msm_dai,
	.num_links	= ARRAY_SIZE(msm_dai),
	.controls = msm_controls,
	.num_controls = ARRAY_SIZE(msm_controls),
};

static struct snd_soc_card snd_soc_card_msm_auto = {
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
	if (!(machine_is_apq8064_adp_2() || machine_is_apq8064_adp2_es2() ||
		 machine_is_apq8064_adp2_es2p5())) {
		pr_err("%s: Not APQ8064-auto machine type\n", __func__);
		ret = -ENODEV;
		goto err;
	}

	if (machine_is_apq8064_mplatform()) {
		is_mplatform = 1;
		pr_info("%s: apq8064 mplatform\n", __func__);
	} else {
		pr_info("%s: apq8064-auto platform\n", __func__);
	}

	msm_snd_device = platform_device_alloc("soc-audio", 0);
	if (!msm_snd_device) {
		pr_err("Platform device allocation failed\n");
		ret = -ENOMEM;
		goto err;
	}

	if (!is_mplatform) {
		platform_set_drvdata(msm_snd_device, &snd_soc_card_msm_auto);
	} else {
		memcpy(msm_dai + REPLACEMENT_FRONTEND_DAI_INDEX,
		       msm_mplatform_dai, sizeof(msm_mplatform_dai));
		platform_set_drvdata(msm_snd_device,
				     &snd_soc_card_msm_mplatform);
	}

	ret = platform_device_add(msm_snd_device);
	if (ret) {
		platform_device_put(msm_snd_device);
		pr_err("%s: platform_device_add device0 failed\n", __func__);
		goto err;
	}

	atomic_set(&auxpcm_rsc_ref, 0);
	atomic_set(&mi2s_rsc_ref, 0);
err:
	return ret;

}
module_init(msm_audio_init);

static void __exit msm_audio_exit(void)
{
	if (!machine_is_apq8064_adp_2()) {
		pr_err("%s: Not APQ8064-auto machine type\n", __func__);
		return ;
	}

	pr_info("%s: apq8064-auto exit\n", __func__);
	platform_device_unregister(msm_snd_device);
}
module_exit(msm_audio_exit);

MODULE_DESCRIPTION("ALSA SoC msm");
MODULE_LICENSE("GPL v2");
