/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
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
#include <linux/err.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/time.h>
#include <linux/math64.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <sound/q6asm.h>
#include <sound/pcm_params.h>
#include <asm/dma.h>
#include <linux/dma-mapping.h>
#include <sound/timer.h>
#include <sound/tlv.h>
#include <sound/apr_audio.h>
#include <sound/compress_params.h>
#include <sound/compress_offload.h>
#include <sound/compress_driver.h>
#include "msm-pcm-routing.h"

/* Default values used if user space does not set */
#define COMPR_PLAYBACK_MIN_FRAGMENT_SIZE (8 * 1024)
#define COMPR_PLAYBACK_MAX_FRAGMENT_SIZE (128 * 1024)
#define COMPR_PLAYBACK_MIN_NUM_FRAGMENTS (4)
#define COMPR_PLAYBACK_MAX_NUM_FRAGMENTS (16 * 4)

#define COMPRESSED_LR_VOL_MAX_STEPS	0x2000
const DECLARE_TLV_DB_LINEAR(msm_compr_vol_gain, 0,
				COMPRESSED_LR_VOL_MAX_STEPS);

struct msm_compr_pdata {
	struct snd_compr_stream *cstream[MSM_FRONTEND_DAI_MAX];
	uint32_t volume[MSM_FRONTEND_DAI_MAX][2]; /* For both L & R */
};

struct msm_compr_audio {
	struct snd_compr_stream *cstream;
	struct snd_compr_caps compr_cap;
	struct snd_compr_codec_caps codec_caps;
	struct snd_compr_params codec_param;
	struct audio_client *audio_client;

	uint32_t codec;
	void    *buffer; /* virtual address */
	uint32_t buffer_paddr; /* physical address */
	uint32_t app_pointer;
	uint32_t buffer_size;
	uint32_t byte_offset;
	uint32_t copied_total; /* bytes consumed by DSP */
	uint32_t bytes_received; /* from userspace */
	uint32_t bytes_sent; /* to DSP */

	uint16_t session_id;

	uint32_t sample_rate;
	uint32_t num_channels;

	uint32_t cmd_ack;
	uint32_t cmd_interrupt;
	uint32_t drain_ready;
	uint32_t eos_ack;

	uint64_t marker_timestamp;
	atomic_t start;
	atomic_t eos;
	atomic_t drain;
	atomic_t xrun;
	atomic_t error;

	wait_queue_head_t eos_wait;
	wait_queue_head_t drain_wait;

	spinlock_t lock;
};

static int msm_compr_set_volume(struct snd_compr_stream *cstream,
				uint32_t volume_l, uint32_t volume_r)
{
	struct msm_compr_audio *prtd;
	int rc = 0;
	uint32_t avg_vol;

	pr_debug("%s: volume_l %d volume_r %d\n",
		__func__, volume_l, volume_r);
	if (!cstream || !cstream->runtime) {
		pr_err("%s: session not active\n", __func__);
		return -EPERM;
	}
	prtd = cstream->runtime->private_data;

	if (!prtd || !prtd->audio_client) {
		pr_err("%s: invalid session prtd or no audio client", __func__);
		return rc;
	}

	if (prtd->num_channels > 2) {
		/*
		 * Currently the left and right gains are averaged an applied
		 * to all channels. This might not be desirable. But currently,
		 * there exists no API in userspace to send a list of gains for
		 * each channel either. If such an API does become available,
		 * the mixer control must be updated to accept more than 2
		 * channel gains.
		 *
		 */
		pr_debug("%s: call q6asm_set_volume for multichannel\n",
			 __func__);
		avg_vol = (volume_l + volume_r) / 2;
		rc = q6asm_set_volume(prtd->audio_client, avg_vol);
	} else {
		pr_debug("%s: call q6asm_set_lrgain\n", __func__);
		rc = q6asm_set_lrgain(prtd->audio_client, volume_l, volume_r);
		if (rc < 0)
			pr_err("%s: Send LR gain command failed rc=%d\n",
				__func__, rc);
	}

	if (rc < 0) {
		pr_err("%s: Send Volume command failed rc=%d\n",
			__func__, rc);
	}

	return rc;
}

static int msm_compr_send_buffer(struct msm_compr_audio *prtd)
{
	int buffer_length;
	int bytes_available;
	struct audio_aio_write_param param;

	if (!atomic_read(&prtd->start)) {
		pr_err("%s: stream is not in started state\n", __func__);
		return -EINVAL;
	}


	if (atomic_read(&prtd->xrun)) {
		WARN(1, "%s called while xrun is true", __func__);
		return -EPERM;
	}

	pr_debug("%s: bytes_received = %d copied_total = %d\n",
		__func__, prtd->bytes_received, prtd->copied_total);
	buffer_length = prtd->codec_param.buffer.fragment_size;
	bytes_available = prtd->bytes_received - prtd->copied_total;
	if (bytes_available < prtd->codec_param.buffer.fragment_size)
		buffer_length = bytes_available;

	if (prtd->byte_offset + buffer_length > prtd->buffer_size) {
		buffer_length = (prtd->buffer_size - prtd->byte_offset);
		pr_debug("wrap around situation, send partial data %d now",
			 buffer_length);
	}

	if (buffer_length) {
		param.paddr	= prtd->buffer_paddr + prtd->byte_offset;
		WARN(prtd->byte_offset % 32 != 0, "offset %x not multiple of 32",
		prtd->byte_offset);
	} else
		param.paddr	= prtd->buffer_paddr;

	param.len	= buffer_length;
	param.msw_ts	= 0;
	param.lsw_ts	= 0;
	param.flags	= NO_TIMESTAMP;
	param.uid	= buffer_length;

	pr_debug("%s: sending %d bytes to DSP byte_offset = %d\n",
		__func__, buffer_length, prtd->byte_offset);
	if (q6asm_async_write(prtd->audio_client, &param) < 0)
		pr_err("%s:q6asm_async_write failed\n", __func__);
	else
		prtd->bytes_sent += buffer_length;

	return 0;
}

static void compr_event_handler(uint32_t opcode,
		uint32_t token, uint32_t *payload, void *priv)
{
	struct msm_compr_audio *prtd = priv;
	struct snd_compr_stream *cstream;
	struct audio_client *ac;
	int bytes_available;
	unsigned long flags;

	if (!prtd) {
		pr_err("%s: prtd is NULL\n", __func__);
		return;
	}
	cstream = prtd->cstream;
	ac = prtd->audio_client;

	pr_debug("%s opcode =%08x\n", __func__, opcode);
	switch (opcode) {
	case ASM_DATA_EVENT_WRITE_DONE:
		pr_debug("ASM_DATA_EVENT_WRITE_DONE\n");
		spin_lock_irqsave(&prtd->lock, flags);

		if (payload[1]) {
			pr_err("WRITE FAILED w/ err 0x%x !, paddr 0x%x byte_offset=%d, copied_total=%d, token=%d\n",
			       payload[1], payload[0],
				prtd->byte_offset, prtd->copied_total, token);
			atomic_set(&prtd->start, 0);
		} else {
			pr_debug("ASM_DATA_EVENT_WRITE_DONE offset %d, length %d\n",
				 prtd->byte_offset, token);
		}

		prtd->byte_offset += token;
		prtd->copied_total += token;
		if (prtd->byte_offset >= prtd->buffer_size)
			prtd->byte_offset -= prtd->buffer_size;

		snd_compr_fragment_elapsed(cstream);

		if (!atomic_read(&prtd->start)) {
			/* Writes must be restarted from _copy() */
			pr_debug("write_done received while not started, treat as xrun");
			atomic_set(&prtd->xrun, 1);
			spin_unlock_irqrestore(&prtd->lock, flags);
			break;
		}

		bytes_available = prtd->bytes_received - prtd->copied_total;
		if (bytes_available < cstream->runtime->fragment_size) {
			pr_debug("WRITE_DONE Insufficient data to send. break out\n");
			atomic_set(&prtd->xrun, 1);
			if (atomic_read(&prtd->drain)) {
				pr_debug("wake up on drain\n");
				prtd->drain_ready = 1;
				wake_up(&prtd->drain_wait);
				atomic_set(&prtd->drain, 0);
			}
		} else if ((bytes_available == cstream->runtime->fragment_size)
			   && atomic_read(&prtd->drain)) {
			msm_compr_send_buffer(prtd);
		} else
			msm_compr_send_buffer(prtd);

		spin_unlock_irqrestore(&prtd->lock, flags);
		break;
	case ASM_DATA_CMDRSP_EOS:
		spin_lock_irqsave(&prtd->lock, flags);
		pr_debug("ASM_DATA_CMDRSP_EOS wake up\n");
		prtd->eos_ack = 1;
		wake_up(&prtd->eos_wait);
		atomic_set(&prtd->eos, 0);
		spin_unlock_irqrestore(&prtd->lock, flags);
		break;
	case APR_BASIC_RSP_RESULT: {
		switch (payload[0]) {
		case ASM_SESSION_CMD_RUN:
			/* check if the first buffer need to be sent to DSP */
			pr_debug("ASM_SESSION_CMD_RUN\n");
			/* FIXME: A state is a better way, dealing with this*/
			spin_lock_irqsave(&prtd->lock, flags);
			if (!prtd->bytes_sent) {
				bytes_available = prtd->bytes_received -
						  prtd->copied_total;
				if (bytes_available <
					cstream->runtime->fragment_size) {
					pr_debug("CMD_RUN Insufficient data to send. break out\n");
					atomic_set(&prtd->xrun, 1);
				} else
					msm_compr_send_buffer(prtd);
			}

			/*
			 * The condition below ensures playback finishes in the
			 * follow cornercase
			 * WAIT_FOR_DRAIN
			 * PAUSE
			 * WRITE_DONE(X)
			 * RESUME
			 */
			if ((prtd->copied_total == prtd->bytes_sent) &&
			    atomic_read(&prtd->drain)) {
				pr_debug("RUN ack, wake up & continue pending drain\n");
				prtd->drain_ready = 1;
				wake_up(&prtd->drain_wait);
				atomic_set(&prtd->drain, 0);
			}

			spin_unlock_irqrestore(&prtd->lock, flags);
			break;
		case ASM_STREAM_CMD_FLUSH:
			pr_debug("%s: ASM_STREAM_CMD_FLUSH", __func__);
			prtd->cmd_ack = 1;
			break;
		default:
			break;
		}
		break;
	}
	default:
		pr_debug("Not Supported Event opcode[0x%x]\n", opcode);
		break;
	}
}

static void populate_codec_list(struct msm_compr_audio *prtd)
{
	pr_debug("%s\n", __func__);
	prtd->compr_cap.direction = SND_COMPRESS_PLAYBACK;
	prtd->compr_cap.min_fragment_size =
			COMPR_PLAYBACK_MIN_FRAGMENT_SIZE;
	prtd->compr_cap.max_fragment_size =
			COMPR_PLAYBACK_MAX_FRAGMENT_SIZE;
	prtd->compr_cap.min_fragments =
			COMPR_PLAYBACK_MIN_NUM_FRAGMENTS;
	prtd->compr_cap.max_fragments =
			COMPR_PLAYBACK_MAX_NUM_FRAGMENTS;
	prtd->compr_cap.num_codecs = 3;
	prtd->compr_cap.codecs[0] = SND_AUDIOCODEC_MP3;
	prtd->compr_cap.codecs[1] = SND_AUDIOCODEC_AAC;
	prtd->compr_cap.codecs[2] = SND_AUDIOCODEC_PCM;
	prtd->compr_cap.codecs[3] = SND_AUDIOCODEC_FLAC;
	prtd->compr_cap.codecs[4] = SND_AUDIOCODEC_WMA;
	prtd->compr_cap.codecs[5] = SND_AUDIOCODEC_WMA_PRO;
}

static int msm_compr_send_media_format_block(
	struct snd_compr_stream *cstream)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct msm_compr_audio *prtd = runtime->private_data;
	struct asm_aac_cfg aac_cfg;
	struct asm_flac_cfg flac_cfg;
	struct asm_wma_cfg wma_cfg;
	struct asm_wmapro_cfg wma_pro_cfg;
	int ret = 0;
	uint16_t bit_width = 16;

	switch (prtd->codec) {
	case FORMAT_LINEAR_PCM:
		pr_debug("SND_AUDIOCODEC_PCM\n");
		if (prtd->codec_param.codec.format == SNDRV_PCM_FORMAT_S24_LE)
			bit_width = 24;
		ret = q6asm_media_format_block_pcm_v2(prtd->audio_client,
						      prtd->sample_rate,
						      prtd->num_channels,
						      bit_width);
		if (ret < 0)
			pr_err("%s: CMD Format block failed\n", __func__);

		break;
	case FORMAT_MP3:
		pr_debug("SND_AUDIOCODEC_MP3\n");
		/* no media format block needed */
		break;
	case FORMAT_MPEG4_MULTI_AAC:
		pr_debug("SND_AUDIOCODEC_AAC\n");
		memset(&aac_cfg, 0x0, sizeof(struct asm_aac_cfg));
		aac_cfg.aot = AAC_ENC_MODE_EAAC_P;
		if (prtd->codec_param.codec.format ==
					SND_AUDIOSTREAMFORMAT_MP4ADTS)
			aac_cfg.format = 0x0;
		else
			aac_cfg.format = 0x03;
		aac_cfg.ch_cfg = prtd->num_channels;
		aac_cfg.sample_rate = prtd->sample_rate;
		ret = q6asm_media_format_block_multi_aac(prtd->audio_client,
						   &aac_cfg);
		if (ret < 0)
			pr_err("%s: CMD Format block failed\n", __func__);
		break;
	case FORMAT_FLAC:
		pr_info("%s: SND_AUDIOCODEC_FLAC\n", __func__);
		memset(&flac_cfg, 0x0, sizeof(struct asm_flac_cfg));
		flac_cfg.num_channels = prtd->num_channels;
		flac_cfg.sample_rate = prtd->sample_rate;
		flac_cfg.is_stream_info_present = 1;
		flac_cfg.sample_size =
			prtd->codec_param.codec.options.flac_dec.sample_size;
		flac_cfg.min_blk_size =
			prtd->codec_param.codec.options.flac_dec.min_blk_size;
		flac_cfg.max_blk_size =
			prtd->codec_param.codec.options.flac_dec.max_blk_size;
		flac_cfg.max_frame_size =
			prtd->codec_param.codec.options.flac_dec.max_frame_size;
		flac_cfg.min_frame_size =
			prtd->codec_param.codec.options.flac_dec.min_frame_size;

		ret = q6asm_media_format_block_flac(prtd->audio_client,
							&flac_cfg);
		if (ret < 0)
			pr_err("%s: CMD Format block failed ret %d\n",
				__func__, ret);

		break;
	case FORMAT_WMA_V9:
		pr_debug("SND_AUDIOCODEC_WMA\n");
		memset(&wma_cfg, 0x0, sizeof(struct asm_wma_cfg));
		wma_cfg.format_tag = prtd->codec_param.codec.format;
		wma_cfg.ch_cfg = prtd->codec_param.codec.ch_in;
		wma_cfg.sample_rate = prtd->sample_rate;
		wma_cfg.avg_bytes_per_sec =
			prtd->codec_param.codec.bit_rate/8;
		wma_cfg.block_align =
			prtd->codec_param.codec.options.wma.super_block_align;
		wma_cfg.valid_bits_per_sample =
		prtd->codec_param.codec.options.wma.bits_per_sample;
		wma_cfg.ch_mask =
			prtd->codec_param.codec.options.wma.channelmask;
		wma_cfg.encode_opt =
			prtd->codec_param.codec.options.wma.encodeopt;
		ret = q6asm_media_format_block_wma(prtd->audio_client,
					&wma_cfg);
		if (ret < 0)
			pr_err("%s: CMD Format block failed\n", __func__);
		break;
	case FORMAT_WMA_V10PRO:
		pr_debug("SND_AUDIOCODEC_WMA_PRO\n");
		memset(&wma_pro_cfg, 0x0, sizeof(struct asm_wmapro_cfg));
		wma_pro_cfg.format_tag = prtd->codec_param.codec.format;
		wma_pro_cfg.ch_cfg = prtd->codec_param.codec.ch_in;
		wma_pro_cfg.sample_rate =
			prtd->sample_rate;
		wma_pro_cfg.avg_bytes_per_sec =
			prtd->codec_param.codec.bit_rate/8;
		wma_pro_cfg.block_align =
			prtd->codec_param.codec.options.wma.super_block_align;
		wma_pro_cfg.valid_bits_per_sample =
			prtd->codec_param.codec.options.wma.bits_per_sample;
		wma_pro_cfg.ch_mask =
			prtd->codec_param.codec.options.wma.channelmask;
		wma_pro_cfg.encode_opt =
			prtd->codec_param.codec.options.wma.encodeopt;
		wma_pro_cfg.adv_encode_opt =
			prtd->codec_param.codec.options.wma.encodeopt1;
		wma_pro_cfg.adv_encode_opt2 =
			prtd->codec_param.codec.options.wma.encodeopt2;
		ret = q6asm_media_format_block_wmapro(prtd->audio_client,
				&wma_pro_cfg);
		if (ret < 0)
			pr_err("%s: CMD Format block failed\n", __func__);
		break;

	default:
		pr_debug("%s, unsupported format, skip", __func__);
		break;
	}
	return ret;
}


static int msm_compr_configure_dsp(struct snd_compr_stream *cstream)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct msm_compr_audio *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *soc_prtd = cstream->private_data;
	uint16_t bits_per_sample = 16;
	int dir = IN, ret = 0;
	struct asm_softpause_params softpause = {
		.enable = SOFT_PAUSE_ENABLE,
		.period = SOFT_PAUSE_PERIOD,
		.step = SOFT_PAUSE_STEP,
		.rampingcurve = SOFT_PAUSE_CURVE_LINEAR,
	};
	struct asm_softvolume_params softvol = {
		.period = SOFT_VOLUME_PERIOD,
		.step = SOFT_VOLUME_STEP,
		.rampingcurve = SOFT_VOLUME_CURVE_LINEAR,
	};

	pr_debug("%s\n", __func__);

	if (prtd->codec_param.codec.format == SNDRV_PCM_FORMAT_S24_LE)
		bits_per_sample = 24;
	else if (prtd->codec_param.codec.format == SNDRV_PCM_FORMAT_S32_LE)
		bits_per_sample = 32;

	pr_debug("%s: bits_per_sample %d\n", __func__, bits_per_sample);

	ret = q6asm_open_write_v2(prtd->audio_client,
				prtd->codec, bits_per_sample);
	if (ret < 0) {
		pr_err("%s: Session out open failed\n", __func__);
		 return -ENOMEM;
	}

	pr_debug("%s be_id %d\n", __func__, soc_prtd->dai_link->be_id);
	msm_pcm_routing_reg_phy_stream(soc_prtd->dai_link->be_id,
				prtd->audio_client->perf_mode,
				prtd->session_id,
				SNDRV_PCM_STREAM_PLAYBACK);

	ret = msm_compr_set_volume(cstream, 0, 0);
	if (ret < 0)
		pr_err("%s : Set Volume failed : %d", __func__, ret);

	ret = q6asm_set_softpause(prtd->audio_client,
					&softpause);
	if (ret < 0)
		pr_err("%s: Send SoftPause Param failed ret=%d\n",
			__func__, ret);

	ret = q6asm_set_softvolume(prtd->audio_client, &softvol);
	if (ret < 0)
		pr_err("%s: Send SoftVolume Param failed ret=%d\n",
			__func__, ret);

	ret = q6asm_set_high_thd_resampler(prtd->audio_client, 1);
	if (ret < 0)
		pr_err("%s: Send HIGH_THD_RESAMPLER_ENABLE Param failed ret=%d\n",
			__func__, ret);

	ret = q6asm_set_io_mode(prtd->audio_client,
				(COMPRESSED_IO | ASYNC_IO_MODE));
	if (ret < 0) {
		pr_err("%s: Set IO mode failed\n", __func__);
		return -EINVAL;
	}

	runtime->fragments = prtd->codec_param.buffer.fragments;
	runtime->fragment_size = prtd->codec_param.buffer.fragment_size;
	pr_debug("allocate %d buffers each of size %d\n",
			runtime->fragments,
			runtime->fragment_size);
	ret = q6asm_audio_client_buf_alloc_contiguous(dir,
					prtd->audio_client,
					runtime->fragment_size,
					runtime->fragments);
	if (ret < 0) {
		pr_err("Audio Start: Buffer Allocation failed rc = %d\n", ret);
		return -ENOMEM;
	}

	prtd->byte_offset  = 0;
	prtd->copied_total = 0;
	prtd->app_pointer  = 0;
	prtd->bytes_received = 0;
	prtd->bytes_sent = 0;
	prtd->buffer       = prtd->audio_client->port[dir].buf[0].data;
	prtd->buffer_paddr = prtd->audio_client->port[dir].buf[0].phys;
	prtd->buffer_size  = runtime->fragments * runtime->fragment_size;

	ret = msm_compr_send_media_format_block(cstream);
	if (ret < 0)
		pr_err("%s, failed to send media format block\n", __func__);

	return ret;
}

static int msm_compr_open(struct snd_compr_stream *cstream)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct msm_compr_audio *prtd;
	struct msm_compr_pdata *pdata =
			snd_soc_platform_get_drvdata(rtd->platform);

	pr_debug("%s\n", __func__);
	prtd = kzalloc(sizeof(struct msm_compr_audio), GFP_KERNEL);
	if (prtd == NULL) {
		pr_err("Failed to allocate memory for msm_compr_audio\n");
		return -ENOMEM;
	}

	prtd->cstream = cstream;
	pdata->cstream[rtd->dai_link->be_id] = cstream;
	prtd->audio_client = q6asm_audio_client_alloc(
				(app_cb)compr_event_handler, prtd);
	if (!prtd->audio_client) {
		pr_err("%s: Could not allocate memory\n", __func__);
		kfree(prtd);
		return -ENOMEM;
	}

	pr_debug("%s: session ID %d\n", __func__, prtd->audio_client->session);
	prtd->audio_client->perf_mode = false;
	prtd->session_id = prtd->audio_client->session;
	prtd->codec = FORMAT_MP3;
	prtd->bytes_received = 0;
	prtd->bytes_sent = 0;
	prtd->copied_total = 0;
	prtd->byte_offset = 0;
	prtd->sample_rate = 44100;
	prtd->num_channels = 2;
	prtd->drain_ready = 0;

	spin_lock_init(&prtd->lock);

	atomic_set(&prtd->eos, 0);
	atomic_set(&prtd->start, 0);
	atomic_set(&prtd->drain, 0);
	atomic_set(&prtd->xrun, 0);
	atomic_set(&prtd->error, 0);

	init_waitqueue_head(&prtd->eos_wait);
	init_waitqueue_head(&prtd->drain_wait);

	runtime->private_data = prtd;
	populate_codec_list(prtd);

	return 0;
}

static int msm_compr_free(struct snd_compr_stream *cstream)
{
	struct snd_compr_runtime *runtime;
	struct msm_compr_audio *prtd;
	struct snd_soc_pcm_runtime *soc_prtd;
	struct msm_compr_pdata *pdata;
	struct audio_client *ac;
	int dir = IN, ret = 0;

	pr_debug("%s\n", __func__);
	if (!cstream) {
		pr_err("%s cstream is null\n", __func__);
		return 0;
	}
	runtime = cstream->runtime;
	soc_prtd = cstream->private_data;
	if (!runtime || !soc_prtd || !(soc_prtd->platform)) {
		pr_err("%s runtime or soc_prtd or platform is null\n",
			__func__);
		return 0;
	}
	prtd = runtime->private_data;
	if (!prtd) {
		pr_err("%s prtd is null\n", __func__);
		return 0;
	}
	pdata = snd_soc_platform_get_drvdata(soc_prtd->platform);
	ac = prtd->audio_client;
	if (!pdata || !ac) {
		pr_err("%s pdata or ac is null\n", __func__);
		return 0;
	}
	if (atomic_read(&prtd->eos)) {
		ret = wait_event_timeout(prtd->eos_wait,
					 prtd->eos_ack, 5 * HZ);
		if (!ret)
			pr_err("%s: CMD_EOS failed\n", __func__);
	}
	pdata->cstream[soc_prtd->dai_link->be_id] = NULL;
	if (cstream->direction == SND_COMPRESS_PLAYBACK) {
		msm_pcm_routing_dereg_phy_stream(soc_prtd->dai_link->be_id,
						SNDRV_PCM_STREAM_PLAYBACK);
	}

	q6asm_cmd(prtd->audio_client, CMD_CLOSE);

	q6asm_audio_client_buf_free_contiguous(dir,
					prtd->audio_client);

	q6asm_audio_client_free(prtd->audio_client);

	kfree(prtd);

	return 0;
}

/* compress stream operations */
static int msm_compr_set_params(struct snd_compr_stream *cstream,
				struct snd_compr_params *params)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct msm_compr_audio *prtd = runtime->private_data;
	int ret = 0;

	pr_debug("%s\n", __func__);

	memcpy(&prtd->codec_param, params, sizeof(struct snd_compr_params));

	/* ToDo: remove duplicates */
	prtd->num_channels = prtd->codec_param.codec.ch_in;

	switch (prtd->codec_param.codec.sample_rate) {
	case SNDRV_PCM_RATE_8000:
		prtd->sample_rate = 8000;
		break;
	case SNDRV_PCM_RATE_11025:
		prtd->sample_rate = 11025;
		break;
	/* ToDo: What about 12K and 24K sample rates ? */
	case SNDRV_PCM_RATE_16000:
		prtd->sample_rate = 16000;
		break;
	case SNDRV_PCM_RATE_22050:
		prtd->sample_rate = 22050;
		break;
	case SNDRV_PCM_RATE_32000:
		prtd->sample_rate = 32000;
		break;
	case SNDRV_PCM_RATE_44100:
		prtd->sample_rate = 44100;
		break;
	case SNDRV_PCM_RATE_48000:
		prtd->sample_rate = 48000;
		break;
	case SNDRV_PCM_RATE_64000:
		prtd->sample_rate = 64000;
		break;
	case SNDRV_PCM_RATE_88200:
		prtd->sample_rate = 88200;
		break;
	case SNDRV_PCM_RATE_96000:
		prtd->sample_rate = 96000;
		break;
	case SNDRV_PCM_RATE_176400:
		prtd->sample_rate = 176400;
		break;
	case SNDRV_PCM_RATE_192000:
		prtd->sample_rate = 192000;
		break;
	}

	pr_debug("%s: sample_rate %d\n", __func__, prtd->sample_rate);

	switch (params->codec.id) {
	case SND_AUDIOCODEC_PCM: {
		pr_debug("SND_AUDIOCODEC_PCM\n");
		prtd->codec = FORMAT_LINEAR_PCM;
		break;
	}

	case SND_AUDIOCODEC_MP3: {
		pr_debug("SND_AUDIOCODEC_MP3\n");
		prtd->codec = FORMAT_MP3;
		break;
	}

	case SND_AUDIOCODEC_AAC: {
		pr_debug("SND_AUDIOCODEC_AAC\n");
		prtd->codec = FORMAT_MPEG4_MULTI_AAC;
		break;
	}

	case SND_AUDIOCODEC_FLAC: {
		pr_debug("SND_AUDIOCODEC_FLAC\n");
		prtd->codec = FORMAT_FLAC;
		break;
	}

	case SND_AUDIOCODEC_WMA: {
		pr_debug("SND_AUDIOCODEC_WMA\n");
		prtd->codec = FORMAT_WMA_V9;
		break;
	}

	case SND_AUDIOCODEC_WMA_PRO: {
		pr_debug("SND_AUDIOCODEC_WMA_PRO\n");
		prtd->codec = FORMAT_WMA_V10PRO;
		break;
	}

	default:
		pr_err("codec not supported, id =%d\n", params->codec.id);
		return -EINVAL;
	}

	ret = msm_compr_configure_dsp(cstream);

	return ret;
}

static int msm_compr_drain_buffer(struct msm_compr_audio *prtd,
				  unsigned long *flags)
{
	int rc = 0;

	atomic_set(&prtd->drain, 1);
	prtd->drain_ready = 0;
	spin_unlock_irqrestore(&prtd->lock, *flags);
	pr_debug("%s: wait for buffer to be drained\n",  __func__);
	rc = wait_event_interruptible(prtd->drain_wait,
					prtd->drain_ready ||
					prtd->cmd_interrupt ||
					atomic_read(&prtd->xrun) ||
					atomic_read(&prtd->error));
	pr_debug("%s: out of buffer drain wait with ret %d\n", __func__, rc);
	spin_lock_irqsave(&prtd->lock, *flags);
	if (prtd->cmd_interrupt) {
		pr_debug("%s: buffer drain interrupted by flush)\n", __func__);
		rc = -EINTR;
		prtd->cmd_interrupt = 0;
	}
	if (atomic_read(&prtd->error)) {
		pr_err("%s: Got RESET EVENTS notification, return\n",
			__func__);
		rc = -ENETRESET;
	}
	return rc;
}

static int msm_compr_trigger(struct snd_compr_stream *cstream, int cmd)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct msm_compr_audio *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct msm_compr_pdata *pdata =
			snd_soc_platform_get_drvdata(rtd->platform);
	uint32_t *volume = pdata->volume[rtd->dai_link->be_id];
	int rc = 0;
	int bytes_to_write;
	unsigned long flags;

	if (cstream->direction != SND_COMPRESS_PLAYBACK) {
		pr_err("%s: Unsupported stream type\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&prtd->lock, flags);
	if (atomic_read(&prtd->error)) {
		pr_err("%s Got RESET EVENTS notification, return immediately",
			__func__);
		spin_unlock_irqrestore(&prtd->lock, flags);
		return 0;
	}
	spin_unlock_irqrestore(&prtd->lock, flags);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		pr_debug("%s: SNDRV_PCM_TRIGGER_START\n", __func__);
		atomic_set(&prtd->start, 1);

		/* set volume for the stream before RUN */
		rc = msm_compr_set_volume(cstream, volume[0], volume[1]);
		if (rc)
			pr_err("%s : Set Volume failed : %d\n",
				__func__, rc);
		/* issue RUN command for the stream */
		q6asm_run_nowait(prtd->audio_client, 0, 0, 0);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		spin_lock_irqsave(&prtd->lock, flags);
		pr_debug("%s: SNDRV_PCM_TRIGGER_STOP\n", __func__);
		atomic_set(&prtd->start, 0);
		if (atomic_read(&prtd->eos)) {
			pr_debug("%s: interrupt eos wait queues", __func__);
			prtd->cmd_interrupt = 1;
			wake_up(&prtd->eos_wait);
			atomic_set(&prtd->eos, 0);
		}
		if (atomic_read(&prtd->drain)) {
			pr_debug("%s: interrupt drain wait queues", __func__);
			prtd->cmd_interrupt = 1;
			prtd->drain_ready = 1;
			wake_up(&prtd->drain_wait);
			atomic_set(&prtd->drain, 0);
		}
		prtd->cmd_ack = 0;
		pr_debug("issue CMD_FLUSH\n");
		spin_unlock_irqrestore(&prtd->lock, flags);
		rc = q6asm_cmd(prtd->audio_client, CMD_FLUSH);
		if (rc < 0) {
			pr_err("%s: flush cmd failed rc=%d\n",
				__func__, rc);
			return rc;
		}
		spin_lock_irqsave(&prtd->lock, flags);
		prtd->byte_offset  = 0;
		prtd->copied_total = 0;
		prtd->app_pointer  = 0;
		prtd->bytes_received = 0;
		prtd->bytes_sent = 0;
		prtd->marker_timestamp = 0;

		atomic_set(&prtd->xrun, 0);
		spin_unlock_irqrestore(&prtd->lock, flags);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		pr_debug("SNDRV_PCM_TRIGGER_PAUSE_PUSH\n");
		q6asm_cmd_nowait(prtd->audio_client, CMD_PAUSE);
		atomic_set(&prtd->start, 0);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		pr_debug("SNDRV_PCM_TRIGGER_PAUSE_RELEASE\n");
		atomic_set(&prtd->start, 1);
		q6asm_run_nowait(prtd->audio_client, 0, 0, 0);
		break;
	case SND_COMPR_TRIGGER_PARTIAL_DRAIN:
		pr_debug("%s: SND_COMPR_TRIGGER_PARTIAL_DRAIN\n", __func__);
	case SND_COMPR_TRIGGER_DRAIN:
		pr_debug("%s: SNDRV_COMPRESS_DRAIN\n", __func__);
		/* Make sure all the data is sent to DSP before sending EOS */
		spin_lock_irqsave(&prtd->lock, flags);

		if (!atomic_read(&prtd->start)) {
			pr_err("%s: stream is not in started state\n",
				__func__);
			rc = -EPERM;
			spin_unlock_irqrestore(&prtd->lock, flags);
			break;
		}
		if (prtd->bytes_received > prtd->copied_total) {
			pr_debug("%s: wait till all the data is sent to dsp\n",
				__func__);
			rc = msm_compr_drain_buffer(prtd, &flags);
			if (rc || !atomic_read(&prtd->start)) {
				if (rc != -ENETRESET)
					rc = -EINTR;
				spin_unlock_irqrestore(&prtd->lock, flags);
				break;
			}
			bytes_to_write = prtd->bytes_received
						- prtd->copied_total;
			WARN(bytes_to_write > runtime->fragment_size,
			     "last write %d cannot be > than fragment_size",
			     bytes_to_write);

			if (bytes_to_write > 0) {
				pr_debug("%s: send %d partial bytes at the end",
				       __func__, bytes_to_write);
				atomic_set(&prtd->xrun, 0);
				msm_compr_send_buffer(prtd);
			}
		}


		prtd->eos_ack = 0;
		atomic_set(&prtd->eos, 1);
		pr_debug("%s: CMD_EOS\n", __func__);
		q6asm_cmd_nowait(prtd->audio_client, CMD_EOS);
		spin_unlock_irqrestore(&prtd->lock, flags);


		/* Wait indefinitely for  DRAIN. Flush can also signal this*/
		rc = wait_event_interruptible(prtd->eos_wait,
						(prtd->eos_ack ||
						prtd->cmd_interrupt ||
						atomic_read(&prtd->error)));

		if (rc < 0)
			pr_err("%s: EOS wait failed\n", __func__);

		pr_debug("%s: SNDRV_COMPRESS_DRAIN  out of wait for EOS\n",
			  __func__);

		if (prtd->cmd_interrupt)
			rc = -EINTR;

		if (atomic_read(&prtd->error)) {
			pr_err("%s: Got RESET EVENTS notification, return\n",
				__func__);
			rc = -ENETRESET;
		}

		if (rc == 0) {
			spin_lock_irqsave(&prtd->lock, flags);
			q6asm_cmd_nowait(prtd->audio_client, CMD_PAUSE);
			prtd->cmd_ack = 0;
			spin_unlock_irqrestore(&prtd->lock, flags);

			/*
			 * Cache this time as last known time
			 */
			q6asm_get_session_time(prtd->audio_client,
					       &prtd->marker_timestamp);
			spin_lock_irqsave(&prtd->lock, flags);
			prtd->byte_offset = 0;
			prtd->app_pointer  = 0;
			atomic_set(&prtd->drain, 0);
			atomic_set(&prtd->xrun, 1);
			spin_unlock_irqrestore(&prtd->lock, flags);
			q6asm_cmd(prtd->audio_client, CMD_FLUSH);
			q6asm_run_nowait(prtd->audio_client, 0, 0, 0);
		}
		prtd->cmd_interrupt = 0;
		break;
	case SND_COMPR_TRIGGER_NEXT_TRACK:
		pr_debug("%s: SND_COMPR_TRIGGER_NEXT_TRACK\n", __func__);
		break;
	}

	return 0;
}

static int msm_compr_pointer(struct snd_compr_stream *cstream,
				struct snd_compr_tstamp *arg)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct msm_compr_audio *prtd = runtime->private_data;
	struct snd_compr_tstamp tstamp;
	uint64_t timestamp = 0;
	unsigned long flags;

	pr_debug("%s\n", __func__);
	memset(&tstamp, 0x0, sizeof(struct snd_compr_tstamp));

	spin_lock_irqsave(&prtd->lock, flags);
	tstamp.sampling_rate = prtd->sample_rate;
	tstamp.byte_offset = prtd->byte_offset;
	tstamp.copied_total = prtd->copied_total;
	if (atomic_read(&prtd->error)) {
		pr_err("%s Got RESET EVENTS notification, return error",
			__func__);
		tstamp.pcm_io_frames = 0;
		memcpy(arg, &tstamp, sizeof(struct snd_compr_tstamp));
		spin_unlock_irqrestore(&prtd->lock, flags);
		return -ENETRESET;
	}

	spin_unlock_irqrestore(&prtd->lock, flags);
	q6asm_get_session_time(prtd->audio_client, &prtd->marker_timestamp);
	timestamp = prtd->marker_timestamp;

	/* DSP returns timestamp in usec */
	pr_debug("%s: timestamp = %lld usec\n", __func__, timestamp);
	timestamp *= prtd->sample_rate;
	tstamp.pcm_io_frames = (snd_pcm_uframes_t)div64_u64(timestamp, 1000000);
	memcpy(arg, &tstamp, sizeof(struct snd_compr_tstamp));

	return 0;
}

static int msm_compr_ack(struct snd_compr_stream *cstream,
			size_t count)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct msm_compr_audio *prtd = runtime->private_data;
	void *src, *dstn;
	size_t copy;
	unsigned long flags;

	WARN(1, "This path is untested");
	return -EINVAL;

	pr_debug("%s: count = %d\n", __func__, count);
	if (!prtd->buffer) {
		pr_err("%s: Buffer is not allocated yet ??\n", __func__);
		return -EINVAL;
	}
	src = runtime->buffer + prtd->app_pointer;
	dstn = prtd->buffer + prtd->app_pointer;
	if (count < prtd->buffer_size - prtd->app_pointer) {
		memcpy(dstn, src, count);
		prtd->app_pointer += count;
	} else {
		copy = prtd->buffer_size - prtd->app_pointer;
		memcpy(dstn, src, copy);
		memcpy(prtd->buffer, runtime->buffer, count - copy);
		prtd->app_pointer = count - copy;
	}

	/*
	 * If the stream is started and all the bytes received were
	 * copied to DSP, the newly received bytes should be
	 * sent right away
	 */
	spin_lock_irqsave(&prtd->lock, flags);

	if (atomic_read(&prtd->start) &&
		prtd->bytes_received == prtd->copied_total) {
		prtd->bytes_received += count;
		msm_compr_send_buffer(prtd);
	} else
		prtd->bytes_received += count;

	spin_unlock_irqrestore(&prtd->lock, flags);

	return 0;
}

static int msm_compr_copy(struct snd_compr_stream *cstream,
				char __user *buf, size_t count)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct msm_compr_audio *prtd = runtime->private_data;
	void *dstn;
	size_t copy;
	size_t bytes_available = 0;
	unsigned long flags;

	pr_debug("%s: count = %d\n", __func__, count);
	if (!prtd->buffer) {
		pr_err("%s: Buffer is not allocated yet ??", __func__);
		return 0;
	}

	spin_lock_irqsave(&prtd->lock, flags);
	if (atomic_read(&prtd->error)) {
		pr_err("%s Got RESET EVENTS notification", __func__);
		spin_unlock_irqrestore(&prtd->lock, flags);
		return -ENETRESET;
	}
	spin_unlock_irqrestore(&prtd->lock, flags);

	dstn = prtd->buffer + prtd->app_pointer;
	if (count < prtd->buffer_size - prtd->app_pointer) {
		if (copy_from_user(dstn, buf, count))
			return -EFAULT;
		prtd->app_pointer += count;
	} else {
		copy = prtd->buffer_size - prtd->app_pointer;
		if (copy_from_user(dstn, buf, copy))
			return -EFAULT;
		if (copy_from_user(prtd->buffer, buf + copy, count - copy))
			return -EFAULT;
		prtd->app_pointer = count - copy;
	}

	/*
	 * If stream is started and there has been an xrun,
	 * since the available bytes fits fragment_size,
	 * copy the data right away
	 */
	spin_lock_irqsave(&prtd->lock, flags);
	prtd->bytes_received += count;
	if (atomic_read(&prtd->start)) {
		if (atomic_read(&prtd->xrun)) {
			pr_debug("%s: in xrun, count = %zd\n", __func__, count);
			bytes_available = prtd->bytes_received -
					  prtd->copied_total;
			if (bytes_available >= runtime->fragment_size) {
				pr_debug("%s: handle xrun, bytes_to_write = %zd\n",
					 __func__,
					 bytes_available);
				atomic_set(&prtd->xrun, 0);
				msm_compr_send_buffer(prtd);
			} /* else not sufficient data */
		} /* writes will continue on the next write_done */
	}

	spin_unlock_irqrestore(&prtd->lock, flags);

	return count;
}

static int msm_compr_get_caps(struct snd_compr_stream *cstream,
				struct snd_compr_caps *arg)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct msm_compr_audio *prtd = runtime->private_data;
	int ret = 0;

	pr_debug("%s\n", __func__);
	if ((arg != NULL) && (prtd != NULL)) {
		memcpy(arg, &prtd->compr_cap, sizeof(struct snd_compr_caps));
	} else {
		ret = -EINVAL;
		pr_err("%s: arg (0x%p), prtd (0x%p)\n", __func__, arg, prtd);
	}

	return ret;
}

static int msm_compr_get_codec_caps(struct snd_compr_stream *cstream,
				struct snd_compr_codec_caps *codec)
{
	pr_debug("%s\n", __func__);

	switch (codec->codec) {
	case SND_AUDIOCODEC_MP3:
		codec->num_descriptors = 2;
		codec->descriptor[0].max_ch = 2;
		codec->descriptor[0].sample_rates = SNDRV_PCM_RATE_8000_48000;
		codec->descriptor[0].bit_rate[0] = 320; /* 320kbps */
		codec->descriptor[0].bit_rate[1] = 128;
		codec->descriptor[0].num_bitrates = 2;
		codec->descriptor[0].profiles = 0;
		codec->descriptor[0].modes = SND_AUDIOCHANMODE_MP3_STEREO;
		codec->descriptor[0].formats = 0;
		break;
	case SND_AUDIOCODEC_AAC:
		codec->num_descriptors = 2;
		codec->descriptor[1].max_ch = 2;
		codec->descriptor[1].sample_rates = SNDRV_PCM_RATE_8000_48000;
		codec->descriptor[1].bit_rate[0] = 320; /* 320kbps */
		codec->descriptor[1].bit_rate[1] = 128;
		codec->descriptor[1].num_bitrates = 2;
		codec->descriptor[1].profiles = 0;
		codec->descriptor[1].modes = 0;
		codec->descriptor[1].formats =
			(SND_AUDIOSTREAMFORMAT_MP4ADTS |
				SND_AUDIOSTREAMFORMAT_RAW);
		break;
	case SND_AUDIOCODEC_FLAC:
		break;
	default:
		pr_err("%s: Unsupported audio codec %d\n",
			__func__, codec->codec);
		return -EINVAL;
	}

	return 0;
}

static int msm_compr_set_metadata(struct snd_compr_stream *cstream,
				struct snd_compr_metadata *metadata)
{
	pr_debug("%s\n", __func__);
	return -ENXIO;
}

static int msm_compr_volume_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_platform *platform = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct msm_compr_pdata *pdata = (struct msm_compr_pdata *)
			snd_soc_platform_get_drvdata(platform);
	struct snd_compr_stream *cstream = pdata->cstream[mc->reg];
	uint32_t *volume = pdata->volume[mc->reg];

	volume[0] = ucontrol->value.integer.value[0];
	volume[1] = ucontrol->value.integer.value[1];
	pr_debug("%s: mc->reg %d left_vol %d right_vol %d\n",
		__func__, mc->reg, volume[0], volume[1]);
	if (cstream)
		msm_compr_set_volume(cstream, volume[0], volume[1]);
	return 0;
}

static int msm_compr_volume_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_platform *platform = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct msm_compr_pdata *pdata =
		snd_soc_platform_get_drvdata(platform);
	uint32_t *volume = pdata->volume[mc->reg];
	pr_debug("%s: mc->reg %d\n", __func__, mc->reg);
	ucontrol->value.integer.value[0] = volume[0];
	ucontrol->value.integer.value[1] = volume[1];

	return 0;
}

/* System Pin has no volume control */
static const struct snd_kcontrol_new msm_compr_volume_controls[] = {
	SOC_DOUBLE_EXT_TLV("Compress Playback Volume",
			MSM_FRONTEND_DAI_MULTIMEDIA4,
			0, 8, COMPRESSED_LR_VOL_MAX_STEPS, 0,
			msm_compr_volume_get,
			msm_compr_volume_put,
			msm_compr_vol_gain),
	SOC_DOUBLE_EXT_TLV("RES Compress Playback Volume",
			MSM_FRONTEND_DAI_MULTIMEDIA8,
			0, 8, COMPRESSED_LR_VOL_MAX_STEPS, 0,
			msm_compr_volume_get,
			msm_compr_volume_put,
			msm_compr_vol_gain),
};

static int msm_compr_probe(struct snd_soc_platform *platform)
{
	struct msm_compr_pdata *pdata;
	int i;

	pr_debug("%s\n", __func__);
	pdata = (struct msm_compr_pdata *)
			kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	snd_soc_platform_set_drvdata(platform, pdata);

	for (i = 0; i < MSM_FRONTEND_DAI_MAX; i++) {
		pdata->volume[i][0] = COMPRESSED_LR_VOL_MAX_STEPS;
		pdata->volume[i][1] = COMPRESSED_LR_VOL_MAX_STEPS;
		pdata->cstream[i] = NULL;
	}

	return 0;
}

static struct snd_compr_ops msm_compr_ops = {
	.open		= msm_compr_open,
	.free		= msm_compr_free,
	.trigger	= msm_compr_trigger,
	.pointer	= msm_compr_pointer,
	.set_params	= msm_compr_set_params,
	.set_metadata	= msm_compr_set_metadata,
	.ack		= msm_compr_ack,
	.copy		= msm_compr_copy,
	.get_caps	= msm_compr_get_caps,
	.get_codec_caps = msm_compr_get_codec_caps,
};

static struct snd_soc_platform_driver msm_soc_platform = {
	.probe		= msm_compr_probe,
	.compr_ops	= &msm_compr_ops,
	.controls	= msm_compr_volume_controls,
	.num_controls	= ARRAY_SIZE(msm_compr_volume_controls),
};

static __devinit int msm_compr_dev_probe(struct platform_device *pdev)
{
	if (pdev->dev.of_node)
		dev_set_name(&pdev->dev, "%s", "msm-compress-dsp");

	pr_debug("%s: dev name %s\n", __func__, dev_name(&pdev->dev));
	return snd_soc_register_platform(&pdev->dev,
					&msm_soc_platform);
}

static int msm_compr_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static const struct of_device_id msm_compr_dt_match[] = {
	{.compatible = "qcom,msm-compress-dsp"},
	{}
};
MODULE_DEVICE_TABLE(of, msm_compr_dt_match);

static struct platform_driver msm_compr_driver = {
	.driver = {
		.name = "msm-compress-dsp",
		.owner = THIS_MODULE,
		.of_match_table = msm_compr_dt_match,
	},
	.probe = msm_compr_dev_probe,
	.remove = __devexit_p(msm_compr_remove),
};

static int __init msm_soc_platform_init(void)
{
	return platform_driver_register(&msm_compr_driver);
}
module_init(msm_soc_platform_init);

static void __exit msm_soc_platform_exit(void)
{
	platform_driver_unregister(&msm_compr_driver);
}
module_exit(msm_soc_platform_exit);

MODULE_DESCRIPTION("Compress Offload platform driver");
MODULE_LICENSE("GPL v2");
