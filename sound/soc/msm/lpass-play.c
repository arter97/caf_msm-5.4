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

#include <linux/file.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/fb.h>
#include <linux/types.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/io.h>
#include <linux/soundcard.h>
#include <sound/core.h>
#include <sound/minors.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <sound/info.h>
#include <sound/asound.h>

#define ALSA_HW_MINOR    2
#define WAV_PLAY_FILE	 "notification.wav"

struct alsa_play_data {
	int wav_file_fd;
	struct file *dummy_file;
	struct snd_pcm *pcm;
	struct snd_pcm_substream *substream;
	char *buffer;
	int count;
	int valid;
	wait_queue_head_t w_wait;
	struct task_struct *playback_thread;
};

static struct alsa_play_data *play_data;

static void release_resources(void)
{
	sys_close(play_data->wav_file_fd);
	if (play_data->substream)
		snd_pcm_release_substream(play_data->substream);
	kfree(play_data->buffer);
	kfree(play_data);
}

static int snd_pcm_open_by_minor(unsigned int minor,
		struct alsa_play_data *play_data)
{
	struct snd_pcm  *pcm;
	pcm = snd_lookup_minor_data(minor,
			SNDRV_DEVICE_TYPE_PCM_PLAYBACK);
	if (!pcm)
		return -EBADFD;

	play_data->pcm = pcm;
	return 0;
}

static int snd_pcm_open_stream(struct alsa_play_data *play_data)
{
	int idx, err = 0;
	struct snd_pcm_substream *substream;

	play_data->dummy_file = kmalloc(sizeof(struct file), GFP_KERNEL);
	if (play_data->dummy_file == NULL) {
		err = -ENOMEM;
		goto out;
	}

	play_data->dummy_file->f_flags &= ~O_APPEND;

	for (idx = 0; idx < 2; idx++) {
		if (!(play_data->pcm->streams[idx].substream_count))
			continue;

		if (idx != SNDRV_PCM_STREAM_PLAYBACK)
			continue;
		/* TODO: do not need to check file permission,
		 * dummy file will be used here
		 */
		err = snd_pcm_open_substream(play_data->pcm, idx,
				play_data->dummy_file, &substream);
		if (err < 0)
			goto free_buffer;

		substream->f_flags = ~O_NONBLOCK;
		play_data->substream = substream;
	}

	return 0;

free_buffer:
	kfree(play_data->dummy_file);
out:
	return err;
}

static void _snd_pcm_hw_param_set(struct snd_pcm_hw_params *params,
				snd_pcm_hw_param_t var, unsigned int val,
				int flags)
{
	if (hw_is_mask(var)) {
		struct snd_mask *m = hw_param_mask(params, var);
		m->bits[0] = 0;
		m->bits[1] = 0;
		m->bits[val >> 5] |= (1 << (val & 31));
	} else if (hw_is_interval(var)) {
		struct snd_interval *i = hw_param_interval(params, var);
		if (flags & 0x01) {	/*min */
			i->min = val;
		} else if (flags & 0x02) {	/*max */
			i->max = val;
		} else {	/* both min and max */
			i->min = val;
			i->max = val;
			i->integer = 1;
		}
	} else

	params->cmask |= 1 << var;
	params->rmask |= 1 << var;
}

static int snd_pcm_set_params(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_pcm_hw_params *hw_params;
	struct snd_pcm_sw_params *sw_params;
	int err;

	hw_params = kmalloc(sizeof(*hw_params), GFP_KERNEL);
	sw_params = kmalloc(sizeof(*sw_params), GFP_KERNEL);

	if (!hw_params || !sw_params) {
		pr_err("No memory\n");
		err = -ENOMEM;
		goto failure;
	}

	_snd_pcm_hw_params_any(hw_params);
	_snd_pcm_hw_param_set(hw_params, SNDRV_PCM_HW_PARAM_ACCESS,
				SNDRV_PCM_ACCESS_RW_INTERLEAVED, 0);
	_snd_pcm_hw_param_set(hw_params, SNDRV_PCM_HW_PARAM_FORMAT,
				SNDRV_PCM_FORMAT_S16_LE, 0);
	_snd_pcm_hw_param_set(hw_params, SNDRV_PCM_HW_PARAM_SUBFORMAT,
				SNDRV_PCM_SUBFORMAT_STD, 0);
	_snd_pcm_hw_param_set(hw_params, SNDRV_PCM_HW_PARAM_CHANNELS,
				2, 0);
	_snd_pcm_hw_param_set(hw_params, SNDRV_PCM_HW_PARAM_RATE,
				48000, 0);
	_snd_pcm_hw_param_set(hw_params, SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
				1024, 1);
	_snd_pcm_hw_param_set(hw_params, SNDRV_PCM_HW_PARAM_PERIODS,
				4, 0);
	_snd_pcm_hw_param_set(hw_params, SNDRV_PCM_HW_PARAM_FRAME_BITS,
				32, 0);
	_snd_pcm_hw_param_set(hw_params, SNDRV_PCM_HW_PARAM_SAMPLE_BITS,
				16, 0);

	snd_pcm_kernel_ioctl(substream, SNDRV_PCM_IOCTL_DROP, NULL);

	err = snd_pcm_kernel_ioctl(substream, SNDRV_PCM_IOCTL_HW_PARAMS,
			hw_params);
	if (err < 0) {
		pr_err("HW param setting error\n");
		goto failure;
	}

	memset(sw_params, 0, sizeof(*sw_params));
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		sw_params->start_threshold = runtime->period_size;
		sw_params->stop_threshold = runtime->buffer_size;
	} else {
		sw_params->start_threshold = 1;
		sw_params->stop_threshold = runtime->buffer_size;
	}

	sw_params->tstamp_mode = SNDRV_PCM_TSTAMP_NONE;
	sw_params->period_step = 1;
	sw_params->avail_min = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
					1 : runtime->period_size;

	err = snd_pcm_kernel_ioctl(substream, SNDRV_PCM_IOCTL_SW_PARAMS,
			sw_params);
	if (err < 0) {
		pr_err("SW param setting error");
		goto failure;
	}

failure:
	kfree(hw_params);
	kfree(sw_params);
	return err;
}

static inline mm_segment_t snd_enter_user(void)
{
	mm_segment_t fs = get_fs();
	set_fs(get_ds());
	return fs;
}

static inline void snd_leave_user(mm_segment_t fs)
{
	set_fs(fs);
}

static int snd_pcm_write_file(struct snd_pcm_substream *substream,
		char *ptr, int count)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int remaining = count;
	int buffer_size = snd_pcm_lib_buffer_bytes(substream);
	snd_pcm_uframes_t send_size;
	char *pos = ptr;
	int size;
	int ret = 0;
	mm_segment_t fs;

	while (remaining) {
		if (!snd_pcm_running(substream)) {
			ret = snd_pcm_kernel_ioctl(substream,
					SNDRV_PCM_IOCTL_PREPARE, NULL);
			if (ret < 0)
				break;
		}

		if (remaining > buffer_size)
			size = buffer_size;
		else
			size = remaining;
		send_size = size / 4;

		fs = snd_enter_user();
		ret = snd_pcm_lib_write(substream,
				(void __force __user *)pos, send_size);
		snd_leave_user(fs);
		if (ret < 0)
			break;

		if (runtime->status->state == SNDRV_PCM_STATE_PREPARED)
			return -EAGAIN;

		remaining -= buffer_size;
		pos += size;

		if (remaining <= 0)
			break;
	}

	return ret;
}

static int playback_thread(void *data)
{
	int rc = 0;
	struct alsa_play_data *play_data = data;
	for (;;) {
		rc = wait_event_interruptible(play_data->w_wait,
				play_data->valid == 1);
		if (rc < 0)
			break;

		if (kthread_should_stop())
			break;
		pr_debug("start notification sound play\n");
		rc = snd_pcm_write_file(play_data->substream,
				play_data->buffer, play_data->count);

		if (rc < 0)
			break;
		release_resources();
		do_exit(0);
	}
	return rc;
}

static int load_wav_file(char *filename, struct alsa_play_data *play_data)
{
	int fd, count, err = 0;
	unsigned char *buffer;

	fd = sys_open(filename, O_RDONLY, 0);
	if (fd < 0) {
		pr_err("open file error\n");
		return -ENOENT;
	}

	count = sys_lseek(fd, (off_t)0, 2);
	if (count <= 0) {
		pr_err("file count to short\n");
		err = -EIO;
		goto out;
	}

	sys_lseek(fd, (off_t)0, 0);
	buffer = kmalloc(count, GFP_KERNEL);
	if (play_data->buffer) {
		pr_err("memory alloc error\n");
		err = -ENOMEM;
		goto out;
	}

	if (sys_read(fd, buffer, count) != count) {
		err = -EIO;
		goto out2;
	}

	play_data->wav_file_fd = fd;
	play_data->count = count;
	play_data->buffer = buffer;
	play_data->valid = 1;
	wake_up(&play_data->w_wait);
	return 0;
out2:
	kfree(play_data->buffer);
out:
	sys_close(fd);
	return err;
}

static int __init alsa_play_init(void)
{
	int ret;

	play_data = kmalloc(sizeof(*play_data), GFP_KERNEL);
	if (unlikely(play_data == NULL)) {
		ret = -ENOMEM;
		goto out;
	}

	memset(play_data, 0, sizeof(*play_data));

	ret = snd_pcm_open_by_minor((unsigned int)ALSA_HW_MINOR, play_data);
	if (ret < 0)
		goto free_buffer;

	ret = snd_pcm_open_stream(play_data);
	if (ret < 0)
		goto free_buffer;

	ret = snd_pcm_set_params(play_data->substream);
	if (ret < 0)
		goto release_substream;

	init_waitqueue_head(&play_data->w_wait);

	play_data->playback_thread =
		kthread_create(playback_thread, (void *)play_data, "alsa_play");
	if (IS_ERR(play_data->playback_thread)) {
		play_data->playback_thread = NULL;
		goto release_substream;
	}

	wake_up_process(play_data->playback_thread);

	ret = load_wav_file((char *)WAV_PLAY_FILE, play_data);
	if (ret < 0)
		goto release_kthread;

	return 0;

release_kthread:
	kthread_stop(play_data->playback_thread);
release_substream:
	snd_pcm_release_substream(play_data->substream);
free_buffer:
	kfree(play_data);
out:
	return ret;
}
late_initcall(alsa_play_init);

static void  __exit alsa_play_exit(void)
{
	if (play_data->playback_thread)
		;
	kthread_stop(play_data->playback_thread);
	release_resources();
}

module_exit(alsa_play_exit);

