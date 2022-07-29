// SPDX-License-Identifier: GPL-2.0
/*
 * Virtio RPMB Front End Driver
 *
 * Copyright (c) 2018-2019 Intel Corporation.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#include <linux/err.h>
#include <linux/scatterlist.h>
#include <linux/spinlock.h>
#include <linux/virtio.h>
#include <linux/module.h>
#include <linux/virtio_ids.h>
#include <linux/fs.h>
#include <linux/virtio_config.h>
#include <linux/uaccess.h>
#include <linux/rpmb.h>

/*
 * Set to get an OASIS virtio draft spec compliant virtio RPMB driver.
 * The macro may be removed later and treated as always been set.
 */
#define VIRTIO_RPMB_DRAFT_SPEC

static const char id[] = "RPMB:VIRTIO";
#ifndef VIRTIO_ID_RPMB
#define	VIRTIO_ID_RPMB		28
#endif

#define RPMB_SEQ_CMD_MAX 3  /* support up to 3 cmds */

#ifdef VIRTIO_RPMB_DRAFT_SPEC
struct virtio_rpmb_config {
	u8 capacity;
	u8 max_wr_cnt;
	u8 max_rd_cnt;
};
#endif

struct virtio_rpmb_info {
	struct virtqueue *vq;
	struct mutex lock; /* info lock */
	wait_queue_head_t have_data;
	struct rpmb_dev *rdev;
};

struct virtio_rpmb_ioc {
	unsigned int ioc_cmd;
	int result;
	u8 target;
	u8 reserved[3];
};

#ifdef DEBUG
static void virtio_rpmb_hexdump(const void *data, size_t length, size_t base)
{
	const unsigned int max_bytes_per_line = 16u;
	const char *cdata = data;
	char *bp;
	size_t offset = 0u;
	unsigned int per_line = max_bytes_per_line;
	unsigned int col;
	int c;
	char buf[32u + 4u * max_bytes_per_line];

	do {
		bp = &buf[0];
		bp += scnprintf(bp, PAGE_SIZE, "%zX", base + offset);
		if (length > 0u) {
			if (per_line > length)
				per_line = (unsigned int)length;
			for (col = 0u; col < per_line; col++) {
				c = cdata[offset + col] & 0xff;
				bp += scnprintf(bp, PAGE_SIZE, " %02X", c);
			}
			bp += scnprintf(bp, PAGE_SIZE, "");
			for (col = 0u; col < per_line; col++) {
				c = cdata[offset + col] & 0xff;
				if (c < 32 || c >=  127)
					c = '.';
				bp += scnprintf(bp, PAGE_SIZE, "%c", c);
			}

			offset += per_line;
			length -= per_line;
		}
		pr_debug("%s\n", buf);
	} while (length > 0u);
}
#endif

static void virtio_rpmb_recv_done(struct virtqueue *vq)
{
	struct virtio_rpmb_info *vi;
	struct virtio_device *vdev = vq->vdev;

	vi = vq->vdev->priv;
	if (!vi) {
		dev_err(&vdev->dev, "Error: no found vi data.\n");
		return;
	}

	wake_up(&vi->have_data);
}

static int rpmb_virtio_cmd_seq(struct device *dev, u8 target,
			       struct rpmb_cmd *cmds, u32 ncmds)
{
	struct virtio_device *vdev = dev_to_virtio(dev);
	struct virtio_rpmb_info *vi = vdev->priv;
	unsigned int i;
#ifndef VIRTIO_RPMB_DRAFT_SPEC
	struct virtio_rpmb_ioc *vio_cmd = NULL;
	struct rpmb_ioc_seq_cmd *seq_cmd = NULL;
	size_t seq_cmd_sz;
	struct scatterlist vio_ioc, vio_seq, frame[3];
	struct scatterlist *sgs[5];
#else
	struct scatterlist frame[3];
	struct scatterlist *sgs[3];
#endif
	unsigned int num_out = 0, num_in = 0;
	size_t sz;
	int ret;
	unsigned int len;
#ifdef DEBUG
	struct rpmb_frame_jdec *rpmb_frame_jdec;
	u16 req_type;
#endif

	if (ncmds > RPMB_SEQ_CMD_MAX)
		return -EINVAL;

	mutex_lock(&vi->lock);
	if (IS_ERR(vi->vq)) {
		ret = PTR_ERR(vi->vq);
		goto unlock_and_out;
	}

#ifndef VIRTIO_RPMB_DRAFT_SPEC
	vio_cmd = kzalloc(sizeof(*vio_cmd), GFP_KERNEL);
	seq_cmd_sz = sizeof(*seq_cmd) + sizeof(struct rpmb_ioc_cmd) * ncmds;
	seq_cmd = kzalloc(seq_cmd_sz, GFP_KERNEL);
	if (!vio_cmd || !seq_cmd) {
		ret = -ENOMEM;
		goto out;
	}

	vio_cmd->ioc_cmd = RPMB_IOC_SEQ_CMD;
	vio_cmd->result = 0;
	vio_cmd->target = target;
	sg_init_one(&vio_ioc, vio_cmd, sizeof(*vio_cmd));
	sgs[num_out++ + num_in] = &vio_ioc;
#ifdef DEBUG
	pr_debug("RPMB vio_cmd (size = %zu):\n", sizeof(*vio_cmd));
	virtio_rpmb_hexdump(vio_cmd, sizeof(*vio_cmd), 0u);
#endif

	pr_debug("ncmds = %u\n", ncmds);
	seq_cmd->num_of_cmds = ncmds;
	for (i = 0; i < ncmds; i++) {
		seq_cmd->cmds[i].flags   = cmds[i].flags;
		seq_cmd->cmds[i].nframes = cmds[i].nframes;
		seq_cmd->cmds[i].frames_ptr = i;
	}
	sg_init_one(&vio_seq, seq_cmd, seq_cmd_sz);
	sgs[num_out++ + num_in] = &vio_seq;
#ifdef DEBUG
	pr_debug("RPMB seq_cmd (size = %zu):\n", seq_cmd_sz);
	virtio_rpmb_hexdump(seq_cmd, seq_cmd_sz, 0u);
#endif
#endif /* #ifndef VIRTIO_RPMB_DRAFT_SPEC */

	for (i = 0; i < ncmds; i++) {
		sz = sizeof(struct rpmb_frame_jdec) * (cmds[i].nframes ?: 1);
		sg_init_one(&frame[i], cmds[i].frames, sz);

#ifdef DEBUG
		rpmb_frame_jdec = cmds[i].frames;
		req_type = be16_to_cpu(rpmb_frame_jdec->req_resp);
		pr_debug("req_type frame[%u] = 0x%04X\n", i, req_type);
		pr_debug("RPMB frame %u (size = %zu):\n", i, sz);
		virtio_rpmb_hexdump(cmds[i].frames, sz, 0u);
#endif

		if (cmds[i].flags & RPMB_F_WRITE)
			sgs[num_out++ + num_in] = &frame[i];
		else
			sgs[num_out + num_in++] = &frame[i];
	}

	virtqueue_add_sgs(vi->vq, sgs, num_out, num_in, vi, GFP_KERNEL);
	virtqueue_kick(vi->vq);

	wait_event(vi->have_data, virtqueue_get_buf(vi->vq, &len));

	ret = 0;

#ifndef VIRTIO_RPMB_DRAFT_SPEC
	if (vio_cmd->result != 0) {
		dev_err(dev, "Error: command error = %d.\n", vio_cmd->result);
		ret = -EIO;
	}

out:
	kfree(vio_cmd);
	kfree(seq_cmd);
#endif
unlock_and_out:
	mutex_unlock(&vi->lock);
	return ret;
}

#if 0 /* This function is *never* used, compiler complains */
static int rpmb_virtio_cmd_cap(struct device *dev, u8 target)
{
	struct virtio_device *vdev = dev_to_virtio(dev);
	struct virtio_rpmb_info *vi = vdev->priv;
	struct virtio_rpmb_ioc *vio_cmd;
	struct rpmb_ioc_cap_cmd *cap_cmd;
	struct scatterlist vio_ioc, cap_ioc;
	struct scatterlist *sgs[2];
	unsigned int num_out = 0, num_in = 0;
	unsigned int len;
	int ret;

	mutex_lock(&vi->lock);

	vio_cmd = kzalloc(sizeof(*vio_cmd), GFP_KERNEL);
	cap_cmd = kzalloc(sizeof(*cap_cmd), GFP_KERNEL);
	if (!vio_cmd || !cap_cmd) {
		ret = -ENOMEM;
		goto out;
	}

	vio_cmd->ioc_cmd = RPMB_IOC_CAP_CMD;
	vio_cmd->result = 0;
	vio_cmd->target = target;
	sg_init_one(&vio_ioc, vio_cmd, sizeof(*vio_cmd));
	sgs[num_out + num_in++] = &vio_ioc;

	sg_init_one(&cap_ioc, cap_cmd, sizeof(*cap_cmd));
	sgs[num_out + num_in++] = &cap_ioc;

	virtqueue_add_sgs(vi->vq, sgs, num_out, num_in, vi, GFP_KERNEL);
	virtqueue_kick(vi->vq);

	wait_event(vi->have_data, virtqueue_get_buf(vi->vq, &len));

	ret = 0;

	if (vio_cmd->result != 0) {
		dev_err(dev, "Error: command error = %d.\n", vio_cmd->result);
		ret = -EIO;
	}

out:
	kfree(vio_cmd);
	kfree(cap_cmd);

	mutex_unlock(&vi->lock);
	return ret;
}
#endif /* #if 0 */

static int rpmb_virtio_get_capacity(struct device *dev, u8 target)
{
#ifdef VIRTIO_RPMB_DRAFT_SPEC
	struct virtio_device *vdev = dev_to_virtio(dev);
	u8 capacity;

	virtio_cread(vdev, struct virtio_rpmb_config, capacity, &capacity);
	pr_debug("Config capacity = %u [128K units]\n", capacity);
	return capacity;
#else
	return 0;
#endif
}

static struct rpmb_ops rpmb_virtio_ops = {
	.cmd_seq = rpmb_virtio_cmd_seq,
	.get_capacity = rpmb_virtio_get_capacity,
	.type = RPMB_TYPE_EMMC,
	.auth_method = RPMB_HMAC_ALGO_SHA_256,
};

static int rpmb_virtio_dev_init(struct virtio_rpmb_info *vi)
{
	int ret = 0;
	struct device *dev = &vi->vq->vdev->dev;
#ifdef VIRTIO_RPMB_DRAFT_SPEC
	struct virtio_device *vdev = dev_to_virtio(dev);
	u8 max_wr_cnt, max_rd_cnt;
#endif

	rpmb_virtio_ops.dev_id_len = strlen(id);
	rpmb_virtio_ops.dev_id = id;
#ifdef VIRTIO_RPMB_DRAFT_SPEC
	virtio_cread(vdev, struct virtio_rpmb_config, max_wr_cnt, &max_wr_cnt);
	virtio_cread(vdev, struct virtio_rpmb_config, max_rd_cnt, &max_rd_cnt);
	/* With the coding below zero means unlimited as in the virtio spec */
	rpmb_virtio_ops.wr_cnt_max = max_wr_cnt;
	rpmb_virtio_ops.rd_cnt_max = max_rd_cnt;
#else
	rpmb_virtio_ops.wr_cnt_max = 1;
	rpmb_virtio_ops.rd_cnt_max = 1;
#endif
	rpmb_virtio_ops.block_size = 1;

	vi->rdev = rpmb_dev_register(dev, 0, &rpmb_virtio_ops);
	if (IS_ERR(vi->rdev)) {
		ret = PTR_ERR(vi->rdev);
		goto err;
	}

	dev_set_drvdata(dev, vi);
err:
	return ret;
}

static int virtio_rpmb_init_vq(struct virtio_device *vdev)
{
	struct virtio_rpmb_info *vi;

	vi = vdev->priv;
	if (!vi)
		return -EINVAL;

	vi->vq = virtio_find_single_vq(vdev, virtio_rpmb_recv_done, "request");
	if (IS_ERR(vi->vq)) {
		dev_err(&vdev->dev, "get single vq failed!\n");
		return PTR_ERR(vi->vq);
	}
	return 0;
}

static int virtio_rpmb_del_vq(struct virtio_device *vdev)
{
	struct virtio_rpmb_info *vi;

	vi = vdev->priv;
	if (!vi)
		return -EINVAL;

	if (vdev->config->reset)
		vdev->config->reset(vdev);

	if (vdev->config->del_vqs)
		vdev->config->del_vqs(vdev);

	vi->vq = ERR_PTR(-EAGAIN);

	return 0;
}

static void virtio_rpmb_remove(struct virtio_device *vdev)
{
	struct virtio_rpmb_info *vi;

	vi = vdev->priv;
	if (!vi)
		return;

	if (wq_has_sleeper(&vi->have_data))
		wake_up(&vi->have_data);

	rpmb_dev_unregister(vi->rdev);

	virtio_rpmb_del_vq(vdev);

	vdev->priv = NULL;
	kfree(vi);
}

static int virtio_rpmb_probe(struct virtio_device *vdev)
{
	int ret;
	struct virtio_rpmb_info *vi;

	vi = kzalloc(sizeof(*vi), GFP_KERNEL);
	if (!vi)
		return -ENOMEM;

	init_waitqueue_head(&vi->have_data);
	mutex_init(&vi->lock);
	vdev->priv = vi;

	ret = virtio_rpmb_init_vq(vdev);
	if (ret)
		goto err;

	/* create vrpmb device. */
	ret = rpmb_virtio_dev_init(vi);
	if (ret) {
		dev_err(&vdev->dev, "create vrpmb device failed.\n");
		goto err;
	}

	dev_info(&vdev->dev, "init done!\n");

	return 0;
err:
	vdev->priv = NULL;
	kfree(vi);
	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int virtio_rpmb_freeze(struct virtio_device *vdev)
{
	return virtio_rpmb_del_vq(vdev);
}

static int virtio_rpmb_restore(struct virtio_device *vdev)
{
	return virtio_rpmb_init_vq(vdev);
}
#endif

static struct virtio_device_id id_table[] = {
	{ VIRTIO_ID_RPMB, VIRTIO_DEV_ANY_ID },
	{ 0 },
};

static struct virtio_driver virtio_rpmb_driver = {
	.driver.name =	KBUILD_MODNAME,
	.driver.owner =	THIS_MODULE,
	.id_table =	id_table,
	.probe =	virtio_rpmb_probe,
	.remove =	virtio_rpmb_remove,
#ifdef CONFIG_PM_SLEEP
	.freeze =	virtio_rpmb_freeze,
	.restore =	virtio_rpmb_restore,
#endif
};

module_virtio_driver(virtio_rpmb_driver);
MODULE_DEVICE_TABLE(virtio, id_table);

MODULE_DESCRIPTION("Virtio rpmb frontend driver");
MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("Dual BSD/GPL");
