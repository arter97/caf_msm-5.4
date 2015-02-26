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
 *
 */

#include "msm_ba_debug.h"

#define MAX_DBG_BUF_SIZE 4096

int msm_ba_debug = BA_ERR | BA_WARN;
int msm_ba_debug_out = BA_OUT_PRINTK;

struct debug_buffer {
	char ptr[MAX_DBG_BUF_SIZE];
	char *curr;
	u32 filled_size;
};

static struct debug_buffer dbg_buf;

#define INIT_DBG_BUF(__buf) ({ \
	__buf.curr = __buf.ptr;\
	__buf.filled_size = 0; \
})

static int dev_info_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static u32 write_str(struct debug_buffer *buffer, const char *fmt, ...)
{
	va_list args;
	u32 size = 0;
	size_t buf_size = 0;

	if (MAX_DBG_BUF_SIZE - 1 > buffer->filled_size) {
		buf_size = MAX_DBG_BUF_SIZE - 1 - buffer->filled_size;
		va_start(args, fmt);
		size = vscnprintf(buffer->curr, buf_size, fmt, args);
		va_end(args);
		buffer->curr += size;
		buffer->filled_size += size;
	}
	return size;
}

static ssize_t dev_info_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct msm_ba_dev *dev_ctxt = file->private_data;

	if (!dev_ctxt) {
		dprintk(BA_ERR, "Invalid params, dev: 0x%p\n", dev_ctxt);
		return 0;
	}
	INIT_DBG_BUF(dbg_buf);
	write_str(&dbg_buf, "===============================\n");
	write_str(&dbg_buf, "DEV: 0x%p\n", dev_ctxt);
	write_str(&dbg_buf, "===============================\n");
	write_str(&dbg_buf, "state: %d\n", dev_ctxt->state);

	return simple_read_from_buffer(buf, count, ppos,
			dbg_buf.ptr, dbg_buf.filled_size);
}

static const struct file_operations dev_info_fops = {
	.open = dev_info_open,
	.read = dev_info_read,
};

struct dentry *msm_ba_debugfs_init_drv(void)
{
	struct dentry *dir = debugfs_create_dir("msm_ba", NULL);
	struct ba_ctxt *ba_ctxt;

	if (IS_ERR_OR_NULL(dir)) {
		dir = NULL;
		goto failed_create_dir;
	}

	if (!debugfs_create_u32("debug_level", S_IRUGO | S_IWUSR,
			dir, &msm_ba_debug)) {
		dprintk(BA_ERR, "debugfs_create_file: fail\n");
		goto failed_create_dir;
	}
	return dir;

failed_create_dir:
	if (dir) {
		ba_ctxt = msm_ba_get_ba_context();
		debugfs_remove_recursive(ba_ctxt->debugfs_root);
	}
	return NULL;
}

struct dentry *msm_ba_debugfs_init_dev(struct msm_ba_dev *dev_ctxt,
					struct dentry *parent)
{
	struct dentry *dir = NULL;
	char debugfs_name[MAX_DEBUGFS_NAME];

	if (!dev_ctxt) {
		dprintk(BA_ERR, "Invalid params, core: %p\n", dev_ctxt);
		goto failed_create_dir;
	}

	snprintf(debugfs_name, MAX_DEBUGFS_NAME, "dev_%p", dev_ctxt);
	dir = debugfs_create_dir(debugfs_name, parent);
	if (!dir) {
		dprintk(BA_ERR, "Failed to create debugfs for msm_ba\n");
		goto failed_create_dir;
	}
	if (!debugfs_create_file("info", S_IRUGO, dir, dev_ctxt,
		&dev_info_fops)) {
		dprintk(BA_ERR, "debugfs_create_file: fail\n");
		goto failed_create_dir;
	}
failed_create_dir:
	return dir;
}

static int inst_info_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t inst_info_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct msm_ba_inst *inst = file->private_data;

	if (!inst) {
		dprintk(BA_ERR, "Invalid params, dev: 0x%p\n", inst);
		return 0;
	}
	INIT_DBG_BUF(dbg_buf);
	write_str(&dbg_buf, "===============================\n");
	write_str(&dbg_buf, "INSTANCE: 0x%p (%s)\n", inst,
								"BA device");
	write_str(&dbg_buf, "===============================\n");
	write_str(&dbg_buf, "dev: 0x%p\n", inst->dev_ctxt);
	write_str(&dbg_buf, "state: %d\n", inst->state);

	return simple_read_from_buffer(buf, count, ppos,
		dbg_buf.ptr, dbg_buf.filled_size);
}

static const struct file_operations inst_info_fops = {
	.open = inst_info_open,
	.read = inst_info_read,
};

struct dentry *msm_ba_debugfs_init_inst(struct msm_ba_inst *inst,
					struct dentry *parent)
{
	struct dentry *dir = NULL;
	char debugfs_name[MAX_DEBUGFS_NAME];

	if (!inst) {
		dprintk(BA_ERR, "Invalid params, inst: %p\n", inst);
		goto failed_create_dir;
	}
	snprintf(debugfs_name, MAX_DEBUGFS_NAME, "inst_%p", inst);
	dir = debugfs_create_dir(debugfs_name, parent);
	if (!dir) {
		dprintk(BA_ERR, "Failed to create debugfs for msm_ba\n");
		goto failed_create_dir;
	}
	if (!debugfs_create_file("info", S_IRUGO, dir, inst, &inst_info_fops)) {
		dprintk(BA_ERR, "debugfs_create_file: fail\n");
		goto failed_create_dir;
	}
	inst->debug.pdata[SESSION_INIT].sampling = true;
failed_create_dir:
	return dir;
}
