/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _VIRTIO_SCSI_QTI_CRYPTO_H
#define _VIRTIO_SCSI_QTI_CRYPTO_H

#include <linux/device.h>
#include <linux/blkdev.h>

struct virtio_scsi_ice_info {
	__u8 ice_slot; /* ice slot to program the key */
	__u8 activate;
	__u64 dun; /* initilaization vector */
} __packed;

struct virtio_scsi_cmd_req_fbe {
	__u8 lun[8];            /* Logical Unit Number */
	__virtio64 tag;         /* Command identifier */
	__u8 task_attr;         /* Task attribute */
	__u8 prio;              /* SAM command priority field */
	__u8 crn;
	__u8 cdb[VIRTIO_SCSI_CDB_SIZE];
	union {
		__u8 raw[16];
		struct virtio_scsi_ice_info ice_info; /*inline crypto params*/
	} keyinfo;
} __packed;

/**
 * This function intializes the supported crypto capabilities
 * and create keyslot manager to manage keyslots for virtual
 * disks.
 *
 * Return: zero on success, else a -errno value
 */
int virtscsi_init_crypto_qti_spec(void);

/**
 * set up a keyslot manager in the virtual disks request_queue
 *
 * @request_queue: virtual disk request queue
 */
void virtscsi_crypto_qti_setup_rq_keyslot_manager(struct request_queue *q);
/**
 * destroy keyslot manager
 *
 * @request_queue: virtual disk request queue
 */
void virtscsi_crypto_qti_destroy_rq_keyslot_manager(struct request_queue *q);

#endif /* _VIRTIO_SCSI_QTI_CRYPTO_H */
