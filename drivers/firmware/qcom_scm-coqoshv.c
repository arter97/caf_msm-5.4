/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2023 OpenSynergy GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/qcom_scm.h>
#include <linux/arm-smccc.h>
#include <linux/dma-mapping.h>

#include <soc/qcom/qseecom_scm.h>
#include <soc/qcom/qseecomi.h>

#include "qcom_scm.h"
#include "qcom_scm-coqoshv.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt)  "qcom_scm-coqoshv: " fmt

/* "opsy/smc" */
#define COQOSHV_SMC_MAGIC   0x636d732f79737070ul
#define COQOSHV_SMC_MAX 54

/* HVC trigger capability (e.g. interrupt injection) */
#define HVC_P_TRIGGER_CAP 0x1

/* 16 byte */
struct coqoshv_smc_header {
        __u64 magic;
        __u16 mailboxes;
        __u16 boxsize;
        __u16 offset_mheader;
        __u16 offset_mailbox;
} __packed;

/* 32bits in case guest is 32bit -> only 32bit atomics */
struct coqoshv_smc_mailbox_header {
        __u32 req_id;
        __u32 res_id;
} __packed;

/* short reference to all relevant parts */
struct coqoshv_smc_desc {
        struct coqoshv_smc_header         *bheader;
        struct coqoshv_smc_mailbox_header *mheader;
        struct coqoshv_smc_mailbox        *mailbox;
};

struct coqoshv_notification {
	__u32 id;
	struct semaphore sem;
};

static struct coqoshv_notification coqoshv_notification[COQOSHV_SMC_MAX];

DEFINE_SEMAPHORE(coqoshv_buffer_semaphore);

static int coqoshv_initialized = 0;
static unsigned int coqoshv_sender_local_id = 0;
static uintptr_t coqoshv_qcpe_pshm = 0;
static u64       coqoshv_qcpe_pshm_size = 0;
static void*     coqoshv_qcpe_vshm = NULL;
static struct coqoshv_smc_desc coqoshv_buffer;
static unsigned long coqoshv_mailbox_alloc = 0;
static unsigned long coqoshv_mailbox_max = 0;
static unsigned long coqoshv_mailbox_notified = ~0ul;

static int coqoshv_init_qcpe(void);

static void coqoshv_notify_mailboxes()
{
	__u32 shmem_res_id, local_res_id;
	int i;

	mb();
	for (i = 0; i < coqoshv_mailbox_max; ++i) {
		shmem_res_id = READ_ONCE(coqoshv_buffer.mheader[i].res_id);
		local_res_id = READ_ONCE(coqoshv_notification[i].id);
		if (local_res_id != shmem_res_id)
			continue;

		/* make sure only one caller of this function updates the id and
		 * and signals the semaphore
		 */
		if (!test_and_set_bit_lock(i, &coqoshv_mailbox_notified)) {
			WRITE_ONCE(coqoshv_notification[i].id, local_res_id + 1);
			mb();
			up(&coqoshv_notification[i].sem);
		}
	}
}

static irqreturn_t coqoshv_irq_handler(int irq, void *args)
{
	mb();
	if (READ_ONCE(coqoshv_mailbox_notified) != ~0ul)
		coqoshv_notify_mailboxes();

	return IRQ_HANDLED;
}

inline static unsigned coqoshv_acquire_mailbox()
{
	/* fast path: try to allocate first mailbox
	 * since this allocator is protected by a semaphore,
	 * not a lot of contention is expected
	 */
	unsigned desired = 0;

	while (test_and_set_bit_lock(desired, &coqoshv_mailbox_alloc)) {
		desired = (desired + 1) % coqoshv_mailbox_max;
		cpu_relax();
	}

	return desired;
}

inline static void coqoshv_release_mailbox(unsigned mailbox)
{
	clear_bit_unlock(mailbox, &coqoshv_mailbox_alloc);
}

inline static void coqoshv_wait_sem(struct semaphore *sem,
				    const bool atomic)
{
	if (!atomic) {
		down(sem);
	} else	{
		/* Can not sleep in semaphore in atomic context,
		 * but need to wait for response.
		 * Returns 1 if cannot be acquired.
		 */
		while (down_trylock(sem))
			cpu_relax();
	}
}

static void coqoshv_send_smc(__u32 mbox_id, struct arm_smccc_res *res)
{
	clear_bit_unlock(mbox_id, &coqoshv_mailbox_notified);
	arm_smccc_hvc(HVC_P_TRIGGER_CAP,
		      coqoshv_sender_local_id, 0, 0, 0, 0, 0, 0, res);
	down(&coqoshv_notification[mbox_id].sem);
}

static void coqoshv_send_smc_atomic(__u32 mbox_id, struct arm_smccc_res *res)
{
	__u32 local_res_id = READ_ONCE(coqoshv_notification[mbox_id].id);
	__u32 shmem_res_id;

	arm_smccc_hvc(HVC_P_TRIGGER_CAP,
		      coqoshv_sender_local_id, 0, 0, 0, 0, 0, 0, res);

	do {
		cpu_relax();
		mb();
		shmem_res_id = READ_ONCE(coqoshv_buffer.mheader[mbox_id].res_id);
	} while (local_res_id != shmem_res_id);

	WRITE_ONCE(coqoshv_notification[mbox_id].id, local_res_id + 1);
}

void coqoshv_call_qcpe(const struct coqoshv_smc_mailbox *smc,
		       struct arm_smccc_res *res,
		       const bool atomic)
{
	struct coqoshv_smc_mailbox *shmem;
	__u32 msg_id;
	__u32 mbox_id;

	coqoshv_wait_sem(&coqoshv_buffer_semaphore, atomic);

	if (unlikely(!coqoshv_initialized))
		coqoshv_init_qcpe();

	mbox_id = coqoshv_acquire_mailbox();
	msg_id = READ_ONCE(coqoshv_buffer.mheader[mbox_id].req_id) + 1;
	shmem = &coqoshv_buffer.mailbox[mbox_id];
	memcpy(shmem, smc, sizeof(struct coqoshv_smc_mailbox));
	WRITE_ONCE(coqoshv_buffer.mheader[mbox_id].req_id, msg_id);
	mb();

	pr_debug("x0: %lx, x1: 0x%lx, x2: 0x%lx, x3: 0x%lx, "
		 "x4: %lx, x5: 0x%lx, x6: 0x%lx, x7: 0x%lx\n",
		 shmem->args[0], shmem->args[1], shmem->args[2], shmem->args[3],
		 shmem->args[4], shmem->args[5], shmem->args[6], shmem->args[7]);

	do {
		if (!atomic)
			coqoshv_send_smc(mbox_id, res);
		else
			coqoshv_send_smc_atomic(mbox_id, res);

		mb();
		/* the signal might have been triggered again
		 * during the interrupt handler that woke us up,
		 * so check all the mailboxes if there are waiters
		 */
		if (unlikely(READ_ONCE(coqoshv_mailbox_notified) != ~0ul))
			coqoshv_notify_mailboxes();
	} while (shmem->args[0] == QCOM_SCM_INTERRUPTED);

	pr_debug("x0: %lx, x1: 0x%lx, x2: 0x%lx, x3: 0x%lx, "
		 "x4: %lx, x5: 0x%lx, x6: 0x%lx, x7: 0x%lx\n",
		 shmem->args[0], shmem->args[1], shmem->args[2], shmem->args[3],
		 shmem->args[4], shmem->args[5], shmem->args[6], shmem->args[7]);

	memcpy(res, shmem, sizeof(struct arm_smccc_res));

	coqoshv_release_mailbox(mbox_id);
	up(&coqoshv_buffer_semaphore);
}

static int coqoshv_platform_init(void)
{
	int mbox_id;
	int ret, virq;
	struct device_node *node;
	struct resource resource, *res = NULL;
	const char *compat = "coqoshv,smc-chan";
	u8 *addr;

#if IS_ENABLED(CONFIG_QCOM_SCM_LOCK_DISABLE)
	pr_info("qcom_scm_lock disabled\n");
#else
	pr_info("qcom_scm_lock enabled\n");
#endif
	node = of_find_compatible_node(NULL, NULL, compat);
	if (!node) {
		pr_err("Could not find %s node\n", compat);
		return -EINVAL;
	}

	if (of_property_read_u32(node, "sender-local-id",
				 &coqoshv_sender_local_id)) {
		pr_err("Could not parse \"sender-local-id\"\n" );
		return -EINVAL;
	}

	virq = irq_of_parse_and_map(node, 0);
	if (virq <= 0) {
		pr_err("irq_of_parse_and_map failed\n");
		return -EINVAL;
	}

	if (request_irq(virq, coqoshv_irq_handler, 0, "coqoshv_smc", NULL)) {
		pr_err("Could not request irq\n");
		return -ENODEV;
	}

	if (of_address_to_resource(node, 0, &resource)) {
		pr_err("Could not get resource from \"reg\" property\n");
		ret = -EINVAL;
		goto error_of_address;
	}

	coqoshv_qcpe_pshm = resource.start;
	coqoshv_qcpe_pshm_size = resource_size(&resource);

	res = request_mem_region(coqoshv_qcpe_pshm,
				 coqoshv_qcpe_pshm_size, "coqoshv-qcpe");
	if (!res) {
		pr_err("Could not request region\n");
		ret = -ENODEV;
		goto error_request_mem;
	}

	coqoshv_qcpe_vshm = memremap(coqoshv_qcpe_pshm,
				     coqoshv_qcpe_pshm_size, MEMREMAP_WB);
	if (!coqoshv_qcpe_vshm) {
		pr_err("Could not map mem region\n");
		ret = -EINVAL;
		goto error_memremap;
	}

	coqoshv_buffer.bheader = coqoshv_qcpe_vshm;

	addr = (u8 *) coqoshv_qcpe_vshm +
		READ_ONCE(coqoshv_buffer.bheader->offset_mheader);

	if (addr + sizeof(struct coqoshv_smc_mailbox_header) >
	    (u8 *)coqoshv_qcpe_vshm + coqoshv_qcpe_pshm_size) {
		pr_err("Mailbox header offset out of shm range");
		goto error_io;
	}

	coqoshv_buffer.mheader = (struct coqoshv_smc_mailbox_header *) addr;

	addr = (u8 *) coqoshv_qcpe_vshm +
		 READ_ONCE(coqoshv_buffer.bheader->offset_mailbox);

	if (addr + sizeof(struct coqoshv_smc_mailbox) >
	    (u8 *)coqoshv_qcpe_vshm + coqoshv_qcpe_pshm_size) {
		pr_err("SMC mailbox offset out of shm range");
		goto error_io;
	}

	coqoshv_buffer.mailbox = (struct coqoshv_smc_mailbox *) addr;

	if (unlikely(READ_ONCE(coqoshv_buffer.bheader->magic) !=
	    COQOSHV_SMC_MAGIC)) {
		pr_err("Invalid header in SMC message buffer");
		goto error_io;
	}

	if (unlikely(READ_ONCE(coqoshv_buffer.bheader->boxsize) != 64)) {
		pr_err("Invalid mailbox size for SMC message buffer");
		goto error_io;
	}

	coqoshv_mailbox_alloc = 0;

	/* minimum of: length of array == COQOSHV_SMC_MAX and
	 *              number of mailboxes from the header
         */
	coqoshv_mailbox_max = READ_ONCE(coqoshv_buffer.bheader->mailboxes);
	coqoshv_mailbox_max = min(coqoshv_mailbox_max,
				  (unsigned long) COQOSHV_SMC_MAX);
	if (unlikely(!coqoshv_mailbox_max)) {
		pr_err("Zero mailboxes for SMC message buffer");
		goto error_io;
	}

	for (mbox_id = 0; mbox_id < coqoshv_mailbox_max; ++mbox_id) {
		sema_init(&coqoshv_notification[mbox_id].sem, 0);
		coqoshv_notification[mbox_id].id = 1;
	}

	pr_info("Initialized\n");

	coqoshv_initialized = 1;

	/* semaphore is initialized to 1, increase to coqoshv_mailbox_max */
	for (mbox_id = 1; mbox_id < coqoshv_mailbox_max; ++mbox_id) {
		up(&coqoshv_buffer_semaphore);
	}

	return 0;

error_io:
	ret = -EIO;
	memunmap(coqoshv_qcpe_vshm);
error_memremap:
	release_mem_region(coqoshv_qcpe_pshm, coqoshv_qcpe_pshm_size);
error_request_mem:
error_of_address:
	free_irq(virq, NULL);
	return ret;
}

static int coqoshv_init_qcpe(void)
{
	if (coqoshv_platform_init())
		panic("Could not initialize.\n");

	return 0;
}
