/*
 * Copyright (c) 2015-2017, The Linux Foundation. All rights reserved.
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/swap.h>
#include <linux/sort.h>
#include <linux/oom.h>
#include <linux/sched.h>
#include <linux/rcupdate.h>
#include <linux/notifier.h>
#include <linux/vmpressure.h>
#include <linux/freezer.h>
#include <linux/kthread.h>

#define CREATE_TRACE_POINTS
#include <trace/events/process_reclaim.h>

#define MAX_SWAP_TASKS SWAP_CLUSTER_MAX

/* Tasks which can fit on half a page */
#define MAX_RECLAIM_TASKS 85

static void swap_fn(struct work_struct *work);
DECLARE_WORK(swap_work, swap_fn);

/* User knob to enable/disable process reclaim feature */
static int enable_process_reclaim;
module_param_named(enable_process_reclaim, enable_process_reclaim, int,
	S_IRUGO | S_IWUSR);

/* The max number of pages tried to be reclaimed in a single run */
int per_swap_size = SWAP_CLUSTER_MAX * 32;
module_param_named(per_swap_size, per_swap_size, int, S_IRUGO | S_IWUSR);

int reclaim_avg_efficiency;
module_param_named(reclaim_avg_efficiency, reclaim_avg_efficiency,
			int, S_IRUGO);

struct selected_task_data {
	struct task_struct *p;
	unsigned long nr_file;
	unsigned long nr_anon;
};

#define MAX_RANGES 10
static int reclaim_filter_size;
enum {
	OOM_RANGE_BEGIN,
	OOM_RANGE_END,
	ANON_RECLAIM_PERCENT,
	FILE_RECLAIM_PERCENT,
	NR_FILTER_ITEMS,
};
static int reclaim_filter[MAX_RANGES * NR_FILTER_ITEMS];

enum {
	SCAN_ANON,
	RECLAIM_ANON,
	SCAN_FILE,
	RECLAIM_FILE,
	RECLAIMED_TASKS,
	RECLAIM_TIME,
	MAX_RECLAIM_STATS,
};

unsigned long long ppr_stats[MAX_RECLAIM_STATS];

/* The vmpressure region where process reclaim operates */
static unsigned long pressure_min = 50;
static unsigned long pressure_max = 90;
module_param_named(pressure_min, pressure_min, ulong, S_IRUGO | S_IWUSR);
module_param_named(pressure_max, pressure_max, ulong, S_IRUGO | S_IWUSR);

/*
 * Scheduling process reclaim workqueue unecessarily
 * when the reclaim efficiency is low does not make
 * sense. We try to detect a drop in efficiency and
 * disable reclaim for a time period. This period and the
 * period for which we monitor a drop in efficiency is
 * defined by swap_eff_win. swap_opt_eff is the optimal
 * efficincy used as theshold for this.
 */
static int swap_eff_win = 2;
module_param_named(swap_eff_win, swap_eff_win, int, S_IRUGO | S_IWUSR);

static int swap_opt_eff = 50;
module_param_named(swap_opt_eff, swap_opt_eff, int, S_IRUGO | S_IWUSR);

static atomic_t skip_reclaim = ATOMIC_INIT(0);
/* Not atomic since only a single instance of swap_fn run at a time */
static int monitor_eff;

static struct task_struct *kpprd;

struct selected_task {
	struct task_struct *p;
	int tasksize;
	short oom_score_adj;
};

int selected_cmp(const void *a, const void *b)
{
	const struct selected_task *x = a;
	const struct selected_task *y = b;
	int ret;

	ret = x->tasksize < y->tasksize ? -1 : 1;

	return ret;
}

static int test_task_flag(struct task_struct *p, int flag)
{
	struct task_struct *t = p;

	rcu_read_lock();
	for_each_thread(p, t) {
		task_lock(t);
		if (test_tsk_thread_flag(t, flag)) {
			task_unlock(t);
			rcu_read_unlock();
			return 1;
		}
		task_unlock(t);
	}
	rcu_read_unlock();

	return 0;
}

static void swap_fn(struct work_struct *work)
{
	struct task_struct *tsk;
	struct reclaim_param rp;

	/* Pick the best MAX_SWAP_TASKS tasks in terms of anon size */
	struct selected_task selected[MAX_SWAP_TASKS] = {{0, 0, 0},};
	int si = 0;
	int i;
	int tasksize;
	int total_sz = 0;
	short min_score_adj = 360;
	int total_scan = 0;
	int total_reclaimed = 0;
	int nr_to_reclaim;
	int efficiency;

	rcu_read_lock();
	for_each_process(tsk) {
		struct task_struct *p;
		short oom_score_adj;

		if (tsk->flags & PF_KTHREAD)
			continue;

		if (test_task_flag(tsk, TIF_MEMDIE))
			continue;

		p = find_lock_task_mm(tsk);
		if (!p)
			continue;

		oom_score_adj = p->signal->oom_score_adj;
		if (oom_score_adj < min_score_adj) {
			task_unlock(p);
			continue;
		}

		tasksize = get_mm_counter(p->mm, MM_ANONPAGES);
		task_unlock(p);

		if (tasksize <= 0)
			continue;

		if (si == MAX_SWAP_TASKS) {
			sort(&selected[0], MAX_SWAP_TASKS,
					sizeof(struct selected_task),
					&selected_cmp, NULL);
			if (tasksize < selected[0].tasksize)
				continue;
			selected[0].p = p;
			selected[0].oom_score_adj = oom_score_adj;
			selected[0].tasksize = tasksize;
		} else {
			selected[si].p = p;
			selected[si].oom_score_adj = oom_score_adj;
			selected[si].tasksize = tasksize;
			si++;
		}
	}

	for (i = 0; i < si; i++)
		total_sz += selected[i].tasksize;

	/* Skip reclaim if total size is too less */
	if (total_sz < SWAP_CLUSTER_MAX) {
		rcu_read_unlock();
		return;
	}

	for (i = 0; i < si; i++)
		get_task_struct(selected[i].p);

	rcu_read_unlock();

	while (si--) {
		nr_to_reclaim =
			(selected[si].tasksize * per_swap_size) / total_sz;
		/* scan atleast a page */
		if (!nr_to_reclaim)
			nr_to_reclaim = 1;

		rp = reclaim_task(selected[si].p, nr_to_reclaim, 1);

		trace_process_reclaim(selected[si].tasksize,
				selected[si].oom_score_adj, rp.nr_scanned,
				rp.nr_reclaimed, per_swap_size, total_sz,
				nr_to_reclaim);
		total_scan += rp.nr_scanned;
		total_reclaimed += rp.nr_reclaimed;
		put_task_struct(selected[si].p);
	}

	if (total_scan) {
		efficiency = (total_reclaimed * 100) / total_scan;

		if (efficiency < swap_opt_eff) {
			if (++monitor_eff == swap_eff_win) {
				atomic_set(&skip_reclaim, swap_eff_win);
				monitor_eff = 0;
			}
		} else {
			monitor_eff = 0;
		}

		reclaim_avg_efficiency =
			(efficiency + reclaim_avg_efficiency) / 2;
		trace_process_reclaim_eff(efficiency, reclaim_avg_efficiency);
	}
}

static int vmpressure_notifier(struct notifier_block *nb,
			unsigned long action, void *data)
{
	unsigned long pressure = action;

	if (!enable_process_reclaim)
		return 0;

	if (!current_is_kswapd())
		return 0;

	if (0 <= atomic_dec_if_positive(&skip_reclaim))
		return 0;

	if ((pressure >= pressure_min) && (pressure < pressure_max))
		if (!work_pending(&swap_work))
			queue_work(system_unbound_wq, &swap_work);
	return 0;
}

static int ppr_thread(void *data)
{
	ktime_t start;
	int i;
	int si;
	int ovflw;
	struct task_struct *tsk;
	int rclm_fil[MAX_RANGES * NR_FILTER_ITEMS];
	int oom_fsize;
	struct selected_task_data *selected;
	set_freezable();

begin:
	try_to_freeze();
	if (kthread_should_stop())
		goto end;

	start = ktime_get();

	if (!reclaim_filter_size)
		goto nap;

	oom_fsize = reclaim_filter_size;
	reclaim_filter_size = 0;

	/*
	 * Entries should count to a multiple of 4
	 * and at least 1 oom_score_adj range.
	 */
	if (oom_fsize % 4)
		goto nap;

	memcpy(rclm_fil, reclaim_filter, oom_fsize * sizeof(int));

	selected = kmalloc(sizeof(struct selected_task_data) *
					MAX_RECLAIM_TASKS, GFP_KERNEL);
	if (!selected)
		goto nap;
	si = 0;
	ovflw = 1;

	rcu_read_lock();
	for_each_process(tsk) {
		struct task_struct *p;
		short oom_score_adj;

		if (tsk->flags & PF_KTHREAD)
			continue;

		if (test_task_flag(tsk, TIF_MEMDIE))
			continue;
		if (si == (MAX_RECLAIM_TASKS * ovflw)) {
			struct selected_task_data *sel;
			size_t sz = sizeof(struct selected_task_data) *
				MAX_RECLAIM_TASKS * ++ovflw;

			sel = krealloc(selected, sz, GFP_ATOMIC);
			if (!sel) /* If krealloc fails, "selected" is intact */
				break;
			selected = sel;
		}

		p = find_lock_task_mm(tsk);
		if (!p)
			continue;

		oom_score_adj = p->signal->oom_score_adj;

		for (i = OOM_RANGE_BEGIN; i < oom_fsize; i += NR_FILTER_ITEMS) {
			if ((oom_score_adj >= rclm_fil[i]) &&
				(oom_score_adj <= rclm_fil[i + 1])) {
				short anon_p;
				short file_p;

				selected[si].p = p;
				anon_p = rclm_fil[i + ANON_RECLAIM_PERCENT];
				file_p = rclm_fil[i + FILE_RECLAIM_PERCENT];

				if (anon_p > 0) {
					unsigned long anon;
					anon = get_mm_counter(p->mm,
							MM_ANONPAGES);
					selected[si].nr_anon =
							(anon * anon_p) / 100;
				} else
					selected[si].nr_anon = 0;

				if (file_p > 0)	{
					unsigned long file;
					file = get_mm_counter(p->mm,
							MM_FILEPAGES);
					selected[si].nr_file =
							(file * file_p) / 100;
				} else
					selected[si].nr_file = 0;
				si++;
				break;
			}
		}

		task_unlock(p);
	}

	for (i = 0; i < si; i++)
		get_task_struct(selected[i].p);

	rcu_read_unlock();

	for (i = 0; i < si; i++) {
		struct reclaim_param rp;

		if ((selected[i].nr_anon > 0) &&
				(get_nr_swap_pages() > 0)) {
			int nr_to_reclaim = selected[i].nr_anon;

			rp = reclaim_task(selected[i].p, nr_to_reclaim, 1);
			ppr_stats[SCAN_ANON] += rp.nr_scanned;
			ppr_stats[RECLAIM_ANON] += rp.nr_reclaimed;
		}

		if (selected[i].nr_file > 0) {
			int nr_to_reclaim = selected[i].nr_file;

			rp = reclaim_task(selected[i].p, nr_to_reclaim, 0);
			ppr_stats[SCAN_FILE] += rp.nr_scanned;
			ppr_stats[RECLAIM_FILE] += rp.nr_reclaimed;
		}
		put_task_struct(selected[i].p);
	}

	ppr_stats[RECLAIMED_TASKS] += si;
	kfree(selected);
nap:
	ppr_stats[RECLAIM_TIME] += ktime_us_delta(ktime_get(), start);
	set_current_state(TASK_INTERRUPTIBLE);
	schedule();
	goto begin;
end:
	return 0;
}

void reclaim_handler(void)
{
	if (reclaim_filter_size && (kpprd->state != TASK_RUNNING))
		wake_up_process(kpprd);
}

static struct notifier_block vmpr_nb = {
	.notifier_call = vmpressure_notifier,
};

static int __init process_reclaim_init(void)
{
	vmpressure_notifier_register(&vmpr_nb);
	kpprd = kthread_run(ppr_thread, NULL, "kpprd");
	return 0;
}

static void __exit process_reclaim_exit(void)
{
	vmpressure_notifier_unregister(&vmpr_nb);
	kthread_stop(kpprd);
}

static int reclaim_filter_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	ret = param_array_ops.set(val, kp);
	reclaim_handler();
	return ret;
}

static int reclaim_filter_get(char *buffer, const struct kernel_param *kp)
{
	return param_array_ops.get(buffer, kp);
}

static void reclaim_filter_free(void *arg)
{
	param_array_ops.free(arg);
}

static struct kernel_param_ops reclaim_filter_ops = {
	.set = reclaim_filter_set,
	.get = reclaim_filter_get,
	.free = reclaim_filter_free,
};

static const struct kparam_array __param_arr_reclaim = {
	.max = ARRAY_SIZE(reclaim_filter),
	.num = &reclaim_filter_size,
	.ops = &param_ops_int,
	.elemsize = sizeof(reclaim_filter[0]),
	.elem = reclaim_filter,
};

__module_param_call(MODULE_PARAM_PREFIX, reclaim_filter,
		    &reclaim_filter_ops,
		    .arr = &__param_arr_reclaim,
		    S_IRUGO | S_IWUSR, -1);
__MODULE_PARM_TYPE(reclaim_filter, "array of int");

module_param_array(ppr_stats, ullong, NULL, S_IRUGO);

module_init(process_reclaim_init);
module_exit(process_reclaim_exit);
