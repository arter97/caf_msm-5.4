/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
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
#ifndef __ASM_SETUP_H
#define __ASM_SETUP_H

#include <linux/types.h>

#define COMMAND_LINE_SIZE	2048

#ifdef CONFIG_BOOT_TIME_MARKER
#define BOOT_MARKER_MAX_LEN 20
#define TIMER_KHZ 32768
#define MAX_PRINT_LEN 50
#define MAX_SS_LK_MARKER_SIZE 16
#define MPM2_MPM_SLEEP_TIMETICK_COUNT_VAL  0x4A3000
extern char lk_splash_val[MAX_SS_LK_MARKER_SIZE];
uint32_t msm_timer_get_sclk_ticks(void);
extern unsigned long kernel_start_marker;
void place_marker(char *name);
int init_marker_sys_fs(void);
#endif

#endif

