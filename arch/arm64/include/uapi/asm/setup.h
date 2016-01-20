/*
 * Based on arch/arm/include/asm/setup.h
 *
 * Copyright (C) 1997-1999 Russell King
 * Copyright (C) 2012 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __ASM_SETUP_H
#define __ASM_SETUP_H

#include <linux/types.h>
#include <linux/slab.h>
#include <linux/delay.h>
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
