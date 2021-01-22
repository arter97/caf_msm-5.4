/* SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2011-2020, The Linux Foundation. All rights reserved.
 */

#ifdef CONFIG_MSM_BOOT_STATS

#define TIMER_KHZ 32768
extern struct boot_stats __iomem *boot_stats;

struct boot_stats {
	uint32_t bootloader_start;
	uint32_t bootloader_end;
	uint32_t kernel_entry;
	uint32_t load_kernel_start;
	uint32_t load_kernel_done;
	uint32_t bootloader_chksum_start;
	uint32_t bootloader_chksum_done;
};

int boot_stats_init(void);
int boot_stats_exit(void);
unsigned long long msm_timer_get_sclk_ticks(void);
phys_addr_t msm_timer_get_pa(void);
#else
static inline int boot_stats_init(void) { return 0; }
static inline unsigned long long msm_timer_get_sclk_ticks(void)
{
	return 0;
}
static inline phys_addr_t msm_timer_get_pa(void) { return 0; }
#endif

#ifdef CONFIG_MSM_BOOT_TIME_MARKER
static inline int boot_marker_enabled(void) { return 1; }
void place_marker(const char *name);
void destroy_marker(const char *name);
void measure_wake_up_time(void);
uint64_t get_sleep_exit_time(void);
#else
static inline void place_marker(char *name) { };
static inline void destroy_marker(const char *name) { };
static inline int boot_marker_enabled(void) { return 0; }
static inline void measure_wake_up_time(void) { };
static inline uint64_t get_sleep_exit_time(void) { return 0; }
#endif
