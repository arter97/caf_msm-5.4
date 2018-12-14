/*
 * include/uapi/linux/buzzer.h
 *
 * Author: Leo Han <lhan@gopro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef BUZZER_H
#define BUZZER_H

#include <linux/types.h>

#define BUZZER_IOCTL_MAGIC      'G'

/* Read / Write of buzzer frequency, range 200-20000 */
#define BUZZER_FREQ_GET			_IOR(BUZZER_IOCTL_MAGIC, 1, unsigned)
#define BUZZER_FREQ_SET			_IOW(BUZZER_IOCTL_MAGIC, 1, unsigned)

/* Read / Write of buzzer duty cycle, range 0-100 */
#define BUZZER_DUTYCYCLE_GET    _IOR(BUZZER_IOCTL_MAGIC, 2, unsigned)
#define BUZZER_DUTYCYCLE_SET    _IOW(BUZZER_IOCTL_MAGIC, 2, unsigned)

/* Start buzzer */
#define BUZZER_START            _IO(BUZZER_IOCTL_MAGIC, 3)

/* Stop buzzer */
#define BUZZER_STOP             _IO(BUZZER_IOCTL_MAGIC, 4)

#endif /* BUZZER_H */

