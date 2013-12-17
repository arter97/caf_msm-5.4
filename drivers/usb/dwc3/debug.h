/**
 * debug.h - DesignWare USB3 DRD Controller Debug Header
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com
 *
 * Authors: Felipe Balbi <balbi@ti.com>,
 *	    Sebastian Andrzej Siewior <bigeasy@linutronix.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "core.h"

#ifdef CONFIG_DEBUG_FS
extern void dbg_event(u8, const char*, int);
extern void dbg_print(u8, const char*, int, const char*);
extern void dbg_done(u8, const u32, int);
extern void dbg_queue(u8, const struct usb_request*, int);
extern void dbg_setup(u8, const struct usb_ctrlrequest*);
extern int dwc3_debugfs_init(struct dwc3 *);
extern void dwc3_debugfs_exit(struct dwc3 *);
extern void dbg_print_reg(const char *name, int reg);
#else
static inline void dbg_event(u8 ep_num, const char *name, int status)
{  }
static inline void dbg_print(u8 ep_num, const char *name, int status,
			     const char *extra)
{  }
static inline void dbg_done(u8 ep_num, const u32 count, int status)
{  }
static inline void dbg_queue(u8 ep_num, const struct usb_request *req,
			     int status)
{  }
static inline void dbg_setup(u8 ep_num, const struct usb_ctrlrequest *req)
{  }
static inline void dbg_print_reg(const char *name, int reg)
{  }
static inline int dwc3_debugfs_init(struct dwc3 *d)
{  return 0;  }
static inline void dwc3_debugfs_exit(struct dwc3 *d)
{  }
#endif

