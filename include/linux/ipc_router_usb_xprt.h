/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
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

#ifndef _IPC_ROUTER_USB_XPRT_H_
#define _IPC_ROUTER_USB_XPRT_H_

/*
 * The USB IPC XPRT implements a transport layer for IPC Router core.
 * This module bridges the USB layer and IPC Router core, and provides the
 * glue for send receving IPC Router control messages to/from USB.
 * This module can work with both USB in device or host mode. It works in
 * conjunction with ipcrtr USB drivers.
 *
 * None of the APIs should be called from interrupt context, since we are using
 * mutex as a form of syncronization, which would not work if called under
 * interrupt context.
 */

/**
 * struct usb_ipc_xprt_ops - USB IPC XPRT layer related operations.
 * 	This structure contains glue between the USB layer and IPC XPRT layer.
 *
 * @max_read_size: The maximum possible read size.
 * @max_write_size: The maximum possible write size.
 * @open: The open must be called before starting I/O.  The IPC bridge
 * 		driver use the platform device pointer to identify the
 * 		underlying transport channel. The IPC bridge driver may
 * 		notify that remote processor that it is ready to receive
 * 		data. Returns 0 upon success and appropriate error code
 * 		upon failure.
 * @read: The read is done synchronously and should be called from process
 * 		context. Returns the number of bytes read from remote
 * 		processor or error code upon failure. The IPC transport
 * 		driver may pass the buffer of max_read_size length if the
 * 		available data size is not known in advance.
 * @write: The write is done synchronously and should be called from process
 * 		context. The IPC bridge driver uses the same buffer for DMA
 * 		to avoid additional memcpy. So it must be physically contiguous.
 * 		Returns the number of bytes written or error code upon failure.
 * @close: The close must be called when the IPC bridge platform device
 * 		is removed. The IPC transport driver may call close when
 * 		it is no longer required to communicate with remote processor.
 */
struct usb_ipc_xprt_ops {
	unsigned int max_read_size;
	unsigned int max_write_size;
	int (*open)(void);
	int (*read)(char *buf, unsigned int count);
	int (*write)(char *buf, unsigned int count);
	void (*close)(void);
};

/* msm_ipc_router_usb_ipc_xprt_register() - Register USB IPC XPRT
 *
 * Should be called from USB layer when corresponding IPC Router
 * endpoint is detected and registered by USB layer.
 */
int msm_ipc_router_usb_ipc_xprt_register(struct usb_ipc_xprt_ops *xprt_ops);

/* msm_ipc_router_usb_ipc_xprt_unregister() - Unregister USB IPC XPRT
 * This function should be called when USB IPC endpoint deregisters and is
 * not needed, i.e. Cable disconnect or composition change, etc.
 */
int msm_ipc_router_usb_ipc_xprt_unregister(struct usb_ipc_xprt_ops *xprt_ops);

#endif /* _IPC_ROUTER_USB_XPRT_H_ */
