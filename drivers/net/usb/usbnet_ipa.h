/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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

#ifndef _USBNET_IPA_H_
#define _USBNET_IPA_H_

#ifdef CONFIG_USB_USBNET_IPA_SUPPORT
int usbnet_ipa_init(struct net_device *dev);
int usbnet_ipa_connect(struct usb_interface *intf,
	unsigned int rx_usb_pipe_handle, unsigned int tx_usb_pipe_handle);
int usbnet_ipa_disconnect(struct usb_interface *intf,
	unsigned int rx_usb_pipe_handle, unsigned int tx_usb_pipe_handle);
void usbnet_ipa_cleanup(void);

#else /* CONFIG_USB_USBNET_IPA_SUPPORT */
inline int usbnet_ipa_init(struct net_device *dev) {return 0; }
inline int usbnet_ipa_connect(struct usb_interface *intf,
	unsigned int rx_usb_pipe_handle, unsigned int tx_usb_pipe_handle)
{
	return 0;
}
inline int usbnet_ipa_disconnect(struct usb_interface *intf,
	unsigned int rx_usb_pipe_handle, unsigned int tx_usb_pipe_handle)
{
	return 0;
}

inline void usbnet_ipa_cleanup(void) {}

#endif /* CONFIG_USB_USBNET_IPA_SUPPORT */

#endif /* _USBNET_IPA_H_ */
