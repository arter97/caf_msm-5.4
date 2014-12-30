/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
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

#ifndef _UAPI_MHI_H
#define _UAPI_MHI_H

enum peripheral_ep_type {
	DATA_EP_TYPE_RESERVED,
	DATA_EP_TYPE_HSIC,
	DATA_EP_TYPE_HSUSB,
	DATA_EP_TYPE_PCIE,
	DATA_EP_TYPE_EMBEDDED,
	DATA_EP_TYPE_BAM_DMUX,
};

struct peripheral_ep_info {
	enum peripheral_ep_type		ep_type;
	u32				peripheral_iface_id;
};

struct ipa_ep_pair {
	u32 cons_pipe_num;
	u32 prod_pipe_num;
};

struct ep_info {
	struct peripheral_ep_info	ph_ep_info;
	struct ipa_ep_pair		ipa_ep_pair;

};

#define MHI_UCI_IOCTL_MAGIC	'm'

#define MHI_UCI_EP_LOOKUP _IOR(MHI_UCI_IOCTL_MAGIC, 2, struct ep_info)

#endif /* _UAPI_MHI_H */

