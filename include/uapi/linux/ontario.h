/*Qualcomm Technologies, Inc. Ontario Fingerprint driver
 *
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
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
#ifndef _UAPI_ONTARIO_H_
#define _UAPI_ONTARIO_H_

#define ONTARIO_SPI_CLKS_CMD NULL

#define ONTARIO_SPI_CLKS_ON  0x47

#define ONTARIO_SPI_CLKS_OFF 0x48

struct ontario_clk {
	/*Need set to ONTARIO_SPI_CLKS_ON or ONTARIO_SPI_CLKS_OFF */
	unsigned action;
	/* only used for ONTARIO_SPI_CLKS_ON */
	unsigned SpiFrequency;
};

#endif /* _UAPI_ONTARIO_H_ */
