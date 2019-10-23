/* SPDX-License-Identifier: GPL-2.0 */
/*
 *  Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com
 */

#ifndef K3_PSIL_H_
#define K3_PSIL_H_

#include <linux/types.h>

#define K3_PSIL_DST_THREAD_ID_OFFSET 0x8000

struct device;

/* Channel Throughput Levels */
enum udma_tp_level {
	UDMA_TP_NORMAL = 0,
	UDMA_TP_HIGH = 1,
	UDMA_TP_ULTRAHIGH = 2,
	UDMA_TP_LAST,
};

enum psil_endpoint_type {
	PSIL_EP_NATIVE = 0,
	PSIL_EP_PDMA_XY,
	PSIL_EP_PDMA_MCAN,
	PSIL_EP_PDMA_AASRC,
};

struct psil_endpoint_config {
	enum psil_endpoint_type ep_type;

	unsigned pkt_mode:1;
	unsigned notdpkt:1;
	unsigned needs_epib:1;
	u32 psd_size;
	enum udma_tp_level channel_tpl;

	/* PDMA properties, valid for PSIL_EP_PDMA_* */
	unsigned pdma_acc32:1;
	unsigned pdma_burst:1;
};

int psil_set_new_ep_config(struct device *dev, const char *name,
			   struct psil_endpoint_config *ep_config);

#endif /* K3_PSIL_H_ */
