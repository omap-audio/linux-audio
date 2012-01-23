/*
 * omap-abe  --  OMAP ABE Platform Data
 *
 * Author: Liam Girdwood <lrg@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _OMAP4_ABE_H
#define _OMAP4_ABE_H

struct omap_abe_pdata {
	bool (*was_context_lost)(struct device *dev);
	int (*device_scale)(struct device *req_dev,
			    struct device *target_dev,
			    unsigned long rate);
};

#endif
