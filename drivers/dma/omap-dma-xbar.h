/*
 *  Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com
 *  Author: Sricharan R <r.sricharan@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
struct dma_req {
	uint32_t req_line;
	struct list_head node;
};

/*
 * @crossbar_base: crossbar base address
 * @register_offsets: offsets for each dma request number
 * @write: function to write in to the dma crossbar
 * @free_reqs: list of free dma request lines
 */
struct dma_xbar_device {
	void __iomem *dma_xbar_base;
	int *reg_offs;
	void (*write)(int, int, struct dma_xbar_device *);
	struct list_head free_reqs;
	const struct xbar_ops *ops;
};

struct xbar_ops {
	uint32_t (*map)(uint32_t, struct dma_xbar_device *);
	void (*unmap)(uint32_t,  struct dma_xbar_device *);
};
