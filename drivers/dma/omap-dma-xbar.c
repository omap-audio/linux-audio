/*
 *  Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com
 *  Author: Sricharan R <r.sricharan@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/module.h>

#include "omap-dma-xbar.h"

static struct platform_device *pd_xbar;

static void dma_xbar_writeb(int dma_sig, int dma_xbar,
			    struct dma_xbar_device *xbar)
{
	writeb(dma_xbar, xbar->dma_xbar_base + xbar->reg_offs[dma_sig]);
}

static void dma_xbar_writew(int dma_sig, int dma_xbar,
			    struct dma_xbar_device *xbar)
{
	writew(dma_xbar, xbar->dma_xbar_base + xbar->reg_offs[dma_sig]);
}

static void dma_xbar_writel(int dma_sig, int dma_xbar,
			    struct dma_xbar_device *xbar)
{
	writel(dma_xbar, xbar->dma_xbar_base + xbar->reg_offs[dma_sig]);
}

static uint32_t dma_xbar_map(uint32_t dma_xbar, struct dma_xbar_device *xbar)
{
	struct dma_req *r;
	uint32_t dma_sig;

	if (!list_empty(&xbar->free_reqs))
		r = list_first_entry(&xbar->free_reqs, struct dma_req, node);
	else
		return -ENODEV;

	dma_sig = r->req_line;
	list_del(&r->node);

	xbar->write(dma_sig - 1, dma_xbar, xbar);

	return dma_sig;
}

static void dma_xbar_unmap(uint32_t dma_sig, struct dma_xbar_device *xbar)
{
	struct dma_req *r;

	r = devm_kzalloc(&pd_xbar->dev, sizeof(*r), GFP_KERNEL);
	r->req_line = dma_sig;

	list_add_tail(&r->node, &xbar->free_reqs);
}

const struct xbar_ops dma_xbar_ops = {
	.map = dma_xbar_map,
	.unmap = dma_xbar_unmap,
};

static int omap_dma_xbar_probe(struct platform_device *pdev)
{
	int i, j, reserved = 0;
	const __be32 *dmar;
	uint32_t *dma_map, max, size, entry, range;
	struct dma_req *p;
	struct resource *res;
	struct dma_xbar_device *xbar;

	pd_xbar = pdev;

	xbar = devm_kzalloc(&pdev->dev, sizeof(*xbar), GFP_KERNEL);
	if (!xbar)
		return -ENOMEM;

	xbar->ops = &dma_xbar_ops;

	INIT_LIST_HEAD(&xbar->free_reqs);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	xbar->dma_xbar_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(xbar->dma_xbar_base))
		return PTR_ERR(xbar->dma_xbar_base);

	of_property_read_u32(pdev->dev.of_node, "ti,dma-reqs", &max);
	dma_map = kzalloc(max * sizeof(*dma_map), GFP_KERNEL);
	if (!dma_map)
		return -ENOMEM;

	/* Get and mark reserved dma req lines */
	dmar = of_get_property(pdev->dev.of_node, "ti,dmas-reserved", &size);
	if (dmar) {
		size /= sizeof(__be32);

		for (i = 0; i < size; i++) {
			of_property_read_u32_index(pdev->dev.of_node,
						   "ti,dmas-reserved",
						   i++, &entry);
			of_property_read_u32_index(pdev->dev.of_node,
						   "ti,dmas-reserved",
						   i, &range);
			if ((entry + range > max) ||
			    ((entry + range) <= entry)) {
				pr_err("Invalid reserved entry\n");
				return -ENODEV;
			}

			for (j = entry; j <= range; j++)
				dma_map[j] = 1;

			/* For a single entry */
			if (!range)
				dma_map[entry] = 1;
		}
	}

	/* Add the free dma req lines to free list */
	for (j = 1; j < max; j++) {
		if (!dma_map[j]) {
			p = devm_kzalloc(&pdev->dev, sizeof(*p), GFP_KERNEL);
			p->req_line = j;
			list_add_tail(&p->node, &xbar->free_reqs);
		}
	}

	xbar->reg_offs = devm_kzalloc(&pdev->dev, max * sizeof(int),
				      GFP_KERNEL);
	if (!xbar->reg_offs)
		return -ENOMEM;

	of_property_read_u32(pdev->dev.of_node, "ti,reg-size", &size);

	switch (size) {
	case 1:
		xbar->write = dma_xbar_writeb;
		break;
	case 2:
		xbar->write = dma_xbar_writew;
		break;
	case 4:
		xbar->write = dma_xbar_writel;
		break;
	default:
		pr_err("Invalid reg-size property\n");
		return -ENODEV;
		break;
	}

	/*
	 * Register offsets are not linear because of the
	 * reserved lines. so find and store the offsets once.
	 */
	for (i = 0; i < max; i++) {
		if (dma_map[i])
			continue;

		xbar->reg_offs[i] = reserved;
		reserved += size;
	}

	if (of_dma_router_register(pdev->dev.of_node, xbar))
		return -ENODEV;

	kfree(dma_map);
	return 0;
}

static int omap_dma_xbar_remove(struct platform_device *pdev)
{
	if (pdev->dev.of_node)
		of_dma_router_free(pdev->dev.of_node);

	return 0;
}

static const struct of_device_id dma_xbar_match[] = {
	{ .compatible = "ti,dma-xbar" },
	{},
};

static struct platform_driver omap_dma_xbar_driver = {
	.probe	= omap_dma_xbar_probe,
	.remove	= omap_dma_xbar_remove,
	.driver = {
		.name = "omap-dma-xbar",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(dma_xbar_match),
	},
};

int omap_dmaxbar_init(void)
{
	return platform_driver_register(&omap_dma_xbar_driver);
}
arch_initcall(omap_dmaxbar_init);

static void __exit omap_dmaxbar_exit(void)
{
	platform_driver_unregister(&omap_dma_xbar_driver);
}
module_exit(omap_dmaxbar_exit);

MODULE_DESCRIPTION("OMAP DMA XBAR");
MODULE_AUTHOR("Sricharan R");
MODULE_LICENSE("GPL");
