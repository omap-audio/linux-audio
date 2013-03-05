/*
 * omap-abe.c  --  OMAP ALSA SoC DAI driver using Audio Backend
 *
 * Copyright (C) 2010 Texas Instruments
 *
 * Contact: Liam Girdwood <lrg@ti.com>
 *          Misael Lopez Cruz <misael.lopez@ti.com>
 *          Sebastien Guiriec <s-guiriec@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/opp.h>
#include <linux/dma-mapping.h>
#include <linux/of_device.h>

#include <sound/soc.h>
#include <sound/soc-fw.h>
#include "../../../arch/arm/mach-omap2/omap-pm.h"

#include "omap-abe-priv.h"

extern struct snd_soc_platform_driver omap_aess_platform;
extern struct snd_soc_dai_driver omap_abe_dai[6];

static u64 omap_abe_dmamask = DMA_BIT_MASK(32);

static const char *abe_memory_bank[5] = {
	"dmem",
	"cmem",
	"smem",
	"pmem",
	"mpu"
};

void driver_deferred_probe_trigger(void);

static void abe_fw_ready(const struct firmware *fw, void *context)
{
	struct platform_device *pdev = (struct platform_device *)context;
	struct omap_abe *abe = dev_get_drvdata(&pdev->dev);
	int err;

	abe->fw = fw;

	err = snd_soc_register_platform(&pdev->dev, &omap_aess_platform);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to register ABE platform %d\n", err);
		release_firmware(fw);
		return;
	}

	err = snd_soc_register_dais(&pdev->dev, omap_abe_dai,
			ARRAY_SIZE(omap_abe_dai));
	if (err < 0) {
		dev_err(&pdev->dev, "failed to register ABE DAIs %d\n", err);
		snd_soc_unregister_platform(&pdev->dev);
		release_firmware(fw);
	}
	driver_deferred_probe_trigger();

}

static int abe_engine_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct omap_abe *abe;
	int ret, i;

	abe = devm_kzalloc(&pdev->dev, sizeof(struct omap_abe), GFP_KERNEL);
	if (abe == NULL)
		return -ENOMEM;

	for (i = 0; i < OMAP_ABE_IO_RESOURCES; i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   abe_memory_bank[i]);
		if (res == NULL) {
			dev_err(&pdev->dev, "no resource %s\n",
				abe_memory_bank[i]);
			return -ENODEV;
		}
		if (!devm_request_mem_region(&pdev->dev, res->start,
					resource_size(res), abe_memory_bank[i]))
			return -EBUSY;

		abe->io_base[i] = devm_ioremap(&pdev->dev, res->start,
					       resource_size(res));
		if (!abe->io_base[i])
			return -ENOMEM;
	}

	for (i = 0; i < OMAP_ABE_DMA_RESOURCES; i++) {
		char name[8];

		sprintf(name, "fifo%d", i);
		res = platform_get_resource_byname(pdev, IORESOURCE_DMA, name);
		if (res == NULL) {
			dev_err(&pdev->dev, "no resource %s\n", name);
			return -ENODEV;
		}
		abe->dma_lines[i] = res->start;
	}

	abe->irq = platform_get_irq(pdev, 0);
	if (abe->irq < 0)
		return abe->irq;

	dev_set_drvdata(&pdev->dev, abe);

#ifdef CONFIG_PM
	abe->get_context_lost_count = omap_pm_get_dev_context_loss_count;
	abe->device_scale = NULL;
#endif
	abe->dev = &pdev->dev;

	mutex_init(&abe->mutex);
	mutex_init(&abe->opp.mutex);
	mutex_init(&abe->opp.req_mutex);
	INIT_LIST_HEAD(&abe->opp.req);

	get_device(abe->dev);
	abe->dev->dma_mask = &omap_abe_dmamask;
	abe->dev->coherent_dma_mask = omap_abe_dmamask;
	put_device(abe->dev);

	ret = request_firmware_nowait(THIS_MODULE, 1, "omap4_abe_new", abe->dev,
				      GFP_KERNEL, pdev, abe_fw_ready);
	if (!ret)
		dev_err(abe->dev, "Failed to load firmware %d\n", ret);

	return ret;
}

static int abe_engine_remove(struct platform_device *pdev)
{
	struct omap_abe *abe = dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_dais(&pdev->dev, ARRAY_SIZE(omap_abe_dai));
	snd_soc_unregister_platform(&pdev->dev);

	abe->fw_data = NULL;
	abe->fw_config = NULL;
	release_firmware(abe->fw);

	return 0;
}

static const struct of_device_id omap_aess_of_match[] = {
	{ .compatible = "ti,omap4-aess", },
	{ }
};
MODULE_DEVICE_TABLE(of, omap_aess_of_match);

static struct platform_driver omap_aess_driver = {
	.driver = {
		.name = "aess",
		.owner = THIS_MODULE,
		.of_match_table = omap_aess_of_match,
	},
	.probe = abe_engine_probe,
	.remove = abe_engine_remove,
};

module_platform_driver(omap_aess_driver);

MODULE_ALIAS("platform:omap-aess");
MODULE_DESCRIPTION("ASoC OMAP4 ABE");
MODULE_AUTHOR("Liam Girdwood <lrg@ti.com>");
MODULE_LICENSE("GPL");
