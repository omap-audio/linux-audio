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
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/opp.h>
#include <linux/dma-mapping.h>
#include <linux/of_device.h>

#include <sound/soc.h>
#include <sound/soc-fw.h>
#include "../../../arch/arm/mach-omap2/omap-pm.h"

#include "omap-aess-priv.h"

#define ABE_FW_NAME	"omap_aess-adfw.bin"

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

static const struct snd_soc_component_driver omap_aess_component = {
	.name		= "omap-aess",
};

#ifndef CONFIG_SND_OMAP_SOC_AESS_MODULE
void driver_deferred_probe_trigger(void);
#endif

void omap_abe_pm_get(struct snd_soc_platform *platform)
{
	struct omap_aess *aess = snd_soc_platform_get_drvdata(platform);
	pm_runtime_get_sync(aess->dev);
}
EXPORT_SYMBOL_GPL(omap_abe_pm_get);

void omap_abe_pm_put(struct snd_soc_platform *platform)
{
	struct omap_aess *aess = snd_soc_platform_get_drvdata(platform);
	pm_runtime_put_sync(aess->dev);
}
EXPORT_SYMBOL_GPL(omap_abe_pm_put);

void omap_abe_pm_shutdown(struct snd_soc_platform *platform)
{
	struct omap_aess *aess = snd_soc_platform_get_drvdata(platform);
	int ret;

	if (aess->active && omap_aess_check_activity(aess))
		return;

	omap_aess_set_opp_processing(aess, OMAP_ABE_OPP_25);
	aess->opp.level = 25;

	omap_aess_write_event_generator(aess, EVENT_STOP);
	udelay(250);
	if (aess->device_scale) {
		ret = aess->device_scale(aess->dev, aess->dev, aess->opp.freqs[0]);
		if (ret)
			dev_err(aess->dev, "failed to scale to lowest OPP\n");
	}
}
EXPORT_SYMBOL_GPL(omap_abe_pm_shutdown);

void omap_abe_pm_set_mode(struct snd_soc_platform *platform, int mode)
{
	struct omap_aess *aess = snd_soc_platform_get_drvdata(platform);

	aess->dc_offset.power_mode = mode;
}
EXPORT_SYMBOL(omap_abe_pm_set_mode);

static void abe_fw_ready(const struct firmware *fw, void *context)
{
	struct platform_device *pdev = (struct platform_device *)context;
	struct omap_aess *aess = dev_get_drvdata(&pdev->dev);
	int ret;

	if (unlikely(!fw)) {
		dev_warn(&pdev->dev, "%s firmware is not loaded. Retry.\n",
			ABE_FW_NAME);

		ret = request_firmware(&fw, ABE_FW_NAME, &pdev->dev);
		if (ret) {
			dev_err(&pdev->dev, "%s firmware loading error %d\n",
				ABE_FW_NAME, ret);
			return;
		}
	}

	if (unlikely(!fw->data)) {
		dev_err(&pdev->dev, "Loaded %s firmware is empty\n",
			ABE_FW_NAME);
		release_firmware(fw);
		return;
	}

	aess->fw = fw;

	ret = snd_soc_register_platform(&pdev->dev, &omap_aess_platform);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register ABE platform %d\n", ret);
		release_firmware(fw);
		return;
	}

	ret = snd_soc_register_component(&pdev->dev, &omap_aess_component,
					 omap_abe_dai, ARRAY_SIZE(omap_abe_dai));
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register ABE DAIs %d\n", ret);
		snd_soc_unregister_platform(&pdev->dev);
		release_firmware(fw);
	}
#ifndef CONFIG_SND_OMAP_SOC_AESS_MODULE
	driver_deferred_probe_trigger();
#endif
}

static int abe_engine_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct omap_aess *aess;
	int ret, i;

	aess = devm_kzalloc(&pdev->dev, sizeof(struct omap_aess), GFP_KERNEL);
	if (aess == NULL)
		return -ENOMEM;

	for (i = 0; i < OMAP_ABE_IO_RESOURCES; i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   abe_memory_bank[i]);
		if (res == NULL) {
			dev_err(&pdev->dev, "no resource: %s\n",
				abe_memory_bank[i]);
			return -ENODEV;
		}
		if (!devm_request_mem_region(&pdev->dev, res->start,
					resource_size(res), abe_memory_bank[i]))
			return -EBUSY;

		aess->io_base[i] = devm_ioremap(&pdev->dev, res->start,
					       resource_size(res));
		if (!aess->io_base[i])
			return -ENOMEM;

		if (i == 0)
			aess->dmem_l4 = res->start;
	}

	/* Get needed L3 I/O addresses */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dmem_dma");
	if (res == NULL) {
		dev_err(&pdev->dev, "no L3 resource: dmem\n");
		return -ENODEV;
	}
	aess->dmem_l3 = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dma");
	if (res == NULL) {
		dev_err(&pdev->dev, "no L3 resource: AESS configuration\n");
		return -ENODEV;
	}
	aess->aess_config_l3 = res->start;

	for (i = 0; i < OMAP_ABE_DMA_RESOURCES; i++) {
		char name[8];

		sprintf(name, "fifo%d", i);
		res = platform_get_resource_byname(pdev, IORESOURCE_DMA, name);
		if (res == NULL) {
			dev_err(&pdev->dev, "no resource %s\n", name);
			return -ENODEV;
		}
		aess->dma_lines[i] = res->start;
	}

	aess->irq = platform_get_irq(pdev, 0);
	if (aess->irq < 0)
		return aess->irq;

	dev_set_drvdata(&pdev->dev, aess);

#ifdef CONFIG_PM
	aess->get_context_lost_count = omap_pm_get_dev_context_loss_count;
	aess->device_scale = NULL;
#endif
	aess->dev = &pdev->dev;

	mutex_init(&aess->mutex);
	mutex_init(&aess->opp.mutex);
	mutex_init(&aess->opp.req_mutex);
	INIT_LIST_HEAD(&aess->opp.req);

	spin_lock_init(&aess->lock);
	INIT_LIST_HEAD(&aess->ports);

	omap_abe_port_mgr_init(aess);

	get_device(aess->dev);
	aess->dev->dma_mask = &omap_abe_dmamask;
	aess->dev->coherent_dma_mask = omap_abe_dmamask;
	put_device(aess->dev);

	ret = request_firmware_nowait(THIS_MODULE, 1, ABE_FW_NAME, aess->dev,
				      GFP_KERNEL, pdev, abe_fw_ready);
	if (ret)
		dev_err(aess->dev, "Failed to load firmware %s: %d\n",
			ABE_FW_NAME, ret);

	return ret;
}

static int abe_engine_remove(struct platform_device *pdev)
{
	struct omap_aess *aess = dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_component(&pdev->dev);
	snd_soc_unregister_platform(&pdev->dev);

	aess->fw_data = NULL;
	aess->fw_config = NULL;
	release_firmware(aess->fw);
	omap_abe_port_mgr_cleanup(aess);

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
