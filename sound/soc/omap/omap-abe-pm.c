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

#include <linux/export.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>

#include <sound/soc.h>

#include "omap-abe-priv.h"

void omap_abe_pm_get(struct snd_soc_platform *platform)
{
	struct omap_abe *abe = snd_soc_platform_get_drvdata(platform);
	pm_runtime_get_sync(abe->dev);
}
EXPORT_SYMBOL_GPL(omap_abe_pm_get);

void omap_abe_pm_put(struct snd_soc_platform *platform)
{
	struct omap_abe *abe = snd_soc_platform_get_drvdata(platform);
	pm_runtime_put_sync(abe->dev);
}
EXPORT_SYMBOL_GPL(omap_abe_pm_put);

void omap_abe_pm_shutdown(struct snd_soc_platform *platform)
{
	struct omap_abe *abe = snd_soc_platform_get_drvdata(platform);
	int ret;

	if (abe->active && omap_aess_check_activity(abe->aess))
		return;

	omap_aess_set_opp_processing(abe->aess, ABE_OPP25);
	abe->opp.level = 25;

	omap_aess_stop_event_generator(abe->aess);
	udelay(250);
	if (abe->device_scale) {
		ret = abe->device_scale(abe->dev, abe->dev, abe->opp.freqs[0]);
		if (ret)
			dev_err(abe->dev, "failed to scale to lowest OPP\n");
	}
}
EXPORT_SYMBOL_GPL(omap_abe_pm_shutdown);

void omap_abe_pm_set_mode(struct snd_soc_platform *platform, int mode)
{
	struct omap_abe *abe = snd_soc_platform_get_drvdata(platform);

	abe->dc_offset.power_mode = mode;
}
EXPORT_SYMBOL(omap_abe_pm_set_mode);
