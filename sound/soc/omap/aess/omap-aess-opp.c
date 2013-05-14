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

#include <linux/pm_runtime.h>
#include <linux/opp.h>
#include <linux/slab.h>
#include <linux/export.h>
#include <linux/delay.h>

#include <sound/soc.h>

#include "omap-aess-priv.h"
#include "abe_mem.h"

struct abe_opp_req {
	struct device *dev;
	struct list_head node;
	int opp;
};

static struct abe_opp_req *abe_opp_lookup_requested(struct omap_aess *aess,
					struct device *dev)
{
	struct abe_opp_req *req;

	list_for_each_entry(req, &aess->opp.req, node) {
		if (req->dev == dev)
			return req;
	}

	return NULL;
}

static int abe_opp_get_requested(struct omap_aess *aess)
{
	struct abe_opp_req *req;
	int opp = 0;

	list_for_each_entry(req, &aess->opp.req, node)
		opp |= req->opp;

	opp = (1 << (fls(opp) - 1)) * 25;

	return opp;
}

int abe_opp_init_initial_opp(struct omap_aess *aess)
{
	struct opp *opp;
	int opp_count, ret = 0, i;
	unsigned long freq = ULONG_MAX;

	aess->opp.req_count = 0;

	/* query supported opps */
	rcu_read_lock();
	opp_count = opp_get_opp_count(aess->dev);
	if (opp_count <= 0) {
		dev_err(aess->dev, "opp: no OPP data\n");
		ret = opp_count;
		goto out;
	} else if (opp_count > OMAP_ABE_OPP_COUNT) {
		dev_err(aess->dev, "opp: unsupported OPP count %d (max:%d)\n",
			opp_count, OMAP_ABE_OPP_COUNT);
		ret = -EINVAL;
		goto out;
	}

	/* assume provided opps are always higher */
	for (i = OMAP_ABE_OPP_COUNT - 1; i >= 0; i--) {

		opp = opp_find_freq_floor(aess->dev, &freq);
		if (IS_ERR_OR_NULL(opp))
			break;

		aess->opp.freqs[i] = freq;

		/* prepare to obtain next available opp */
		freq--;
	}

	/* use lowest available opp for non-populated items */
	for (freq++; i >= 0; i--)
		aess->opp.freqs[i] = freq;

out:
	rcu_read_unlock();
	return ret;
}

/**
 * abe_set_opp_processing - Set OPP mode for ABE Firmware
 * @aess: Pointer on aess handle
 * @level: OOPP level
 *
 * New processing network and OPP:
 * 0: Ultra Lowest power consumption audio player (no post-processing, no mixer)
 * 1: OPP 25% (simple multimedia features, including low-power player)
 * 2: OPP 50% (multimedia and voice calls)
 * 3: OPP100% ( multimedia complex use-cases)
 *
 * Rearranges the FW task network to the corresponding OPP list of features.
 * The corresponding AE ports are supposed to be set/reset accordingly before
 * this switch.
 *
 */
int omap_aess_set_opp_processing(struct omap_aess *aess, enum opp_level level)
{
	u32 dOppMode32;

	switch (level) {
	case OMAP_ABE_OPP_25:
		/* OPP25% */
		dOppMode32 = DOPPMODE32_OPP25;
		break;
	case OMAP_ABE_OPP_50:
		/* OPP50% */
		dOppMode32 = DOPPMODE32_OPP50;
		break;
	default:
		dev_warn(aess->dev, "Bad OPP value requested (%d)\n", level);
	case OMAP_ABE_OPP_100:
		/* OPP100% */
		dOppMode32 = DOPPMODE32_OPP100;
		break;
	}
	/* Write Multiframe inside DMEM */
	omap_aess_write_map(aess, OMAP_AESS_DMEM_MAXTASKBYTESINSLOT_ID,
			    &dOppMode32);

	return 0;

}

int abe_opp_set_level(struct omap_aess *aess, int opp)
{
	int ret = 0;

	if (aess->opp.level > opp) {
		/* Decrease OPP mode - no need of OPP100% */
		switch (opp) {
		case 25:
			omap_aess_set_opp_processing(aess, OMAP_ABE_OPP_25);
			udelay(250);
			if (aess->device_scale) {
				ret = aess->device_scale(aess->dev, aess->dev,
					aess->opp.freqs[OMAP_ABE_OPP_25]);
				if (ret)
					goto err_down_scale;
			}

			break;
		case 50:
		default:
			omap_aess_set_opp_processing(aess, OMAP_ABE_OPP_50);
			udelay(250);
			if (aess->device_scale) {
				ret = aess->device_scale(aess->dev, aess->dev,
					aess->opp.freqs[OMAP_ABE_OPP_50]);
				if (ret)
					goto err_down_scale;
			}

			break;
		}
	} else if (aess->opp.level < opp) {
		/* Increase OPP mode */
		switch (opp) {
		case 25:
			if (aess->device_scale) {
				aess->device_scale(aess->dev, aess->dev,
					aess->opp.freqs[OMAP_ABE_OPP_25]);
				if (ret)
					goto err_up_scale;
			}

			omap_aess_set_opp_processing(aess, OMAP_ABE_OPP_25);
			break;
		case 50:
			if (aess->device_scale) {
				ret = aess->device_scale(aess->dev, aess->dev,
					aess->opp.freqs[OMAP_ABE_OPP_50]);
				if (ret)
					goto err_up_scale;
			}
			omap_aess_set_opp_processing(aess, OMAP_ABE_OPP_50);
			break;
		case 100:
		default:
			if (aess->device_scale) {
				ret = aess->device_scale(aess->dev, aess->dev,
					aess->opp.freqs[OMAP_ABE_OPP_100]);
				if (ret)
					goto err_up_scale;
			}
			omap_aess_set_opp_processing(aess, OMAP_ABE_OPP_100);
			break;
		}
	}
	aess->opp.level = opp;
	dev_dbg(aess->dev, "opp: new OPP level is %d\n", opp);

	return 0;

err_down_scale:
	/* revert old to OPP ABE processing to keep ABE and MPU in sync */
	dev_err(aess->dev, "opp: failed to scale OPP - reverting to %d\n",
			aess->opp.level);
	switch (aess->opp.level) {
	case 25:
		omap_aess_set_opp_processing(aess, OMAP_ABE_OPP_25);
		break;
	case 50:
		omap_aess_set_opp_processing(aess, OMAP_ABE_OPP_50);
		break;
	case 100:
		omap_aess_set_opp_processing(aess, OMAP_ABE_OPP_100);
		break;
	}
	udelay(250);

err_up_scale:
	dev_err(aess->dev, "opp: failed to scale to OPP%d\n", opp);
	return ret;
}

int abe_opp_recalc_level(struct omap_aess *aess)
{
	int i, requested_opp, opp = 0;

	mutex_lock(&aess->opp.mutex);

	/* now calculate OPP level based upon DAPM widget status */
	for (i = 0; i < OMAP_ABE_NUM_WIDGETS; i++) {
		if (aess->opp.widget[OMAP_ABE_WIDGET(i)]) {
			dev_dbg(aess->dev, "opp: id %d = %d%%\n", i,
					aess->opp.widget[OMAP_ABE_WIDGET(i)] * 25);
			opp |= aess->opp.widget[OMAP_ABE_WIDGET(i)];
		}
	}
	opp = (1 << (fls(opp) - 1)) * 25;

	/* OPP requested outside ABE driver (e.g. McPDM) */
	requested_opp = abe_opp_get_requested(aess);
	dev_dbg(aess->dev, "opp: calculated %d requested %d selected %d\n",
		opp, requested_opp, max(opp, requested_opp));

	pm_runtime_get_sync(aess->dev);
	abe_opp_set_level(aess, max(opp, requested_opp));
	pm_runtime_put_sync(aess->dev);

	mutex_unlock(&aess->opp.mutex);
	return 0;
}

int abe_opp_stream_event(struct snd_soc_dapm_context *dapm, int event)
{
	struct snd_soc_platform *platform = dapm->platform;
	struct omap_aess *aess = snd_soc_platform_get_drvdata(platform);

	if (aess->active) {
		dev_dbg(aess->dev, "opp: stream event %d\n", event);
		abe_opp_recalc_level(aess);
	}

	return 0;
}

int omap_abe_opp_new_request(struct snd_soc_platform *platform,
		struct device *dev, int opp)
{
	struct omap_aess *aess = snd_soc_platform_get_drvdata(platform);
	struct abe_opp_req *req;
	int ret = 0;

	mutex_lock(&aess->opp.req_mutex);

	req = abe_opp_lookup_requested(aess, dev);
	if (!req) {
		req = kzalloc(sizeof(struct abe_opp_req), GFP_KERNEL);
		if (!req) {
			ret = -ENOMEM;
			goto out;
		}

		req->dev = dev;
		req->opp = 1 << opp; /* use the same convention as ABE DSP DAPM */

		list_add(&req->node, &aess->opp.req);
		dev_dbg(aess->dev, "opp: new constraint %d from %s\n", opp,
			dev_name(dev));
		aess->opp.req_count++;
	} else
		req->opp = opp;

	abe_opp_recalc_level(aess);

out:
	mutex_unlock(&aess->opp.req_mutex);
	return ret;
}

int omap_abe_opp_free_request(struct snd_soc_platform *platform,
		struct device *dev)
{
	struct omap_aess *aess = snd_soc_platform_get_drvdata(platform);
	struct abe_opp_req *req;
	int ret;

	mutex_lock(&aess->opp.req_mutex);

	req = abe_opp_lookup_requested(aess, dev);
	if (!req) {
		dev_err(dev, "opp: trying to remove an invalid OPP request\n");
		ret = -EINVAL;
		goto out;
	}

	dev_dbg(aess->dev, "opp: free constraint %d from %s\n", req->opp,
			dev_name(dev));

	list_del(&req->node);
	aess->opp.req_count--;
	kfree(req);

	abe_opp_recalc_level(aess);

out:
	mutex_unlock(&aess->opp.req_mutex);
	return ret;
}
