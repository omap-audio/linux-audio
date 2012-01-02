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
#include <sound/omap-abe.h>

#include "omap-abe-priv.h"
#include "abe/abe_main.h"

int abe_mixer_enable_mono(struct omap_abe* abe, int id, int enable);
int abe_mixer_set_equ_profile(struct omap_abe* abe,
		unsigned int id, unsigned int profile);

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
	struct omap_abe_pdata *pdata = abe->pdata;
	int ret;

	if (abe->active && abe_check_activity())
		return;

	abe_set_opp_processing(ABE_OPP25);
	abe->opp.level = 25;

	abe_stop_event_generator();
	udelay(250);

	if (pdata && pdata->device_scale) {
		ret = pdata->device_scale(abe->dev, abe->dev, abe->opp.freqs[0]);
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

int abe_pm_save_context(struct omap_abe *abe)
{
	/* mute gains not associated with FEs/BEs */
	abe_mute_gain(MIXAUDUL, MIX_AUDUL_INPUT_MM_DL);
	abe_mute_gain(MIXAUDUL, MIX_AUDUL_INPUT_TONES);
	abe_mute_gain(MIXAUDUL, MIX_AUDUL_INPUT_VX_DL);
	abe_mute_gain(MIXVXREC, MIX_VXREC_INPUT_TONES);
	abe_mute_gain(MIXVXREC, MIX_VXREC_INPUT_VX_DL);
	abe_mute_gain(MIXVXREC, MIX_VXREC_INPUT_MM_DL);
	abe_mute_gain(MIXVXREC, MIX_VXREC_INPUT_VX_UL);
	abe_mute_gain(MIXECHO, MIX_ECHO_DL1);
	abe_mute_gain(MIXECHO, MIX_ECHO_DL2);

	return 0;
}

int abe_pm_restore_context(struct omap_abe *abe)
{
	struct omap_abe_pdata *pdata = abe->pdata;
	int i, ret;

	if (pdata && pdata->device_scale) {
		ret = pdata->device_scale(abe->dev, abe->dev,
				abe->opp.freqs[OMAP_ABE_OPP_50]);
		if (ret) {
			dev_err(abe->dev, "failed to scale to OPP 50\n");
			return ret;
		}
	}

	/* unmute gains not associated with FEs/BEs */
	abe_unmute_gain(MIXAUDUL, MIX_AUDUL_INPUT_MM_DL);
	abe_unmute_gain(MIXAUDUL, MIX_AUDUL_INPUT_TONES);
	abe_unmute_gain(MIXAUDUL, MIX_AUDUL_INPUT_VX_DL);
	abe_unmute_gain(MIXVXREC, MIX_VXREC_INPUT_TONES);
	abe_unmute_gain(MIXVXREC, MIX_VXREC_INPUT_VX_DL);
	abe_unmute_gain(MIXVXREC, MIX_VXREC_INPUT_MM_DL);
	abe_unmute_gain(MIXVXREC, MIX_VXREC_INPUT_VX_UL);
	abe_unmute_gain(MIXECHO, MIX_ECHO_DL1);
	abe_unmute_gain(MIXECHO, MIX_ECHO_DL2);
	abe_set_router_configuration(UPROUTE, 0, (u32 *)abe->mixer.route_ul);

	/* DC offset cancellation setting */
	if (abe->dc_offset.power_mode)
		abe_write_pdmdl_offset(1, abe->dc_offset.hsl * 2, abe->dc_offset.hsr * 2);
	else
		abe_write_pdmdl_offset(1, abe->dc_offset.hsl, abe->dc_offset.hsr);

	abe_write_pdmdl_offset(2, abe->dc_offset.hfl, abe->dc_offset.hfr);

	for (i = 0; i < abe->hdr.num_equ; i++)
		abe_mixer_set_equ_profile(abe, i, abe->equ.profile[i]);

	for (i = 0; i < OMAP_ABE_NUM_MONO_MIXERS; i++)
		abe_mixer_enable_mono(abe, MIX_DL1_MONO + i, abe->mixer.mono[i]);

       return 0;
}

#ifdef CONFIG_PM
int abe_pm_suspend(struct snd_soc_dai *dai)
{
	struct omap_abe *abe = snd_soc_dai_get_drvdata(dai);
	int ret = 0;

	dev_dbg(dai->dev, "%s: %s active %d\n",
		__func__, dai->name, dai->active);

	if (!dai->active)
		return 0;

	pm_runtime_get_sync(abe->dev);

	switch (dai->id) {
	case OMAP_ABE_DAI_PDM_UL:
		abe_mute_gain(GAINS_AMIC, GAIN_LEFT_OFFSET);
		abe_mute_gain(GAINS_AMIC, GAIN_RIGHT_OFFSET);
		break;
	case OMAP_ABE_DAI_PDM_DL1:
	case OMAP_ABE_DAI_PDM_DL2:
	case OMAP_ABE_DAI_PDM_VIB:
		break;
	case OMAP_ABE_DAI_BT_VX:
		abe_mute_gain(GAINS_BTUL, GAIN_LEFT_OFFSET);
		abe_mute_gain(GAINS_BTUL, GAIN_RIGHT_OFFSET);
		break;
	case OMAP_ABE_DAI_MM_FM:
	case OMAP_ABE_DAI_MODEM:
		break;
	case OMAP_ABE_DAI_DMIC0:
		abe_mute_gain(GAINS_DMIC1, GAIN_LEFT_OFFSET);
		abe_mute_gain(GAINS_DMIC1, GAIN_RIGHT_OFFSET);
		break;
	case OMAP_ABE_DAI_DMIC1:
		abe_mute_gain(GAINS_DMIC2, GAIN_LEFT_OFFSET);
		abe_mute_gain(GAINS_DMIC2, GAIN_RIGHT_OFFSET);
		break;
	case OMAP_ABE_DAI_DMIC2:
		abe_mute_gain(GAINS_DMIC3, GAIN_LEFT_OFFSET);
		abe_mute_gain(GAINS_DMIC3, GAIN_RIGHT_OFFSET);
		break;
	default:
		dev_err(dai->dev, "%s: invalid DAI id %d\n",
				__func__, dai->id);
		break;
	}

	pm_runtime_put_sync(abe->dev);
	return ret;
}

int abe_pm_resume(struct snd_soc_dai *dai)
{
	struct omap_abe *abe = snd_soc_dai_get_drvdata(dai);
	struct omap_abe_pdata *pdata = abe->pdata;
	int i, ret = 0;

	dev_dbg(dai->dev, "%s: %s active %d\n",
		__func__, dai->name, dai->active);

	if (!dai->active)
		return 0;

	/* context retained, no need to restore */
	if (pdata->was_context_lost && !pdata->was_context_lost(abe->dev))
		return 0;

	pm_runtime_get_sync(abe->dev);

	if (pdata && pdata->device_scale) {
		ret = pdata->device_scale(abe->dev, abe->dev,
				abe->opp.freqs[OMAP_ABE_OPP_50]);
		if (ret) {
			dev_err(abe->dev, "failed to scale to OPP 50\n");
			goto out;
		}
	}

	abe_reload_fw(abe->firmware);

	switch (dai->id) {
	case OMAP_ABE_DAI_PDM_UL:
		abe_unmute_gain(GAINS_AMIC, GAIN_LEFT_OFFSET);
		abe_unmute_gain(GAINS_AMIC, GAIN_RIGHT_OFFSET);
		break;
	case OMAP_ABE_DAI_PDM_DL1:
	case OMAP_ABE_DAI_PDM_DL2:
	case OMAP_ABE_DAI_PDM_VIB:
		break;
	case OMAP_ABE_DAI_BT_VX:
		abe_unmute_gain(GAINS_BTUL, GAIN_LEFT_OFFSET);
		abe_unmute_gain(GAINS_BTUL, GAIN_RIGHT_OFFSET);
		break;
	case OMAP_ABE_DAI_MM_FM:
	case OMAP_ABE_DAI_MODEM:
		break;
	case OMAP_ABE_DAI_DMIC0:
		abe_unmute_gain(GAINS_DMIC1, GAIN_LEFT_OFFSET);
		abe_unmute_gain(GAINS_DMIC1, GAIN_RIGHT_OFFSET);
		break;
	case OMAP_ABE_DAI_DMIC1:
		abe_unmute_gain(GAINS_DMIC2, GAIN_LEFT_OFFSET);
		abe_unmute_gain(GAINS_DMIC2, GAIN_RIGHT_OFFSET);
		break;
	case OMAP_ABE_DAI_DMIC2:
		abe_unmute_gain(GAINS_DMIC3, GAIN_LEFT_OFFSET);
		abe_unmute_gain(GAINS_DMIC3, GAIN_RIGHT_OFFSET);
		break;
	default:
		dev_err(dai->dev, "%s: invalid DAI id %d\n",
				__func__, dai->id);
		ret = -EINVAL;
		goto out;
	}

	abe_set_router_configuration(UPROUTE, 0, (u32 *)abe->mixer.route_ul);

	if (abe->dc_offset.power_mode)
		abe_write_pdmdl_offset(1, abe->dc_offset.hsl * 2, abe->dc_offset.hsr * 2);
	else
		abe_write_pdmdl_offset(1, abe->dc_offset.hsl, abe->dc_offset.hsr);

	abe_write_pdmdl_offset(2, abe->dc_offset.hfl, abe->dc_offset.hfr);

	for (i = 0; i < abe->hdr.num_equ; i++)
		abe_mixer_set_equ_profile(abe, i, abe->equ.profile[i]);

	for (i = 0; i < OMAP_ABE_NUM_MONO_MIXERS; i++)
		abe_mixer_enable_mono(abe, MIX_DL1_MONO + i, abe->mixer.mono[i]);
out:
	pm_runtime_put_sync(abe->dev);
	return ret;
}
#else
#define abe_suspend	NULL
#define abe_resume	NULL
#endif

