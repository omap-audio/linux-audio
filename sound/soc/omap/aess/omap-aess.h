/*
 * omap-aess.h
 *
 * Copyright (C) 2010 Texas Instruments
 *
 * Author: Liam Girdwood <lrg@ti.com>
 * Contact: Peter Ujfalusi <peter.ujfalusi@ti.com>
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

#ifndef __OMAP_AESS_H__
#define __OMAP_AESS_H__

/* This must currently match the BE order in DSP */
#define OMAP_ABE_DAI_PDM_UL			0
#define OMAP_ABE_DAI_PDM_DL1			1
#define OMAP_ABE_DAI_PDM_DL2			2
#define OMAP_ABE_DAI_BT_VX			3
#define OMAP_ABE_DAI_MM_FM			4
#define OMAP_ABE_DAI_MODEM			5
#define OMAP_ABE_DAI_DMIC0			6
#define OMAP_ABE_DAI_DMIC1			7
#define OMAP_ABE_DAI_DMIC2			8
#define OMAP_ABE_DAI_VXREC			9
#define OMAP_ABE_DAI_NUM			10

/* Power Management */
void omap_abe_pm_shutdown(struct snd_soc_platform *platform);
void omap_abe_pm_get(struct snd_soc_platform *platform);
void omap_abe_pm_put(struct snd_soc_platform *platform);
void omap_abe_pm_set_mode(struct snd_soc_platform *platform, int mode);

/* Operating Point */
int omap_abe_opp_new_request(struct snd_soc_platform *platform,
		struct device *dev, int opp);
int omap_abe_opp_free_request(struct snd_soc_platform *platform,
		struct device *dev);

/* DC Offset */
void omap_abe_dc_set_hs_offset(struct snd_soc_platform *platform,
	int left, int right, int step_mV);
void omap_abe_dc_set_hf_offset(struct snd_soc_platform *platform,
	int left, int right);
void omap_abe_set_dl1_gains(struct snd_soc_platform *platform,
	int left, int right);

#endif	/* End of __OMAP_AESS_H__ */
