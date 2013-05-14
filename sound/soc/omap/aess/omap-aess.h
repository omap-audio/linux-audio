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
enum dai_id {
	OMAP_ABE_DAI_PDM_UL = 0,
	OMAP_ABE_DAI_PDM_DL1,
	OMAP_ABE_DAI_PDM_DL2,
	OMAP_ABE_DAI_BT_VX,
	OMAP_ABE_DAI_MM_FM,
	OMAP_ABE_DAI_MODEM,
	OMAP_ABE_DAI_DMIC0,
	OMAP_ABE_DAI_DMIC1,
	OMAP_ABE_DAI_DMIC2,
	OMAP_ABE_DAI_VXREC,

	OMAP_ABE_DAI_NUM,
};

enum port_id {
	/* Logical PORT IDs - Backend */
	OMAP_ABE_BE_PORT_DMIC0 = 0,
	OMAP_ABE_BE_PORT_DMIC1,
	OMAP_ABE_BE_PORT_DMIC2,
	OMAP_ABE_BE_PORT_PDM_DL1,
	OMAP_ABE_BE_PORT_PDM_DL2,
	OMAP_ABE_BE_PORT_MCASP,
	OMAP_ABE_BE_PORT_PDM_UL1,
	OMAP_ABE_BE_PORT_BT_VX_DL,
	OMAP_ABE_BE_PORT_BT_VX_UL,
	OMAP_ABE_BE_PORT_MM_EXT_UL,
	OMAP_ABE_BE_PORT_MM_EXT_DL,

	/* Logical PORT IDs - Frontend */
	OMAP_ABE_FE_PORT_MM_DL1,
	OMAP_ABE_FE_PORT_MM_UL1,
	OMAP_ABE_FE_PORT_MM_UL2,
	OMAP_ABE_FE_PORT_VX_DL,
	OMAP_ABE_FE_PORT_VX_UL,
	OMAP_ABE_FE_PORT_TONES,
	OMAP_ABE_FE_PORT_MM_DL_LP,

	OMAP_ABE_PORT_ID_LAST,
};

struct snd_soc_platform;
struct omap_aess;

#if IS_ENABLED(CONFIG_SND_OMAP_SOC_AESS)
int omap_abe_port_open(struct omap_aess *aess, int logical_id);
void omap_abe_port_close(struct omap_aess *aess, int logical_id);
int omap_abe_port_enable(struct omap_aess *aess, int logical_id);
int omap_abe_port_disable(struct omap_aess *aess, int logical_id);
int omap_abe_port_is_enabled(struct omap_aess *aess, int logical_id);

struct omap_aess *omap_abe_port_mgr_get(void);
void omap_abe_port_mgr_put(struct omap_aess *aess);
#else
static inline int omap_abe_port_open(struct omap_aess *aess, int logical_id)
{
	return 0;
}

static inline void omap_abe_port_close(struct omap_aess *aess, int logical_id)
{
}

static inline int omap_abe_port_enable(struct omap_aess *aess, int logical_id)
{
	return 0;
}

static inline int omap_abe_port_disable(struct omap_aess *aess, int logical_id)
{
	return 0;
}

static inline int omap_abe_port_is_enabled(struct omap_aess *aess,
					   int logical_id)
{
	return 0;
}

static inline struct omap_aess *omap_abe_port_mgr_get(void)
{
	return NULL;
}

static inline void omap_abe_port_mgr_put(struct omap_aess *aess)
{
}
#endif /* IS_ENABLED(CONFIG_SND_OMAP_SOC_AESS) */

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
