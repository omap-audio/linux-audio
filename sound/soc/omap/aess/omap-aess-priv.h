/*
 * omap-aess-priv.h
 *
 * Copyright (C) 2010 Texas Instruments
 *
 * Contact: Liam Girdwood <lrg@ti.com>
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

#ifndef __OMAP_AESS_PRIV_H__
#define __OMAP_AESS_PRIV_H__

#ifdef __KERNEL__

#include <sound/soc.h>
#include <sound/soc-fw.h>

#include "omap-aess.h"
#include "aess-fw.h"

#endif

#define OMAP_ABE_FRONTEND_DAI_MEDIA		0
#define OMAP_ABE_FRONTEND_DAI_MEDIA_CAPTURE	1
#define OMAP_ABE_FRONTEND_DAI_VOICE		2
#define OMAP_ABE_FRONTEND_DAI_TONES		3
#define OMAP_ABE_FRONTEND_DAI_MODEM		4
#define OMAP_ABE_FRONTEND_DAI_LP_MEDIA		5
#define OMAP_ABE_FRONTEND_DAI_NUM		6

#define OMAP_ABE_MIXER(x)		(x)


#define MIX_SWITCH_PDM_DL		OMAP_ABE_MIXER(1)
#define MIX_SWITCH_BT_VX_DL		OMAP_ABE_MIXER(2)
#define MIX_SWITCH_MM_EXT_DL		OMAP_ABE_MIXER(3)
#define MIX_DL1_MONO		OMAP_ABE_MIXER(4)
#define MIX_DL2_MONO		OMAP_ABE_MIXER(5)
#define MIX_AUDUL_MONO		OMAP_ABE_MIXER(6)

#define OMAP_ABE_VIRTUAL_SWITCH	36

#define OMAP_ABE_NUM_MONO_MIXERS	(MIX_AUDUL_MONO - MIX_DL1_MONO + 1)
#define OMAP_ABE_NUM_MIXERS		(MIX_AUDUL_MONO + 1)

#define OMAP_ABE_MUX(x)		(x + 37)

#define MUX_MM_UL10		OMAP_ABE_MUX(0)
#define MUX_MM_UL11		OMAP_ABE_MUX(1)
#define MUX_MM_UL12		OMAP_ABE_MUX(2)
#define MUX_MM_UL13		OMAP_ABE_MUX(3)
#define MUX_MM_UL14		OMAP_ABE_MUX(4)
#define MUX_MM_UL15		OMAP_ABE_MUX(5)
#define MUX_MM_UL20		OMAP_ABE_MUX(6)
#define MUX_MM_UL21		OMAP_ABE_MUX(7)
#define MUX_VX_UL0		OMAP_ABE_MUX(8)
#define MUX_VX_UL1		OMAP_ABE_MUX(9)

#define OMAP_ABE_NUM_MUXES		(MUX_VX_UL1 - MUX_MM_UL10)

#define OMAP_ABE_WIDGET(x)		(x + OMAP_ABE_NUM_MIXERS + OMAP_ABE_NUM_MUXES)

/* ABE AIF Frontend Widgets */
#define OMAP_ABE_AIF_TONES_DL		OMAP_ABE_WIDGET(0)
#define OMAP_ABE_AIF_VX_DL		OMAP_ABE_WIDGET(1)
#define OMAP_ABE_AIF_VX_UL		OMAP_ABE_WIDGET(2)
#define OMAP_ABE_AIF_MM_UL1		OMAP_ABE_WIDGET(3)
#define OMAP_ABE_AIF_MM_UL2		OMAP_ABE_WIDGET(4)
#define OMAP_ABE_AIF_MM_DL		OMAP_ABE_WIDGET(5)
#define OMAP_ABE_AIF_MM_DL_LP		OMAP_ABE_AIF_MM_DL
#define OMAP_ABE_AIF_MODEM_DL		OMAP_ABE_WIDGET(6)
#define OMAP_ABE_AIF_MODEM_UL		OMAP_ABE_WIDGET(7)

/* ABE AIF Backend Widgets */
#define OMAP_ABE_AIF_PDM_UL1		OMAP_ABE_WIDGET(8)
#define OMAP_ABE_AIF_PDM_DL1		OMAP_ABE_WIDGET(9)
#define OMAP_ABE_AIF_PDM_DL2		OMAP_ABE_WIDGET(10)
#define OMAP_ABE_AIF_BT_VX_UL		OMAP_ABE_WIDGET(11)
#define OMAP_ABE_AIF_BT_VX_DL		OMAP_ABE_WIDGET(12)
#define OMAP_ABE_AIF_MM_EXT_UL	OMAP_ABE_WIDGET(13)
#define OMAP_ABE_AIF_MM_EXT_DL	OMAP_ABE_WIDGET(14)
#define OMAP_ABE_AIF_DMIC0		OMAP_ABE_WIDGET(15)
#define OMAP_ABE_AIF_DMIC1		OMAP_ABE_WIDGET(16)
#define OMAP_ABE_AIF_DMIC2		OMAP_ABE_WIDGET(17)

/* ABE ROUTE_UL MUX Widgets */
#define OMAP_ABE_MUX_UL00		OMAP_ABE_WIDGET(18)
#define OMAP_ABE_MUX_UL01		OMAP_ABE_WIDGET(19)
#define OMAP_ABE_MUX_UL02		OMAP_ABE_WIDGET(20)
#define OMAP_ABE_MUX_UL03		OMAP_ABE_WIDGET(21)
#define OMAP_ABE_MUX_UL04		OMAP_ABE_WIDGET(22)
#define OMAP_ABE_MUX_UL05		OMAP_ABE_WIDGET(23)
#define OMAP_ABE_MUX_UL10		OMAP_ABE_WIDGET(24)
#define OMAP_ABE_MUX_UL11		OMAP_ABE_WIDGET(25)
#define OMAP_ABE_MUX_VX00		OMAP_ABE_WIDGET(26)
#define OMAP_ABE_MUX_VX01		OMAP_ABE_WIDGET(27)

/* ABE Volume and Mixer Widgets */
#define OMAP_ABE_MIXER_DL1		OMAP_ABE_WIDGET(28)
#define OMAP_ABE_MIXER_DL2		OMAP_ABE_WIDGET(29)
#define OMAP_ABE_VOLUME_DL1		OMAP_ABE_WIDGET(30)
#define OMAP_ABE_MIXER_AUDIO_UL	OMAP_ABE_WIDGET(31)
#define OMAP_ABE_MIXER_VX_REC		OMAP_ABE_WIDGET(32)
#define OMAP_ABE_MIXER_SDT		OMAP_ABE_WIDGET(33)
#define OMAP_ABE_VSWITCH_DL1_PDM	OMAP_ABE_WIDGET(34)
#define OMAP_ABE_VSWITCH_DL1_BT_VX	OMAP_ABE_WIDGET(35)
#define OMAP_ABE_VSWITCH_DL1_MM_EXT	OMAP_ABE_WIDGET(36)
#define OMAP_ABE_AIF_VXREC		OMAP_ABE_WIDGET(37)

#define OMAP_ABE_NUM_WIDGETS		(OMAP_ABE_AIF_VXREC - OMAP_ABE_AIF_TONES_DL)
#define OMAP_ABE_WIDGET_LAST		OMAP_ABE_AIF_VXREC

#define OMAP_ABE_NUM_DAPM_REG		\
	(OMAP_ABE_NUM_MIXERS + OMAP_ABE_NUM_MUXES + OMAP_ABE_NUM_WIDGETS)

#define OMAP_ABE_ROUTES_UL		14

/* Firmware coefficients and equalizers */
#define OMAP_ABE_MAX_FW_SIZE		(1024 * 128)
#define OMAP_ABE_MAX_COEFF_SIZE	(1024 * 4)
#define OMAP_ABE_COEFF_NAME_SIZE	20
#define OMAP_ABE_COEFF_TEXT_SIZE	20
#define OMAP_ABE_COEFF_NUM_TEXTS	10
#define OMAP_ABE_MAX_EQU		10
#define OMAP_ABE_MAX_PROFILES	30

/* TODO: we need the names for each array memeber */
#define OMAP_ABE_IO_RESOURCES	5
#define OMAP_ABE_IO_DMEM 0
#define OMAP_ABE_IO_CMEM 1
#define OMAP_ABE_IO_SMEM 2
#define OMAP_ABE_IO_PMEM 3
#define OMAP_ABE_IO_AESS 4

#define OMAP_ABE_EQU_AMIC	0
#define OMAP_ABE_EQU_DL1	0
#define OMAP_ABE_EQU_DL2L	0
#define OMAP_ABE_EQU_DL2R	0
#define OMAP_ABE_EQU_DMIC	0
#define OMAP_ABE_EQU_SDT	0

/* TODO: combine / sort */
#define OMAP_ABE_MIXER_DEFAULT	128
#define OMAP_ABE_MIXER_MONO	129
#define OMAP_ABE_MIXER_ROUTER	130
#define OMAP_ABE_MIXER_EQU	131
#define OMAP_ABE_MIXER_SWITCH	132
#define OMAP_ABE_MIXER_GAIN	133
#define OMAP_ABE_MIXER_VOLUME	134

#define OMAP_CONTROL_DEFAULT \
	SOC_CONTROL_ID(OMAP_ABE_MIXER_DEFAULT, \
		OMAP_ABE_MIXER_DEFAULT, \
		SOC_CONTROL_TYPE_EXT)
#define OMAP_CONTROL_MONO \
	SOC_CONTROL_ID(OMAP_ABE_MIXER_MONO, \
		OMAP_ABE_MIXER_MONO, \
		SOC_CONTROL_TYPE_VOLSW)
#define OMAP_CONTROL_ROUTER \
	SOC_CONTROL_ID(OMAP_ABE_MIXER_ROUTER, \
		OMAP_ABE_MIXER_ROUTER, \
		SOC_CONTROL_TYPE_ENUM)
#define OMAP_CONTROL_EQU \
	SOC_CONTROL_ID(OMAP_ABE_MIXER_EQU, \
		OMAP_ABE_MIXER_EQU, \
		SOC_CONTROL_TYPE_ENUM)
#define OMAP_CONTROL_SWITCH \
	SOC_CONTROL_ID(OMAP_ABE_MIXER_DEFAULT, \
		OMAP_ABE_MIXER_SWITCH, \
		SOC_CONTROL_TYPE_VOLSW)
#define OMAP_CONTROL_GAIN \
	SOC_CONTROL_ID(OMAP_ABE_MIXER_GAIN, \
		OMAP_ABE_MIXER_GAIN, \
		SOC_CONTROL_TYPE_VOLSW)
#define OMAP_CONTROL_VOLUME \
	SOC_CONTROL_ID(OMAP_ABE_MIXER_VOLUME, \
		OMAP_ABE_MIXER_VOLUME, \
		SOC_CONTROL_TYPE_VOLSW)

#define OMAP_ABE_DMA_RESOURCES	8

#ifdef __KERNEL__

/*
 * ABE Firmware Header.
 * The ABE firmware blob has a header that describes each data section. This
 * way we can store coefficients etc in the firmware.
 */
struct fw_header {
	u32 version;	/* min version of ABE firmware required */
	u32 pmem_size;
	u32 cmem_size;
	u32 dmem_size;
	u32 smem_size;
};

struct omap_aess_dc_offset {
	/* DC offset cancellation */
	int power_mode;
	u32 hsl;
	u32 hsr;
	u32 hfl;
	u32 hfr;
};

enum opp_level {
	OMAP_ABE_OPP_25 = 0,
	OMAP_ABE_OPP_50,
	OMAP_ABE_OPP_100,
	OMAP_ABE_OPP_COUNT,
};

struct omap_aess_opp {
	struct mutex mutex;
	struct mutex req_mutex;
	int level;
	unsigned long freqs[OMAP_ABE_OPP_COUNT];
	u32 widget[OMAP_ABE_NUM_DAPM_REG + 1];
	struct list_head req;
	int req_count;
};

struct omap_aess_modem {
	struct snd_pcm_substream *substream[2];
	struct snd_soc_dai *dai;
};

struct omap_aess_coeff {
	int profile; /* current enabled profile */
	int num_profiles;
	int profile_size;
	void *coeff_data;
};

struct omap_aess_equ {
	struct omap_aess_coeff dl1;
	struct omap_aess_coeff dl2l;
	struct omap_aess_coeff dl2r;
	struct omap_aess_coeff sdt;
	struct omap_aess_coeff amic;
	struct omap_aess_coeff dmic;
};

struct omap_aess_dai {
	int num_active;
	int num_suspended;
};

struct omap_aess_mixer {
	int mono[OMAP_ABE_NUM_MONO_MIXERS];
	u16 route_ul[16];
};

struct omap_aess_mapping {
	struct omap_aess_addr *map;
	u32 *fct_id;
	u32 *label_id;
	int nb_init_task;
	struct omap_aess_task *init_table;
	struct omap_aess_port *port;
	struct omap_aess_port *ping_pong;
	struct omap_aess_task *dl1_mono_mixer;
	struct omap_aess_task *dl2_mono_mixer;
	struct omap_aess_task *audul_mono_mixer;
	int *asrc;
};

struct omap_aess_pingppong {
	int buf_id;
	int buf_id_next;
	int buf_addr[4];
	int first_irq;

	/* size of each ping/pong buffers */
	u32 size;
	/* base addresses of the ping pong buffers in bytes addresses */
	u32 base_address[MAX_PINGPONG_BUFFERS];
};

struct omap_aess_gain {
	u8  muted_indicator;
	u32 desired_decibel;
	u32 muted_decibel;
	u32 desired_linear;
	u32 desired_ramp_delay_ms;
};

struct omap_aess_debug;

/*
 * ABE private data.
 */
struct omap_aess {
	struct device *dev;

	struct clk *clk;
	void __iomem *io_base[OMAP_ABE_IO_RESOURCES];
	u32 dmem_l4;
	u32 dmem_l3;
	u32 aess_config_l3;
	int dma_lines[OMAP_ABE_DMA_RESOURCES];
	int irq;
	int active;
	struct mutex mutex;
	int (*get_context_lost_count)(struct device *dev);
	int (*device_scale)(struct device *req_dev,
			    struct device *target_dev,
			    unsigned long rate);
	u32 context_lost;

	struct omap_aess_opp opp;
	struct omap_aess_dc_offset dc_offset;
	struct omap_aess_modem modem;
	struct omap_aess_equ equ;
	struct omap_aess_dai dai;
	struct omap_aess_mixer mixer;

	/* firmware */
	struct fw_header hdr;
	const void *fw_config;
	const void *fw_data;
	const struct firmware *fw;

	/* from former omap_aess struct */
	u32 firmware_version_number;
	u16 MultiFrame[25][8];

	/* Housekeeping for gains */
	struct omap_aess_gain gains[MAX_NBGAIN_CMEM];

	/* Ping-Pong mode */
	struct omap_aess_pingppong pingpong;

	u32 irq_dbg_read_ptr;
	struct omap_aess_mapping fw_info;

	/* List of open ABE logical ports */
	struct list_head ports;

	/* spinlock */
	spinlock_t lock;

#ifdef CONFIG_DEBUG_FS
	struct omap_aess_debug *debug;
	struct dentry *debugfs_root;
#endif
};

extern struct snd_soc_fw_platform_ops soc_fw_ops;

extern int abe_mixer_enable_mono(struct omap_aess *aess, int id, int enable);
extern int abe_mixer_set_equ_profile(struct omap_aess *aess, unsigned int id,
				     unsigned int profile);


/* From former abe.h file */
void omap_abe_port_mgr_init(struct omap_aess *aess);
void omap_abe_port_mgr_cleanup(struct omap_aess *aess);
void omap_abe_port_set_substream(struct omap_aess *aess, int logical_id,
				 struct snd_pcm_substream *substream);
struct snd_pcm_substream *omap_abe_port_get_substream(struct omap_aess *aess,
						      int logical_id);
struct omap_aess_dma {
	void *data;
	u32 iter;
};

int omap_aess_set_opp_processing(struct omap_aess *aess, enum opp_level level);
int omap_aess_connect_debug_trace(struct omap_aess *aess,
				  struct omap_aess_dma *dma2);

/* gain */
int omap_aess_use_compensated_gain(struct omap_aess *aess, int on_off);

int omap_aess_disable_gain(struct omap_aess *aess, u32 id);
int omap_aess_enable_gain(struct omap_aess *aess, u32 id);
int omap_aess_mute_gain(struct omap_aess *aess, u32 id);
int omap_aess_unmute_gain(struct omap_aess *aess, u32 id);

int omap_aess_write_gain(struct omap_aess *aess, u32 id, s32 f_g);
int omap_aess_read_gain(struct omap_aess *aess, u32 id, u32 *f_g);
#define omap_aess_write_mixer(aess, id, f_g) omap_aess_write_gain(aess, id, f_g)
#define omap_aess_read_mixer(aess, id, f_g) omap_aess_read_gain(aess, id, f_g)

int omap_aess_init_mem(struct omap_aess *aess, const void *fw_config);
int omap_aess_reset_hal(struct omap_aess *aess);
int omap_aess_load_fw(struct omap_aess *aess, const void *firmware);
int omap_aess_reload_fw(struct omap_aess *aess, const void *firmware);

/* port */
int omap_aess_mono_mixer(struct omap_aess *aess, u32 id, u32 on_off);
void omap_aess_connect_serial_port(struct omap_aess *aess, u32 id,
				  struct omap_aess_data_format *f,
				  u32 mcbsp_id, struct omap_aess_dma *aess_dma);
void omap_aess_connect_cbpr_dmareq_port(struct omap_aess *aess, u32 id,
				       struct omap_aess_data_format *f, u32 d,
				       struct omap_aess_dma *aess_dma);
int omap_aess_connect_irq_ping_pong_port(struct omap_aess *aess, u32 id,
					 struct omap_aess_data_format *f,
					 u32 subroutine_id, u32 size,
					 u32 *sink, u32 dsp_mcu_flag);
void omap_aess_write_pdmdl_offset(struct omap_aess *aess, u32 path,
				  u32 offset_left, u32 offset_right);
int omap_aess_enable_data_transfer(struct omap_aess *aess, u32 id);
int omap_aess_disable_data_transfer(struct omap_aess *aess, u32 id);

/* core */
int omap_aess_check_activity(struct omap_aess *aess);
int omap_aess_wakeup(struct omap_aess *aess);
int omap_aess_set_router_configuration(struct omap_aess *aess, u32 *param);
int omap_abe_read_next_ping_pong_buffer(struct omap_aess *aess,
					u32 port, u32 *p, u32 *n);
int omap_aess_read_next_ping_pong_buffer(struct omap_aess *aess,
					 u32 port, u32 *p, u32 *n);
int omap_aess_irq_processing(struct omap_aess *aess);
int omap_aess_set_ping_pong_buffer(struct omap_aess *aess,
				   u32 port, u32 n_bytes);
int omap_aess_read_offset_from_ping_buffer(struct omap_aess *aess,
					   u32 id, u32 *n);

int omap_aess_disable_irq(struct omap_aess *aess);
u32 omap_aess_get_label_data(struct omap_aess *aess, int index);
void omap_aess_pp_handler(struct omap_aess *aess, void (*callback)(void *data),
			  void *cb_data);
int omap_aess_write_event_generator(struct omap_aess *aess, u32 e);

#endif  /* __kernel__ */
#endif	/* End of __OMAP_AESS_PRIV_H__ */
