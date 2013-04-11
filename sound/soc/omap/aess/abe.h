/*
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2010-2013 Texas Instruments Incorporated,
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * The full GNU General Public License is included in this distribution
 * in the file called LICENSE.GPL.
 *
 * BSD LICENSE
 *
 * Copyright(c) 2010-2012 Texas Instruments Incorporated,
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Texas Instruments Incorporated nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _ABE_H_
#define _ABE_H_

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/io.h>

#include "aess-fw.h"

#include <linux/debugfs.h>

/*
 * OS DEPENDENT MMU CONFIGURATION
 */
#define ABE_DMEM_BASE_OFFSET_MPU	0x80000
#define ABE_ATC_BASE_OFFSET_MPU		0xF1000

/* default base address for io_base */
#define ABE_DEFAULT_BASE_ADDRESS_L3 0x49000000L
#define ABE_DEFAULT_BASE_ADDRESS_L4 0x40100000L
#define ABE_DEFAULT_BASE_ADDRESS_DEFAULT ABE_DEFAULT_BASE_ADDRESS_L3

struct snd_pcm_substream;

/*
 * TODO: These structures, enums and port ID macros should be moved to the
 * new public ABE API header.
 */

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


#define OMAP_ABE_D_MCUIRQFIFO_SIZE	0x40

/* ports can either be enabled or disabled */
enum port_state {
	PORT_DISABLED = 0,
	PORT_ENABLED,
};

struct omap_aess_mapping {
	struct omap_aess_addr *map;
	int *fct_id;
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

/* structure used for client port info */
struct omap_abe_port;

/* main ABE structure */
struct omap_aess {
	struct device *dev;
	void __iomem *io_base[5];
	u32 firmware_version_number;
	u16 MultiFrame[25][8];
	u8  muted_gains_indicator[MAX_NBGAIN_CMEM];
	u32 desired_gains_decibel[MAX_NBGAIN_CMEM];
	u32 muted_gains_decibel[MAX_NBGAIN_CMEM];
	u32 desired_gains_linear[MAX_NBGAIN_CMEM];
	u32 desired_ramp_delay_ms[MAX_NBGAIN_CMEM];
	int pp_buf_id;
	int pp_buf_id_next;
	int pp_buf_addr[4];
	int pp_first_irq;

	/* base addresses of the ping pong buffers in bytes addresses */
	u32 base_address_pingpong[MAX_PINGPONG_BUFFERS];
	/* size of each ping/pong buffers */
	u32 size_pingpong;
	/* number of ping/pong buffer being used */
	u32 nb_pingpong;

	u32 irq_dbg_read_ptr;
	struct omap_aess_mapping fw_info;

	/* List of open ABE logical ports */
	struct list_head ports;

	/* spinlock */
	spinlock_t lock;

#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_root;
#endif
};

#if IS_ENABLED(CONFIG_SND_OMAP_SOC_AESS)
int omap_abe_port_open(struct omap_aess *aess, int logical_id);
void omap_abe_port_close(struct omap_aess *aess, int logical_id);
int omap_abe_port_enable(struct omap_aess *aess, int logical_id);
int omap_abe_port_disable(struct omap_aess *aess, int logical_id);
int omap_abe_port_is_enabled(struct omap_aess *aess, int logical_id);
void omap_abe_port_set_substream(struct omap_aess *aess, int logical_id,
				 struct snd_pcm_substream *substream);
struct snd_pcm_substream *omap_abe_port_get_substream(struct omap_aess *aess,
						      int logical_id);

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

static inline void omap_abe_port_set_substream(struct omap_aess *aess,
					    int logical_id,
					    struct snd_pcm_substream *substream)
{
}

static inline struct snd_pcm_substream *omap_abe_port_get_substream(
							struct omap_aess *aess,
							int logical_id)
{
	return NULL;
}

static inline struct omap_aess *omap_abe_port_mgr_get(void)
{
	return NULL;
}

static inline void omap_abe_port_mgr_put(struct omap_aess *aess)
{
}
#endif /* IS_ENABLED(CONFIG_SND_OMAP_SOC_AESS) */

struct omap_aess_equ {
	/* type of filter */
	u32 equ_type;
	/* filter length */
	u32 equ_length;
	union {
		/* parameters are the direct and recursive coefficients in */
		/* Q6.26 integer fixed-point format. */
		s32 type1[NBEQ1];
		struct {
			/* center frequency of the band [Hz] */
			s32 freq[NBEQ2];
			/* gain of each band. [dB] */
			s32 gain[NBEQ2];
			/* Q factor of this band [dB] */
			s32 q[NBEQ2];
		} type2;
	} coef;
	s32 equ_param3;
};


struct omap_aess_dma {
	void *data;
	u32 iter;
};

int omap_aess_set_opp_processing(struct omap_aess *aess, u32 opp);
int omap_aess_connect_debug_trace(struct omap_aess *aess,
				  struct omap_aess_dma *dma2);

/* gain */
int omap_aess_use_compensated_gain(struct omap_aess *aess, int on_off);
int omap_aess_write_equalizer(struct omap_aess *aess, u32 id,
			      struct omap_aess_equ *param);

int omap_aess_disable_gain(struct omap_aess *aess, u32 id);
int omap_aess_enable_gain(struct omap_aess *aess, u32 id);
int omap_aess_mute_gain(struct omap_aess *aess, u32 id);
int omap_aess_unmute_gain(struct omap_aess *aess, u32 id);

int omap_aess_write_gain(struct omap_aess *aess, u32 id, s32 f_g);
int omap_aess_read_gain(struct omap_aess *aess, u32 id, u32 *f_g);
#define omap_aess_write_mixer(aess, id, f_g) omap_aess_write_gain(aess, id, f_g)
#define omap_aess_read_mixer(aess, id, f_g) omap_aess_read_gain(aess, id, f_g)

int omap_aess_init_mem(struct omap_aess *aess, struct device *dev,
	void __iomem **_io_base, const void *fw_config);
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

#endif /* _ABE_H_ */
