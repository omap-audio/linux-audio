/*
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2010-2012 Texas Instruments Incorporated,
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
 * Copyright(c) 2010-2013 Texas Instruments Incorporated,
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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>

#include "omap-aess-priv.h"
#include "abe_gain.h"
#include "aess-priv.h"
#include "abe_port.h"
#include "abe_mem.h"
#include "abe_dbg.h"

/* AESS_MCU_IRQENABLE_SET/CLR (0x3c/0x40) bit field */
#define INT_MASK			0x01

/* AESS_DMAENABLE_SET/CLR (0x60/0x64) bit fields */
#define DMA_SELECT(x)			(x & 0xFF)

#define EVENT_SOURCE_DMA 0
#define EVENT_SOURCE_COUNTER 1

/* EVENT_GENERATOR_COUNTER COUNTER_VALUE bit field */

/* PLL output/desired sampling rate = (32768 * 6000)/96000 */
#define EVENT_GENERATOR_COUNTER_DEFAULT	(2048-1)
/* PLL output/desired sampling rate = (32768 * 6000)/88200 */
#define EVENT_GENERATOR_COUNTER_44100	(2228-1)

/*
 * omap_aess_irq_data
 *
 * IRQ FIFO content declaration
 *	APS interrupts : IRQ_FIFO[31:28] = IRQtag_APS,
 *		IRQ_FIFO[27:16] = APS_IRQs, IRQ_FIFO[15:0] = loopCounter
 *	SEQ interrupts : IRQ_FIFO[31:28] OMAP_ABE_IRQTAG_COUNT,
 *		IRQ_FIFO[27:16] = Count_IRQs, IRQ_FIFO[15:0] = loopCounter
 *	Ping-Pong Interrupts : IRQ_FIFO[31:28] = OMAP_ABE_IRQTAG_PP,
 *		IRQ_FIFO[27:16] = PP_MCU_IRQ, IRQ_FIFO[15:0] = loopCounter
 */
struct omap_aess_irq_data {
	unsigned int counter:16;
	unsigned int data:12;
	unsigned int tag:4;
};


/**
 * omap_aess_hw_configuration
 * @aess: Pointer on aess handle
 *
 * Initialize the AESS HW registers for MPU and DMA
 * request visibility.
 */
static void omap_aess_hw_configuration(struct omap_aess *aess)
{
	/* enable AESS auto gating (required to release all AESS clocks) */
	omap_aess_reg_write(aess, OMAP_AESS_AUTO_GATING_ENABLE, 1);
	/* enables the DMAreq from AESS AESS_DMAENABLE_SET = 255 */
	omap_aess_reg_write(aess, OMAP_AESS_DMAENABLE_SET, DMA_SELECT(0xff));
	/* enables the MCU IRQ from AESS to Cortex A9 */
	omap_aess_reg_write(aess, OMAP_AESS_MCU_IRQENABLE_SET, INT_MASK);
}

/**
 * omap_aess_disable_irq - disable MCU/DSP ABE interrupt
 * @aess: Pointer on abe handle
 *
 * This subroutine is disabling ABE MCU/DSP Irq
 */
int omap_aess_disable_irq(struct omap_aess *aess)
{
	/* disables the DMAreq from AESS AESS_DMAENABLE_CLR = 127
	 * DMA_Req7 will still be enabled as it is used for ABE trace */
	omap_aess_reg_write(aess, OMAP_AESS_DMAENABLE_CLR, DMA_SELECT(0x7f));
	/* disables the MCU IRQ from AESS to Cortex A9 */
	omap_aess_reg_write(aess, OMAP_AESS_MCU_IRQENABLE_CLR, INT_MASK);
	return 0;
}
EXPORT_SYMBOL(omap_aess_disable_irq);

/**
 * omap_aess_reset_hal - reset the ABE/HAL
 * @aess: Pointer on aess handle
 *
 * Operations : reset the ABE by reloading the static variables and
 * default AESS registers.
 * Called after a PRCM cold-start reset of ABE
 */
int omap_aess_reset_hal(struct omap_aess *aess)
{
	u32 i;

	/* IRQ & DBG circular read pointer in DMEM */
	aess->irq_dbg_read_ptr = 0;

	/* reset the default gain values */
	for (i = 0; i < MAX_NBGAIN_CMEM; i++) {
		struct omap_aess_gain *gain = &aess->gains[i];

		gain->muted_indicator = 0;
		gain->desired_decibel = (u32) GAIN_MUTE;
		gain->desired_linear = 0;
		gain->desired_ramp_delay_ms = 0;
		gain->muted_decibel = (u32) GAIN_TOOLOW;
	}
	omap_aess_hw_configuration(aess);
	return 0;
}
EXPORT_SYMBOL(omap_aess_reset_hal);

/**
 * abe_write_event_generator - Selects event generator source
 * @aess: Pointer on abe handle
 * @e: Event Generation Counter, McPDM, DMIC or default.
 *
 * Loads the AESS event generator hardware source.
 * Indicates to the FW which data stream is the most important to preserve
 * in case all the streams are asynchronous.
 *
 * the Audio Engine will generaly use its own timer EVENT generator programmed
 * with the EVENT_COUNTER. The event counter will be tuned in order to deliver
 * a pulse frequency at 96 kHz.
 * The DPLL output at 100% OPP is MCLK = (32768kHz x6000) = 196.608kHz
 * The ratio is (MCLK/96000)+(1<<1) = 2050
 * (1<<1) in order to have the same speed at 50% and 100% OPP
 * (only 15 MSB bits are used at OPP50%)
 */
int omap_aess_write_event_generator(struct omap_aess *aess, u32 e)
{
	u32 event, selection;
	u32 counter = EVENT_GENERATOR_COUNTER_DEFAULT;

	switch (e) {
	case EVENT_STOP:
		omap_aess_reg_write(aess, OMAP_AESS_EVENT_GENERATOR_START, 0);
		return 0;
	case EVENT_44100:
		counter = EVENT_GENERATOR_COUNTER_44100;
		/* Fall through */
	case EVENT_TIMER:
		selection = EVENT_SOURCE_COUNTER;
		event = 0;
		break;
	default:
		aess_err("Bad event generator selection (%u)", e);
		return -EINVAL;
	}
	omap_aess_reg_write(aess, OMAP_AESS_EVENT_GENERATOR_COUNTER, counter);
	omap_aess_reg_write(aess, OMAP_AESS_EVENT_SOURCE_SELECTION, selection);
	omap_aess_reg_write(aess, OMAP_AESS_EVENT_GENERATOR_START, 1);
	omap_aess_reg_write(aess, OMAP_AESS_AUDIO_ENGINE_SCHEDULER, event);
	return 0;
}
EXPORT_SYMBOL(omap_aess_write_event_generator);

/**
 * omap_aess_wakeup - Wakeup ABE
 * @aess: Pointer on aess handle
 *
 * Wakeup ABE in case of retention
 */
int omap_aess_wakeup(struct omap_aess *aess)
{
	/* Restart event generator */
	omap_aess_write_event_generator(aess, EVENT_TIMER);

	/* reconfigure DMA Req and MCU Irq visibility */
	omap_aess_hw_configuration(aess);
	return 0;
}
EXPORT_SYMBOL(omap_aess_wakeup);

/**
 * abe_set_router_configuration
 * @aess: Pointer on aess handle
 * @param: list of output index of the route
 *
 * The uplink router takes its input from DMIC (6 samples), AMIC (2 samples)
 * and PORT1/2 (2 stereo ports). Each sample will be individually stored in
 * an intermediate table of 10 elements.
 *
 * Example of router table parameter for voice uplink with phoenix microphones
 *
 * indexes 0 .. 9 = MM_UL description (digital MICs and MMEXTIN)
 *	DMIC1_L_labelID, DMIC1_R_labelID, DMIC2_L_labelID, DMIC2_R_labelID,
 *	MM_EXT_IN_L_labelID, MM_EXT_IN_R_labelID, ZERO_labelID, ZERO_labelID,
 *	ZERO_labelID, ZERO_labelID,
 * indexes 10 .. 11 = MM_UL2 description (recording on DMIC3)
 *	DMIC3_L_labelID, DMIC3_R_labelID,
 * indexes 12 .. 13 = VX_UL description (VXUL based on PDMUL data)
 *	AMIC_L_labelID, AMIC_R_labelID,
 * indexes 14 .. 15 = RESERVED (NULL)
 *	ZERO_labelID, ZERO_labelID,
 */
int omap_aess_set_router_configuration(struct omap_aess *aess, u32 *param)
{
	omap_aess_write_map(aess, OMAP_AESS_DMEM_AUPLINKROUTING_ID, param);
	return 0;
}
EXPORT_SYMBOL(omap_aess_set_router_configuration);

/**
 * abe_set_opp_processing - Set OPP mode for ABE Firmware
 * @aess: Pointer on aess handle
 * @opp: OOPP mode
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
int omap_aess_set_opp_processing(struct omap_aess *aess, u32 opp)
{
	u32 dOppMode32;

	switch (opp) {
	case ABE_OPP25:
		/* OPP25% */
		dOppMode32 = DOPPMODE32_OPP25;
		break;
	case ABE_OPP50:
		/* OPP50% */
		dOppMode32 = DOPPMODE32_OPP50;
		break;
	default:
		aess_warm("Bad OPP value requested");
	case ABE_OPP100:
		/* OPP100% */
		dOppMode32 = DOPPMODE32_OPP100;
		break;
	}
	/* Write Multiframe inside DMEM */
	omap_aess_write_map(aess, OMAP_AESS_DMEM_MAXTASKBYTESINSLOT_ID,
			    &dOppMode32);

	return 0;

}
EXPORT_SYMBOL(omap_aess_set_opp_processing);

#define OMAP_ABE_IRQTAG_COUNT		0x000c
#define OMAP_ABE_IRQTAG_PP		0x000d
#define OMAP_ABE_D_MCUIRQFIFO_SIZE	0x40
#define OMAP_ABE_IRQ_FIFO_MASK		((OMAP_ABE_D_MCUIRQFIFO_SIZE >> 2) - 1)

void omap_aess_pp_handler(struct omap_aess *aess, void (*callback)(void *data),
			  void *cb_data)
{
	struct omap_aess_pingppong *pp = &aess->pingpong;
	struct omap_aess_addr addr;
	struct omap_aess_irq_data IRQ_data;
	u32 abe_irq_dbg_write_ptr, i, cmem_src, sm_cm;

	if (!callback || !cb_data)
		return;

	/* Clear interrupts */
	omap_aess_reg_write(aess, OMAP_AESS_MCU_IRQSTATUS, INT_MASK);

	/*
	 * extract the write pointer index from CMEM memory (INITPTR format)
	 * CMEM address of the write pointer in bytes
	 */
	cmem_src = omap_aess_get_label_data(aess,
				OMAP_AESS_BUFFER_MCU_IRQ_FIFO_PTR_ID) << 2;
	omap_aess_read(aess, OMAP_ABE_CMEM, cmem_src, &sm_cm, sizeof(sm_cm));
	/* AESS left-pointer index located on MSBs */
	abe_irq_dbg_write_ptr = sm_cm >> 16;
	abe_irq_dbg_write_ptr &= 0xFF;
	/* loop on the IRQ FIFO content */
	for (i = 0; i < OMAP_ABE_D_MCUIRQFIFO_SIZE; i++) {
		/* stop when the FIFO is empty */
		if (abe_irq_dbg_write_ptr == aess->irq_dbg_read_ptr)
			break;
		/* read the IRQ/DBG FIFO */
		memcpy(&addr, &aess->fw_info.map[OMAP_AESS_DMEM_MCUIRQFIFO_ID],
		       sizeof(struct omap_aess_addr));
		addr.offset += (aess->irq_dbg_read_ptr << 2);
		addr.bytes = sizeof(IRQ_data);
		omap_aess_mem_read(aess, addr, (u32 *)&IRQ_data);
		aess->irq_dbg_read_ptr =
			  (aess->irq_dbg_read_ptr + 1) & OMAP_ABE_IRQ_FIFO_MASK;
		/* select the source of the interrupt */
		switch (IRQ_data.tag) {
		case OMAP_ABE_IRQTAG_PP:
			/*
			 * first IRQ doesn't represent a buffer transference
			 * completion
			 */
			if (!pp->first_irq)
				pp->buf_id = (pp->buf_id + 1) & 0x03;

			callback(cb_data);

			break;
		case OMAP_ABE_IRQTAG_COUNT:
			break;
		default:
			break;
		}

	}
}
EXPORT_SYMBOL(omap_aess_pp_handler);

u32 omap_aess_get_label_data(struct omap_aess *aess, int index)
{
	return aess->fw_info.label_id[index];
}
EXPORT_SYMBOL(omap_aess_get_label_data);
