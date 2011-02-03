/*

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2010-2011 Texas Instruments Incorporated,
  All rights reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
  The full GNU General Public License is included in this distribution
  in the file called LICENSE.GPL.

  BSD LICENSE

  Copyright(c) 2010-2011 Texas Instruments Incorporated,
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Texas Instruments Incorporated nor the names of
      its contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef _ABE_DBG_H_
#define _ABE_DBG_H_

#include <linux/mutex.h>

#include "abe_typ.h"
#include "abe_dm_addr.h"

/*
 *	Debug trace format
 *	TIME 2 bytes from ABE : 4kHz period of the FW scheduler
 *	SUBID 1 byte : HAL API index
 * From 0 to 16 bytes : parameters of the subroutine
 * on every 32 dumps a tag is pushed on the debug trace : 0x55555555
 */
#define dbg_bitfield_offset 8
#define dbg_api_calls 0
#define dbg_mapi (1L << (dbg_api_calls + dbg_bitfield_offset))
#define dbg_external_data_access 1
#define dbg_mdata (1L << (dbg_external_data_access + dbg_bitfield_offset))
#define dbg_err_codes 2
#define dbg_merr (1L << (dbg_api_calls + dbg_bitfield_offset))
#define ABE_DBG_MAGIC_NUMBER 0x55555555

/*
 * IDs used for error codes
 */
#define NOERR 0
#define ABE_SET_MEMORY_CONFIG_ERR (1 + dbg_merr)
#define ABE_BLOCK_COPY_ERR (2 + dbg_merr)
#define ABE_SEQTOOLONG (3 + dbg_merr)
#define ABE_BADSAMPFORMAT (4 + dbg_merr)
#define ABE_SET_ATC_ABE_BLOCK_COPY_ERR MEMORY_CONFIG_ERR (5 + dbg_merr)
#define ABE_PROTOCOL_ERROR (6 + dbg_merr)
#define ABE_PARAMETER_ERROR (7 + dbg_merr)
/*  port programmed while still running */
#define ABE_PORT_REPROGRAMMING (8 + dbg_merr)
#define ABE_READ_USE_CASE_OPP_ERR (9 + dbg_merr)
#define ABE_PARAMETER_OVERFLOW (10 + dbg_merr)
#define ABE_FW_FIFO_WRITE_PTR_ERR (11 + dbg_merr)

/*
 * IDs used for error codes
 */
#define OMAP_ABE_ERR_LIB   (1 << 1)
#define OMAP_ABE_ERR_API   (1 << 2)
#define OMAP_ABE_ERR_INI   (1 << 3)
#define OMAP_ABE_ERR_SEQ   (1 << 4)
#define OMAP_ABE_ERR_DBG   (1 << 5)
#define OMAP_ABE_ERR_EXT   (1 << 6)

struct omap_abe_dbg {
	/* Debug Data */
	u32 activity_log[OMAP_ABE_D_DEBUG_HAL_TASK_SIZE];
	u32 activity_log_write_pointer;
	u32 mask;
};

struct omap_abe_dma {
	/* OCP L3 pointer to the first address of the */
	void *data;
	/* destination buffer (either DMA or Ping-Pong read/write pointers). */
	/* address L3 when addressing the DMEM buffer instead of CBPr */
	void *l3_dmem;
	/* address L3 translated to L4 the ARM memory space */
	void *l4_dmem;
	/* number of iterations for the DMA data moves. */
	u32 iter;
};

struct omap_abe {
	void __iomem *io_base[5];
	u32 firmware_version_number;
	u16 MultiFrame[25][8];
	u32 compensated_mixer_gain;
	u8  muted_gains_indicator[MAX_NBGAIN_CMEM];
	u32 desired_gains_decibel[MAX_NBGAIN_CMEM];
	u32 muted_gains_decibel[MAX_NBGAIN_CMEM];
	u32 desired_gains_linear[MAX_NBGAIN_CMEM];
	u32 desired_ramp_delay_ms[MAX_NBGAIN_CMEM];
	int pp_buf_id;
	int pp_buf_id_next;
	int pp_buf_addr[4];
	int pp_first_irq;
	struct mutex mutex;
	u32 warm_boot;

	u32 irq_dbg_read_ptr;
	u32 dbg_param;

	struct omap_abe_dbg dbg;
};

/**
 * omap_abe_dbg_reset
 * @dbg: Pointer on abe debug handle
 *
 * Called in order to reset Audio Back End debug global data.
 * This ensures that ABE debug trace pointer is reset correctly.
 */
int omap_abe_dbg_reset(struct omap_abe_dbg *dbg);

#endif /* _ABE_DBG_H_ */
