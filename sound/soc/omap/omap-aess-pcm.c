/*
 * omap-abe-pcm.c  --  OMAP ALSA SoC platform driver using Audio Backend
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


#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "omap-aess-priv.h"
#include "omap-pcm.h"

int abe_opp_recalc_level(struct omap_abe *abe);
int abe_opp_set_level(struct omap_abe *abe, int opp);

int abe_opp_stream_event(struct snd_soc_dapm_context *dapm, int event);
void abe_init_debugfs(struct omap_abe *abe);
void abe_cleanup_debugfs(struct omap_abe *abe);
int abe_opp_init_initial_opp(struct omap_abe *abe);

int abe_mixer_enable_mono(struct omap_abe *abe, int id, int enable);
int abe_mixer_set_equ_profile(struct omap_abe *abe,
		unsigned int id, unsigned int profile);

extern struct snd_soc_fw_platform_ops soc_fw_ops;

/* Ping pong buffer DMEM offset - we should read this from future FWs */
#define OMAP_ABE_DMEM_BASE_OFFSET_PING_PONG	0x4000

static const struct snd_pcm_hardware omap_abe_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_BLOCK_TRANSFER |
				  SNDRV_PCM_INFO_PAUSE |
				  SNDRV_PCM_INFO_RESUME,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S32_LE,
	.period_bytes_min	= 4 * 1024,
	.period_bytes_max	= 24 * 1024,
	.periods_min		= 4,
	.periods_max		= 4,
	.buffer_bytes_max	= 24 * 1024 * 2,
};

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

static int abe_save_context(struct omap_abe *abe)
{
	/* mute gains not associated with FEs/BEs */
	omap_aess_mute_gain(abe->aess, OMAP_AESS_MIXAUDUL_MM_DL);
	omap_aess_mute_gain(abe->aess, OMAP_AESS_MIXAUDUL_TONES);
	omap_aess_mute_gain(abe->aess, OMAP_AESS_MIXAUDUL_VX_DL);
	omap_aess_mute_gain(abe->aess, OMAP_AESS_MIXVXREC_TONES);
	omap_aess_mute_gain(abe->aess, OMAP_AESS_MIXVXREC_VX_DL);
	omap_aess_mute_gain(abe->aess, OMAP_AESS_MIXVXREC_MM_DL);
	omap_aess_mute_gain(abe->aess, OMAP_AESS_MIXVXREC_VX_UL);
	omap_aess_mute_gain(abe->aess, OMAP_AESS_MIXECHO_DL1);
	omap_aess_mute_gain(abe->aess, OMAP_AESS_MIXECHO_DL2);

	/*
	 * mute gains associated with DL1 BE
	 * ideally, these gains should be muted/saved when BE is muted, but
	 * when ABE McPDM is started for DL1 or DL2, PDM_DL1 port gets enabled
	 * which prevents to mute these gains since two ports on DL1 path are
	 * active when mute is called for BT_VX_DL or MM_EXT_DL.
	 *
	 * These gains are not restored along with the context because they
	 * are properly unmuted/restored when any of the DL1 BEs is unmuted
	 */
	omap_aess_mute_gain(abe->aess, OMAP_AESS_GAIN_DL1_LEFT);
	omap_aess_mute_gain(abe->aess, OMAP_AESS_GAIN_DL1_RIGHT);
	omap_aess_mute_gain(abe->aess, OMAP_AESS_MIXSDT_DL);

	return 0;
}

int abe_restore_context(struct omap_abe *abe)
{
	int i, ret;

	if (abe->device_scale) {
		ret = abe->device_scale(abe->dev, abe->dev,
				abe->opp.freqs[OMAP_ABE_OPP_50]);
		if (ret) {
			dev_err(abe->dev, "failed to scale to OPP 50\n");
			return ret;
		}
	}

	/* unmute gains not associated with FEs/BEs */
	omap_aess_unmute_gain(abe->aess, OMAP_AESS_MIXAUDUL_MM_DL);
	omap_aess_unmute_gain(abe->aess, OMAP_AESS_MIXAUDUL_TONES);
	omap_aess_unmute_gain(abe->aess, OMAP_AESS_MIXAUDUL_VX_DL);
	omap_aess_unmute_gain(abe->aess, OMAP_AESS_MIXVXREC_TONES);
	omap_aess_unmute_gain(abe->aess, OMAP_AESS_MIXVXREC_VX_DL);
	omap_aess_unmute_gain(abe->aess, OMAP_AESS_MIXVXREC_MM_DL);
	omap_aess_unmute_gain(abe->aess, OMAP_AESS_MIXVXREC_VX_UL);
	omap_aess_unmute_gain(abe->aess, OMAP_AESS_MIXECHO_DL1);
	omap_aess_unmute_gain(abe->aess, OMAP_AESS_MIXECHO_DL2);
	omap_aess_set_router_configuration(abe->aess, (u32 *)abe->mixer.route_ul);

	/* DC offset cancellation setting */
	if (abe->dc_offset.power_mode)
		omap_aess_write_pdmdl_offset(abe->aess, 1, abe->dc_offset.hsl * 2, abe->dc_offset.hsr * 2);
	else
		omap_aess_write_pdmdl_offset(abe->aess, 1, abe->dc_offset.hsl, abe->dc_offset.hsr);

	omap_aess_write_pdmdl_offset(abe->aess, 2, abe->dc_offset.hfl, abe->dc_offset.hfr);

	abe_mixer_set_equ_profile(abe, OMAP_AESS_CMEM_DL1_COEFS_ID, abe->equ.dl1.profile);
	abe_mixer_set_equ_profile(abe, OMAP_AESS_CMEM_DL2_L_COEFS_ID, abe->equ.dl2l.profile);
	abe_mixer_set_equ_profile(abe, OMAP_AESS_CMEM_DL2_R_COEFS_ID, abe->equ.dl2r.profile);
	abe_mixer_set_equ_profile(abe, OMAP_AESS_CMEM_SDT_COEFS_ID, abe->equ.sdt.profile);
	abe_mixer_set_equ_profile(abe, OMAP_AESS_CMEM_96_48_AMIC_COEFS_ID, abe->equ.amic.profile);
	abe_mixer_set_equ_profile(abe, OMAP_AESS_CMEM_96_48_DMIC_COEFS_ID, abe->equ.dmic.profile);

	for (i = 0; i < OMAP_ABE_NUM_MONO_MIXERS; i++)
		abe_mixer_enable_mono(abe, MIX_DL1_MONO + i, abe->mixer.mono[i]);

       return 0;
}


#define OMAP_ABE_IRQTAG_COUNT	0x000c
#define OMAP_ABE_IRQTAG_PP	0x000d
#define OMAP_ABE_IRQ_FIFO_MASK	((OMAP_ABE_D_MCUIRQFIFO_SIZE >> 2) - 1)

static void abe_irq_pingpong_subroutine(struct snd_pcm_substream *substream,
					struct omap_abe *abe)
{

	u32 dst, n_bytes;

	omap_aess_read_next_ping_pong_buffer(abe->aess, OMAP_ABE_MM_DL_PORT,
					     &dst, &n_bytes);
	omap_aess_set_ping_pong_buffer(abe->aess, OMAP_ABE_MM_DL_PORT, n_bytes);

	/* 1st IRQ does not signal completed period */
	if (abe->mmap.first_irq) {
		abe->mmap.first_irq = 0;
	} else {
		if (substream)
			snd_pcm_period_elapsed(substream);
	}
}


irqreturn_t abe_irq_handler(int irq, void *dev_id)
{
	struct omap_abe *abe = dev_id;
	struct omap_aess_addr addr;
	struct omap_aess *aess = abe->aess;
	struct omap_aess_irq_data IRQ_data;
	u32 abe_irq_dbg_write_ptr, i, cmem_src, sm_cm;

	pm_runtime_get_sync(abe->dev);
	omap_aess_clear_irq(abe->aess);

	/* extract the write pointer index from CMEM memory (INITPTR format) */
	/* CMEM address of the write pointer in bytes */
	cmem_src = aess->fw_info.label_id[OMAP_AESS_BUFFER_MCU_IRQ_FIFO_PTR_ID] << 2;
	omap_abe_mem_read(aess, OMAP_ABE_CMEM, cmem_src, &sm_cm, sizeof(sm_cm));
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
		aess->irq_dbg_read_ptr = (aess->irq_dbg_read_ptr + 1) & OMAP_ABE_IRQ_FIFO_MASK;
		/* select the source of the interrupt */
		switch (IRQ_data.tag) {
		case OMAP_ABE_IRQTAG_PP:
			/* first IRQ doesn't represent a buffer transference completion */
			if (aess->pp_first_irq)
				aess->pp_first_irq = 0;
			else
				aess->pp_buf_id = (aess->pp_buf_id + 1) & 0x03;

			abe_irq_pingpong_subroutine(abe->dai.port[OMAP_ABE_FE_PORT_MM_DL_LP]->substream,
						    abe);

			break;
		case OMAP_ABE_IRQTAG_COUNT:
			/*omap_aess_monitoring(aess);*/
			break;
		default:
			break;
		}

	}

	pm_runtime_put_sync_suspend(abe->dev);
	return IRQ_HANDLED;
}

static int aess_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_platform *platform = rtd->platform;
	struct omap_abe *abe = snd_soc_platform_get_drvdata(platform);
	struct snd_soc_dai *dai = rtd->cpu_dai;

	mutex_lock(&abe->mutex);

	dev_dbg(dai->dev, "%s: %s\n", __func__, dai->name);

	if (dai->id == OMAP_ABE_FRONTEND_DAI_LP_MEDIA)
		snd_soc_set_runtime_hwparams(substream, &omap_abe_hardware);

	pm_runtime_get_sync(abe->dev);

	if (!abe->active++) {
		abe->opp.level = 0;
		abe_restore_context(abe);
		abe_opp_set_level(abe, 100);
		omap_aess_wakeup(abe->aess);
	}

	mutex_unlock(&abe->mutex);
	return 0;
}

static int aess_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_platform *platform = rtd->platform;
	struct omap_abe *abe = snd_soc_platform_get_drvdata(platform);
	struct snd_soc_dai *dai = rtd->cpu_dai;
	struct omap_aess_data_format format;
	size_t period_size;
	u32 dst;
	int ret = 0;

	mutex_lock(&abe->mutex);

	dev_dbg(dai->dev, "%s: %s\n", __func__, dai->name);

	if (dai->id != OMAP_ABE_FRONTEND_DAI_LP_MEDIA)
		goto out;

	format.f = params_rate(params);
	if (params_format(params) == SNDRV_PCM_FORMAT_S32_LE)
		format.samp_format = OMAP_AESS_FORMAT_STEREO_MSB;
	else
		format.samp_format = OMAP_AESS_FORMAT_STEREO_16_16;

	period_size = params_period_bytes(params);

	/* Connect a Ping-Pong cache-flush protocol to MM_DL port */
	omap_aess_connect_irq_ping_pong_port(abe->aess, OMAP_ABE_MM_DL_PORT,
					     &format, 0, period_size, &dst,
					     PING_PONG_WITH_MCU_IRQ);

	/* Memory mapping for hw params */
	runtime->dma_area  = abe->io_base[0] + dst;
	runtime->dma_addr  = 0;
	runtime->dma_bytes = period_size * 4;

	/* Need to set the first buffer in order to get interrupt */
	omap_aess_set_ping_pong_buffer(abe->aess, OMAP_ABE_MM_DL_PORT,
				       period_size);
	abe->mmap.first_irq = 1;

out:
	mutex_unlock(&abe->mutex);
	return ret;
}

static int aess_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_platform *platform = rtd->platform;
	struct omap_abe *abe = snd_soc_platform_get_drvdata(platform);
	struct snd_soc_dai *dai = rtd->cpu_dai;

	dev_dbg(dai->dev, "%s: %s\n", __func__, dai->name);

	mutex_lock(&abe->mutex);
	/*
	 * TODO: do we need to set OPP here ? e.g.
	 * Startup() -> OPP100
	 * aess_prepare() -> OPP0 (as widgets are not updated until after)
	 * stream_event() -> OPP25/50/100 (correct value based on widgets)
	 * abe_opp_recalc_level(abe);
	 */
	mutex_unlock(&abe->mutex);
	return 0;
}

static int aess_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_platform *platform = rtd->platform;
	struct omap_abe *abe = snd_soc_platform_get_drvdata(platform);
	struct snd_soc_dai *dai = rtd->cpu_dai;

	dev_dbg(dai->dev, "%s: %s\n", __func__, dai->name);

	mutex_lock(&abe->mutex);

	if (dai->id != OMAP_ABE_FRONTEND_DAI_LP_MEDIA) {
	}

	if (!--abe->active) {
		omap_aess_disable_irq(abe->aess);
		abe_save_context(abe);
		omap_abe_pm_shutdown(platform);
	} else {
		/* Only scale OPP level
		 * if ABE is still active */
		abe_opp_recalc_level(abe);
	}
	pm_runtime_put_sync(abe->dev);

	mutex_unlock(&abe->mutex);
	return 0;
}

static int aess_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	struct snd_soc_pcm_runtime  *rtd = substream->private_data;
	struct snd_soc_dai *dai = rtd->cpu_dai;
	int offset, size, err;

	if (dai->id != OMAP_ABE_FRONTEND_DAI_LP_MEDIA)
		return -EINVAL;

	vma->vm_flags |= VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	size = vma->vm_end - vma->vm_start;
	offset = vma->vm_pgoff << PAGE_SHIFT;

	err = io_remap_pfn_range(vma, vma->vm_start,
			(ABE_DEFAULT_BASE_ADDRESS_L4 + ABE_DMEM_BASE_OFFSET_MPU +
			OMAP_ABE_DMEM_BASE_OFFSET_PING_PONG + offset) >> PAGE_SHIFT,
			size, vma->vm_page_prot);
	if (err)
		return -EAGAIN;

	return 0;
}

static snd_pcm_uframes_t aess_pointer(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_platform *platform = rtd->platform;
	struct omap_abe *abe = snd_soc_platform_get_drvdata(platform);
	snd_pcm_uframes_t offset = 0;
	u32 pingpong;

	if (!abe->mmap.first_irq) {
		omap_aess_read_offset_from_ping_buffer(abe->aess,
						       OMAP_ABE_MM_DL_PORT,
						       &pingpong);
		offset = (snd_pcm_uframes_t)pingpong;
	}

	return offset;
}

static struct snd_pcm_ops omap_aess_pcm_ops = {
	.open           = aess_open,
	.hw_params	= aess_hw_params,
	.prepare	= aess_prepare,
	.close	        = aess_close,
	.pointer	= aess_pointer,
	.mmap		= aess_mmap,
};

static void abe_init_gains(struct omap_aess *aess)
{
	/* Uplink gains */
	omap_aess_mute_gain(aess, OMAP_AESS_MIXAUDUL_MM_DL);
	omap_aess_mute_gain(aess, OMAP_AESS_MIXAUDUL_TONES);
	omap_aess_mute_gain(aess, OMAP_AESS_MIXAUDUL_UPLINK);
	omap_aess_mute_gain(aess, OMAP_AESS_MIXAUDUL_VX_DL);
	omap_aess_mute_gain(aess, OMAP_AESS_MIXVXREC_TONES);
	omap_aess_mute_gain(aess, OMAP_AESS_MIXVXREC_VX_DL);
	omap_aess_mute_gain(aess, OMAP_AESS_MIXVXREC_MM_DL);
	omap_aess_mute_gain(aess, OMAP_AESS_MIXVXREC_VX_UL);
	omap_aess_mute_gain(aess, OMAP_AESS_GAIN_DMIC1_LEFT);
	omap_aess_mute_gain(aess, OMAP_AESS_GAIN_DMIC1_RIGHT);
	omap_aess_mute_gain(aess, OMAP_AESS_GAIN_DMIC2_LEFT);
	omap_aess_mute_gain(aess, OMAP_AESS_GAIN_DMIC2_RIGHT);
	omap_aess_mute_gain(aess, OMAP_AESS_GAIN_DMIC3_LEFT);
	omap_aess_mute_gain(aess, OMAP_AESS_GAIN_DMIC3_RIGHT);

	omap_aess_mute_gain(aess, OMAP_AESS_GAIN_AMIC_LEFT);
	omap_aess_mute_gain(aess, OMAP_AESS_GAIN_AMIC_RIGHT);

	omap_aess_mute_gain(aess, OMAP_AESS_GAIN_BTUL_LEFT);
	omap_aess_mute_gain(aess, OMAP_AESS_GAIN_BTUL_RIGHT);

	/* Downlink gains */
	omap_aess_write_gain(aess, OMAP_AESS_GAIN_DL1_LEFT, GAIN_0dB);
	omap_aess_write_gain(aess, OMAP_AESS_GAIN_DL1_RIGHT, GAIN_0dB);
	/*SEBG: Ramp RAMP_2MS */

	omap_aess_mute_gain(aess, OMAP_AESS_GAIN_DL1_LEFT);
	omap_aess_mute_gain(aess, OMAP_AESS_GAIN_DL1_RIGHT);

	omap_aess_write_gain(aess, OMAP_AESS_GAIN_DL2_LEFT, GAIN_M7dB);
	omap_aess_write_gain(aess, OMAP_AESS_GAIN_DL2_RIGHT, GAIN_M7dB);
	/*SEBG: Ramp RAMP_2MS */

	omap_aess_mute_gain(aess, OMAP_AESS_GAIN_DL2_LEFT);
	omap_aess_mute_gain(aess, OMAP_AESS_GAIN_DL2_RIGHT);
	omap_aess_mute_gain(aess, OMAP_AESS_MIXDL1_MM_DL);
	omap_aess_mute_gain(aess, OMAP_AESS_MIXDL1_MM_UL2);
	omap_aess_mute_gain(aess, OMAP_AESS_MIXDL1_VX_DL);
	omap_aess_mute_gain(aess, OMAP_AESS_MIXDL1_TONES);
	omap_aess_mute_gain(aess, OMAP_AESS_MIXDL2_TONES);
	omap_aess_mute_gain(aess, OMAP_AESS_MIXDL2_VX_DL);
	omap_aess_mute_gain(aess, OMAP_AESS_MIXDL2_MM_DL);
	omap_aess_mute_gain(aess, OMAP_AESS_MIXDL2_MM_UL2);
	omap_aess_mute_gain(aess, OMAP_AESS_MIXECHO_DL1);
	omap_aess_mute_gain(aess, OMAP_AESS_MIXECHO_DL2);

	/* Sidetone gains */
	omap_aess_mute_gain(aess, OMAP_AESS_MIXSDT_UL);
	omap_aess_mute_gain(aess, OMAP_AESS_MIXSDT_DL);
}

static int abe_probe(struct snd_soc_platform *platform)
{
	struct omap_abe *abe = snd_soc_platform_get_drvdata(platform);
	int ret = 0, i;

	pm_runtime_enable(abe->dev);
	pm_runtime_irq_safe(abe->dev);

	ret = snd_soc_fw_load_platform(platform, &soc_fw_ops, abe->fw, 0);
	if (ret < 0) {
		dev_err(platform->dev, "request for ABE FW failed %d\n", ret);
		goto out;
	}

	ret = devm_request_threaded_irq(abe->dev, abe->irq, NULL,
					abe_irq_handler, IRQF_ONESHOT, "ABE",
					(void *)abe);
	if (ret) {
		dev_err(platform->dev, "request for ABE IRQ %d failed %d\n",
				abe->irq, ret);
		goto out;
	}

	ret = abe_opp_init_initial_opp(abe);
	if (ret < 0) {
		dev_info(platform->dev, "No OPP definition\n");
		ret = 0;
	}
	/* aess_clk has to be enabled to access hal register.
	 * Disable the clk after it has been used.
	 */
	pm_runtime_get_sync(abe->dev);

	/* lrg - rework for better init flow */
	abe->aess = omap_abe_port_mgr_get();
	omap_aess_init_mem(abe->aess, abe->dev, abe->io_base, abe->fw_config);

	omap_aess_reset_hal(abe->aess);

	/* ZERO_labelID should really be 0 */
	for (i = 0; i < OMAP_ABE_ROUTES_UL + 2; i++)
		abe->mixer.route_ul[i] = abe->aess->fw_info.label_id[OMAP_AESS_BUFFER_ZERO_ID];

	omap_aess_load_fw(abe->aess, abe->fw_data);

	/* "tick" of the audio engine */
	omap_aess_write_event_generator(abe->aess, EVENT_TIMER);
	abe_init_gains(abe->aess);

	/* Stop the engine */
	omap_aess_write_event_generator(abe->aess, EVENT_STOP);
	omap_aess_disable_irq(abe->aess);

	pm_runtime_put_sync(abe->dev);
	abe_init_debugfs(abe);

out:
	if (ret)
		pm_runtime_disable(abe->dev);

	return ret;
}

static int abe_remove(struct snd_soc_platform *platform)
{
	struct omap_abe *abe = snd_soc_platform_get_drvdata(platform);

	abe_cleanup_debugfs(abe);
	pm_runtime_disable(abe->dev);

	return 0;
}

/* TODO: map IO directly into ABE memories */
static unsigned int omap_abe_oppwidget_read(struct snd_soc_platform *platform,
		unsigned int reg)
{
	struct omap_abe *abe = snd_soc_platform_get_drvdata(platform);

	if (reg > OMAP_ABE_NUM_DAPM_REG)
		return 0;

	dev_dbg(platform->dev, "read R%d (Ox%x) = 0x%x\n",
			reg, reg, abe->opp.widget[reg]);
	return abe->opp.widget[reg];
}

static int omap_abe_oppwidget_write(struct snd_soc_platform *platform,
				    unsigned int reg, unsigned int val)
{
	struct omap_abe *abe = snd_soc_platform_get_drvdata(platform);

	if (reg > OMAP_ABE_NUM_DAPM_REG)
		return 0;

	abe->opp.widget[reg] = val;
	dev_dbg(platform->dev, "write R%d (Ox%x) = 0x%x\n", reg, reg, val);
	return 0;
}

#ifdef CONFIG_PM
static int abe_suspend(struct snd_soc_dai *dai)
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
		omap_aess_mute_gain(abe->aess, OMAP_AESS_GAIN_AMIC_LEFT);
		omap_aess_mute_gain(abe->aess, OMAP_AESS_GAIN_AMIC_RIGHT);
		break;
	case OMAP_ABE_DAI_PDM_DL1:
	case OMAP_ABE_DAI_PDM_DL2:
		break;
	case OMAP_ABE_DAI_BT_VX:
		omap_aess_mute_gain(abe->aess, OMAP_AESS_GAIN_BTUL_LEFT);
		omap_aess_mute_gain(abe->aess, OMAP_AESS_GAIN_BTUL_RIGHT);
		break;
	case OMAP_ABE_DAI_MM_FM:
	case OMAP_ABE_DAI_MODEM:
		break;
	case OMAP_ABE_DAI_DMIC0:
		omap_aess_mute_gain(abe->aess, OMAP_AESS_GAIN_DMIC1_LEFT);
		omap_aess_mute_gain(abe->aess, OMAP_AESS_GAIN_DMIC1_RIGHT);
		break;
	case OMAP_ABE_DAI_DMIC1:
		omap_aess_mute_gain(abe->aess, OMAP_AESS_GAIN_DMIC2_LEFT);
		omap_aess_mute_gain(abe->aess, OMAP_AESS_GAIN_DMIC2_RIGHT);
		break;
	case OMAP_ABE_DAI_DMIC2:
		omap_aess_mute_gain(abe->aess, OMAP_AESS_GAIN_DMIC3_LEFT);
		omap_aess_mute_gain(abe->aess, OMAP_AESS_GAIN_DMIC3_RIGHT);
		break;
	default:
		dev_err(dai->dev, "%s: invalid DAI id %d\n",
				__func__, dai->id);
		break;
	}

	pm_runtime_put_sync(abe->dev);
	return ret;
}

static int abe_resume(struct snd_soc_dai *dai)
{
	struct omap_abe *abe = snd_soc_dai_get_drvdata(dai);
	int i, ret = 0;

	dev_dbg(dai->dev, "%s: %s active %d\n",
		__func__, dai->name, dai->active);

	if (!dai->active)
		return 0;

	/* context retained, no need to restore */
	if (abe->get_context_lost_count && abe->get_context_lost_count(abe->dev) == abe->context_lost)
		return 0;
	abe->context_lost = abe->get_context_lost_count(abe->dev);
	pm_runtime_get_sync(abe->dev);
	if (abe->device_scale) {
		ret = abe->device_scale(abe->dev, abe->dev,
				abe->opp.freqs[OMAP_ABE_OPP_50]);
		if (ret) {
			dev_err(abe->dev, "failed to scale to OPP 50\n");
			goto out;
		}
	}

	omap_aess_reload_fw(abe->aess, abe->fw_data);

	switch (dai->id) {
	case OMAP_ABE_DAI_PDM_UL:
		omap_aess_unmute_gain(abe->aess, OMAP_AESS_GAIN_AMIC_LEFT);
		omap_aess_unmute_gain(abe->aess, OMAP_AESS_GAIN_AMIC_RIGHT);
		break;
	case OMAP_ABE_DAI_PDM_DL1:
	case OMAP_ABE_DAI_PDM_DL2:
		break;
	case OMAP_ABE_DAI_BT_VX:
		omap_aess_unmute_gain(abe->aess, OMAP_AESS_GAIN_BTUL_LEFT);
		omap_aess_unmute_gain(abe->aess, OMAP_AESS_GAIN_BTUL_RIGHT);
		break;
	case OMAP_ABE_DAI_MM_FM:
	case OMAP_ABE_DAI_MODEM:
		break;
	case OMAP_ABE_DAI_DMIC0:
		omap_aess_unmute_gain(abe->aess, OMAP_AESS_GAIN_DMIC1_LEFT);
		omap_aess_unmute_gain(abe->aess, OMAP_AESS_GAIN_DMIC1_RIGHT);
		break;
	case OMAP_ABE_DAI_DMIC1:
		omap_aess_unmute_gain(abe->aess, OMAP_AESS_GAIN_DMIC2_LEFT);
		omap_aess_unmute_gain(abe->aess, OMAP_AESS_GAIN_DMIC2_RIGHT);
		break;
	case OMAP_ABE_DAI_DMIC2:
		omap_aess_unmute_gain(abe->aess, OMAP_AESS_GAIN_DMIC3_LEFT);
		omap_aess_unmute_gain(abe->aess, OMAP_AESS_GAIN_DMIC3_RIGHT);
		break;
	default:
		dev_err(dai->dev, "%s: invalid DAI id %d\n",
				__func__, dai->id);
		ret = -EINVAL;
		goto out;
	}

	omap_aess_set_router_configuration(abe->aess, (u32 *)abe->mixer.route_ul);

	if (abe->dc_offset.power_mode)
		omap_aess_write_pdmdl_offset(abe->aess, 1, abe->dc_offset.hsl * 2, abe->dc_offset.hsr * 2);
	else
		omap_aess_write_pdmdl_offset(abe->aess, 1, abe->dc_offset.hsl, abe->dc_offset.hsr);

	omap_aess_write_pdmdl_offset(abe->aess, 2, abe->dc_offset.hfl, abe->dc_offset.hfr);

	abe_mixer_set_equ_profile(abe, OMAP_AESS_CMEM_DL1_COEFS_ID, abe->equ.dl1.profile);
	abe_mixer_set_equ_profile(abe, OMAP_AESS_CMEM_DL2_L_COEFS_ID, abe->equ.dl2l.profile);
	abe_mixer_set_equ_profile(abe, OMAP_AESS_CMEM_DL2_R_COEFS_ID, abe->equ.dl2r.profile);
	abe_mixer_set_equ_profile(abe, OMAP_AESS_CMEM_SDT_COEFS_ID, abe->equ.sdt.profile);
	abe_mixer_set_equ_profile(abe, OMAP_AESS_CMEM_96_48_AMIC_COEFS_ID, abe->equ.amic.profile);
	abe_mixer_set_equ_profile(abe, OMAP_AESS_CMEM_96_48_DMIC_COEFS_ID, abe->equ.dmic.profile);

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

struct snd_soc_platform_driver omap_aess_platform = {
	.ops		= &omap_aess_pcm_ops,
	.probe		= abe_probe,
	.remove		= abe_remove,
	.suspend	= abe_suspend,
	.resume		= abe_resume,
	.read		= omap_abe_oppwidget_read,
	.write		= omap_abe_oppwidget_write,
	.stream_event	= abe_opp_stream_event,
};
