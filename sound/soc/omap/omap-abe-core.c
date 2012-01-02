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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/opp.h>

#include <sound/soc.h>
#include <sound/omap-abe.h>

#include "omap-abe-priv.h"
#include "abe/abe_main.h"

int abe_opp_stream_event(struct snd_soc_dapm_context *dapm, int event);
int abe_pm_suspend(struct snd_soc_dai *dai);
int abe_pm_resume(struct snd_soc_dai *dai);
int abe_mixer_add_widgets(struct snd_soc_platform *platform);
int abe_mixer_write(struct snd_soc_platform *platform, unsigned int reg,
		unsigned int val);
unsigned int abe_mixer_read(struct snd_soc_platform *platform,
		unsigned int reg);
irqreturn_t abe_irq_handler(int irq, void *dev_id);
void abe_init_debugfs(struct omap_abe *abe);
void abe_cleanup_debugfs(struct omap_abe *abe);
int abe_opp_init_initial_opp(struct omap_abe *abe);
extern struct snd_pcm_ops omap_aess_pcm_ops;
extern struct snd_soc_dai_driver omap_abe_dai[7];

static const char *abe_memory_bank[5] = {
	"dmem",
	"cmem",
	"smem",
	"pmem",
	"mpu"
};

static void abe_init_gains(struct omap_abe *abe)
{
	/* Uplink gains */
	abe_mute_gain(MIXAUDUL, MIX_AUDUL_INPUT_MM_DL);
	abe_mute_gain(MIXAUDUL, MIX_AUDUL_INPUT_TONES);
	abe_mute_gain(MIXAUDUL, MIX_AUDUL_INPUT_UPLINK);
	abe_mute_gain(MIXAUDUL, MIX_AUDUL_INPUT_VX_DL);
	abe_mute_gain(MIXVXREC, MIX_VXREC_INPUT_TONES);
	abe_mute_gain(MIXVXREC, MIX_VXREC_INPUT_VX_DL);
	abe_mute_gain(MIXVXREC, MIX_VXREC_INPUT_MM_DL);
	abe_mute_gain(MIXVXREC, MIX_VXREC_INPUT_VX_UL);
	abe_mute_gain(GAINS_DMIC1, GAIN_LEFT_OFFSET);
	abe_mute_gain(GAINS_DMIC1, GAIN_RIGHT_OFFSET);
	abe_mute_gain(GAINS_DMIC2, GAIN_LEFT_OFFSET);
	abe_mute_gain(GAINS_DMIC2, GAIN_RIGHT_OFFSET);
	abe_mute_gain(GAINS_DMIC3, GAIN_LEFT_OFFSET);
	abe_mute_gain(GAINS_DMIC3, GAIN_RIGHT_OFFSET);

	abe_mute_gain(GAINS_AMIC, GAIN_LEFT_OFFSET);
	abe_mute_gain(GAINS_AMIC, GAIN_RIGHT_OFFSET);

	abe_mute_gain(GAINS_BTUL, GAIN_LEFT_OFFSET);
	abe_mute_gain(GAINS_BTUL, GAIN_RIGHT_OFFSET);

	/* Downlink gains */
	abe_write_gain(GAINS_DL1, GAIN_0dB, RAMP_2MS, GAIN_LEFT_OFFSET);
	abe_write_gain(GAINS_DL1, GAIN_0dB, RAMP_2MS, GAIN_RIGHT_OFFSET);
	abe_mute_gain(GAINS_DL1, GAIN_LEFT_OFFSET);
	abe_mute_gain(GAINS_DL1, GAIN_RIGHT_OFFSET);

	abe_write_gain(GAINS_DL2, GAIN_M7dB, RAMP_2MS, GAIN_LEFT_OFFSET);
	abe_write_gain(GAINS_DL2, GAIN_M7dB, RAMP_2MS, GAIN_RIGHT_OFFSET);
	abe_mute_gain(GAINS_DL2, GAIN_LEFT_OFFSET);
	abe_mute_gain(GAINS_DL2, GAIN_RIGHT_OFFSET);
	abe_mute_gain(MIXDL1, MIX_DL1_INPUT_MM_DL);
	abe_mute_gain(MIXDL1, MIX_DL1_INPUT_MM_UL2);
	abe_mute_gain(MIXDL1, MIX_DL1_INPUT_VX_DL);
	abe_mute_gain(MIXDL1, MIX_DL1_INPUT_TONES);
	abe_mute_gain(MIXDL2, MIX_DL2_INPUT_TONES);
	abe_mute_gain(MIXDL2, MIX_DL2_INPUT_VX_DL);
	abe_mute_gain(MIXDL2, MIX_DL2_INPUT_MM_DL);
	abe_mute_gain(MIXDL2, MIX_DL2_INPUT_MM_UL2);
	abe_mute_gain(MIXECHO, MIX_ECHO_DL1);
	abe_mute_gain(MIXECHO, MIX_ECHO_DL2);

	/* Sidetone gains */
	abe_mute_gain(MIXSDT, MIX_SDT_INPUT_UP_MIXER);
	abe_mute_gain(MIXSDT, MIX_SDT_INPUT_DL1_MIXER);
}

static int abe_init_fw(struct omap_abe *abe)
{
#if defined(CONFIG_SND_OMAP_SOC_ABE_MODULE)
	const struct firmware *fw;
#endif
	const u8 *fw_data;
	int ret, i, offset;

#if defined(CONFIG_SND_OMAP_SOC_ABE_MODULE)
	/* request firmware & coefficients */
	ret = request_firmware(&fw, "omap4_abe", abe->dev);
	if (ret != 0) {
		dev_err(abe->dev, "Failed to load firmware: %d\n", ret);
		return ret;
	}
	fw_data = fw->data;
#else
	fw_data = (u8 *)abe_get_default_fw();
#endif

	/* get firmware and coefficients header info */
	memcpy(&abe->hdr, fw_data, sizeof(struct fw_header));
	if (abe->hdr.firmware_size > OMAP_ABE_MAX_FW_SIZE) {
		dev_err(abe->dev, "Firmware too large at %d bytes: %d\n",
					abe->hdr.firmware_size, ret);
		ret = -EINVAL;
		goto err_fw;
	}
	dev_dbg(abe->dev, "ABE firmware size %d bytes\n", abe->hdr.firmware_size);

	dev_info(abe->dev, "ABE Firmware version %x\n", abe->hdr.firmware_version);
	if (omap_abe_get_supported_fw_version() != abe->hdr.firmware_version) {
		dev_err(abe->dev, "firmware version mismatch. Need %x have %x\n",
			omap_abe_get_supported_fw_version(), abe->hdr.firmware_version);
		return -EINVAL;
	}

	if (abe->hdr.coeff_size > OMAP_ABE_MAX_COEFF_SIZE) {
		dev_err(abe->dev, "Coefficients too large at %d bytes: %d\n",
					abe->hdr.coeff_size, ret);
		ret = -EINVAL;
		goto err_fw;
	}
	dev_dbg(abe->dev, "ABE coefficients size %d bytes\n", abe->hdr.coeff_size);

	/* get coefficient EQU mixer strings */
	if (abe->hdr.num_equ >= OMAP_ABE_MAX_EQU) {
		dev_err(abe->dev, "Too many equalizers got %d\n", abe->hdr.num_equ);
		ret = -EINVAL;
		goto err_fw;
	}
	abe->equ.texts = kzalloc(abe->hdr.num_equ * sizeof(struct coeff_config),
			GFP_KERNEL);
	if (abe->equ.texts == NULL) {
		ret = -ENOMEM;
		goto err_fw;
	}

	offset = sizeof(struct fw_header);
	memcpy(abe->equ.texts, fw_data + offset,
			abe->hdr.num_equ * sizeof(struct coeff_config));

	/* get coefficients from firmware */
	abe->equ.equ[0] = kmalloc(abe->hdr.coeff_size, GFP_KERNEL);
	if (abe->equ.equ[0] == NULL) {
		ret = -ENOMEM;
		goto err_equ;
	}

	offset += abe->hdr.num_equ * sizeof(struct coeff_config);
	memcpy(abe->equ.equ[0], fw_data + offset, abe->hdr.coeff_size);

	/* allocate coefficient mixer texts */
	dev_dbg(abe->dev, "loaded %d equalizers\n", abe->hdr.num_equ);
	for (i = 0; i < abe->hdr.num_equ; i++) {
		dev_dbg(abe->dev, "equ %d: %s profiles %d\n", i,
				abe->equ.texts[i].name, abe->equ.texts[i].count);
		if (abe->equ.texts[i].count >= OMAP_ABE_MAX_PROFILES) {
			dev_err(abe->dev, "Too many profiles got %d for equ %d\n",
					abe->equ.texts[i].count, i);
			ret = -EINVAL;
			goto err_texts;
		}
		abe->equ.senum[i].dtexts =
				kzalloc(abe->equ.texts[i].count * sizeof(char *), GFP_KERNEL);
		if (abe->equ.senum[i].dtexts == NULL) {
			ret = -ENOMEM;
			goto err_texts;
		}
	}

	/* initialise coefficient equalizers */
	for (i = 1; i < abe->hdr.num_equ; i++) {
		abe->equ.equ[i] = abe->equ.equ[i - 1] +
			abe->equ.texts[i - 1].count * abe->equ.texts[i - 1].coeff;
	}

	/* store ABE firmware for later context restore */
	abe->firmware = kzalloc(abe->hdr.firmware_size, GFP_KERNEL);
	if (abe->firmware == NULL) {
		ret = -ENOMEM;
		goto err_texts;
	}
	memcpy(abe->firmware,
		fw_data + sizeof(struct fw_header) + abe->hdr.coeff_size,
		abe->hdr.firmware_size);

#if defined(CONFIG_SND_OMAP_SOC_ABE_MODULE)
	release_firmware(fw);
#endif

	return 0;

err_texts:
	for (i = 0; i < abe->hdr.num_equ; i++)
		kfree(abe->equ.senum[i].texts);
	kfree(abe->equ.equ[0]);
err_equ:
	kfree(abe->equ.texts);
err_fw:
#if defined(CONFIG_SND_OMAP_SOC_ABE_MODULE)
	release_firmware(fw);
#endif
	return ret;
}

static void abe_free_fw(struct omap_abe *abe)
{
	int i;

	for (i = 0; i < abe->hdr.num_equ; i++)
		kfree(abe->equ.senum[i].texts);

	kfree(abe->equ.equ[0]);
	kfree(abe->equ.texts);
	kfree(abe->firmware);
}

static int abe_probe(struct snd_soc_platform *platform)
{
	struct omap_abe *abe = snd_soc_platform_get_drvdata(platform);
	int ret, i;

	pm_runtime_enable(abe->dev);
	pm_runtime_irq_safe(abe->dev);

	/* ZERO_labelID should really be 0 */
	for (i = 0; i < OMAP_ABE_ROUTES_UL + 2; i++)
		abe->mixer.route_ul[i] = ZERO_labelID;

	ret = abe_init_fw(abe);
	if (ret < 0)
		return ret;

	ret = request_threaded_irq(abe->irq, NULL, abe_irq_handler,
				IRQF_ONESHOT, "ABE", (void *)abe);
	if (ret) {
		dev_err(platform->dev, "request for ABE IRQ %d failed %d\n",
				abe->irq, ret);
		goto err_irq;
	}

	ret = abe_opp_init_initial_opp(abe);
	if (ret < 0)
		goto err_opp;

	/* aess_clk has to be enabled to access hal register.
	 * Disable the clk after it has been used.
	 */
	pm_runtime_get_sync(abe->dev);

	abe_init_mem(abe->io_base);

	abe_reset_hal();

	abe_load_fw(abe->firmware);

	/* "tick" of the audio engine */
	abe_write_event_generator(EVENT_TIMER);
	abe_init_gains(abe);

	/* Stop the engine */
	abe_stop_event_generator();
	abe_disable_irq();

	pm_runtime_put_sync(abe->dev);
	abe_mixer_add_widgets(platform);
	abe_init_debugfs(abe);

	return ret;

err_opp:
	free_irq(abe->irq, (void *)abe);
err_irq:
	abe_free_fw(abe);
	return ret;
}

static int abe_remove(struct snd_soc_platform *platform)
{
	struct omap_abe *abe = snd_soc_platform_get_drvdata(platform);

	abe_cleanup_debugfs(abe);
	free_irq(abe->irq, (void *)abe);
	abe_free_fw(abe);
	pm_runtime_disable(abe->dev);

	return 0;
}

static struct snd_soc_platform_driver omap_aess_platform = {
	.ops		= &omap_aess_pcm_ops,
	.probe		= abe_probe,
	.remove		= abe_remove,
	.suspend	= abe_pm_suspend,
	.resume		= abe_pm_resume,
	.read		= abe_mixer_read,
	.write		= abe_mixer_write,
	.stream_event = abe_opp_stream_event,
};

static int __devinit abe_engine_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct omap_abe_pdata *pdata = pdev->dev.platform_data;
	struct omap_abe *abe;
	int ret = -EINVAL, i;

	abe = devm_kzalloc(&pdev->dev, sizeof(struct omap_abe), GFP_KERNEL);
	if (abe == NULL)
		return -ENOMEM;
	dev_set_drvdata(&pdev->dev, abe);

	for (i = 0; i < OMAP_ABE_IO_RESOURCES; i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   abe_memory_bank[i]);
		if (res == NULL) {
			dev_err(&pdev->dev, "no resource %s\n",
				abe_memory_bank[i]);
			goto err;
		}
		abe->io_base[i] = ioremap(res->start, resource_size(res));
		if (!abe->io_base[i]) {
			ret = -ENOMEM;
			goto err;
		}
	}

	abe->irq = platform_get_irq(pdev, 0);
	if (abe->irq < 0) {
		ret = abe->irq;
		goto err;
	}

	abe->pdata = pdata;
	abe->dev = &pdev->dev;
	mutex_init(&abe->mutex);
	mutex_init(&abe->opp.mutex);
	mutex_init(&abe->opp.req_mutex);
	INIT_LIST_HEAD(&abe->opp.req);

	ret = snd_soc_register_platform(abe->dev, &omap_aess_platform);
	if (ret < 0)
		goto err;
	ret = snd_soc_register_dais(abe->dev, omap_abe_dai,
			ARRAY_SIZE(omap_abe_dai));
	if (ret < 0)
		goto dai_err;

	return ret;

dai_err:
	snd_soc_unregister_dais(&pdev->dev, ARRAY_SIZE(omap_abe_dai));
err:
	for (--i; i >= 0; i--)
		iounmap(abe->io_base[i]);

	return ret;
}

static int __devexit abe_engine_remove(struct platform_device *pdev)
{
	struct omap_abe *abe = dev_get_drvdata(&pdev->dev);
	int i;

	snd_soc_unregister_dais(&pdev->dev, ARRAY_SIZE(omap_abe_dai));
	snd_soc_unregister_platform(&pdev->dev);

	for (i = 0; i < OMAP_ABE_IO_RESOURCES; i++)
		iounmap(abe->io_base[i]);

	return 0;
}

static struct platform_driver omap_aess_driver = {
	.driver = {
		.name = "aess",
		.owner = THIS_MODULE,
	},
	.probe = abe_engine_probe,
	.remove = __devexit_p(abe_engine_remove),
};

module_platform_driver(omap_aess_driver);

MODULE_ALIAS("platform:omap-aess");
MODULE_DESCRIPTION("ASoC OMAP4 ABE");
MODULE_AUTHOR("Liam Girdwood <lrg@ti.com>");
MODULE_LICENSE("GPL");
