// SPDX-License-Identifier: GPL-2.0
/*
 * ALSA SoC Texas Instruments TAS2555 High Performance 4W Smart Amplifier
 *
 * Copyright (C) 2015-2018 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Author: Peter Ujfalusi <peter.ujfalusi@ti.com>
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <dt-bindings/sound/tas2555.h>

#include "tas2555.h"

#define TAS2555_FW_MAME		"tas2555_uCDSP.bin"

#define TAS2555_DEVICE_ID	(2555)

struct ti_dsp_fw_header {
	uint16_t deviceID;
	uint16_t customerID;
	struct {
		uint8_t major;
		uint8_t minor;
	} fw_version;
} __packed;

struct ti_dsp_fw_entry {
	uint8_t book;
	uint8_t page;
	uint8_t reg;
	uint8_t len;
	uint8_t data[];
} __packed;

struct tas2555_dai_cfg {
	unsigned int dai_fmt;
	unsigned int tdm_delay;
};

#define TAS2555_NUM_SUPPLIES	4
static const char *tas2555_supply_names[TAS2555_NUM_SUPPLIES] = {
	"vbat",		/* Battery power supply */
	"iovdd",	/* I/O power supply */
	"avdd",		/* Analog power supply */
	"dvdd",		/* Digital power supply */
};

struct tas2555_priv {
	struct snd_soc_component *component;
	u8 i2c_regs_status;
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *nreset;
	struct regulator_bulk_data supplies[TAS2555_NUM_SUPPLIES];
	struct tas2555_dai_cfg dai_cfg[3];
	unsigned int mclk_clkin;
	int mclk_clk_id;
	int current_book;
	bool enabled;
};

static void tas2555_change_book(struct tas2555_priv *tas2555, int book)
{
	if (tas2555->current_book == book)
		return;

	regmap_write(tas2555->regmap, TAS2555_BOOKCTL_REG, book);
	tas2555->current_book = book;
}

static unsigned int tas2555_read(struct snd_soc_component *component,
				 unsigned int reg)
{
	struct tas2555_priv *tas2555 = snd_soc_component_get_drvdata(component);
	unsigned int value = 0;
	int ret;

	dev_dbg(component->dev, "%s: BOOK:PAGE:REG %u:%u:%u\n", __func__,
		TAS2555_BOOK_ID(reg), TAS2555_PAGE_ID(reg),
		TAS2555_PAGE_REG(reg));
	tas2555_change_book(tas2555, TAS2555_BOOK_ID(reg));
	ret = regmap_read(tas2555->regmap, TAS2555_BOOK_REG(reg), &value);
	return value;
}

static int tas2555_write(struct snd_soc_component *component, unsigned int reg,
			 unsigned int value)
{
	struct tas2555_priv *tas2555 = snd_soc_component_get_drvdata(component);
	int ret;

	dev_dbg(component->dev, "%s: BOOK:PAGE:REG %u:%u:%u, VAL: 0x%02x\n",
		__func__, TAS2555_BOOK_ID(reg), TAS2555_PAGE_ID(reg),
		TAS2555_PAGE_REG(reg), value);
	tas2555_change_book(tas2555, TAS2555_BOOK_ID(reg));
	ret = regmap_write(tas2555->regmap, TAS2555_BOOK_REG(reg), value);
	return ret;
}

static int tas2555_bulk_write(struct snd_soc_component *component,
			      unsigned int reg, u8 *data, size_t len)
{
	struct tas2555_priv *tas2555 = snd_soc_component_get_drvdata(component);
	int ret;

	dev_dbg(component->dev, "%s: BOOK:PAGE:REG %u:%u:%u, len: %zu\n",
		__func__, TAS2555_BOOK_ID(reg), TAS2555_PAGE_ID(reg),
		TAS2555_PAGE_REG(reg), len);
	tas2555_change_book(tas2555, TAS2555_BOOK_ID(reg));
	ret = regmap_bulk_write(tas2555->regmap, TAS2555_BOOK_REG(reg), data,
				len);
	return ret;
}

static const struct regmap_range_cfg tas2555_ranges[] = {
	{
		.range_min = 0,
		.range_max = 255 * 128,
		.selector_reg = TAS2555_PAGECTL_REG,
		.selector_mask = 0xff,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 128,
	},
};

static const struct regmap_config tas2555_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	/* Until better way has been found to support Books/Pages/Registers */
	.cache_type = REGCACHE_NONE,
	.ranges = tas2555_ranges,
	.num_ranges = ARRAY_SIZE(tas2555_ranges),
	.max_register = 255 * 128,
};

#define TAS2555_PCTRL1_MASK	(TAS2555_MADC_POWER_UP | \
				 TAS2555_MDAC_POWER_UP | \
				 TAS2555_DSP_POWER_UP)
#define TAS2555_PCTRL2_MASK	(TAS2555_VSENSE_ENABLE | \
				 TAS2555_ISENSE_ENABLE | \
				 TAS2555_BOOST_ENABLE)
#define TAS2555_MUTE_MASK	(TAS2555_ISENSE_MUTE | TAS2555_CLASSD_MUTE)
#define TAS2555_SOFT_MUTE_MASK	(TAS2555_PDM_SOFT_MUTE | \
				 TAS2555_VSENSE_SOFT_MUTE | \
				 TAS2555_ISENSE_SOFT_MUTE | \
				 TAS2555_CLASSD_SOFT_MUTE)
static void tas2555_enable(struct snd_soc_component *component, bool enable)
{
	struct tas2555_priv *tas2555 = snd_soc_component_get_drvdata(component);

	if (enable == tas2555->enabled)
		return;

	if (enable) {
		snd_soc_component_update_bits(component, TAS2555_POWER_CTRL1_REG,
				    TAS2555_PCTRL1_MASK, TAS2555_PCTRL1_MASK);
		snd_soc_component_update_bits(component, TAS2555_MUTE_REG,
				    TAS2555_MUTE_MASK, 0);
		snd_soc_component_update_bits(component, TAS2555_POWER_CTRL2_REG,
				    TAS2555_PCTRL2_MASK, TAS2555_PCTRL2_MASK);
		snd_soc_component_update_bits(component, TAS2555_SOFT_MUTE_REG,
				    TAS2555_SOFT_MUTE_MASK, 0);
	} else {
		snd_soc_component_update_bits(component, TAS2555_SOFT_MUTE_REG,
				    TAS2555_SOFT_MUTE_MASK,
				    TAS2555_SOFT_MUTE_MASK);
		usleep_range(100, 300);
		snd_soc_component_update_bits(component, TAS2555_POWER_CTRL2_REG,
				    TAS2555_PCTRL2_MASK, 0);
		snd_soc_component_update_bits(component, TAS2555_MUTE_REG,
				    TAS2555_MUTE_MASK, TAS2555_MUTE_MASK);
		snd_soc_component_update_bits(component, TAS2555_POWER_CTRL1_REG,
				    TAS2555_PCTRL1_MASK, 0);
	}

	tas2555->enabled = enable;
}

static int tas2555_dapm_pre_event(struct snd_soc_dapm_widget *w,
				       struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMD:
		/*
		 * We need to do power down sequence before turning off other
		 * DAPM widgets to reduce pop noise.
		 */
		if (snd_soc_component_get_bias_level(component) != SND_SOC_BIAS_ON)
			tas2555_enable(component, false);
		break;
	}
	return 0;
}

static const struct snd_soc_dapm_widget tas2555_dapm_widgets[] =
{
	SND_SOC_DAPM_AIF_IN("ASI1", "ASI1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("ASI2", "ASI2 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("ASIM", "ASIM Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC", NULL, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_OUT_DRV("ClassD", TAS2555_POWER_CTRL2_REG, 7, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("PLL", TAS2555_POWER_CTRL1_REG, 6, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("NDivider", TAS2555_POWER_CTRL1_REG, 5, 0, NULL, 0),

	SND_SOC_DAPM_PRE("Pre Event", tas2555_dapm_pre_event),

	SND_SOC_DAPM_OUTPUT("OUT")
};

static const struct snd_soc_dapm_route tas2555_audio_map[] = {
	{"DAC", NULL, "ASI1"},
	{"DAC", NULL, "ASI2"},
	{"DAC", NULL, "ASIM"},
	{"ClassD", NULL, "DAC"},
	{"OUT", NULL, "ClassD"},
	{"DAC", NULL, "PLL"},
	{"DAC", NULL, "NDivider"},
};

static int tas2555_setup_clocks(struct snd_soc_component *component,
				struct snd_pcm_hw_params *params)
{
	struct tas2555_priv *tas2555 = snd_soc_component_get_drvdata(component);
	int main_clk, i;
	int dac_mod_clk = params_rate(params) * 64;
	bool bypass_pll = true;
	u8 pll_p = 1, pll_j = 1, madc_val;
	u16 pll_d = 0;

	for (i = 1; i < 20; i++) {
		main_clk = params_rate(params) * 512 * i;
		if (main_clk > 24000000)
			break;
	}

	if (tas2555->mclk_clkin % main_clk)
		bypass_pll = false;

	if (bypass_pll) {
		dev_dbg(component->dev, "PLL bypass\n");
		snd_soc_component_write(component, TAS2555_MAIN_CLKIN_REG,
			      tas2555->mclk_clk_id);
		snd_soc_component_write(component, TAS2555_PLL_N_VAL_REG,
			      tas2555->mclk_clkin / main_clk);
		snd_soc_component_update_bits(component, TAS2555_CLK_MISC_REG,
				    TAS2555_DSP_CLK_FROM_PLL, 0);
		goto madc;
	} else {
		u8 p, j;
		u16 d = 0;
		int target_clk = main_clk;
		int mclk_clk = tas2555->mclk_clkin;
		int last_clk = 0;

		dev_dbg(component->dev, "PLL setup\n");
		snd_soc_component_write(component, TAS2555_MAIN_CLKIN_REG,
			      TAS2555_NDIV_CLKIN_PLL);
		snd_soc_component_write(component, TAS2555_PLL_N_VAL_REG, 1);
		snd_soc_component_write(component, TAS2555_PLL_CLKIN_REG,
			      tas2555->mclk_clk_id);
		snd_soc_component_update_bits(component, TAS2555_CLK_MISC_REG,
				    TAS2555_DSP_CLK_FROM_PLL,
				    TAS2555_DSP_CLK_FROM_PLL);

		for (p = 1; p <= 64; p++) {
			for (j = 1; j <= 63; j++) {
				int tmp_clk = mclk_clk * j / p;

				if (abs(target_clk - tmp_clk) <
					abs(target_clk - last_clk)) {
					pll_j = j;
					pll_p = p;
					last_clk = tmp_clk;
				}

				if (tmp_clk == target_clk)
					goto found;
			}
		}

		for (p = 1; p <= 64; p++) {
			int tmp_clk;

			j = target_clk / (mclk_clk * p);
			if (j < 1 || j > 63)
				continue;
			d = ((target_clk * p) - (mclk_clk * j)) / (mclk_clk / 10000);
			tmp_clk = ((mclk_clk / 10000) * (j * 10000 + d)) / (p);

			if (abs(target_clk - tmp_clk) <
				abs(target_clk - last_clk)) {
				pll_j = j;
				pll_p = p;
				pll_d = d;
				last_clk = tmp_clk;
			}

			if (tmp_clk == target_clk) {
				goto found;
			}
		}
	}

found:
	if (pll_p == 64)
		pll_p = 0;

	dev_dbg(component->dev, "PLL divider configuration for MCLK %d, rate %d:",
		tas2555->mclk_clkin, params_rate(params));
	dev_dbg(component->dev, "  J.D = %u.%04u, P = %u\n", pll_j, pll_d, pll_p);
	snd_soc_component_update_bits(component, TAS2555_PLL_P_VAL_REG,
			    TAS2555_PLL_P_VAL_MASK, pll_p);
	snd_soc_component_update_bits(component, TAS2555_PLL_J_VAL_REG,
			    TAS2555_PLL_J_VAL_MASK, pll_j);
	snd_soc_component_write(component, TAS2555_PLL_D_VAL_MSB_REG,
		      TAS2555_PLL_D_MSB_VAL(pll_d));
	snd_soc_component_write(component, TAS2555_PLL_D_VAL_LSB_REG,
		      TAS2555_PLL_D_LSB_VAL(pll_d));
madc:
	madc_val = main_clk / dac_mod_clk;
	dev_dbg(component->dev, "  MADC = %u\n", madc_val);
	if (madc_val > 128) {
		dev_warn(component->dev, "MDAC divider is too high: %u\n",
			 madc_val);
		madc_val = 0;
	} else if (madc_val == 128) {
		madc_val = 0;
	}
	snd_soc_component_write(component, TAS2555_DAC_MADC_VAL_REG, madc_val);

	return 0;
}

static int tas2555_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	unsigned int format_reg;
	u8 w_len;

	switch (dai->id) {
		case 0: /* ASI1 */
			format_reg = TAS2555_ASI1_DAC_FORMAT_REG;
			break;
		case 1: /* ASI2 */
			format_reg = TAS2555_ASI2_DAC_FORMAT_REG;
			break;
		case 2: /* ASI3/ASIM */
			format_reg = TAS2555_ASIM_FORMAT_REG;
			break;
		default:
			dev_err(component->dev, "Invalid DAI%d\n", dai->id);
			return -EINVAL;
	}

	switch (params_width(params)) {
	case 16:
		w_len = TAS2555_WORDLENGTH_16BIT;
		break;
	case 20:
		w_len = TAS2555_WORDLENGTH_20BIT;
		break;
	case 24:
		w_len = TAS2555_WORDLENGTH_24BIT;
		break;
	case 32:
		w_len = TAS2555_WORDLENGTH_32BIT;
		break;
	default:
		dev_err(component->dev, "Not supported sample size: %d\n",
			params_width(params));
		return -EINVAL;
	}

	snd_soc_component_update_bits(component, format_reg,
				      TAS2555_WORDLENGTH_MASK, w_len);

	tas2555_setup_clocks(component, params);
	return 0;
}

static int tas2555_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	struct tas2555_priv *tas2555 = snd_soc_component_get_drvdata(component);
	struct tas2555_dai_cfg *dai_cfg;
	unsigned int format_reg;
	u8 serial_format;

	switch (dai->id) {
		case 0: /* ASI1 */
			format_reg = TAS2555_ASI1_DAC_FORMAT_REG;
			break;
		case 1: /* ASI2 */
			format_reg = TAS2555_ASI2_DAC_FORMAT_REG;
			break;
		case 2: /* ASI3/ASIM */
			format_reg = TAS2555_ASIM_FORMAT_REG;
			break;
		default:
			dev_err(component->dev, "Invalid DAI%d\n", dai->id);
			return -EINVAL;
	}

	dai_cfg = &tas2555->dai_cfg[dai->id];

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		dev_err(component->dev, "Only Codec slave mode is supported\n");
		return -EINVAL;
	}

	switch (fmt & (SND_SOC_DAIFMT_FORMAT_MASK |
		       SND_SOC_DAIFMT_INV_MASK)) {
	case (SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF):
		serial_format = TAS2555_FORMAT_I2S;
		break;
	case (SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_IB_NF):
	case (SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_IB_NF):
		serial_format = TAS2555_FORMAT_DSP;
		break;
	case (SND_SOC_DAIFMT_RIGHT_J | SND_SOC_DAIFMT_NB_NF):
		serial_format = TAS2555_FORMAT_RIGHT_J;
		break;
	case (SND_SOC_DAIFMT_LEFT_J | SND_SOC_DAIFMT_NB_NF):
		serial_format = TAS2555_FORMAT_LEFT_J;
		break;
	default:
		dev_err(component->dev, "DAI Format is not found\n");
		return -EINVAL;
	}

	dai_cfg->dai_fmt = fmt & SND_SOC_DAIFMT_FORMAT_MASK;

	snd_soc_component_update_bits(component, format_reg, TAS2555_FORMAT_MASK,
			    serial_format);
	return 0;
}

static int tas2555_set_dai_sysclk(struct snd_soc_dai *dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_component *component = dai->component;
	struct tas2555_priv *tas2555 = snd_soc_component_get_drvdata(component);

	switch (clk_id) {
	case TAS2555_MCLK_CLKIN_SRC_GPIO1:
		tas2555->mclk_clk_id = TAS2555_XXX_CLKIN_GPIO1;
		break;
	case TAS2555_MCLK_CLKIN_SRC_GPIO2:
		tas2555->mclk_clk_id = TAS2555_XXX_CLKIN_GPIO2;
		break;
	case TAS2555_MCLK_CLKIN_SRC_GPIO3:
		tas2555->mclk_clk_id = TAS2555_XXX_CLKIN_GPIO3;
		break;
	case TAS2555_MCLK_CLKIN_SRC_GPIO4:
		tas2555->mclk_clk_id = TAS2555_XXX_CLKIN_GPIO4;
		break;
	case TAS2555_MCLK_CLKIN_SRC_GPIO5:
		tas2555->mclk_clk_id = TAS2555_XXX_CLKIN_GPIO5;
		break;
	case TAS2555_MCLK_CLKIN_SRC_GPIO6:
		tas2555->mclk_clk_id = TAS2555_XXX_CLKIN_GPIO6;
		break;
	case TAS2555_MCLK_CLKIN_SRC_GPIO7:
		tas2555->mclk_clk_id = TAS2555_XXX_CLKIN_GPIO7;
		break;
	case TAS2555_MCLK_CLKIN_SRC_GPIO8:
		tas2555->mclk_clk_id = TAS2555_XXX_CLKIN_GPIO8;
		break;
	case TAS2555_MCLK_CLKIN_SRC_GPIO9:
		tas2555->mclk_clk_id = TAS2555_XXX_CLKIN_GPIO9;
		break;
	case TAS2555_MCLK_CLKIN_SRC_GPIO10:
		tas2555->mclk_clk_id = TAS2555_XXX_CLKIN_GPIO10;
		break;
	case TAS2555_MCLK_CLKIN_SRC_GPI1:
		tas2555->mclk_clk_id = TAS2555_XXX_CLKIN_GPI1;
		break;
	case TAS2555_MCLK_CLKIN_SRC_GPI2:
		tas2555->mclk_clk_id = TAS2555_XXX_CLKIN_GPI2;
		break;
	case TAS2555_MCLK_CLKIN_SRC_GPI3:
		tas2555->mclk_clk_id = TAS2555_XXX_CLKIN_GPI3;
		break;
	default:
		return -EINVAL;
	}

	tas2555->mclk_clkin = freq;
	return 0;
}

static int tas2555_prepare(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct tas2555_priv *tas2555 = snd_soc_component_get_drvdata(component);
	struct tas2555_dai_cfg *dai_cfg;
	unsigned int delay_reg;
	int delay = 0;

	switch (dai->id) {
		case 0: /* ASI1 */
			delay_reg = TAS2555_ASI1_OFFSET1_REG;
			break;
		case 1: /* ASI2 */
			delay_reg = TAS2555_ASI2_OFFSET1_REG;
			break;
		case 2: /* ASI3/ASIM */
			return 0;
		default:
			dev_err(component->dev, "Invalid DAI%d\n", dai->id);
			return -EINVAL;
	}

	dai_cfg = &tas2555->dai_cfg[dai->id];

	/* TDM slot selection only valid in DSP_A/_B mode */
	if (dai_cfg->dai_fmt == SND_SOC_DAIFMT_DSP_A)
		delay += (dai_cfg->tdm_delay + 1);
	else if (dai_cfg->dai_fmt == SND_SOC_DAIFMT_DSP_B)
		delay += dai_cfg->tdm_delay;

	/* Configure data delay */
	snd_soc_component_write(component, delay_reg, delay);

	return 0;
}

static void tas2555_fw_ready(const struct firmware *fw, void *context)
{
	struct snd_soc_component *component = (struct snd_soc_component *)context;
	struct ti_dsp_fw_header *header;
	struct ti_dsp_fw_entry *cmd;
	size_t count;
	unsigned int reg;
	const u8 *data;

	if (unlikely(!fw) || unlikely(!fw->data)) {
		dev_info(component->dev, "%s firmware is not loaded.\n",
			 TAS2555_FW_MAME);
		return;
	}
	data = fw->data;

	header = (struct ti_dsp_fw_header *) fw->data;
	if (header->deviceID != TAS2555_DEVICE_ID) {
		dev_err(component->dev, "FW device ID (%u) is not valid.\n",
			header->deviceID);
		return;
	}

	dev_dbg(component->dev, "Firmware version %02u.%02u for customer: %u.\n",
		header->fw_version.major, header->fw_version.minor,
		header->customerID);

	count = sizeof(*header);

	snd_soc_component_write(component, TAS2555_CRC_RESET_REG, 1);
	do {
		cmd = (struct ti_dsp_fw_entry *) &data[count];
		reg = TAS2555_REG((unsigned int)cmd->book,
				  (unsigned int)cmd->page,
				  (unsigned int)cmd->reg);
		if (cmd->len == 1)
			tas2555_write(component, reg, cmd->data[0]);
		else
			tas2555_bulk_write(component, reg, cmd->data, cmd->len);
		count += sizeof(*cmd) + cmd->len;
	} while(count < fw->size);

	release_firmware(fw);

	dev_info(component->dev, "uCDSP Checksum: 0x%02x\n",
		 snd_soc_component_read32(component, TAS2555_CRC_CHECKSUM_REG));
	return;
}

static int tas2555_init(struct snd_soc_component *component)
{
	/* Reset the chip */
	snd_soc_component_write(component, TAS2555_SW_RESET_REG, 1);

	/* Set DAC gain to 0dB (default is 15dB ) */
	snd_soc_component_update_bits(component, TAS2555_SPK_CTRL_REG,
			    TAS2555_DAC_GAIN_MASK, 0);
	/*
	 * to increase 1st integrator gm in classD ( this is a performance
	 * improvement for classD )
	 */
	snd_soc_component_write(component, TAS2555_HACK01_REG, 0x1f);
	/* Test mode for Isense */
	snd_soc_component_write(component, TAS2555_HACK_GP02_REG, 0x20);

	return request_firmware_nowait(THIS_MODULE, 1, TAS2555_FW_MAME,
				      component->dev, GFP_KERNEL, component,
				      tas2555_fw_ready);
}

static int tas2555_codec_probe(struct snd_soc_component *component)
{
	struct tas2555_priv *tas2555 = snd_soc_component_get_drvdata(component);
	int ret;

	tas2555->component = component;

	ret = regulator_bulk_enable(ARRAY_SIZE(tas2555->supplies),
				    tas2555->supplies);

	gpiod_set_value(tas2555->nreset, 1);

	ret = regmap_read(tas2555->regmap, TAS2555_BOOKCTL_REG,
			  &tas2555->current_book);
	if (ret < 0)
		tas2555->current_book = 0;

	dev_info(component->dev, "TAS2555 PGID: 0x%02x\n",
		 tas2555_read(component, TAS2555_REV_PGID_REG));

	return tas2555_init(component);
}

static void tas2555_codec_remove(struct snd_soc_component *component)
{
	struct tas2555_priv *tas2555 = snd_soc_component_get_drvdata(component);

	gpiod_set_value(tas2555->nreset, 0);

	regulator_bulk_disable(ARRAY_SIZE(tas2555->supplies),
			       tas2555->supplies);

}

/*
 * DAC digital volumes. From 0 to 15 dB in 1 dB steps
 */
static DECLARE_TLV_DB_SCALE(dac_tlv, 0, 100, 0);

static const char * const tas2555_dsp_modes[] = {
	"RAM mode",
	"ROM mode1 - Digital Playback",
	"ROM mode2 - Analog Playback",
	"ROM mode3 - Analog Playback with iSense",
	"ROM mode4 - PDM Playback",
	"ROM mode5 - Digital Playback with I and Vsense",
	"ROM mode6 - Digital Playback LP",
};
static SOC_ENUM_SINGLE_DECL(tas2555_dsp_mode_enum, TAS2555_DSP_MODE_SELECT_REG,
			    0, tas2555_dsp_modes);

static const char * const tas2555_coeff_select[] = {
	"Downloaded custom",
	"Default from ZROM",
};
static SOC_ENUM_SINGLE_DECL(tas2555_coeff_select_enum,
			    TAS2555_DSP_MODE_SELECT_REG, 5,
			    tas2555_coeff_select);

static const char * const tas2555_asix_source_select[] = {
	"Left",
	"Right",
	"(Left + Right) / 2",
	"monoPCM",
};
static SOC_ENUM_SINGLE_DECL(tas2555_asi1_source_enum,
			    TAS2555_ASIX_SOURCE_REG, 1,
			    tas2555_asix_source_select);
static SOC_ENUM_SINGLE_DECL(tas2555_asi2_source_enum,
			    TAS2555_ASIX_SOURCE_REG, 3,
			    tas2555_asix_source_select);

static const struct snd_kcontrol_new tas2555_snd_controls[] = {
	SOC_SINGLE_TLV("DAC Playback Volume",
			 TAS2555_SPK_CTRL_REG, 3, 0x0f, 0, dac_tlv),
	SOC_ENUM("DSP mode", tas2555_dsp_mode_enum),
	SOC_ENUM("Coefficient selection", tas2555_coeff_select_enum),
	SOC_ENUM("ASI1 digital source", tas2555_asi1_source_enum),
	SOC_ENUM("ASI2 digital source", tas2555_asi2_source_enum),
};

static int tas2555_set_bias_level(struct snd_soc_component *component,
				  enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
		/*
		 * Turn on the tas2555 after the DAPM sequence and clock
		 * configuration
		 */
		tas2555_enable(component, true);
		break;
	case SND_SOC_BIAS_STANDBY:
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_OFF:
		tas2555_enable(component, false);
		break;
	}

	return 0;
}

static struct snd_soc_component_driver soc_codec_driver_tas2555 = {
	.probe			= tas2555_codec_probe,
	.remove			= tas2555_codec_remove,
	.read			= tas2555_read,
	.write			= tas2555_write,
	.set_bias_level		= tas2555_set_bias_level,

	.controls		= tas2555_snd_controls,
	.num_controls		= ARRAY_SIZE(tas2555_snd_controls),
	.dapm_widgets		= tas2555_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(tas2555_dapm_widgets),
	.dapm_routes		= tas2555_audio_map,
	.num_dapm_routes	= ARRAY_SIZE(tas2555_audio_map),

	.idle_bias_on		= 0,
	.use_pmdown_time	= 0,
	.endianness		= 1,
	.non_legacy_dai_naming	= 1,
};

static struct snd_soc_dai_ops tas2555_dai_ops = {
	.hw_params	= tas2555_hw_params,
	.prepare	= tas2555_prepare,
	.set_sysclk	= tas2555_set_dai_sysclk,
	.set_fmt	= tas2555_set_dai_fmt,
};

#define TAS2555_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
			 SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)
static struct snd_soc_dai_driver tas2555_dai_driver[] = {
	{
		.name = "tas2555 ASI1",
		.id = 0,
		.playback = {
			.stream_name	= "ASI1 Playback",
			.channels_min	= 2,
			.channels_max	= 2,
			.rates		= SNDRV_PCM_RATE_8000_192000,
			.formats	= TAS2555_FORMATS,
		},
		.ops = &tas2555_dai_ops,
	},
	{
		.name = "tas2555 ASI2",
		.id = 1,
		.playback = {
			.stream_name	= "ASI2 Playback",
			.channels_min	= 2,
			.channels_max	= 2,
			.rates		= SNDRV_PCM_RATE_8000_192000,
			.formats	= TAS2555_FORMATS,
		},
		.ops = &tas2555_dai_ops,
	},
	{
		.name = "tas2555 ASIM",
		.id = 2,
		.playback = {
			.stream_name	= "ASIM Playback",
			.channels_min	= 2,
			.channels_max	= 2,
			.rates		= SNDRV_PCM_RATE_8000_192000,
			.formats	= TAS2555_FORMATS,
		},
		.ops = &tas2555_dai_ops,
	},
};

static int tas2555_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct tas2555_priv *tas2555;
	int i, ret;
	const struct regmap_config *regmap_config;

	regmap_config = &tas2555_i2c_regmap;

	tas2555 = devm_kzalloc(dev, sizeof(*tas2555), GFP_KERNEL);
	if (tas2555 == NULL)
		return -ENOMEM;

	tas2555->nreset = devm_gpiod_get_optional(dev, "nreset",
						  GPIOD_OUT_LOW);
	if (IS_ERR(tas2555->nreset))
		return PTR_ERR(tas2555->nreset);

	for (i = 0; i < ARRAY_SIZE(tas2555->supplies); i++) {
		tas2555->supplies[i].supply = tas2555_supply_names[i];
		/* vbat supply is optional */
// 		if (i == 0)
// 			tas2555->supplies[i].optional = true;
	}

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(tas2555->supplies),
				      tas2555->supplies);
	if (ret != 0) {
		dev_err(dev, "Failed to request supplies: %d\n", ret);
		return ret;
	}

	tas2555->regmap = devm_regmap_init_i2c(client, regmap_config);
	if (IS_ERR(tas2555->regmap)) {
		ret = PTR_ERR(tas2555->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}
	tas2555->dev = dev;

	dev_set_drvdata(dev, tas2555);

	return devm_snd_soc_register_component(dev, &soc_codec_driver_tas2555,
					       tas2555_dai_driver,
					       ARRAY_SIZE(tas2555_dai_driver));
}

static const struct i2c_device_id tas2555_i2c_id[] = {
	{ "tas2555", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, tas2555_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id tas2555_of_match[] = {
	{ .compatible = "ti,tas2555" },
	{},
};
MODULE_DEVICE_TABLE(of, tas2555_of_match);
#endif

static struct i2c_driver tas2555_i2c_driver = {
	.driver = {
		.name	= "tas2555",
		.of_match_table = of_match_ptr(tas2555_of_match),
	},
	.probe		= tas2555_i2c_probe,
	.id_table	= tas2555_i2c_id,
};

module_i2c_driver(tas2555_i2c_driver);

MODULE_AUTHOR("Peter Ujfalusi <peter.ujfalusi@ti.com>");
MODULE_DESCRIPTION("TAS2555 Smart Amplifier driver");
MODULE_LICENSE("GPL");
