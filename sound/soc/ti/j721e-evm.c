// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com
 *  Author: Peter Ujfalusi <peter.ujfalusi@ti.com>
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>

#include <sound/pcm_params.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <dt-bindings/sound/ti-mcasp.h>
#include "davinci-mcasp.h"

#define J721E_MAX_DAI_LINK	3

#define J721E_AUDIO_DOMAIN_CPB	0
#define J721E_AUDIO_DOMAIN_IVI	1

#define J721E_DAI_LINK_CODEC1		J721E_AUDIO_DOMAIN_CPB
#define J721E_DAI_LINK_CODECA		(J721E_AUDIO_DOMAIN_IVI | (0 << 4))
#define J721E_DAI_LINK_CODECB		(J721E_AUDIO_DOMAIN_IVI | (1 << 4))
#define J721E_DAI_LINK_IVI_MULTICODEC	(J721E_AUDIO_DOMAIN_IVI | (2 << 4))

#define J721E_DAI_LINK_ID_TO_DOMAIN(x)	((x) & 0xf)

#define J721E_CLK_PARENT_48000	0
#define J721E_CLK_PARENT_44100	1

#define J721E_MAX_CLK_HSDIV	128
#define PCM1368A_MAX_SYSCLK	36864000

#define J721E_DAI_FMT		(SND_SOC_DAIFMT_RIGHT_J | \
				 SND_SOC_DAIFMT_NB_NF |   \
				 SND_SOC_DAIFMT_CBS_CFS)

enum j721e_board_type {
	J721E_BOARD_CPB = 1,
	J721E_BOARD_CPB_IVI,
	J721E_BOARD_CPB_IVI_MULTICODEC,
};

struct j721e_audio_match_data {
	enum j721e_board_type board_type;
	int num_links;
};

static unsigned int ratios_for_pcm3168a[] = {
	256,
	512,
	768,
};

struct j721e_audio_clocks {
	struct clk *target;
	struct clk *parent[2];
};

struct j721e_audio_domain {
	struct j721e_audio_clocks codec;
	struct j721e_audio_clocks mcasp;
	int parent_clk_id;

	int active;
	unsigned int active_link;
	unsigned int rate;
};

struct j721e_priv {
	struct device *dev;
	struct snd_soc_card card;
	struct snd_soc_dai_link *dai_links;
	struct snd_soc_codec_conf codec_conf[J721E_MAX_DAI_LINK];
	struct snd_interval rate_range;
	const struct j721e_audio_match_data *match_data;
	u32 pll_rates[2];
	unsigned int hsdiv_rates[2];

	struct j721e_audio_domain audio_domains[2];

	struct mutex mutex;
};

static const struct snd_soc_dapm_widget j721e_cpb_dapm_widgets[] = {
	SND_SOC_DAPM_HP("CPB Stereo HP 1", NULL),
	SND_SOC_DAPM_HP("CPB Stereo HP 2", NULL),
	SND_SOC_DAPM_HP("CPB Stereo HP 3", NULL),
	SND_SOC_DAPM_LINE("CPB Line Out", NULL),
	SND_SOC_DAPM_MIC("CPB Stereo Mic 1", NULL),
	SND_SOC_DAPM_MIC("CPB Stereo Mic 2", NULL),
	SND_SOC_DAPM_LINE("CPB Line In", NULL),
};

static const struct snd_soc_dapm_route j721e_cpb_dapm_routes[] = {
	{"CPB Stereo HP 1", NULL, "codec1 AOUT1L"},
	{"CPB Stereo HP 1", NULL, "codec1 AOUT1R"},
	{"CPB Stereo HP 2", NULL, "codec1 AOUT2L"},
	{"CPB Stereo HP 2", NULL, "codec1 AOUT2R"},
	{"CPB Stereo HP 3", NULL, "codec1 AOUT3L"},
	{"CPB Stereo HP 3", NULL, "codec1 AOUT3R"},
	{"CPB Line Out", NULL, "codec1 AOUT4L"},
	{"CPB Line Out", NULL, "codec1 AOUT4R"},

	{"codec1 AIN1L", NULL, "CPB Stereo Mic 1"},
	{"codec1 AIN1R", NULL, "CPB Stereo Mic 1"},
	{"codec1 AIN2L", NULL, "CPB Stereo Mic 2"},
	{"codec1 AIN2R", NULL, "CPB Stereo Mic 2"},
	{"codec1 AIN3L", NULL, "CPB Line In"},
	{"codec1 AIN3R", NULL, "CPB Line In"},
};

static const struct snd_soc_dapm_widget j721e_ivi_codec_a_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("IVI A Line Out 1", NULL),
	SND_SOC_DAPM_LINE("IVI A Line Out 2", NULL),
	SND_SOC_DAPM_LINE("IVI A Line Out 3", NULL),
	SND_SOC_DAPM_LINE("IVI A Line Out 4", NULL),
	SND_SOC_DAPM_MIC("IVI A Stereo Mic 1", NULL),
	SND_SOC_DAPM_MIC("IVI A Stereo Mic 2", NULL),
	SND_SOC_DAPM_LINE("IVI A Line In", NULL),
};

static const struct snd_soc_dapm_route j721e_codec_a_dapm_routes[] = {
	{"IVI A Line Out 1", NULL, "codeca AOUT1L"},
	{"IVI A Line Out 1", NULL, "codeca AOUT1R"},
	{"IVI A Line Out 2", NULL, "codeca AOUT2L"},
	{"IVI A Line Out 2", NULL, "codeca AOUT2R"},
	{"IVI A Line Out 3", NULL, "codeca AOUT3L"},
	{"IVI A Line Out 3", NULL, "codeca AOUT3R"},
	{"IVI A Line Out 4", NULL, "codeca AOUT4L"},
	{"IVI A Line Out 4", NULL, "codeca AOUT4R"},

	{"codeca AIN1L", NULL, "IVI A Stereo Mic 1"},
	{"codeca AIN1R", NULL, "IVI A Stereo Mic 1"},
	{"codeca AIN2L", NULL, "IVI A Stereo Mic 2"},
	{"codeca AIN2R", NULL, "IVI A Stereo Mic 2"},
	{"codeca AIN3L", NULL, "IVI A Line In"},
	{"codeca AIN3R", NULL, "IVI A Line In"},
};

static const struct snd_soc_dapm_widget j721e_ivi_codec_b_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("IVI B Line Out 1", NULL),
	SND_SOC_DAPM_LINE("IVI B Line Out 2", NULL),
	SND_SOC_DAPM_LINE("IVI B Line Out 3", NULL),
	SND_SOC_DAPM_LINE("IVI B Line Out 4", NULL),
	SND_SOC_DAPM_MIC("IVI B Stereo Mic 1", NULL),
	SND_SOC_DAPM_MIC("IVI B Stereo Mic 2", NULL),
	SND_SOC_DAPM_LINE("IVI B Line In", NULL),
};

static const struct snd_soc_dapm_route j721e_codec_b_dapm_routes[] = {
	{"IVI B Line Out 1", NULL, "codecb AOUT1L"},
	{"IVI B Line Out 1", NULL, "codecb AOUT1R"},
	{"IVI B Line Out 2", NULL, "codecb AOUT2L"},
	{"IVI B Line Out 2", NULL, "codecb AOUT2R"},
	{"IVI B Line Out 3", NULL, "codecb AOUT3L"},
	{"IVI B Line Out 3", NULL, "codecb AOUT3R"},
	{"IVI B Line Out 4", NULL, "codecb AOUT4L"},
	{"IVI B Line Out 4", NULL, "codecb AOUT4R"},

	{"codecb AIN1L", NULL, "IVI B Stereo Mic 1"},
	{"codecb AIN1R", NULL, "IVI B Stereo Mic 1"},
	{"codecb AIN2L", NULL, "IVI B Stereo Mic 2"},
	{"codecb AIN2R", NULL, "IVI B Stereo Mic 2"},
	{"codecb AIN3L", NULL, "IVI B Line In"},
	{"codecb AIN3R", NULL, "IVI B Line In"},
};

static int j721e_configure_refclk(struct j721e_priv *priv,
				  unsigned int audio_domain, unsigned int rate)
{
	struct j721e_audio_domain *domain = &priv->audio_domains[audio_domain];
	unsigned int scki;
	int ret = -EINVAL;
	int i, clk_id;

	if (!(rate % 8000))
		clk_id = J721E_CLK_PARENT_48000;
	else if (!(rate % 11025))
		clk_id = J721E_CLK_PARENT_44100;
	else
		return ret;

	for (i = 0; i < ARRAY_SIZE(ratios_for_pcm3168a); i++) {
		scki = ratios_for_pcm3168a[i] * rate;

		if (priv->pll_rates[clk_id] / scki <= J721E_MAX_CLK_HSDIV) {
			ret = 0;
			break;
		}
	}

	if (ret) {
		dev_err(priv->dev, "No valid clock configuration for %u Hz\n",
			rate);
		return ret;
	}

	if (priv->hsdiv_rates[domain->parent_clk_id] != scki) {
		dev_dbg(priv->dev,
			"%s configuration for %u Hz: %s, %dxFS (SCKI: %u Hz)\n",
			audio_domain == J721E_AUDIO_DOMAIN_CPB ? "CPB" : "IVI",
			rate,
			clk_id == J721E_CLK_PARENT_48000 ? "PLL4" : "PLL15",
			ratios_for_pcm3168a[i], scki);

		if (domain->parent_clk_id != clk_id) {
			ret = clk_set_parent(domain->codec.target,
					     domain->codec.parent[clk_id]);
			if (ret)
				return ret;

			ret = clk_set_parent(domain->mcasp.target,
					     domain->mcasp.parent[clk_id]);
			if (ret)
				return ret;

			domain->parent_clk_id = clk_id;
		}

		ret = clk_set_rate(domain->codec.target, scki);
		if (ret) {
			dev_err(priv->dev, "codec set rate failed for %u Hz\n",
				scki);
			return ret;
		}

		ret = clk_set_rate(domain->mcasp.target, scki);
		if (!ret) {
			priv->hsdiv_rates[domain->parent_clk_id] = scki;
		} else {
			dev_err(priv->dev, "mcasp set rate failed for %u Hz\n",
				scki);
			return ret;
		}
	}

	return ret;
}

static u8 mcasp0_codec_a_serializers[] =
{
	1, 1, 1, 1,
	2, 2, 2, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
};

static u8 mcasp0_codec_b_serializers[] =
{
	0, 0, 0, 0,
	0, 0, 0, 1,
	1, 1, 1, 2,
	2, 2, 0, 0,
};

static int j721e_rule_rate(struct snd_pcm_hw_params *params,
			   struct snd_pcm_hw_rule *rule)
{
	struct snd_interval *t = rule->private;

	return snd_interval_refine(hw_param_interval(params, rule->var), t);
}

static int j721e_audio_startup(struct snd_pcm_substream *substream)
{
	struct 	snd_soc_pcm_runtime *rtd = substream->private_data;
	struct j721e_priv *priv = snd_soc_card_get_drvdata(rtd->card);
	unsigned int domain_id = J721E_DAI_LINK_ID_TO_DOMAIN(rtd->dai_link->id);
	struct j721e_audio_domain *domain = &priv->audio_domains[domain_id];
	unsigned int active_rate;
	int ret = 0;
	int i;

	mutex_lock(&priv->mutex);

	if (!domain->active) {
		u8 *serial_dir = NULL;

		domain->active_link = rtd->dai_link->id;
		if (rtd->dai_link->id == J721E_DAI_LINK_CODECA)
			serial_dir = mcasp0_codec_a_serializers;
		else if (rtd->dai_link->id == J721E_DAI_LINK_CODECB)
			serial_dir = mcasp0_codec_b_serializers;

		if (serial_dir)
			davinci_mcasp_set_serializer_dir(rtd->cpu_dai, 16,
							 serial_dir);

	} else if (domain->active_link &&
		   domain->active_link != rtd->dai_link->id) {
		dev_err(priv->dev, "Codec %s is already opened\n",
			rtd->dai_link->id == J721E_DAI_LINK_CODECA ? "A" : "B");
		mutex_unlock(&priv->mutex);
		return -ENOTSUPP;
	}

	domain->active++;

	if (priv->audio_domains[J721E_AUDIO_DOMAIN_CPB].rate)
		active_rate = priv->audio_domains[J721E_AUDIO_DOMAIN_CPB].rate;
	else
		active_rate = priv->audio_domains[J721E_AUDIO_DOMAIN_IVI].rate;

	if (active_rate)
		ret = snd_pcm_hw_constraint_single(substream->runtime,
						   SNDRV_PCM_HW_PARAM_RATE,
						   active_rate);
	else
		ret = snd_pcm_hw_rule_add(substream->runtime, 0,
					  SNDRV_PCM_HW_PARAM_RATE,
					  j721e_rule_rate, &priv->rate_range,
					  SNDRV_PCM_HW_PARAM_RATE, -1);

	mutex_unlock(&priv->mutex);

	if (ret)
		return ret;

	/* Reset TDM slots to 32 */
	ret = snd_soc_dai_set_tdm_slot(rtd->cpu_dai, 0x3, 0x3, 2, 32);
	if (ret && ret != -ENOTSUPP)
		return ret;

	for (i = 0; i < rtd->num_codecs; i++) {
		ret = snd_soc_dai_set_tdm_slot(rtd->codec_dais[i], 0x3, 0x3, 2,
					       32);
		if (ret && ret != -ENOTSUPP)
			return ret;
	}

	return 0;
}

static int j721e_audio_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	struct 	snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct j721e_priv *priv = snd_soc_card_get_drvdata(card);
	unsigned int domain_id = J721E_DAI_LINK_ID_TO_DOMAIN(rtd->dai_link->id);
	struct j721e_audio_domain *domain = &priv->audio_domains[domain_id];
	int slot_width = 32;
	int ret, i;

	mutex_lock(&priv->mutex);

	if (domain->rate && domain->rate != params_rate(params)) {
		ret = -EINVAL;
		goto out;
	}

	if (params_width(params) == 16)
		slot_width = 16;

	ret = snd_soc_dai_set_tdm_slot(rtd->cpu_dai, 0x3, 0x3, 2, slot_width);
	if (ret && ret != -ENOTSUPP)
		goto out;

	for (i = 0; i < rtd->num_codecs; i++) {
		ret = snd_soc_dai_set_tdm_slot(rtd->codec_dais[i], 0x3, 0x3, 2,
					       slot_width);
		if (ret && ret != -ENOTSUPP)
			return ret;
	}

	ret = j721e_configure_refclk(priv, domain_id, params_rate(params));
	if (ret)
		goto out;

	for (i = 0; i < rtd->num_codecs; i++) {
		ret = snd_soc_dai_set_sysclk(rtd->codec_dais[i], 0,
				priv->hsdiv_rates[domain->parent_clk_id],
				SND_SOC_CLOCK_IN);
		if (ret && ret != -ENOTSUPP) {
			dev_err(priv->dev, "codec set_sysclk failed for %u Hz\n",
				priv->hsdiv_rates[domain->parent_clk_id]);
			goto out;
		}
	}

	ret = snd_soc_dai_set_sysclk(rtd->cpu_dai, MCASP_CLK_HCLK_AUXCLK,
				     priv->hsdiv_rates[domain->parent_clk_id],
				     SND_SOC_CLOCK_IN);

	if (ret && ret != -ENOTSUPP) {
		dev_err(priv->dev, "mcasp set_sysclk failed for %u Hz\n",
			priv->hsdiv_rates[domain->parent_clk_id]);
	} else {
		domain->rate = params_rate(params);
		ret = 0;
	}

out:
	mutex_unlock(&priv->mutex);
	return ret;
}

static void j721e_audio_shutdown(struct snd_pcm_substream *substream)
{
	struct 	snd_soc_pcm_runtime *rtd = substream->private_data;
	struct j721e_priv *priv = snd_soc_card_get_drvdata(rtd->card);
	unsigned int domain_id = J721E_DAI_LINK_ID_TO_DOMAIN(rtd->dai_link->id);
	struct j721e_audio_domain *domain = &priv->audio_domains[domain_id];

	mutex_lock(&priv->mutex);

	domain->active--;
	if (!domain->active) {
		domain->rate = 0;
		domain->active_link = 0;
	}

	mutex_unlock(&priv->mutex);
}

static const struct snd_soc_ops j721e_audio_ops = {
	.startup = j721e_audio_startup,
	.hw_params = j721e_audio_hw_params,
	.shutdown = j721e_audio_shutdown,
};

static int j721e_audio_init(struct snd_soc_pcm_runtime *rtd)
{
	struct j721e_priv *priv = snd_soc_card_get_drvdata(rtd->card);
	unsigned int domain_id = J721E_DAI_LINK_ID_TO_DOMAIN(rtd->dai_link->id);
	struct j721e_audio_domain *domain = &priv->audio_domains[domain_id];
	int ret, i;

	/* Set up initial clock configuration */
	ret = j721e_configure_refclk(priv, domain_id, 48000);
	if (ret)
		return ret;

	for (i = 0; i < rtd->num_codecs; i++) {
		ret = snd_soc_dai_set_sysclk(rtd->codec_dais[i], 0,
				priv->hsdiv_rates[domain->parent_clk_id],
				SND_SOC_CLOCK_IN);
		if (ret && ret != -ENOTSUPP)
			return ret;
	}

	ret = snd_soc_dai_set_sysclk(rtd->cpu_dai, MCASP_CLK_HCLK_AUXCLK,
				     priv->hsdiv_rates[domain->parent_clk_id],
				     SND_SOC_CLOCK_IN);
	if (ret && ret != -ENOTSUPP)
		return ret;

	/* Set initial tdm slots */
	ret = snd_soc_dai_set_tdm_slot(rtd->cpu_dai, 0x3, 0x3, 2, 32);
	if (ret && ret != -ENOTSUPP)
		return ret;

	for (i = 0; i < rtd->num_codecs; i++) {
		ret = snd_soc_dai_set_tdm_slot(rtd->codec_dais[i], 0x3, 0x3, 2,
					       32);
		if (ret && ret != -ENOTSUPP)
			return ret;
	}

	return 0;
}

static int j721e_audio_init_ivi(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dapm_context *dapm = &rtd->card->dapm;

	if (rtd->dai_link->id == J721E_DAI_LINK_CODECA) {
		snd_soc_dapm_new_controls(dapm, j721e_ivi_codec_a_dapm_widgets,
				ARRAY_SIZE(j721e_ivi_codec_a_dapm_widgets));
		snd_soc_dapm_add_routes(dapm, j721e_codec_a_dapm_routes,
					ARRAY_SIZE(j721e_codec_a_dapm_routes));
	} else if (rtd->dai_link->id == J721E_DAI_LINK_CODECB){
		snd_soc_dapm_new_controls(dapm, j721e_ivi_codec_b_dapm_widgets,
				ARRAY_SIZE(j721e_ivi_codec_b_dapm_widgets));
		snd_soc_dapm_add_routes(dapm, j721e_codec_b_dapm_routes,
					ARRAY_SIZE(j721e_codec_b_dapm_routes));
	} else {
		snd_soc_dapm_new_controls(dapm, j721e_ivi_codec_a_dapm_widgets,
				ARRAY_SIZE(j721e_ivi_codec_a_dapm_widgets));
		snd_soc_dapm_add_routes(dapm, j721e_codec_a_dapm_routes,
					ARRAY_SIZE(j721e_codec_a_dapm_routes));
		snd_soc_dapm_new_controls(dapm, j721e_ivi_codec_b_dapm_widgets,
				ARRAY_SIZE(j721e_ivi_codec_b_dapm_widgets));
		snd_soc_dapm_add_routes(dapm, j721e_codec_b_dapm_routes,
					ARRAY_SIZE(j721e_codec_b_dapm_routes));
	}

	return j721e_audio_init(rtd);
}

static int j721e_get_clocks(struct device *dev,
			    struct j721e_audio_clocks *clocks, char *prefix)
{
	struct clk *parent;
	char *clk_name;
	int ret;

	clocks->target = devm_clk_get(dev, prefix);
	if (IS_ERR(clocks->target)) {
		ret = PTR_ERR(clocks->target);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to acquire %s': %d\n",
				prefix, ret);
		return ret;
	}

	clk_name = kasprintf(GFP_KERNEL, "%s-48000", prefix);
	if (clk_name) {
		parent = devm_clk_get(dev, clk_name);
		kfree(clk_name);
		if (IS_ERR(parent)) {
			ret = PTR_ERR(parent);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "failed to acquire %s': %d\n",
					prefix, ret);
			return ret;
		}
		clocks->parent[J721E_CLK_PARENT_48000] = parent;
	} else {
		return -ENOMEM;
	}

	clk_name = kasprintf(GFP_KERNEL, "%s-44100", prefix);
	if (clk_name) {
		parent = devm_clk_get(dev, clk_name);
		kfree(clk_name);
		if (IS_ERR(parent)) {
			ret = PTR_ERR(parent);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "failed to acquire %s': %d\n",
					prefix, ret);
			return ret;
		}
		clocks->parent[J721E_CLK_PARENT_44100] = parent;
	} else {
		return -ENOMEM;
	}

	return 0;
}

#if defined(CONFIG_OF)
static const struct j721e_audio_match_data j721e_cpb_data = {
	.board_type = J721E_BOARD_CPB,
	.num_links = 2, /* CPB pcm3168a */
};

static const struct j721e_audio_match_data j721e_cpb_ivi_data = {
	.board_type = J721E_BOARD_CPB_IVI,
	.num_links = 6, /* CPB pcm3168a + 2x IVI pcm3168a */
};

static const struct j721e_audio_match_data j721e_cpb_ivi_multicodec_data = {
	.board_type = J721E_BOARD_CPB_IVI_MULTICODEC,
	.num_links = 4, /* CPB pcm3168a + 2x IVI pcm3168a */
};

static const struct of_device_id j721e_audio_of_match[] = {
	{
		.compatible = "ti,j721e-cpb-audio",
		.data = &j721e_cpb_data,
	}, {
		.compatible = "ti,j721e-cpb-ivi-audio",
		.data = &j721e_cpb_ivi_data,
	}, {
		.compatible = "ti,j721e-cpb-ivi-multicodec-audio",
		.data = &j721e_cpb_ivi_multicodec_data,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, j721e_audio_of_match);
#endif

static void j721e_calculate_rate_range(struct j721e_priv *priv)
{
	unsigned int min_rate, max_rate, pll_rate;

	pll_rate = priv->pll_rates[J721E_CLK_PARENT_44100];
	min_rate = pll_rate / J721E_MAX_CLK_HSDIV;
	min_rate /= ratios_for_pcm3168a[ARRAY_SIZE(ratios_for_pcm3168a) - 1];

	pll_rate = priv->pll_rates[J721E_CLK_PARENT_48000];
	if (pll_rate > PCM1368A_MAX_SYSCLK)
		pll_rate = PCM1368A_MAX_SYSCLK;

	max_rate = pll_rate / ratios_for_pcm3168a[0];

	snd_interval_any(&priv->rate_range);
	priv->rate_range.min = min_rate;
	priv->rate_range.max = max_rate;
}

static int j721e_soc_probe_cpb(struct j721e_priv *priv, int *link_idx,
			       int *conf_idx)
{
	struct device_node *node = priv->dev->of_node;
	struct device_node *dai_node, *codec_node;
	struct j721e_audio_domain *domain;
	int ret;

	dai_node = of_parse_phandle(node, "ti,cpb-mcasp", 0);
	if (!dai_node) {
		dev_err(priv->dev, "CPB McASP node is not provided\n");
		return -EINVAL;
	}

	codec_node = of_parse_phandle(node, "ti,cpb-codec", 0);
	if (!codec_node) {
		dev_err(priv->dev, "CPB codec node is not provided\n");
		return -EINVAL;
	}

	domain = &priv->audio_domains[J721E_AUDIO_DOMAIN_CPB];
	ret = j721e_get_clocks(priv->dev, &domain->codec, "audio-refclk2");
	if (ret)
		return ret;

	ret = j721e_get_clocks(priv->dev, &domain->mcasp, "cpb-mcasp");
	if (ret)
		return ret;

	/* Playback to codec1 */
	priv->dai_links[*link_idx].name = "CPB pcm3168a DAC";
	priv->dai_links[*link_idx].stream_name = "cpb pcm3168a Playback";
	priv->dai_links[*link_idx].cpu_of_node = dai_node;
	priv->dai_links[*link_idx].platform_of_node = dai_node;
	priv->dai_links[*link_idx].codec_of_node = codec_node;
	priv->dai_links[*link_idx].codec_dai_name = "pcm3168a-dac";
	priv->dai_links[*link_idx].playback_only = 1;
	priv->dai_links[*link_idx].id = J721E_DAI_LINK_CODEC1;
	priv->dai_links[*link_idx].dai_fmt = J721E_DAI_FMT;
	priv->dai_links[*link_idx].init = j721e_audio_init;
	priv->dai_links[*link_idx].ops = &j721e_audio_ops;
	(*link_idx)++;

	/* Capture from codec1 */
	priv->dai_links[*link_idx].name = "CPB pcm3168a ADC";
	priv->dai_links[*link_idx].stream_name = "cpb pcm3168a Capture";
	priv->dai_links[*link_idx].cpu_of_node = dai_node;
	priv->dai_links[*link_idx].platform_of_node = dai_node;
	priv->dai_links[*link_idx].codec_of_node = codec_node;
	priv->dai_links[*link_idx].codec_dai_name = "pcm3168a-adc";
	priv->dai_links[*link_idx].capture_only = 1;
	priv->dai_links[*link_idx].id = J721E_DAI_LINK_CODEC1;
	priv->dai_links[*link_idx].dai_fmt = J721E_DAI_FMT;
	priv->dai_links[*link_idx].init = j721e_audio_init;
	priv->dai_links[*link_idx].ops = &j721e_audio_ops;
	(*link_idx)++;

	priv->codec_conf[*conf_idx].of_node = codec_node;
	priv->codec_conf[*conf_idx].name_prefix = "codec1";
	(*conf_idx)++;

	return 0;
}

static int j721e_soc_probe_ivi(struct j721e_priv *priv, int *link_idx,
			       int *conf_idx)
{
	struct device_node *node = priv->dev->of_node;
	struct device_node *dai_node, *codeca_node, *codecb_node;
	struct j721e_audio_domain *domain;
	int ret;

	if (priv->match_data->board_type != J721E_BOARD_CPB_IVI)
		return 0;

	dai_node = of_parse_phandle(node, "ti,ivi-mcasp", 0);
	if (!dai_node) {
		dev_err(priv->dev, "IVI McASP node is not provided\n");
		return -EINVAL;
	}

	codeca_node = of_parse_phandle(node, "ti,ivi-codec-a", 0);
	if (!codeca_node) {
		dev_err(priv->dev, "IVI codec-a node is not provided\n");
		return -EINVAL;
	}

	codecb_node = of_parse_phandle(node, "ti,ivi-codec-b", 0);
	if (!codecb_node) {
		dev_warn(priv->dev, "IVI codec-b node is not provided\n");
		return 0;
	}

	domain = &priv->audio_domains[J721E_AUDIO_DOMAIN_IVI];
	ret = j721e_get_clocks(priv->dev, &domain->codec,
			       "audio-refclk0");
	if (ret)
		return ret;

	ret = j721e_get_clocks(priv->dev, &domain->mcasp, "ivi-mcasp");
	if (ret)
		return ret;

	/* Playback to codeca */
	priv->dai_links[*link_idx].name = "IVI pcm3168a-a DAC";
	priv->dai_links[*link_idx].stream_name = "ivi pcm3168a-a Playback";
	priv->dai_links[*link_idx].cpu_of_node = dai_node;
	priv->dai_links[*link_idx].platform_of_node = dai_node;
	priv->dai_links[*link_idx].codec_of_node = codeca_node;
	priv->dai_links[*link_idx].codec_dai_name = "pcm3168a-dac";
	priv->dai_links[*link_idx].playback_only = 1;
	priv->dai_links[*link_idx].id = J721E_DAI_LINK_CODECA;
	priv->dai_links[*link_idx].dai_fmt = J721E_DAI_FMT;
	priv->dai_links[*link_idx].init = j721e_audio_init_ivi;
	priv->dai_links[*link_idx].ops = &j721e_audio_ops;
	(*link_idx)++;

	/* Capture from codeca */
	priv->dai_links[*link_idx].name = "IVI pcm3168a-a ADC";
	priv->dai_links[*link_idx].stream_name = "cpb pcm3168a-a Capture";
	priv->dai_links[*link_idx].cpu_of_node = dai_node;
	priv->dai_links[*link_idx].platform_of_node = dai_node;
	priv->dai_links[*link_idx].codec_of_node = codeca_node;
	priv->dai_links[*link_idx].codec_dai_name = "pcm3168a-adc";
	priv->dai_links[*link_idx].capture_only = 1;
	priv->dai_links[*link_idx].id = J721E_DAI_LINK_CODECA;
	priv->dai_links[*link_idx].dai_fmt = J721E_DAI_FMT;
	priv->dai_links[*link_idx].init = j721e_audio_init;
	priv->dai_links[*link_idx].ops = &j721e_audio_ops;
	(*link_idx)++;

	/* Playback to codecb */
	priv->dai_links[*link_idx].name = "IVI pcm3168a-b DAC";
	priv->dai_links[*link_idx].stream_name = "ivi pcm3168a-b Playback";
	priv->dai_links[*link_idx].cpu_of_node = dai_node;
	priv->dai_links[*link_idx].platform_of_node = dai_node;
	priv->dai_links[*link_idx].codec_of_node = codecb_node;
	priv->dai_links[*link_idx].codec_dai_name = "pcm3168a-dac";
	priv->dai_links[*link_idx].playback_only = 1;
	priv->dai_links[*link_idx].id = J721E_DAI_LINK_CODECB;
	priv->dai_links[*link_idx].dai_fmt = J721E_DAI_FMT;
	priv->dai_links[*link_idx].init = j721e_audio_init_ivi;
	priv->dai_links[*link_idx].ops = &j721e_audio_ops;
	(*link_idx)++;

	/* Capture from codecb */
	priv->dai_links[*link_idx].name = "IVI pcm3168a-b ADC";
	priv->dai_links[*link_idx].stream_name = "cpb pcm3168a-b Capture";
	priv->dai_links[*link_idx].cpu_of_node = dai_node;
	priv->dai_links[*link_idx].platform_of_node = dai_node;
	priv->dai_links[*link_idx].codec_of_node = codecb_node;
	priv->dai_links[*link_idx].codec_dai_name = "pcm3168a-adc";
	priv->dai_links[*link_idx].capture_only = 1;
	priv->dai_links[*link_idx].id = J721E_DAI_LINK_CODECB;
	priv->dai_links[*link_idx].dai_fmt = J721E_DAI_FMT;
	priv->dai_links[*link_idx].init = j721e_audio_init;
	priv->dai_links[*link_idx].ops = &j721e_audio_ops;
	(*link_idx)++;

	priv->codec_conf[*conf_idx].of_node = codeca_node;
	priv->codec_conf[*conf_idx].name_prefix = "codeca";
	(*conf_idx)++;
	priv->codec_conf[*conf_idx].of_node = codecb_node;
	priv->codec_conf[*conf_idx].name_prefix = "codecb";
	(*conf_idx)++;

	return 0;
}

static int j721e_soc_probe_ivi_multicodec(struct j721e_priv *priv,
					  int *link_idx, int *conf_idx)
{
	struct device_node *node = priv->dev->of_node;
	struct device_node *dai_node, *codeca_node, *codecb_node;
	struct j721e_audio_domain *domain;
	int ret;

	if (priv->match_data->board_type != J721E_BOARD_CPB_IVI_MULTICODEC)
		return 0;

	dai_node = of_parse_phandle(node, "ti,ivi-mcasp", 0);
	if (!dai_node) {
		dev_err(priv->dev, "IVI McASP node is not provided\n");
		return -EINVAL;
	}

	codeca_node = of_parse_phandle(node, "ti,ivi-codec-a", 0);
	if (!codeca_node) {
		dev_err(priv->dev, "IVI codec-a node is not provided\n");
		return -EINVAL;
	}

	codecb_node = of_parse_phandle(node, "ti,ivi-codec-b", 0);
	if (!codecb_node) {
		dev_warn(priv->dev, "IVI codec-b node is not provided\n");
		return 0;
	}

	domain = &priv->audio_domains[J721E_AUDIO_DOMAIN_IVI];
	ret = j721e_get_clocks(priv->dev, &domain->codec,
			       "audio-refclk0");
	if (ret)
		return ret;

	ret = j721e_get_clocks(priv->dev, &domain->mcasp, "ivi-mcasp");
	if (ret)
		return ret;

	/* Playback to codeca + codecb */
	priv->dai_links[*link_idx].codecs = devm_kcalloc(priv->dev, 2,
				sizeof(*priv->dai_links[*link_idx].codecs),
				GFP_KERNEL);
	if (!priv->dai_links[*link_idx].codecs)
		return -ENOMEM;

	priv->dai_links[*link_idx].name = "IVI Multicodec Playback";
	priv->dai_links[*link_idx].stream_name = "ivi Playback";
	priv->dai_links[*link_idx].cpu_of_node = dai_node;
	priv->dai_links[*link_idx].platform_of_node = dai_node;
	priv->dai_links[*link_idx].playback_only = 1;
	priv->dai_links[*link_idx].id = J721E_DAI_LINK_IVI_MULTICODEC;
	priv->dai_links[*link_idx].dai_fmt = J721E_DAI_FMT;
	priv->dai_links[*link_idx].init = j721e_audio_init_ivi;
	priv->dai_links[*link_idx].ops = &j721e_audio_ops;
	priv->dai_links[*link_idx].codecs[0].of_node = codeca_node;
	priv->dai_links[*link_idx].codecs[0].dai_name = "pcm3168a-dac";
	priv->dai_links[*link_idx].codecs[1].of_node = codecb_node;
	priv->dai_links[*link_idx].codecs[1].dai_name = "pcm3168a-dac";
	priv->dai_links[*link_idx].num_codecs = 2;
	(*link_idx)++;

	/* Capture from codeca + codecb */
	priv->dai_links[*link_idx].codecs = devm_kcalloc(priv->dev, 2,
				sizeof(*priv->dai_links[*link_idx].codecs),
				GFP_KERNEL);
	if (!priv->dai_links[*link_idx].codecs)
		return -ENOMEM;

	priv->dai_links[*link_idx].name = "IVI Multicodec Capture";
	priv->dai_links[*link_idx].stream_name = "ivi Capture";
	priv->dai_links[*link_idx].cpu_of_node = dai_node;
	priv->dai_links[*link_idx].platform_of_node = dai_node;
	priv->dai_links[*link_idx].capture_only = 1;
	priv->dai_links[*link_idx].id = J721E_DAI_LINK_IVI_MULTICODEC;
	priv->dai_links[*link_idx].dai_fmt = J721E_DAI_FMT;
	priv->dai_links[*link_idx].init = j721e_audio_init;
	priv->dai_links[*link_idx].ops = &j721e_audio_ops;
	priv->dai_links[*link_idx].codecs[0].of_node = codeca_node;
	priv->dai_links[*link_idx].codecs[0].dai_name = "pcm3168a-adc";
	priv->dai_links[*link_idx].codecs[1].of_node = codecb_node;
	priv->dai_links[*link_idx].codecs[1].dai_name = "pcm3168a-adc";
	priv->dai_links[*link_idx].num_codecs = 2;
	(*link_idx)++;

	priv->codec_conf[*conf_idx].of_node = codeca_node;
	priv->codec_conf[*conf_idx].name_prefix = "codeca";
	(*conf_idx)++;
	priv->codec_conf[*conf_idx].of_node = codecb_node;
	priv->codec_conf[*conf_idx].name_prefix = "codecb";
	(*conf_idx)++;

	return 0;
}

static int j721e_soc_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct snd_soc_card *card;
	const struct of_device_id *match;
	struct j721e_priv *priv;
	int link_cnt, conf_cnt, ret;

	if (!node) {
		dev_err(&pdev->dev, "of node is missing.\n");
		return -ENODEV;
	}

	match = of_match_node(j721e_audio_of_match, node);
	if (!match) {
		dev_err(&pdev->dev, "No compatible match found\n");
		return -ENODEV;
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	priv->match_data = match->data;

	priv->dai_links = devm_kcalloc(&pdev->dev, priv->match_data->num_links,
				       sizeof(*priv->dai_links), GFP_KERNEL);
	if (priv->dai_links == NULL)
		return -ENOMEM;

	priv->audio_domains[J721E_AUDIO_DOMAIN_CPB].parent_clk_id = -1;
	priv->audio_domains[J721E_AUDIO_DOMAIN_IVI].parent_clk_id = -1;
	priv->dev = &pdev->dev;
	card = &priv->card;
	card->dev = &pdev->dev;
	card->owner = THIS_MODULE;
	card->dapm_widgets = j721e_cpb_dapm_widgets;
	card->num_dapm_widgets = ARRAY_SIZE(j721e_cpb_dapm_widgets);
	card->dapm_routes = j721e_cpb_dapm_routes;
	card->num_dapm_routes = ARRAY_SIZE(j721e_cpb_dapm_routes);
	card->fully_routed = 1;

	if (snd_soc_of_parse_card_name(card, "ti,model")) {
		dev_err(&pdev->dev, "Card name is not provided\n");
		return -ENODEV;
	}

	ret = of_property_read_u32(node, "pll4-rate",
				   &priv->pll_rates[J721E_CLK_PARENT_48000]);
	if (ret)
		return ret;

	ret = of_property_read_u32(node, "pll15-rate",
				   &priv->pll_rates[J721E_CLK_PARENT_44100]);
	if (ret)
		return ret;

	link_cnt = 0;
	conf_cnt = 0;
	ret = j721e_soc_probe_cpb(priv, &link_cnt, &conf_cnt);
	if (ret)
		return ret;

	ret = j721e_soc_probe_ivi(priv, &link_cnt, &conf_cnt);
	if (ret)
		return ret;

	ret = j721e_soc_probe_ivi_multicodec(priv, &link_cnt, &conf_cnt);
	if (ret)
		return ret;

	card->dai_link = priv->dai_links;
	card->num_links = link_cnt;

	card->codec_conf = priv->codec_conf;
	card->num_configs = conf_cnt;

	j721e_calculate_rate_range(priv);

	snd_soc_card_set_drvdata(card, priv);

	mutex_init(&priv->mutex);

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret)
		dev_err(&pdev->dev, "devm_snd_soc_register_card() failed: %d\n",
			ret);

	return ret;
}

static struct platform_driver j721e_soc_driver = {
	.driver = {
		.name = "j721e-audio",
		.pm = &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(j721e_audio_of_match),
	},
	.probe = j721e_soc_probe,
};

module_platform_driver(j721e_soc_driver);

MODULE_AUTHOR("Peter Ujfalusi <peter.ujfalusi@ti.com>");
MODULE_DESCRIPTION("ASoC machine driver for j721e Common Processor Board");
MODULE_LICENSE("GPL v2");
