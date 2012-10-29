/*
 * soc-fw.c  --  ALSA SoC Firmware
 *
 * Copyright (C) 2012 Texas Instruments Inc.
 *
 * Author: Liam Girdwood <lrg@ti.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Support for audio fimrware to contain kcontrols, DAPM graphs, widgets,
 *  DAIs, equalizers, firmware, coefficienst etc.
 *
 *  This file only manages the DAPM and Kcontrol components, all other firmware
 *  data is passed to component drivers for bespoke handling.
 */

#define DEBUG
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/list.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/soc-fw.h>

static DEFINE_MUTEX(client_mutex);
static LIST_HEAD(waiting_list);
static LIST_HEAD(ready_list);

/*
 * We make several passes over the data (since it wont necessarily be ordered)
 * and process objects in the following order. This guarantees the component
 * drivers will be ready with any vendor data before the mixers and DAPM objects
 * are loaded (that may make use of the vemndor data).
 */
#define SOC_FW_PASS_VENDOR	0
#define SOC_FW_PASS_MIXER	1
#define SOC_FW_PASS_COEFF	SOC_FW_PASS_MIXER
#define SOC_FW_PASS_WIDGET	2
#define SOC_FW_PASS_GRAPH	3
#define SOC_FW_PASS_PINS	4

#define SOC_FW_PASS_START	SOC_FW_PASS_VENDOR
#define SOC_FW_PASS_END	SOC_FW_PASS_PINS

struct soc_fw {
	const char *file;
	const struct firmware *fw;
	const u8 *pos;	/* read postion */
	const u8 *hdr_pos;	/* header position */
	unsigned int pass;

	struct device *dev;
	struct snd_soc_codec *codec;
	struct snd_soc_platform *platform;
	struct snd_soc_card *card;

	const struct snd_soc_fw_kcontrol_ops *io_ops;
	int io_ops_count;

	union {
		struct snd_soc_fw_codec_ops *codec_ops;
		struct snd_soc_fw_platform_ops *platform_ops;
		struct snd_soc_fw_card_ops *card_ops;
	};

	struct list_head list; /* client list */
};

static int soc_fw_load_headers(struct soc_fw *sfw);
static void soc_fw_complete(struct soc_fw *sfw);

static const struct snd_soc_fw_kcontrol_ops io_ops[] = {
	{SOC_CONTROL_IO_VOLSW, snd_soc_get_volsw,
		snd_soc_put_volsw, snd_soc_info_volsw},
	{SOC_CONTROL_IO_VOLSW_SX, snd_soc_get_volsw_sx,
		snd_soc_put_volsw_sx, NULL},
	{SOC_CONTROL_IO_VOLSW_S8, snd_soc_get_volsw_s8,
		snd_soc_put_volsw_s8, snd_soc_info_volsw_s8},
	{SOC_CONTROL_IO_ENUM, snd_soc_get_enum_double,
		snd_soc_put_enum_double, snd_soc_info_enum_double},
	{SOC_CONTROL_IO_ENUM_EXT, NULL,
		NULL, snd_soc_info_enum_ext},
	{SOC_CONTROL_IO_BYTES, snd_soc_bytes_get,
		snd_soc_bytes_put, snd_soc_bytes_info},
	{SOC_CONTROL_IO_BOOL_EXT, NULL,
		NULL, snd_ctl_boolean_mono_info},
	{SOC_CONTROL_IO_ENUM_VALUE, snd_soc_get_value_enum_double,
		snd_soc_put_value_enum_double, NULL},
	{SOC_CONTROL_IO_RANGE, snd_soc_get_volsw_range,
		snd_soc_put_volsw_range, snd_soc_info_volsw_range},
	{SOC_CONTROL_IO_VOLSW_XR_SX, snd_soc_get_xr_sx,
		snd_soc_put_xr_sx, snd_soc_info_xr_sx},
	{SOC_CONTROL_IO_STROBE, snd_soc_get_strobe,
		snd_soc_put_strobe, NULL},

	{SOC_DAPM_IO_VOLSW, snd_soc_dapm_get_volsw,
		snd_soc_dapm_put_volsw, NULL},
	{SOC_DAPM_IO_ENUM_DOUBLE, snd_soc_dapm_get_enum_double,
		snd_soc_dapm_put_enum_double, snd_soc_info_enum_double},
	{SOC_DAPM_IO_ENUM_VIRT, snd_soc_dapm_get_enum_virt,
		snd_soc_dapm_put_enum_virt, NULL},
	{SOC_DAPM_IO_ENUM_VALUE, snd_soc_dapm_get_value_enum_double,
		snd_soc_dapm_put_value_enum_double, NULL},
	{SOC_DAPM_IO_PIN, snd_soc_dapm_get_pin_switch,
		snd_soc_dapm_put_pin_switch, snd_soc_dapm_info_pin_switch},
};

static void sfw_dump(struct soc_fw *sfw)
{
	struct snd_soc_fw_hdr *hdr = (struct snd_soc_fw_hdr*)sfw->hdr_pos;
	int i;

	dev_dbg(sfw->dev, "header: magic %x type %d vendor %d version %d size 0x%x\n",
		hdr->magic, hdr->type, hdr->vendor_type, hdr->version, hdr->size);

	for (i = 0; i < 24; i++)
		dev_dbg(sfw->dev, "data: offset 0x%x = 0x%x\n",
		(char*) sfw->hdr_pos - (char*) sfw->fw->data + i,
		*(((char*)sfw->hdr_pos) + i));
}

static inline void soc_fw_list_add_enum(struct soc_fw *sfw, struct soc_enum *se)
{
	if (sfw->codec)
		list_add(&se->list, &sfw->codec->denums);
	else if (sfw->platform)
		list_add(&se->list, &sfw->platform->denums);
	else if (sfw->card)
		list_add(&se->list, &sfw->card->denums);
}

static inline void soc_fw_list_add_mixer(struct soc_fw *sfw,
	struct soc_mixer_control *mc)
{
	if (sfw->codec)
		list_add(&mc->list, &sfw->codec->dmixers);
	else if (sfw->platform)
		list_add(&mc->list, &sfw->platform->dmixers);
	else if (sfw->card)
		list_add(&mc->list, &sfw->card->dmixers);
}

static inline struct snd_soc_dapm_context *soc_fw_dapm_get(struct soc_fw *sfw)
{
	if (sfw->codec)
		return &sfw->codec->dapm;
	else if (sfw->platform)
		return &sfw->platform->dapm;
	else if (sfw->card)
		return &sfw->card->dapm;
	BUG();
}

static inline struct snd_soc_card *soc_fw_card_get(struct soc_fw *sfw)
{
	if (sfw->codec)
		return sfw->codec->card;
	else if (sfw->platform)
		return sfw->platform->card;
	else if (sfw->card)
		return sfw->card;
	BUG();
}

static int soc_fw_request_data(struct soc_fw *sfw)
{
	int ret;

	ret = request_firmware(&sfw->fw, sfw->file, sfw->dev);
	if (ret != 0)
		dev_err(sfw->dev, "Failed to load : %s %d\n", sfw->file, ret);

	return ret;
}

static inline void soc_fw_release_data(struct soc_fw *sfw)
{
	release_firmware(sfw->fw);
}

static void soc_fw_load(const struct firmware *fw, void *context)
{
	struct soc_fw *sfw = (struct soc_fw *)context;

	sfw->fw = fw;
	mutex_lock(&client_mutex);
	list_move(&sfw->list, &ready_list);
	mutex_unlock(&client_mutex);

	driver_deferred_probe_trigger();
}

static int soc_fw_request_data_nowait(struct soc_fw *sfw)
{
	int ret;

	ret = request_firmware_nowait(THIS_MODULE, 1, sfw->file, sfw->dev,
		GFP_KERNEL, sfw, soc_fw_load);
	if (ret != 0) {
		dev_err(sfw->dev, "Failed to load : %s %d\n", sfw->file, ret);
		return ret;
	}

	return -EPROBE_DEFER;
}

/* check we dont overflow the data for this chunk */
static inline int soc_fw_check_count(struct soc_fw *sfw, size_t elem_size,
	unsigned int count, size_t bytes)
{
	const u8 *end = sfw->pos + elem_size * count;

	if (end > sfw->fw->data + sfw->fw->size) {
		dev_err(sfw->dev, "controls overflow end of data\n");
		return -EINVAL;
	}

	/* check there is enough room in chunk for control.
	   extra bytes at the end of control are for vendor data here  */
	if (elem_size * count > bytes) {
		dev_err(sfw->dev, "controls count %d of elem size %d "
			"are bigger than chunk %d\n", count, elem_size, bytes);
		return -EINVAL;
	}

	return 0;
}

static inline int soc_fw_eof(struct soc_fw *sfw)
{
	const u8 *end = sfw->hdr_pos;

	if (end >= sfw->fw->data + sfw->fw->size)
		return 1;
	return 0;
}

static inline unsigned int soc_fw_get_hdr_offset(struct soc_fw *sfw)
{
	return (unsigned int)(sfw->hdr_pos - sfw->fw->data);
}

static inline unsigned int soc_fw_get_offset(struct soc_fw *sfw)
{
	return (unsigned int)(sfw->pos - sfw->fw->data);
}

/* pass vendor data to component driver for processing */
static int soc_fw_vendor_load_(struct soc_fw *sfw, struct snd_soc_fw_hdr *hdr)
{
	int ret = 0;

	if (sfw->codec && sfw->codec_ops && sfw->codec_ops->vendor_load)
		ret = sfw->codec_ops->vendor_load(sfw->codec, hdr);

	if (sfw->platform && sfw->platform_ops && sfw->platform_ops->vendor_load)
		ret = sfw->platform_ops->vendor_load(sfw->platform, hdr);

	if (sfw->card && sfw->card_ops && sfw->card_ops->vendor_load)
		ret = sfw->card_ops->vendor_load(sfw->card, hdr);

	if (ret < 0)
		dev_err(sfw->dev, "vendor load failed at hdr offset %d/0x%x for type %d:%d\n",
			soc_fw_get_hdr_offset(sfw), soc_fw_get_hdr_offset(sfw),
			hdr->type, hdr->vendor_type);
	return ret;
}

/* pass vendor data to component driver for processing */
static int soc_fw_vendor_load(struct soc_fw *sfw, struct snd_soc_fw_hdr *hdr)
{
	if (sfw->pass != SOC_FW_PASS_VENDOR)
		return 0;

	return soc_fw_vendor_load_(sfw, hdr);
}

static int soc_fw_vendor_unload(struct soc_fw *sfw, struct snd_soc_fw_hdr *hdr)
{
	int ret = 0;

	if (sfw->codec && sfw->codec_ops && sfw->codec_ops->vendor_unload)
		ret = sfw->codec_ops->vendor_unload(sfw->codec, hdr);

	if (sfw->platform && sfw->platform_ops && sfw->platform_ops->vendor_unload)
		ret = sfw->platform_ops->vendor_unload(sfw->platform, hdr);

	if (sfw->card && sfw->card_ops && sfw->card_ops->vendor_unload)
		ret = sfw->card_ops->vendor_unload(sfw->card, hdr);

	if (ret < 0)
		dev_err(sfw->dev, "vendor unload failed at offset %d/0x%x for type %d:%d\n",
			soc_fw_get_hdr_offset(sfw), soc_fw_get_hdr_offset(sfw),
			hdr->type, hdr->vendor_type);
	return ret;
}

/* pass new dynamic widget to component driver. mainly for external widgets */
static int soc_fw_widget_load(struct soc_fw *sfw, struct snd_soc_dapm_widget *w)
{
	if (sfw->codec && sfw->codec_ops && sfw->codec_ops->widget_load)
		return sfw->codec_ops->widget_load(sfw->codec, w);

	if (sfw->platform && sfw->platform_ops && sfw->platform_ops->widget_load)
		return sfw->platform_ops->widget_load(sfw->platform, w);

	if (sfw->card && sfw->card_ops && sfw->card_ops->widget_load)
		return sfw->card_ops->widget_load(sfw->card, w);

	dev_info(sfw->dev, "no handler specified for ext widget %s\n", w->name);
	return 0;
}

static void soc_fw_complete(struct soc_fw *sfw)
{
	if (sfw->codec && sfw->codec_ops && sfw->codec_ops->complete)
		sfw->codec_ops->complete(sfw->codec);
	if (sfw->platform && sfw->platform_ops && sfw->platform_ops->complete)
		sfw->platform_ops->complete(sfw->platform);
	if (sfw->card && sfw->card_ops && sfw->card_ops->complete)
		sfw->card_ops->complete(sfw->card);
}

/* add a dynamic kcontrol */
static int soc_fw_add_dcontrol(struct snd_card *card, struct device *dev,
	const struct snd_kcontrol_new *control_new, const char *prefix,
	void *data, struct snd_kcontrol **kcontrol)
{
	int err;

	*kcontrol = snd_soc_cnew(control_new, data, control_new->name, prefix);
	if (*kcontrol == NULL) {
		dev_err(dev, "Failed to create new kcontrol %s\n",
		control_new->name);
		return -ENOMEM;
	}

	err = snd_ctl_add(card, *kcontrol);
	if (err < 0) {
		kfree(*kcontrol);
		dev_err(dev, "Failed to add %s: %d\n", control_new->name, err);
		return err;
	}

	return 0;
}

/* add a dynamic kcontrol for component driver */
static int soc_fw_add_kcontrol(struct soc_fw *sfw, struct snd_kcontrol_new *k,
	struct snd_kcontrol **kcontrol)
{
	if (sfw->codec) {
		struct snd_soc_codec *codec = sfw->codec;

		return soc_fw_add_dcontrol(codec->card->snd_card, codec->dev,
				k, codec->name_prefix, codec, kcontrol);
	} else if (sfw->platform) {
		struct snd_soc_platform *platform = sfw->platform;

		return soc_fw_add_dcontrol(platform->card->snd_card,
				platform->dev, k, NULL, platform, kcontrol);
	} else if (sfw->card) {
		struct snd_soc_card *card = sfw->card;

		return soc_fw_add_dcontrol(card->snd_card, card->dev,
				k, NULL, card, kcontrol);
	} else
		dev_info(sfw->dev, "no handler specified for kcontrol %s\n",
			k->name);
	return 0;
}

static int soc_fw_kcontrol_bind_io(u32 io_type, struct snd_kcontrol_new *k,
	const struct snd_soc_fw_kcontrol_ops *ops, int num_ops)
{
	int i;

	for (i = 0; i < num_ops; i++) {

		if (SOC_CONTROL_GET_ID_PUT(ops[i].id) ==
			SOC_CONTROL_GET_ID_PUT(io_type) && ops[i].put)
			k->put = ops[i].put;
		if (SOC_CONTROL_GET_ID_GET(ops[i].id) ==
			SOC_CONTROL_GET_ID_GET(io_type) && ops[i].get)
			k->get = ops[i].get;
		if (SOC_CONTROL_GET_ID_INFO(ops[i].id) ==
			SOC_CONTROL_GET_ID_INFO(io_type) && ops[i].info)
			k->info = ops[i].info;
	}

	/* do we need to bind external kcontrols */
	if (!k->put || !k->get || !k->info)
		return 1;

	return 0;
}

/* pass new dynamic kcontrol to component driver. mainly for external kcontrols */
static int soc_fw_init_kcontrol(struct soc_fw *sfw, struct snd_kcontrol_new *k)
{
	if (sfw->codec && sfw->codec_ops && sfw->codec_ops->control_load)
		return sfw->codec_ops->control_load(sfw->codec, k);

	if (sfw->platform && sfw->platform_ops && sfw->platform_ops->control_load)
		return sfw->platform_ops->control_load(sfw->platform, k);

	if (sfw->card && sfw->card_ops && sfw->card_ops->control_load)
		return sfw->card_ops->control_load(sfw->card, k);

	dev_info(sfw->dev, "no handler specified for kcontrol %s\n", k->name);
	return 0;
}

static void soc_fw_dmixer_codec_remove(struct soc_fw *sfw, const char *name)
{
	struct snd_soc_codec *codec = sfw->codec;
	struct soc_mixer_control *sm, *next_sm;
	struct snd_card *card = codec->card->snd_card;

	list_for_each_entry_safe(sm, next_sm, &codec->dmixers, list) {

		/* if name is not NULL then remove matching kcontrols */
		if (name && strcmp(name, sm->dcontrol->id.name))
			continue;

		snd_ctl_remove(card, sm->dcontrol);
		list_del(&sm->list);
		kfree(sm);
	}
}

static void soc_fw_dmixer_platform_remove(struct soc_fw *sfw, const char *name)
{
	struct snd_soc_platform *platform = sfw->platform;
	struct soc_mixer_control *sm, *next_sm;
	struct snd_card *card = platform->card->snd_card;

	list_for_each_entry_safe(sm, next_sm, &platform->dmixers, list) {

		/* if name is not NULL then remove matching kcontrols */
		if (name && strcmp(name, sm->dcontrol->id.name))
			continue;

		snd_ctl_remove(card, sm->dcontrol);
		list_del(&sm->list);
		kfree(sm);
	}
}

static void soc_fw_dmixer_card_remove(struct soc_fw *sfw, const char *name)
{
	struct snd_soc_card *soc_card = sfw->card;
	struct soc_mixer_control *sm, *next_sm;
	struct snd_card *card = soc_card->snd_card;

	list_for_each_entry_safe(sm, next_sm, &soc_card->dmixers, list) {

		/* if name is not NULL then remove matching kcontrols */
		if (name && strcmp(name, sm->dcontrol->id.name))
			continue;

		snd_ctl_remove(card, sm->dcontrol);
		list_del(&sm->list);
		kfree(sm);
	}
}

static void soc_fw_dmixer_component_remove(struct soc_fw *sfw, const char *name)
{
	if (sfw->codec)
		soc_fw_dmixer_codec_remove(sfw, name);
	else if (sfw->platform)
		soc_fw_dmixer_platform_remove(sfw, name);
	else if (sfw->card)
		soc_fw_dmixer_card_remove(sfw, name);
}

static int soc_fw_dmixer_remove(struct soc_fw *sfw, unsigned int count,
	size_t size)
{
	struct snd_soc_fw_mixer_control *mc;
	int i;

	if (soc_fw_check_count(sfw,
		sizeof(struct snd_soc_fw_mixer_control), count, size)) {
		dev_err(sfw->dev, "invalid count %d for mixer controls\n", count);
		return -EINVAL;
	}

	for (i = 0; i < count; i++) {
		mc = (struct snd_soc_fw_mixer_control*)sfw->pos;
		sfw->pos += sizeof(struct snd_soc_fw_enum_control);

		soc_fw_dmixer_component_remove(sfw, mc->hdr.name);
	}
	return 0;
}

static int soc_fw_create_tlv(struct soc_fw *sfw, struct snd_kcontrol_new *kc,
	u32 tlv_size)
{
	struct snd_soc_fw_ctl_tlv *fw_tlv;
	struct snd_ctl_tlv *tlv;

	if (tlv_size == 0)
		return 0;

	fw_tlv = (struct snd_soc_fw_ctl_tlv *) sfw->pos;
	sfw->pos += tlv_size;

	tlv = kzalloc(sizeof(*tlv) + tlv_size, GFP_KERNEL);
	if (tlv == NULL)
		return -ENOMEM;

	dev_dbg(sfw->dev, " created TLV type %d size %d bytes\n",
		fw_tlv->numid, fw_tlv->length);
	tlv->numid = fw_tlv->numid;
	tlv->length = fw_tlv->length;
	memcpy(tlv->tlv, fw_tlv + 1, fw_tlv->length);
	kc->tlv.p = (void*)tlv;

	return 0;
}

static inline void soc_fw_free_tlv(struct soc_fw *sfw,
	struct snd_kcontrol_new *kc)
{
	kfree (kc->tlv.p);
}


static int soc_fw_dmixer_create(struct soc_fw *sfw, unsigned int count,
	size_t size)
{
	struct snd_soc_fw_mixer_control *mc;
	struct soc_mixer_control *sm;
	struct snd_kcontrol_new kc;
	int i, err, ext;

	if (soc_fw_check_count(sfw,
		sizeof(struct snd_soc_fw_mixer_control), count, size)) {
		dev_err(sfw->dev, "invalid count %d for controls\n", count);
		return -EINVAL;
	}

	for (i = 0; i < count; i++) {
		mc = (struct snd_soc_fw_mixer_control*)sfw->pos;
		sfw->pos += sizeof(struct snd_soc_fw_mixer_control);

		/* validate kcontrol */
		if (strnlen(mc->hdr.name, SND_SOC_FW_TEXT_SIZE) ==
			SND_SOC_FW_TEXT_SIZE)
			return -EINVAL;

		sm = kzalloc(sizeof(*sm), GFP_KERNEL);
		if (!sm)
			return -ENOMEM;

		dev_dbg(sfw->dev, "adding mixer kcontrol %s with access 0x%x\n",
			mc->hdr.name, mc->hdr.access);

		memset(&kc, 0, sizeof(kc));
		kc.name = mc->hdr.name;
		kc.private_value = (long)sm;
		kc.iface = SNDRV_CTL_ELEM_IFACE_MIXER;
		kc.access = mc->hdr.access;

		sm->reg = mc->reg;
		sm->rreg = mc->rreg;
		sm->shift = mc->shift;
		sm->rshift = mc->rshift;
		sm->max = mc->max;
		sm->min = mc->min;
		sm->invert = mc->invert;
		sm->platform_max = mc->platform_max;
		INIT_LIST_HEAD(&sm->list);

		/* map standard io handlers and check for external handlers */
		ext = soc_fw_kcontrol_bind_io(mc->hdr.index, &kc, io_ops,
			ARRAY_SIZE(io_ops));
		if (ext) {
			/* none exist, so now try and map ext handlers */
			ext = soc_fw_kcontrol_bind_io(mc->hdr.index, &kc,
				sfw->io_ops, sfw->io_ops_count);
			if (ext) {
				dev_err(sfw->dev, "no complete mixer IO handler"
					" for %s type (g,p,i) %d:%d:%d\n",
					mc->hdr.name,
					SOC_CONTROL_GET_ID_GET(mc->hdr.index),
					SOC_CONTROL_GET_ID_PUT(mc->hdr.index),
					SOC_CONTROL_GET_ID_INFO(mc->hdr.index));
				kfree(sm);
				continue;
			}

			err = soc_fw_init_kcontrol(sfw, &kc);
			if (err < 0) {
				dev_err(sfw->dev, "failed to init %s\n",
					mc->hdr.name);
				kfree(sm);
				continue;
			}
		}

		/* create any TLV data */
		soc_fw_create_tlv(sfw, &kc, mc->hdr.tlv_size);

		/* register control here */
		err = soc_fw_add_kcontrol(sfw, &kc, &sm->dcontrol);
		if (err < 0) {
			dev_err(sfw->dev, "failed to add %s\n", mc->hdr.name);
			kfree(sm);
			continue;
		}

		//soc_fw_free_tlv(sfw, &kc);
		soc_fw_list_add_mixer(sfw, sm);
	}

	return 0;
}

static inline void soc_fw_denum_free_data(struct soc_enum *se)
{
	int i;

	if (se->dvalues)
		kfree(se->dvalues);
	else {
		for (i = 0; i < se->max - 1; i++)
			kfree(se->dtexts[i]);
	}
}

static void soc_fw_denum_codec_remove(struct soc_fw *sfw, const char *name)
{
	struct snd_soc_codec *codec = sfw->codec;
	struct soc_enum *se, *next_se;
	struct snd_card *card = codec->card->snd_card;

	list_for_each_entry_safe(se, next_se, &codec->denums, list) {

		/* if name is not NULL then remove matching kcontrols */
		if (name && strcmp(name, se->dcontrol->id.name))
			continue;

		snd_ctl_remove(card, se->dcontrol);
		list_del(&se->list);
		soc_fw_denum_free_data(se);
		kfree(se);
	}
}

static void soc_fw_denum_platform_remove(struct soc_fw *sfw, const char *name)
{
	struct snd_soc_platform *platform = sfw->platform;
	struct soc_enum *se, *next_se;
	struct snd_card *card = platform->card->snd_card;

	list_for_each_entry_safe(se, next_se, &platform->denums, list) {

		/* if name is not NULL then remove matching kcontrols */
		if (name && strcmp(name, se->dcontrol->id.name))
			continue;

		snd_ctl_remove(card, se->dcontrol);
		list_del(&se->list);
		soc_fw_denum_free_data(se);
		kfree(se);
	}
}

static void soc_fw_denum_card_remove(struct soc_fw *sfw, const char *name)
{
	struct snd_soc_card *soc_card = sfw->card;
	struct soc_enum *se, *next_se;
	struct snd_card *card = soc_card->snd_card;

	list_for_each_entry_safe(se, next_se, &soc_card->denums, list) {

		/* if name is not NULL then remove matching kcontrols */
		if (name && strcmp(name, se->dcontrol->id.name))
			continue;

		snd_ctl_remove(card, se->dcontrol);
		list_del(&se->list);
		soc_fw_denum_free_data(se);
		kfree(se);
	}
}

static void soc_fw_denum_component_remove(struct soc_fw *sfw, const char *name)
{
	if (sfw->codec)
		soc_fw_denum_codec_remove(sfw, name);
	else if (sfw->platform)
		soc_fw_denum_platform_remove(sfw, name);
	else if (sfw->card)
		soc_fw_denum_card_remove(sfw, name);
}

static int soc_fw_denum_remove(struct soc_fw *sfw, unsigned int count,
	size_t size)
{
	struct snd_soc_fw_enum_control *ec;
	int i;

	if (soc_fw_check_count(sfw,
		sizeof(struct snd_soc_fw_enum_control), count, size)) {
		dev_err(sfw->dev, "invalid count %d for enum controls\n", count);
		return -EINVAL;
	}

	for (i = 0; i < count; i++) {
		ec = (struct snd_soc_fw_enum_control*)sfw->pos;
		sfw->pos += sizeof(struct snd_soc_fw_enum_control);

		soc_fw_denum_component_remove(sfw, ec->hdr.name);
	}

	return 0;
}

static int soc_fw_denum_create_texts(struct soc_enum *se,
	struct snd_soc_fw_enum_control *ec)
{
	int i, ret;

	se->dtexts = kzalloc(sizeof(char *) * ec->max, GFP_KERNEL);
	if (se->dtexts == NULL)
		return -ENOMEM;

	for (i = 0; i < ec->max; i++) {

		if (strnlen(ec->texts[i], SND_SOC_FW_TEXT_SIZE) ==
			SND_SOC_FW_TEXT_SIZE) {
			ret = -EINVAL;
			goto err;
		}

		se->dtexts[i] = kstrdup(ec->texts[i], GFP_KERNEL);
		if (!se->dtexts[i]) {
			ret = -ENOMEM;
			goto err;
		}
	}
	return 0;

err:
	for (--i; i >= 0; i--)
		kfree(se->dtexts[i]);
	kfree(se->dtexts);
	return ret;
}

static int soc_fw_denum_create_values(struct soc_enum *se,
	struct snd_soc_fw_enum_control *ec)
{
	if (ec->max > sizeof(*ec->values))
		return -EINVAL;

	se->dvalues = kmalloc(ec->max * sizeof(u32), GFP_KERNEL);
	if (!se->dvalues)
		return -ENOMEM;

	memcpy(se->dvalues, ec->values, ec->max * sizeof(u32));
	return 0;
}

static int soc_fw_denum_create(struct soc_fw *sfw, unsigned int count,
	size_t size)
{
	struct snd_soc_fw_enum_control *ec;
	struct soc_enum *se;
	struct snd_kcontrol_new kc;
	int i, ret, err, ext;

	if (soc_fw_check_count(sfw,
		sizeof(struct snd_soc_fw_enum_control), count, size)) {
		dev_err(sfw->dev, "invalid count %d for enum controls\n", count);
		return -EINVAL;
	}

	for (i = 0; i < count; i++) {
		ec = (struct snd_soc_fw_enum_control*)sfw->pos;
		sfw->pos += sizeof(struct snd_soc_fw_enum_control);

		/* validate kcontrol */
		if (strnlen(ec->hdr.name, SND_SOC_FW_TEXT_SIZE) ==
			SND_SOC_FW_TEXT_SIZE)
			return -EINVAL;

		se = kzalloc(sizeof(*se), GFP_KERNEL);
		if (!se)
			return -ENOMEM;

		dev_dbg(sfw->dev, "adding enum kcontrol %s size %d\n",
			ec->hdr.name, ec->max);

		memset(&kc, 0, sizeof(kc));
		kc.name = ec->hdr.name;
		kc.private_value = (long)se;
		kc.iface = SNDRV_CTL_ELEM_IFACE_MIXER;
		kc.access = ec->hdr.access;

		se->reg = ec->reg;
		se->reg2 = ec->reg2;
		se->shift_l = ec->shift_l;
		se->shift_r = ec->shift_r;
		se->max = ec->max;
		se->mask = ec->mask;
		INIT_LIST_HEAD(&se->list);

		switch (SOC_CONTROL_GET_ID_INFO(ec->hdr.index)) {
		case SOC_CONTROL_TYPE_ENUM:
		case SOC_CONTROL_TYPE_ENUM_EXT:
		case SOC_DAPM_TYPE_ENUM_EXT:
		case SOC_DAPM_TYPE_ENUM_DOUBLE:
		case SOC_DAPM_TYPE_ENUM_VIRT:
			err = soc_fw_denum_create_texts(se, ec);
			if (err < 0) {
				dev_err(sfw->dev, "could not create"
					" texts for %s\n", ec->hdr.name);
				kfree(se);
				continue;
			}
			break;
		case SOC_DAPM_TYPE_ENUM_VALUE:
		case SOC_CONTROL_TYPE_ENUM_VALUE:
			err = soc_fw_denum_create_values(se, ec);
			if (err < 0) {
				dev_err(sfw->dev, "could not create"
					" values for %s\n", ec->hdr.name);
				kfree(se);
				continue;
			}
			break;
		default:
			dev_err(sfw->dev, "invalid enum control type %d for %s\n",
				ec->hdr.index, ec->hdr.name);
			kfree(se);
			continue;
		}

		/* map standard io handlers and check for external handlers */
		ext = soc_fw_kcontrol_bind_io(ec->hdr.index, &kc, io_ops,
			ARRAY_SIZE(io_ops));
		if (ext) {
			/* none exist, so now try and map ext handlers */
			ext = soc_fw_kcontrol_bind_io(ec->hdr.index, &kc,
				sfw->io_ops, sfw->io_ops_count);
			if (ext) {
				dev_err(sfw->dev, "no complete enum IO handler"
					" for %s type (g,p,i) %d:%d:%d\n",
					ec->hdr.name,
					SOC_CONTROL_GET_ID_GET(ec->hdr.index),
					SOC_CONTROL_GET_ID_PUT(ec->hdr.index),
					SOC_CONTROL_GET_ID_INFO(ec->hdr.index));
				kfree(se);
				continue;
			}

			err = soc_fw_init_kcontrol(sfw, &kc);
			if (err < 0) {
				dev_err(sfw->dev, "failed to init %s\n",
					ec->hdr.name);
				kfree(se);
				continue;
			}
		}

		/* register control here */
		ret = soc_fw_add_kcontrol(sfw, &kc, &se->dcontrol);
		if (ret < 0) {
			dev_err(sfw->dev, "could not add kcontrol %s\n",
				ec->hdr.name);
			kfree(se);
			continue;
		}
		soc_fw_list_add_enum(sfw, se);
	}

	return 0;
#if 0
err:
	/* free texts */
	if (se->dvalues)
		kfree(se->dvalues);
	else {
		for (i = 0; i < ec->max; i++)
			kfree(se->dtexts[i]);
	}
	kfree(se);

	/* remove other enum controls */
	sfw->pos -= sizeof(struct snd_soc_fw_enum_control) * (i + 1);
	soc_fw_denum_remove(sfw, count, size);
	return ret;
#endif
}

static int soc_fw_kcontrol_load(struct soc_fw *sfw, struct snd_soc_fw_hdr *hdr)
{
	struct snd_soc_fw_kcontrol *sfwk =
		(struct snd_soc_fw_kcontrol*)sfw->pos;
	struct snd_soc_fw_control_hdr *control_hdr;
	int i;

	if (sfw->pass != SOC_FW_PASS_MIXER) {
		sfw->pos += sizeof(struct snd_soc_fw_kcontrol) + hdr->size;
		return 0;
	}

	sfw->pos += sizeof(struct snd_soc_fw_kcontrol);
	control_hdr = (struct snd_soc_fw_control_hdr*)sfw->pos;

	dev_dbg(sfw->dev, "adding %d kcontrols\n", sfwk->count);

	for (i = 0; i < sfwk->count; i++) {
		switch (SOC_CONTROL_GET_ID_INFO(control_hdr->index)) {
		case SOC_CONTROL_TYPE_VOLSW:
		case SOC_CONTROL_TYPE_STROBE:
		case SOC_CONTROL_TYPE_VOLSW_SX:
		case SOC_CONTROL_TYPE_VOLSW_S8:
		case SOC_CONTROL_TYPE_VOLSW_XR_SX:
		case SOC_CONTROL_TYPE_BYTES:
		case SOC_CONTROL_TYPE_BOOL_EXT:
		case SOC_CONTROL_TYPE_RANGE:
		case SOC_DAPM_TYPE_VOLSW:
		case SOC_DAPM_TYPE_PIN:
			soc_fw_dmixer_create(sfw, 1, hdr->size);
			break;
		case SOC_CONTROL_TYPE_ENUM:
		case SOC_CONTROL_TYPE_ENUM_EXT:
		case SOC_CONTROL_TYPE_ENUM_VALUE:
		case SOC_DAPM_TYPE_ENUM_DOUBLE:
		case SOC_DAPM_TYPE_ENUM_VIRT:
		case SOC_DAPM_TYPE_ENUM_VALUE:
		case SOC_DAPM_TYPE_ENUM_EXT:
			soc_fw_denum_create(sfw, 1, hdr->size);
			break;
		default:
			dev_err(sfw->dev, "invalid control type %d:%d:%d count %d\n",
				SOC_CONTROL_GET_ID_GET(control_hdr->index),
				SOC_CONTROL_GET_ID_PUT(control_hdr->index),
				SOC_CONTROL_GET_ID_INFO(control_hdr->index),
				sfwk->count);
		}
	}
	return 0;
}

static int soc_fw_kcontrol_unload(struct soc_fw *sfw, struct snd_soc_fw_hdr *hdr)
{
	struct snd_soc_fw_kcontrol *sfwk =
		(struct snd_soc_fw_kcontrol*)sfw->pos;
	struct snd_soc_fw_control_hdr *control_hdr;
	int i;

	if (sfw->pass != SOC_FW_PASS_MIXER) {
		sfw->pos += sizeof(struct snd_soc_fw_kcontrol) + hdr->size;
		return 0;
	}

	sfw->pos += sizeof(struct snd_soc_fw_kcontrol);
	control_hdr = (struct snd_soc_fw_control_hdr*)sfw->pos;

	for (i = 0; i < sfwk->count; i++) {
		switch (SOC_CONTROL_GET_ID_INFO(control_hdr->index)) {
		case SOC_CONTROL_TYPE_VOLSW:
		case SOC_CONTROL_TYPE_STROBE:
		case SOC_CONTROL_TYPE_VOLSW_SX:
		case SOC_CONTROL_TYPE_VOLSW_S8:
		case SOC_CONTROL_TYPE_VOLSW_XR_SX:
		case SOC_CONTROL_TYPE_BYTES:
		case SOC_CONTROL_TYPE_BOOL_EXT:
		case SOC_CONTROL_TYPE_RANGE:
			return soc_fw_dmixer_remove(sfw, 1, hdr->size);
		case SOC_CONTROL_TYPE_ENUM:
		case SOC_CONTROL_TYPE_ENUM_EXT:
		case SOC_CONTROL_TYPE_ENUM_VALUE:
			return soc_fw_denum_remove(sfw, 1, hdr->size);
		default:
			dev_err(sfw->dev, "invalid control type %d:%d:%d count %d\n",
				SOC_CONTROL_GET_ID_GET(control_hdr->index),
				SOC_CONTROL_GET_ID_PUT(control_hdr->index),
				SOC_CONTROL_GET_ID_INFO(control_hdr->index),
				sfwk->count);
			return -EINVAL;
		}
	}
	return 0;
}

static int soc_fw_dapm_graph_load(struct soc_fw *sfw,
	struct snd_soc_fw_hdr *hdr)
{
	struct snd_soc_dapm_context *dapm = soc_fw_dapm_get(sfw);
	struct snd_soc_dapm_route route;
	struct snd_soc_fw_dapm_elems *elem_info =
		(struct snd_soc_fw_dapm_elems*)sfw->pos;
	struct snd_soc_fw_dapm_graph_elem *elem;
	int count = elem_info->count, i;

	if (sfw->pass != SOC_FW_PASS_GRAPH) {
		sfw->pos += sizeof(struct snd_soc_fw_dapm_elems) + hdr->size;
		return 0;
	}

	sfw->pos += sizeof(struct snd_soc_fw_dapm_elems);

	if (soc_fw_check_count(sfw,
		sizeof(struct snd_soc_fw_dapm_graph_elem), count, hdr->size)) {
		dev_err(sfw->dev, "invalid count %d for DAPM routes\n", count);
		return -EINVAL;
	}

	/* tear down exsiting widgets and graph for this context */
	//soc_dapm_free_widgets(dapm); // lrg - to move

	dev_dbg(sfw->dev, "adding %d DAPM routes\n", count);

	for (i = 0; i < count; i++) {
		elem = (struct snd_soc_fw_dapm_graph_elem *)sfw->pos;
		sfw->pos += sizeof(struct snd_soc_fw_dapm_graph_elem);

		/* validate routes */
		if (strnlen(elem->source, SND_SOC_FW_TEXT_SIZE) ==
			SND_SOC_FW_TEXT_SIZE)
			return -EINVAL;
		if (strnlen(elem->sink, SND_SOC_FW_TEXT_SIZE) ==
			SND_SOC_FW_TEXT_SIZE)
			return -EINVAL;
		if (strnlen(elem->control, SND_SOC_FW_TEXT_SIZE) ==
			SND_SOC_FW_TEXT_SIZE)
			return -EINVAL;

		route.source = elem->source;
		route.sink = elem->sink;
		if (strnlen(elem->control, SND_SOC_FW_TEXT_SIZE) == 0)
			route.control = NULL;
		else
			route.control = elem->control;

		/* add route, but keep going if some fail */
		snd_soc_dapm_add_routes(dapm, &route, 1);
	}

	return 0;
}

static struct snd_kcontrol_new *soc_fw_dapm_widget_dmixer_create(struct soc_fw *sfw,
	int num_kcontrols)
{
	struct snd_kcontrol_new *kc;
	struct soc_mixer_control *sm;
	struct snd_soc_fw_mixer_control *mc;
	int i, err, ext;

	kc = kzalloc(sizeof(*kc) * num_kcontrols, GFP_KERNEL);
	if (!kc)
		return NULL;

	for (i = 0; i < num_kcontrols; i++) {
		sm = kzalloc(sizeof(*sm), GFP_KERNEL);
		if (!sm)
			goto err;

		mc = (struct snd_soc_fw_mixer_control*)sfw->pos;
		sfw->pos += sizeof(struct snd_soc_fw_mixer_control);

		/* validate kcontrol */
		if (strnlen(mc->hdr.name, SND_SOC_FW_TEXT_SIZE) ==
			SND_SOC_FW_TEXT_SIZE)
			goto err_str;

		dev_dbg(sfw->dev, " adding DAPM widget mixer control %s at %d\n",
			mc->hdr.name, i);

		kc[i].name = mc->hdr.name;
		kc[i].private_value = (long)sm;
		kc[i].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
		kc[i].access = mc->hdr.access;

		sm->reg = mc->reg;
		sm->rreg = mc->rreg;
		sm->shift = mc->shift;
		sm->rshift = mc->rshift;
		sm->max = mc->max;
		sm->min = mc->min;
		sm->invert = mc->invert;
		sm->platform_max = mc->platform_max;
		INIT_LIST_HEAD(&sm->list);

		/* map standard io handlers and check for external handlers */
		ext = soc_fw_kcontrol_bind_io(mc->hdr.index, &kc[i], io_ops,
			ARRAY_SIZE(io_ops));
		if (ext) {
			/* none exist, so now try and map ext handlers */
			ext = soc_fw_kcontrol_bind_io(mc->hdr.index, &kc[i],
				sfw->io_ops, sfw->io_ops_count);
			if (ext) {
				dev_err(sfw->dev, "no complete widget mixer IO handler"
					" for %s type (g,p,i) %d:%d:%d\n",
					mc->hdr.name,
					SOC_CONTROL_GET_ID_GET(mc->hdr.index),
					SOC_CONTROL_GET_ID_PUT(mc->hdr.index),
					SOC_CONTROL_GET_ID_INFO(mc->hdr.index));
				kfree(sm);
				continue;
			}

			err = soc_fw_init_kcontrol(sfw, &kc[i]);
			if (err < 0) {
				dev_err(sfw->dev, "failed to init %s\n",
					mc->hdr.name);
				kfree(sm);
				continue;
			}
		}
	}
	return kc;
err_str:
	kfree(sm);
err:
	for (--i; i >= 0; i--)
		kfree((void*)kc[i].private_value);
	kfree(kc);
	return NULL;
}

static struct snd_kcontrol_new *soc_fw_dapm_widget_denum_create(struct soc_fw *sfw)
{
	struct snd_kcontrol_new *kc;
	struct snd_soc_fw_enum_control *ec;
	struct soc_enum *se;
	int i, err, ext;

	ec = (struct snd_soc_fw_enum_control*)sfw->pos;
	sfw->pos += sizeof(struct snd_soc_fw_enum_control);

	/* validate kcontrol */
	if (strnlen(ec->hdr.name, SND_SOC_FW_TEXT_SIZE) ==
		SND_SOC_FW_TEXT_SIZE)
		return NULL;

	kc = kzalloc(sizeof(*kc), GFP_KERNEL);
	if (!kc)
		return NULL;

	se = kzalloc(sizeof(*se), GFP_KERNEL);
	if (!se)
		goto err_se;

	dev_dbg(sfw->dev, " adding DAPM widget enum control %s\n",
		ec->hdr.name);

	kc->name = ec->hdr.name;
	kc->private_value = (long)se;
	kc->iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	kc->access = ec->hdr.access;

	se->reg = ec->reg;
	se->reg2 = ec->reg2;
	se->shift_l = ec->shift_l;
	se->shift_r = ec->shift_r;
	se->max = ec->max;
	se->mask = ec->mask;

	switch (SOC_CONTROL_GET_ID_INFO(ec->hdr.index)) {
	case SOC_CONTROL_TYPE_ENUM:
	case SOC_CONTROL_TYPE_ENUM_EXT:
	case SOC_DAPM_TYPE_ENUM_EXT:
	case SOC_DAPM_TYPE_ENUM_DOUBLE:
	case SOC_DAPM_TYPE_ENUM_VIRT:
		err = soc_fw_denum_create_texts(se, ec);
		if (err < 0) {
			dev_err(sfw->dev, "could not create"
				" texts for %s\n", ec->hdr.name);
			goto err_se;
		}
		break;
	case SOC_CONTROL_TYPE_ENUM_VALUE:
	case SOC_DAPM_TYPE_ENUM_VALUE:
		err = soc_fw_denum_create_values(se, ec);
		if (err < 0) {
			dev_err(sfw->dev, "could not create"
				" values for %s\n", ec->hdr.name);
			goto err_se;
		}
		break;
	default:
		dev_err(sfw->dev, "invalid enum control type %d for %s\n",
			ec->hdr.index, ec->hdr.name);
		goto err_se;
	}

	/* map standard io handlers and check for external handlers */
	ext = soc_fw_kcontrol_bind_io(ec->hdr.index, kc, io_ops,
		ARRAY_SIZE(io_ops));
	if (ext) {
		/* none exist, so now try and map ext handlers */
		ext = soc_fw_kcontrol_bind_io(ec->hdr.index, kc,
			sfw->io_ops, sfw->io_ops_count);
		if (ext) {
			dev_err(sfw->dev, "no complete widget enum IO handler"
				" for %s type (g,p,i) %d:%d:%d\n",
				ec->hdr.name,
				SOC_CONTROL_GET_ID_GET(ec->hdr.index),
				SOC_CONTROL_GET_ID_PUT(ec->hdr.index),
				SOC_CONTROL_GET_ID_INFO(ec->hdr.index));
			goto err_se;
		}

		err = soc_fw_init_kcontrol(sfw, kc);
		if (err < 0) {
			dev_err(sfw->dev, "failed to init %s\n",
				ec->hdr.name);
			goto err_se;
		}
	}

//	soc_fw_list_add_enum(sfw, se);

	return kc;

err_se:
	kfree(kc);
//err:
	/* free texts */
	if (se->dvalues)
		kfree(se->dvalues);
	else {
		for (i = 0; i < ec->max; i++)
			kfree(se->dtexts[i]);
	}
	kfree(se);
	return NULL;
}

static int soc_fw_dapm_widget_create(struct soc_fw *sfw,
	struct snd_soc_fw_dapm_widget *w)
{
	struct snd_soc_dapm_context *dapm = soc_fw_dapm_get(sfw);
	struct snd_soc_dapm_widget widget;
	struct snd_soc_fw_control_hdr *control_hdr;
	int ret = 0;

	if (strnlen(w->name, SND_SOC_FW_TEXT_SIZE) ==
		SND_SOC_FW_TEXT_SIZE)
		return -EINVAL;
	if (strnlen(w->sname, SND_SOC_FW_TEXT_SIZE) ==
		SND_SOC_FW_TEXT_SIZE)
		return -EINVAL;

	dev_dbg(sfw->dev, "creating DAPM widget %s id %d\n", w->name, w->id);

	memset(&widget, 0, sizeof(widget));
	widget.id = w->id;
	widget.name = kstrdup(w->name, GFP_KERNEL);
	if (!widget.name)
		return -ENOMEM;
	widget.sname = kstrdup(w->sname, GFP_KERNEL);
	if (!widget.sname) {
		ret = -ENOMEM;
		goto err;
	}
	widget.reg = w->reg;
	widget.shift = w->shift;
	widget.mask = w->mask;
	widget.invert = w->invert;
	widget.ignore_suspend = w->ignore_suspend;

	sfw->pos += sizeof(struct snd_soc_fw_dapm_widget);
	if (w->kcontrol.count == 0) {
		widget.num_kcontrols = 0;
		goto widget;
	}

	control_hdr = (struct snd_soc_fw_control_hdr*)sfw->pos;
	dev_dbg(sfw->dev, "widget %s has %d controls of type %x\n", w->name,
		w->kcontrol.count, control_hdr->index);

	switch (SOC_CONTROL_GET_ID_INFO(control_hdr->index)) {
	case SOC_CONTROL_TYPE_VOLSW:
	case SOC_CONTROL_TYPE_STROBE:
	case SOC_CONTROL_TYPE_VOLSW_SX:
	case SOC_CONTROL_TYPE_VOLSW_S8:
	case SOC_CONTROL_TYPE_VOLSW_XR_SX:
	case SOC_CONTROL_TYPE_BYTES:
	case SOC_CONTROL_TYPE_BOOL_EXT:
	case SOC_CONTROL_TYPE_RANGE:
	case SOC_DAPM_TYPE_VOLSW:
		widget.num_kcontrols = w->kcontrol.count;
		widget.kcontrol_news = soc_fw_dapm_widget_dmixer_create(sfw,
			widget.num_kcontrols);
		if (!widget.kcontrol_news) {
			ret = -ENOMEM;
			goto hdr_err;
		}
		ret = soc_fw_widget_load(sfw, &widget);
		if (ret < 0)
			goto hdr_err;
		break;
	case SOC_CONTROL_TYPE_ENUM:
	case SOC_CONTROL_TYPE_ENUM_EXT:
	case SOC_CONTROL_TYPE_ENUM_VALUE:
	case SOC_DAPM_TYPE_ENUM_DOUBLE:
	case SOC_DAPM_TYPE_ENUM_VIRT:
	case SOC_DAPM_TYPE_ENUM_VALUE:
	case SOC_DAPM_TYPE_ENUM_EXT:
		widget.num_kcontrols = 1;
		widget.kcontrol_news = soc_fw_dapm_widget_denum_create(sfw);
		if (!widget.kcontrol_news) {
			ret = -ENOMEM;
			goto hdr_err;
		}
		ret = soc_fw_widget_load(sfw, &widget);
		if (ret < 0)
			goto hdr_err;
		break;
	default:
		dev_err(sfw->dev, "invalid widget control type %d:%d:%d\n",
			SOC_CONTROL_GET_ID_GET(control_hdr->index),
			SOC_CONTROL_GET_ID_PUT(control_hdr->index),
			SOC_CONTROL_GET_ID_INFO(control_hdr->index));
		ret = -EINVAL;
		goto hdr_err;
	}

widget:
	ret = snd_soc_dapm_new_controls(dapm, &widget, 1);
	if (ret < 0) {
		dev_err(sfw->dev, "failed to create widget %s controls\n",
			w->name);
		goto hdr_err;
	}

hdr_err:
	//kfree(widget.sname);
err:
	//kfree(widget.name);
	// lrg - free at some point kfree(widget.kcontrol_news);
	return ret;
}

static int soc_fw_dapm_widget_load(struct soc_fw *sfw, struct snd_soc_fw_hdr *hdr)
{
	struct snd_soc_fw_dapm_elems *elem_info =
		(struct snd_soc_fw_dapm_elems*)sfw->pos;
	struct snd_soc_fw_dapm_widget *widget;
	int ret, count = elem_info->count, i;

	if (sfw->pass != SOC_FW_PASS_WIDGET)
		return 0;

	sfw->pos += sizeof(struct snd_soc_fw_dapm_elems);

	if (soc_fw_check_count(sfw,
		sizeof(struct snd_soc_fw_dapm_graph_elem), count, hdr->size)) {
		dev_err(sfw->dev, "invalid count %d for widgets\n", count);
		return -EINVAL;
	}

	dev_dbg(sfw->dev, "adding %d DAPM widgets\n", count);

	for (i = 0; i < count; i++) {
		widget = (struct snd_soc_fw_dapm_widget*) sfw->pos;
		ret = soc_fw_dapm_widget_create(sfw, widget);
		if (ret < 0)
			dev_err(sfw->dev, "failed to load widget %s\n",
				widget->name);
	}

	return 0;
}

static int soc_fw_dapm_complete(struct soc_fw *sfw)
{
	struct snd_soc_dapm_context *dapm = soc_fw_dapm_get(sfw);
	int ret;

	ret = snd_soc_dapm_new_widgets(dapm);
	if (ret < 0)
		dev_err(sfw->dev, "failed to create new widgets %d\n", ret);

	return ret;
}

/*
 * Coefficients with mixer header.
 *
 * Input:
 *  [hdr].<coeff control enum>.<vendor coeff data>
 */
static int soc_fw_coeff_load(struct soc_fw *sfw, struct snd_soc_fw_hdr *hdr)
{
	struct snd_soc_fw_kcontrol *sfwk =
		(struct snd_soc_fw_kcontrol*)sfw->pos;
	struct snd_soc_fw_control_hdr *control_hdr;
	struct snd_soc_fw_hdr *vhdr;
	int ret;

	if (sfw->pass != SOC_FW_PASS_COEFF)
		return 0;

	/* vendor coefficient data is encapsulated with hdrs in generic
	  coefficient controls */
	if (hdr->vendor_type != 0)
		return 0;

	dev_dbg(sfw->dev, "got %d new coefficients\n", sfwk->count);

	sfw->pos += sizeof(struct snd_soc_fw_kcontrol);
	control_hdr = (struct snd_soc_fw_control_hdr*)sfw->pos;

	switch (SOC_CONTROL_GET_ID_INFO(control_hdr->index)) {
	case SOC_CONTROL_TYPE_ENUM:
	case SOC_CONTROL_TYPE_ENUM_EXT:
	case SOC_CONTROL_TYPE_ENUM_VALUE:
		ret = soc_fw_denum_create(sfw, 1, hdr->size);
		if (ret < 0) {
			dev_err(sfw->dev, "failed to create coeff enum %d\n", ret);
			return ret;
		}
		break;
	default:
		dev_err(sfw->dev, "invalid coeff control type %d count %d\n",
			SOC_CONTROL_GET_ID_INFO(control_hdr->index),
			sfwk->count);
		return -EINVAL;
	}

	vhdr = (struct snd_soc_fw_hdr *)sfw->pos;

	ret = soc_fw_vendor_load_(sfw, vhdr);
	if (ret < 0) {
		dev_err(sfw->dev, "unabled to load coeff data %d\n", ret);
		return ret;
	}
	sfw->pos += sizeof(*vhdr) + vhdr->size;
	vhdr = (struct snd_soc_fw_hdr *)sfw->pos;

	return 0;
}

static int soc_fw_dapm_pin_load(struct soc_fw *sfw, struct snd_soc_fw_hdr *hdr)
{
	/* TODO: add static enabled/disabled pins */
	return 0;
}

static int soc_fw_dapm_unload(struct soc_fw *sfw, struct snd_soc_fw_hdr *hdr)
{
	struct snd_soc_dapm_context *dapm = soc_fw_dapm_get(sfw);

	soc_dapm_free_widgets(dapm);
	return 0;
}

static int soc_fw_dai_link_load(struct soc_fw *sfw, struct snd_soc_fw_hdr *hdr)
{
	/* TODO: add DAI links based on FW routing between components */
	return 0;
}

static int soc_fw_dai_link_unload(struct soc_fw *sfw, struct snd_soc_fw_hdr *hdr)
{
	/* TODO: add DAI links based on FW routing between components */
	return 0;
}

static int soc_valid_header(struct soc_fw *sfw, struct snd_soc_fw_hdr *hdr)
{
	if (soc_fw_get_hdr_offset(sfw) >= sfw->fw->size)
		return 0;

	if (hdr->magic != SND_SOC_FW_MAGIC) {
		dev_err(sfw->dev, "%s at pass %d does not have a valid header got %x at offset 0x%x size 0x%x.\n",
			sfw->file, sfw->pass, hdr->magic, soc_fw_get_hdr_offset(sfw), sfw->fw->size);
		return -EINVAL;
	}

	if (hdr->size == 0) {
		dev_err(sfw->dev, "%s header has 0 size at offset 0x%x.\n",
			sfw->file, soc_fw_get_hdr_offset(sfw));
		return -EINVAL;
	}

	if (sfw->pass == hdr->type)
		dev_dbg(sfw->dev, "Got 0x%x bytes of type %d version %d vendor %d at pass %d\n",
			hdr->size, hdr->type, hdr->version, hdr->vendor_type, sfw->pass);

	return 1;
}

static int soc_fw_load_header(struct soc_fw *sfw, struct snd_soc_fw_hdr *hdr)
{
	sfw->pos = sfw->hdr_pos + sizeof(struct snd_soc_fw_hdr);

	switch (hdr->type) {
	case SND_SOC_FW_MIXER:
		return soc_fw_kcontrol_load(sfw, hdr);
	case SND_SOC_FW_DAPM_GRAPH:
		return soc_fw_dapm_graph_load(sfw, hdr);
	case SND_SOC_FW_DAPM_PINS:
		return soc_fw_dapm_pin_load(sfw, hdr);
	case SND_SOC_FW_DAPM_WIDGET:
		return soc_fw_dapm_widget_load(sfw, hdr);
	case SND_SOC_FW_DAI_LINK:
		return soc_fw_dai_link_load(sfw, hdr);
	case SND_SOC_FW_COEFF:
		return soc_fw_coeff_load(sfw, hdr);
	default:
		return soc_fw_vendor_load(sfw, hdr);
	}

	return 0;
}

static int soc_fw_load_headers(struct soc_fw *sfw)
{
	struct snd_soc_fw_hdr *hdr;
	int ret;

	sfw->pass = SOC_FW_PASS_START;

	while (sfw->pass <= SOC_FW_PASS_END) {

		sfw->hdr_pos = sfw->fw->data;
		hdr = (struct snd_soc_fw_hdr *)sfw->hdr_pos;

		while (!soc_fw_eof(sfw)) {

			ret = soc_valid_header(sfw, hdr);
			if (ret < 0) {
				sfw_dump(sfw);
				return ret;
			} else if (ret == 0)
				break;

			ret = soc_fw_load_header(sfw, hdr);
			if (ret < 0)
				return ret;

			sfw->hdr_pos += hdr->size + sizeof(struct snd_soc_fw_hdr);
			hdr = (struct snd_soc_fw_hdr *)sfw->hdr_pos;
		}
		sfw->pass++;
	}

	ret = soc_fw_dapm_complete(sfw);
	if (ret < 0)
		dev_err(sfw->dev, "failed to initialise DAPM\n");

	return ret;
}

static int soc_fw_unload_header(struct soc_fw *sfw, struct snd_soc_fw_hdr *hdr)
{
	if (hdr->magic != SND_SOC_FW_MAGIC) {
		dev_err(sfw->dev, "%s does not have a valid header.\n",
			sfw->file);
		return -EINVAL;
	}

	dev_dbg(sfw->dev, "Got %d bytes of type %d version %d\n", hdr->size,
		hdr->type, hdr->version);

	switch (hdr->type) {
	case SND_SOC_FW_MIXER:
		return soc_fw_kcontrol_unload(sfw, hdr);
	case SND_SOC_FW_DAPM_GRAPH:
	case SND_SOC_FW_DAPM_PINS:
	case SND_SOC_FW_DAPM_WIDGET:
		return soc_fw_dapm_unload(sfw, hdr);
	case SND_SOC_FW_DAI_LINK:
		return soc_fw_dai_link_unload(sfw, hdr);
	default:
		return soc_fw_vendor_unload(sfw, hdr);
	}

	return 0;
}

static int soc_fw_unload_headers(struct soc_fw *sfw)
{
	struct snd_soc_fw_hdr *hdr =
		(struct snd_soc_fw_hdr*)sfw->fw->data;
	int ret;

	sfw->pass = SOC_FW_PASS_START;
	sfw->pos += sizeof(struct snd_soc_fw_hdr);

	while (sfw->pass <= SOC_FW_PASS_END) {
		while (!soc_fw_eof(sfw)) {
			ret = soc_fw_unload_header(sfw, hdr);
			if (ret < 0)
				return ret;
		}
		sfw->pass++;
	}

	soc_fw_complete(sfw);
	return 0;
}

static int soc_fw_load_codec(struct snd_soc_codec *codec,
	struct snd_soc_fw_codec_ops *ops, const char *file, int nowait)
{
	struct soc_fw *sfw;
	int ret;

	sfw = kzalloc(sizeof(struct soc_fw), GFP_KERNEL);
	if (sfw == NULL)
		return -ENOMEM;

	sfw->file = file;
	sfw->codec = codec;
	sfw->dev = codec->dev;
	sfw->codec_ops = ops;
	sfw->io_ops = ops->io_ops;
	sfw->io_ops_count = ops->io_ops_count;

	if (nowait) {
		ret = soc_fw_request_data_nowait(sfw);
		if (ret < 0)
			return ret;
		return -EAGAIN;
	} else {
		ret = soc_fw_request_data(sfw);
		if (ret != 0)
			return ret;

		ret = soc_fw_load_headers(sfw);
		if (ret < 0)
			soc_fw_complete(sfw);
		soc_fw_release_data(sfw);
	}

	kfree(sfw);
	return ret;
}

int snd_soc_fw_load_codec(struct snd_soc_codec *codec,
	struct snd_soc_fw_codec_ops *ops, const char *file)
{
	return soc_fw_load_codec(codec, ops, file, 0);
}
EXPORT_SYMBOL_GPL(snd_soc_fw_load_codec);

int snd_soc_fw_load_codec_nowait(struct snd_soc_codec *codec,
	struct snd_soc_fw_codec_ops *ops, const char *file)
{
	return soc_fw_load_codec(codec, ops, file, 1);
}
EXPORT_SYMBOL_GPL(snd_soc_fw_load_codec_nowait);

static int soc_fw_exec(struct soc_fw *sfw)
{
	int ret;

	ret = soc_fw_load_headers(sfw);
	if (ret == 0)
		soc_fw_complete(sfw);
	soc_fw_release_data(sfw);

	return ret;
}


static int soc_fw_load_platform(struct snd_soc_platform *platform,
	struct snd_soc_fw_platform_ops *ops, const char *file, int nowait)
{
	struct soc_fw *sfw, *t;
	int ret, found = 0;

	mutex_lock(&client_mutex);

	/* check for that we are not waiting on this platfomr */
	list_for_each_entry(sfw, &waiting_list, list) {
		if (sfw->dev == platform->dev) {
			mutex_unlock(&client_mutex);
			return -EPROBE_DEFER;
		}
	}

	list_for_each_entry_safe(sfw, t, &ready_list, list) {
		if (sfw->dev == platform->dev) {
			found = 1;
			list_del(&sfw->list);
			break;
		}
	}
	mutex_unlock(&client_mutex);


	if (found) {
		ret = soc_fw_exec(sfw);
		kfree(sfw);
		return ret;
	}

	sfw = kzalloc(sizeof(struct soc_fw), GFP_KERNEL);
	if (sfw == NULL)
		return -ENOMEM;

	sfw->file = file;
	sfw->dev = platform->dev;
	sfw->platform = platform;
	sfw->platform_ops = ops;
	sfw->io_ops = ops->io_ops;
	sfw->io_ops_count = ops->io_ops_count;

	if (nowait) {
		mutex_lock(&client_mutex);
		list_add(&sfw->list, &waiting_list);
		mutex_unlock(&client_mutex);
		return soc_fw_request_data_nowait(sfw);
	} else {
		ret = soc_fw_request_data(sfw);
		if (ret != 0)
			return ret;
			ret = soc_fw_exec(sfw);
	}

	kfree(sfw);
	return ret;
}

int snd_soc_fw_load_platform(struct snd_soc_platform *platform,
	struct snd_soc_fw_platform_ops *ops, const char *file)
{
	return soc_fw_load_platform(platform, ops, file, 0);
}
EXPORT_SYMBOL_GPL(snd_soc_fw_load_platform);

int snd_soc_fw_load_platform_nowait(struct snd_soc_platform *platform,
	struct snd_soc_fw_platform_ops *ops, const char *file)
{
	return soc_fw_load_platform(platform, ops, file, 1);
}
EXPORT_SYMBOL_GPL(snd_soc_fw_load_platform_nowait);

static int soc_fw_load_card(struct snd_soc_card *card,
	struct snd_soc_fw_card_ops *ops, const char *file, int nowait)
{
	struct soc_fw *sfw;
	int ret;

	sfw = kzalloc(sizeof(struct soc_fw), GFP_KERNEL);
	if (sfw == NULL)
		return -ENOMEM;

	sfw->file = file;
	sfw->card = card;
	sfw->dev = card->dev;
	sfw->card_ops = ops;

	if (nowait)
		return soc_fw_request_data_nowait(sfw);
	else {
		ret = soc_fw_request_data(sfw);
		if (ret != 0)
			return ret;

		ret = soc_fw_load_headers(sfw);
		if (ret < 0)
			soc_fw_complete(sfw);
		soc_fw_release_data(sfw);
	}

	kfree(sfw);
	return ret;
}

int snd_soc_fw_load_card(struct snd_soc_card *card,
	struct snd_soc_fw_card_ops *ops, const char *file)
{
	return soc_fw_load_card(card, ops, file, 0);
}
EXPORT_SYMBOL_GPL(snd_soc_fw_load_card);

int snd_soc_fw_load_card_nowait(struct snd_soc_card *card,
	struct snd_soc_fw_card_ops *ops, const char *file)
{
	return soc_fw_load_card(card, ops, file, 1);
}
EXPORT_SYMBOL_GPL(snd_soc_fw_load_card_nowait);

int snd_soc_fw_unload_card(struct snd_soc_card *card,
	struct snd_soc_fw_card_ops *ops, const char *file)
{
	struct soc_fw *sfw;
	int ret;

	sfw = kzalloc(sizeof(struct soc_fw), GFP_KERNEL);
	if (sfw == NULL)
		return -ENOMEM;

	sfw->file = file;
	sfw->card = card;
	sfw->dev = card->dev;
	sfw->card_ops = ops;

	ret = soc_fw_request_data(sfw);
	if (ret != 0)
		return ret;

	ret = soc_fw_unload_headers(sfw);
	soc_fw_release_data(sfw);
	kfree(sfw);
	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_fw_unload_card);

int snd_soc_fw_unload_platform(struct snd_soc_platform *platform,
	struct snd_soc_fw_platform_ops *ops, const char *file)
{
	struct soc_fw *sfw;
	int ret;

	sfw = kzalloc(sizeof(struct soc_fw), GFP_KERNEL);
	if (sfw == NULL)
		return -ENOMEM;

	sfw->file = file;
	sfw->platform = platform;
	sfw->dev = platform->dev;
	sfw->platform_ops = ops;
	sfw->io_ops = ops->io_ops;
	sfw->io_ops_count = ops->io_ops_count;

	ret = soc_fw_request_data(sfw);
	if (ret != 0)
		return ret;

	ret = soc_fw_unload_headers(sfw);
	soc_fw_release_data(sfw);
	kfree(sfw);
	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_fw_unload_platform);


int snd_soc_fw_unload_codec(struct snd_soc_codec *codec,
	struct snd_soc_fw_codec_ops *ops, const char *file)
{
	struct soc_fw *sfw;
	int ret;

	sfw = kzalloc(sizeof(struct soc_fw), GFP_KERNEL);
	if (sfw == NULL)
		return -ENOMEM;

	sfw->file = file;
	sfw->codec = codec;
	sfw->dev = codec->dev;
	sfw->codec_ops = ops;

	ret = soc_fw_request_data(sfw);
	if (ret != 0)
		return ret;

	ret = soc_fw_unload_headers(sfw);
	soc_fw_release_data(sfw);
	kfree(sfw);
	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_fw_unload_codec);
