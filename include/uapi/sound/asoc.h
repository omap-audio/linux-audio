/*
 * linux/uapi/sound/asoc.h -- ALSA SoC Firmware Controls and DAPM
 *
 * Copyright:	2012 Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Simple file API to load FW that includes mixers, coefficients, DAPM graphs,
 * algorithms, equalisers, DAIs, widgets etc.
 */

#ifndef __LINUX_UAPI_SND_ASOC_H
#define __LINUX_UAPI_SND_ASOC_H

/*
 * Convenience kcontrol builders
 */
#define SOC_DOUBLE_VALUE(xreg, shift_left, shift_right, xmax, xinvert) \
	((unsigned long)&(struct soc_mixer_control) \
	{.reg = xreg, .rreg = xreg, .shift = shift_left, \
	.rshift = shift_right, .max = xmax, .platform_max = xmax, \
	.invert = xinvert})
#define SOC_SINGLE_VALUE(xreg, xshift, xmax, xinvert) \
	SOC_DOUBLE_VALUE(xreg, xshift, xshift, xmax, xinvert)
#define SOC_SINGLE_VALUE_EXT(xreg, xmax, xinvert) \
	((unsigned long)&(struct soc_mixer_control) \
	{.reg = xreg, .max = xmax, .platform_max = xmax, .invert = xinvert})
#define SOC_DOUBLE_R_VALUE(xlreg, xrreg, xshift, xmax, xinvert) \
	((unsigned long)&(struct soc_mixer_control) \
	{.reg = xlreg, .rreg = xrreg, .shift = xshift, .rshift = xshift, \
	.max = xmax, .platform_max = xmax, .invert = xinvert})
#define SOC_DOUBLE_R_RANGE_VALUE(xlreg, xrreg, xshift, xmin, xmax, xinvert) \
	((unsigned long)&(struct soc_mixer_control) \
	{.reg = xlreg, .rreg = xrreg, .shift = xshift, .rshift = xshift, \
	.min = xmin, .max = xmax, .platform_max = xmax, .invert = xinvert})
#define SOC_SINGLE(xname, reg, shift, max, invert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_volsw, .get = snd_soc_get_volsw,\
	.put = snd_soc_put_volsw, .index = SOC_CONTROL_IO_VOLSW, \
	.private_value =  SOC_SINGLE_VALUE(reg, shift, max, invert) }
#define SOC_SINGLE_RANGE(xname, xreg, xshift, xmin, xmax, xinvert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.info = snd_soc_info_volsw_range, .get = snd_soc_get_volsw_range, \
	.put = snd_soc_put_volsw_range, .index = SOC_CONTROL_IO_RANGE, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = xreg, .shift = xshift, .min = xmin,\
		 .max = xmax, .platform_max = xmax, .invert = xinvert} }
#define SOC_SINGLE_TLV(xname, reg, shift, max, invert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, .get = snd_soc_get_volsw,\
	.put = snd_soc_put_volsw, .index = SOC_CONTROL_IO_VOLSW, \
	.private_value =  SOC_SINGLE_VALUE(reg, shift, max, invert) }
#define SOC_SINGLE_SX_TLV(xname, xreg, xshift, xmin, xmax, tlv_array) \
{       .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | \
	SNDRV_CTL_ELEM_ACCESS_READWRITE, \
	.tlv.p  = (tlv_array),\
	.info = snd_soc_info_volsw, \
	.get = snd_soc_get_volsw_sx,\
	.put = snd_soc_put_volsw_sx, \
	.index = SOC_CONTROL_IO_VOLSW_SX, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = xreg, .rreg = xreg, \
		.shift = xshift, .rshift = xshift, \
		.max = xmax, .min = xmin} }
#define SOC_SINGLE_RANGE_TLV(xname, xreg, xshift, xmin, xmax, xinvert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw_range, .index = SOC_CONTROL_IO_RANGE, \
	.get = snd_soc_get_volsw_range, .put = snd_soc_put_volsw_range, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = xreg, .shift = xshift, .min = xmin,\
		 .max = xmax, .platform_max = xmax, .invert = xinvert} }
#define SOC_DOUBLE(xname, reg, shift_left, shift_right, max, invert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.info = snd_soc_info_volsw, .get = snd_soc_get_volsw, \
	.put = snd_soc_put_volsw, .index = SOC_CONTROL_IO_VOLSW, \
	.private_value = SOC_DOUBLE_VALUE(reg, shift_left, shift_right, \
					  max, invert) }
#define SOC_DOUBLE_R(xname, reg_left, reg_right, xshift, xmax, xinvert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.info = snd_soc_info_volsw, .index = SOC_CONTROL_IO_VOLSW, \
	.get = snd_soc_get_volsw, .put = snd_soc_put_volsw, \
	.private_value = SOC_DOUBLE_R_VALUE(reg_left, reg_right, xshift, \
					    xmax, xinvert) }
#define SOC_DOUBLE_R_RANGE(xname, reg_left, reg_right, xshift, xmin, \
			   xmax, xinvert)		\
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.info = snd_soc_info_volsw_range, .index = SOC_CONTROL_IO_RANGE, \
	.get = snd_soc_get_volsw_range, .put = snd_soc_put_volsw_range, \
	.private_value = SOC_DOUBLE_R_RANGE_VALUE(reg_left, reg_right, \
					    xshift, xmin, xmax, xinvert) }
#define SOC_DOUBLE_TLV(xname, reg, shift_left, shift_right, max, invert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, .get = snd_soc_get_volsw, \
	.put = snd_soc_put_volsw, .index = SOC_CONTROL_IO_VOLSW, \
	.private_value = SOC_DOUBLE_VALUE(reg, shift_left, shift_right, \
					  max, invert) }
#define SOC_DOUBLE_R_TLV(xname, reg_left, reg_right, xshift, xmax, xinvert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, .index = SOC_CONTROL_IO_VOLSW, \
	.get = snd_soc_get_volsw, .put = snd_soc_put_volsw, \
	.private_value = SOC_DOUBLE_R_VALUE(reg_left, reg_right, xshift, \
					    xmax, xinvert) }
#define SOC_DOUBLE_R_RANGE_TLV(xname, reg_left, reg_right, xshift, xmin, \
			       xmax, xinvert, tlv_array)		\
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw_range, .index = SOC_CONTROL_IO_RANGE, \
	.get = snd_soc_get_volsw_range, .put = snd_soc_put_volsw_range, \
	.private_value = SOC_DOUBLE_R_RANGE_VALUE(reg_left, reg_right, \
					    xshift, xmin, xmax, xinvert) }
#define SOC_DOUBLE_R_SX_TLV(xname, xreg, xrreg, xshift, xmin, xmax, tlv_array) \
{       .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | \
	SNDRV_CTL_ELEM_ACCESS_READWRITE, \
	.tlv.p  = (tlv_array), \
	.info = snd_soc_info_volsw, \
	.get = snd_soc_get_volsw_sx, \
	.put = snd_soc_put_volsw_sx, \
	.index = SOC_CONTROL_IO_VOLSW, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = xreg, .rreg = xrreg, \
		.shift = xshift, .rshift = xshift, \
		.max = xmax, .min = xmin} }
#define SOC_DOUBLE_S8_TLV(xname, xreg, xmin, xmax, tlv_array) \
{	.iface  = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | \
		  SNDRV_CTL_ELEM_ACCESS_READWRITE, \
	.tlv.p  = (tlv_array), \
	.info   = snd_soc_info_volsw_s8, .get = snd_soc_get_volsw_s8, \
	.put    = snd_soc_put_volsw_s8, .index = SOC_CONTROL_IO_VOLSW_S8, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = xreg, .min = xmin, .max = xmax, \
		 .platform_max = xmax} }
#define SOC_ENUM_DOUBLE(xreg, xshift_l, xshift_r, xmax, xtexts) \
{	.reg = xreg, .shift_l = xshift_l, .shift_r = xshift_r, \
	.max = xmax, .texts = xtexts, \
	.mask = xmax ? roundup_pow_of_two(xmax) - 1 : 0}
#define SOC_ENUM_SINGLE(xreg, xshift, xmax, xtexts) \
	SOC_ENUM_DOUBLE(xreg, xshift, xshift, xmax, xtexts)
#define SOC_ENUM_SINGLE_EXT(xmax, xtexts) \
{	.max = xmax, .texts = xtexts }
#define SOC_VALUE_ENUM_DOUBLE(xreg, xshift_l, xshift_r, xmask, xmax, xtexts, xvalues) \
{	.reg = xreg, .shift_l = xshift_l, .shift_r = xshift_r, \
	.mask = xmask, .max = xmax, .texts = xtexts, .values = xvalues}
#define SOC_VALUE_ENUM_SINGLE(xreg, xshift, xmask, xmax, xtexts, xvalues) \
	SOC_VALUE_ENUM_DOUBLE(xreg, xshift, xshift, xmask, xmax, xtexts, xvalues)
#define SOC_ENUM(xname, xenum) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,\
	.info = snd_soc_info_enum_double, .index = SOC_CONTROL_IO_ENUM, \
	.get = snd_soc_get_enum_double, .put = snd_soc_put_enum_double, \
	.private_value = (unsigned long)&xenum }
#define SOC_VALUE_ENUM(xname, xenum) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,\
	.info = snd_soc_info_enum_double, \
	.get = snd_soc_get_value_enum_double, \
	.put = snd_soc_put_value_enum_double, \
	.index = SOC_CONTROL_IO_ENUM_VALUE, \
	.private_value = (unsigned long)&xenum }
#define SOC_SINGLE_EXT(xname, xreg, xshift, xmax, xinvert,\
	 xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_volsw, .index = SOC_CONTROL_IO_EXT, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = SOC_SINGLE_VALUE(xreg, xshift, xmax, xinvert) }
#define SOC_DOUBLE_EXT(xname, reg, shift_left, shift_right, max, invert,\
	 xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.info = snd_soc_info_volsw, .index = SOC_CONTROL_IO_EXT, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = \
		SOC_DOUBLE_VALUE(reg, shift_left, shift_right, max, invert) }
#define SOC_SINGLE_EXT_TLV(xname, xreg, xshift, xmax, xinvert,\
	 xhandler_get, xhandler_put, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, .index = SOC_CONTROL_IO_EXT, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = SOC_SINGLE_VALUE(xreg, xshift, xmax, xinvert) }
#define SOC_DOUBLE_EXT_TLV(xname, xreg, shift_left, shift_right, xmax, xinvert,\
	 xhandler_get, xhandler_put, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | \
		 SNDRV_CTL_ELEM_ACCESS_READWRITE, \
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, .index = SOC_CONTROL_IO_EXT,\
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = SOC_DOUBLE_VALUE(xreg, shift_left, shift_right, \
					  xmax, xinvert) }
#define SOC_DOUBLE_R_EXT_TLV(xname, reg_left, reg_right, xshift, xmax, xinvert,\
	 xhandler_get, xhandler_put, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | \
		 SNDRV_CTL_ELEM_ACCESS_READWRITE, \
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, .index = SOC_CONTROL_IO_EXT, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = SOC_DOUBLE_R_VALUE(reg_left, reg_right, xshift, \
					    xmax, xinvert) }
#define SOC_SINGLE_BOOL_EXT(xname, xdata, xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_bool_ext, .index = SOC_CONTROL_IO_BOOL_EXT, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = xdata }
#define SOC_ENUM_EXT(xname, xenum, xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_enum_ext, .index = SOC_CONTROL_IO_ENUM_EXT, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = (unsigned long)&xenum }

#define SND_SOC_BYTES(xname, xbase, xregs)		      \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,   \
	.info = snd_soc_bytes_info, .get = snd_soc_bytes_get, \
	.index = SOC_CONTROL_IO_BYTES, \
	.put = snd_soc_bytes_put, .private_value =	      \
		((unsigned long)&(struct soc_bytes)           \
		{.base = xbase, .num_regs = xregs }) }

#define SND_SOC_BYTES_MASK(xname, xbase, xregs, xmask)	      \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,   \
	.info = snd_soc_bytes_info, .get = snd_soc_bytes_get, \
	.index = SOC_CONTROL_IO_BYTES, \
	.put = snd_soc_bytes_put, .private_value =	      \
		((unsigned long)&(struct soc_bytes)           \
		{.base = xbase, .num_regs = xregs,	      \
		 .mask = xmask }) }

#define SOC_SINGLE_XR_SX(xname, xregbase, xregcount, xnbits, \
		xmin, xmax, xinvert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.info = snd_soc_info_xr_sx, .get = snd_soc_get_xr_sx, \
	.put = snd_soc_put_xr_sx, \
	.index = SOC_CONTROL_IO_XR_SX, \
	.private_value = (unsigned long)&(struct soc_mreg_control) \
		{.regbase = xregbase, .regcount = xregcount, .nbits = xnbits, \
		.invert = xinvert, .min = xmin, .max = xmax} }

#define SOC_SINGLE_STROBE(xname, xreg, xshift, xinvert) \
	SOC_SINGLE_EXT(xname, xreg, xshift, 1, xinvert, \
		snd_soc_get_strobe, snd_soc_put_strobe)

/*
 * Simplified versions of above macros, declaring a struct and calculating
 * ARRAY_SIZE internally
 */
#define SOC_ENUM_DOUBLE_DECL(name, xreg, xshift_l, xshift_r, xtexts) \
	struct soc_enum name = SOC_ENUM_DOUBLE(xreg, xshift_l, xshift_r, \
						ARRAY_SIZE(xtexts), xtexts)
#define SOC_ENUM_SINGLE_DECL(name, xreg, xshift, xtexts) \
	SOC_ENUM_DOUBLE_DECL(name, xreg, xshift, xshift, xtexts)
#define SOC_ENUM_SINGLE_EXT_DECL(name, xtexts) \
	struct soc_enum name = SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(xtexts), xtexts)
#define SOC_VALUE_ENUM_DOUBLE_DECL(name, xreg, xshift_l, xshift_r, xmask, xtexts, xvalues) \
	struct soc_enum name = SOC_VALUE_ENUM_DOUBLE(xreg, xshift_l, xshift_r, xmask, \
							ARRAY_SIZE(xtexts), xtexts, xvalues)
#define SOC_VALUE_ENUM_SINGLE_DECL(name, xreg, xshift, xmask, xtexts, xvalues) \
	SOC_VALUE_ENUM_DOUBLE_DECL(name, xreg, xshift, xshift, xmask, xtexts, xvalues)


/*
 * Numeric IDs for stock mixer types that are used to enumerate FW based mixers.
 */

#define SOC_CONTROL_ID_PUT(p)	((p & 0xff) << 16)
#define SOC_CONTROL_ID_GET(g)	((g & 0xff) << 8)
#define SOC_CONTROL_ID_INFO(i)	((i & 0xff) << 0)
#define SOC_CONTROL_ID(g, p, i)	\
	(SOC_CONTROL_ID_PUT(p) | SOC_CONTROL_ID_GET(g) |\
	SOC_CONTROL_ID_INFO(i))

#define SOC_CONTROL_GET_ID_PUT(id)	((id & 0xff0000) >> 16)
#define SOC_CONTROL_GET_ID_GET(id)	((id & 0x00ff00) >> 8)
#define SOC_CONTROL_GET_ID_INFO(id)	((id & 0x0000ff) >> 0)

/* individual kcontrol info types - can be mixed with other types */
#define SOC_CONTROL_TYPE_EXT		0	/* driver defined */
#define SOC_CONTROL_TYPE_VOLSW		1
#define SOC_CONTROL_TYPE_VOLSW_SX	2
#define SOC_CONTROL_TYPE_VOLSW_S8	3
#define SOC_CONTROL_TYPE_VOLSW_XR_SX	4
#define SOC_CONTROL_TYPE_ENUM		6
#define SOC_CONTROL_TYPE_ENUM_EXT	7
#define SOC_CONTROL_TYPE_BYTES		8
#define SOC_CONTROL_TYPE_BOOL_EXT	9
#define SOC_CONTROL_TYPE_ENUM_VALUE	10
#define SOC_CONTROL_TYPE_RANGE		11
#define SOC_CONTROL_TYPE_STROBE		12

/* compound control IDs */
#define SOC_CONTROL_IO_VOLSW \
	SOC_CONTROL_ID(SOC_CONTROL_TYPE_VOLSW, \
		SOC_CONTROL_TYPE_VOLSW, \
		SOC_CONTROL_TYPE_VOLSW)
#define SOC_CONTROL_IO_VOLSW_SX \
	SOC_CONTROL_ID(SOC_CONTROL_TYPE_VOLSW_SX, \
		SOC_CONTROL_TYPE_VOLSW_SX, \
		SOC_CONTROL_TYPE_VOLSW)
#define SOC_CONTROL_IO_VOLSW_S8 \
	SOC_CONTROL_ID(SOC_CONTROL_TYPE_VOLSW_S8, \
		SOC_CONTROL_TYPE_VOLSW_S8, \
		SOC_CONTROL_TYPE_VOLSW_S8)
#define SOC_CONTROL_IO_VOLSW_XR_SX \
	SOC_CONTROL_ID(SOC_CONTROL_TYPE_VOLSW_XR_SX, \
		SOC_CONTROL_TYPE_VOLSW_XR_SX, \
		SOC_CONTROL_TYPE_VOLSW_XR_SX)
#define SOC_CONTROL_IO_EXT \
	SOC_CONTROL_ID(SOC_CONTROL_TYPE_EXT, \
		SOC_CONTROL_TYPE_EXT, \
		SOC_CONTROL_TYPE_VOLSW)
#define SOC_CONTROL_IO_ENUM \
	SOC_CONTROL_ID(SOC_CONTROL_TYPE_ENUM, \
		SOC_CONTROL_TYPE_ENUM, \
		SOC_CONTROL_TYPE_ENUM)
#define SOC_CONTROL_IO_ENUM_EXT \
	SOC_CONTROL_ID(SOC_CONTROL_TYPE_EXT, \
		SOC_CONTROL_TYPE_EXT, \
		SOC_CONTROL_TYPE_ENUM_EXT)
#define SOC_CONTROL_IO_BYTES \
	SOC_CONTROL_ID(SOC_CONTROL_TYPE_BYTES, \
		SOC_CONTROL_TYPE_BYTES, \
		SOC_CONTROL_TYPE_BYTES)
#define SOC_CONTROL_IO_BOOL_EXT \
	SOC_CONTROL_ID(SOC_CONTROL_TYPE_EXT, \
		SOC_CONTROL_TYPE_EXT, \
		SOC_CONTROL_TYPE_BOOL_EXT)
#define SOC_CONTROL_IO_ENUM_VALUE \
	SOC_CONTROL_ID(SOC_CONTROL_TYPE_ENUM_VALUE, \
		SOC_CONTROL_TYPE_ENUM_VALUE, \
		SOC_CONTROL_TYPE_ENUM)
#define SOC_CONTROL_IO_RANGE \
	SOC_CONTROL_ID(SOC_CONTROL_TYPE_RANGE, \
		SOC_CONTROL_TYPE_RANGE, \
		SOC_CONTROL_TYPE_RANGE)
#define SOC_CONTROL_IO_STROBE \
	SOC_CONTROL_ID(SOC_CONTROL_TYPE_STROBE, \
		SOC_CONTROL_TYPE_STROBE, \
		SOC_CONTROL_TYPE_STROBE)


/* Header magic number and string sizes */
#define SND_SOC_FW_MAGIC	0x41536F43 /* ASoC */

/* string sizes */
#define SND_SOC_FW_TEXT_SIZE	32
#define SND_SOC_FW_NUM_TEXTS	16

/* ABI version */
#define SND_SOC_FW_ABI_VERSION		1

/*
 * File andBlock header data types.
 * Add new generic and vendor types to end of list.
 * Generic types are handled by the core whilst vendors types are passed
 * to the component drivers for handling.
 */
#define SND_SOC_FW_MIXER		1
#define SND_SOC_FW_DAPM_GRAPH		2
#define SND_SOC_FW_DAPM_PINS		3
#define SND_SOC_FW_DAPM_WIDGET		4
#define SND_SOC_FW_DAI_LINK		5
#define SND_SOC_FW_COEFF		6

#define SND_SOC_FW_VENDOR_FW		1000
#define SND_SOC_FW_VENDOR_CONFIG	1001
#define SND_SOC_FW_VENDOR_COEFF	1002
#define SND_SOC_FW_VENDOR_CODEC	1003

struct firmware;

/*
 * File and Block Header
 */
struct snd_soc_fw_hdr {
	u32 magic;
	u32 type;
	u32 vendor_type; /* optional vendor specific type info */
	u32 version; /* optional vendor specific version details */
	u32 size; /* data bytes, excluding this header */
	/* file data contents start here */
} __attribute__((packed));


struct snd_soc_fw_ctl_tlv {
	u32 numid;	/* control element numeric identification */
	u32 length;	/* in bytes aligned to 4 */
	/* tlv data starts here */
} __attribute__((packed));

struct snd_soc_fw_control_hdr {
	char name[SND_SOC_FW_TEXT_SIZE];
	u32 index;
	u32 access;
	u32 tlv_size;
} __attribute__((packed));

/*
 * Mixer KControl.
 */
struct snd_soc_fw_mixer_control {
	struct snd_soc_fw_control_hdr hdr;
	s32 min;
	s32 max;
	s32 platform_max;
	u32 reg;
	u32 rreg;
	u32 shift;
	u32 rshift;
	u32 invert;
} __attribute__((packed));

/*
 * Enumerated KControl
 */
struct snd_soc_fw_enum_control {
	struct snd_soc_fw_control_hdr hdr;
	u32 reg;
	u32 reg2;
	u32 shift_l;
	u32 shift_r;
	u32 max;
	u32 mask;
	union {	/* both texts and values are the same size */
		char texts[SND_SOC_FW_NUM_TEXTS][SND_SOC_FW_TEXT_SIZE];
		u32 values[SND_SOC_FW_NUM_TEXTS * SND_SOC_FW_TEXT_SIZE / 4];
	};
} __attribute__((packed));

/*
 * Kcontrol Header
 */
struct snd_soc_fw_kcontrol {
	u32 count; /* in kcontrols (based on type) */
	/* kcontrols here */
} __attribute__((packed));

/*
 * DAPM Graph Element
 */
struct snd_soc_fw_dapm_graph_elem {
	char sink[SND_SOC_FW_TEXT_SIZE];
	char control[SND_SOC_FW_TEXT_SIZE];
	char source[SND_SOC_FW_TEXT_SIZE];
} __attribute__((packed));

/*
 * DAPM Pin Element.
 */
struct snd_soc_fw_dapm_pin_elem {
	char name[SND_SOC_FW_TEXT_SIZE];
	u32 disconnect:1;
	u32 ignore_suspend:1;
} __attribute__((packed));


/*
 * DAPM Widget.
 */
struct snd_soc_fw_dapm_widget {
	u32 id;		/* snd_soc_dapm_type */
	char name[SND_SOC_FW_TEXT_SIZE];
	char sname[SND_SOC_FW_TEXT_SIZE];

	s32 reg;		/* negative reg = no direct dapm */
	u32 shift;		/* bits to shift */
	u32 mask;		/* non-shifted mask */
	u32 invert:1;		/* invert the power bit */
	u32 ignore_suspend:1;	/* kept enabled over suspend */

	/* kcontrols that relate to this widget */
	struct snd_soc_fw_kcontrol kcontrol;
	/* controls follow here */
} __attribute__((packed));

/*
 * DAPM Graph and Pins.
 */
struct snd_soc_fw_dapm_elems {
	u32 count; /* in elements */
	/* elements here */
} __attribute__((packed));

/*
 * Coeffcient File Data.
 */
struct snd_soc_file_coeff_data {
	u32 count; /* in elems */
	u32 size;	/* total data size */
	u32 id; /* associated mixer ID */
	/* data here */
} __attribute__((packed));

#endif

