/*
 * OMAP5 Clock data
 *
 * Copyright (C) 2012-2013 Texas Instruments, Inc.
 *
 * Paul Walmsley (paul@pwsan.com)
 * Rajendra Nayak (rnayak@ti.com)
 * Benoit Cousson (b-cousson@ti.com)
 * Vincent Stehlé (v-stehle@ti.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * XXX Some of the ES1 clocks have been removed/changed; once support
 * is added for discriminating clocks by ES level, these should be added back
 * in.
 */

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/clk-private.h>
#include <linux/clkdev.h>
#include <linux/io.h>

#include "soc.h"
#include "iomap.h"
#include "clock.h"
#include "clock54xx.h"
#include "cm1_54xx.h"
#include "cm2_54xx.h"
#include "cm-regbits-54xx.h"
#include "prm54xx.h"
#include "prm-regbits-54xx.h"
#include "control.h"
#include "scrm54xx.h"

/* OMAP5 modulemode control */
#define OMAP54XX_MODULEMODE_HWCTRL			0
#define OMAP54XX_MODULEMODE_SWCTRL			1

/* Root clocks */

DEFINE_CLK_FIXED_RATE(pad_clks_src_ck, CLK_IS_ROOT, 12000000, 0x0);

DEFINE_CLK_GATE(pad_clks_ck, "pad_clks_src_ck", &pad_clks_src_ck, 0x0,
		OMAP54XX_CM_CLKSEL_ABE, OMAP54XX_PAD_CLKS_GATE_SHIFT,
		0x0, NULL);

DEFINE_CLK_FIXED_RATE(pad_slimbus_core_clks_ck, CLK_IS_ROOT, 12000000, 0x0);

DEFINE_CLK_FIXED_RATE(slimbus_src_clk, CLK_IS_ROOT, 12000000, 0x0);

DEFINE_CLK_GATE(slimbus_clk, "slimbus_src_clk", &slimbus_src_clk, 0x0,
		OMAP54XX_CM_CLKSEL_ABE, OMAP54XX_SLIMBUS1_CLK_GATE_SHIFT,
		0x0, NULL);

DEFINE_CLK_FIXED_RATE(sys_32k_ck, CLK_IS_ROOT, 32768, 0x0);

DEFINE_CLK_FIXED_RATE(virt_12000000_ck, CLK_IS_ROOT, 12000000, 0x0);

DEFINE_CLK_FIXED_RATE(virt_13000000_ck, CLK_IS_ROOT, 13000000, 0x0);

DEFINE_CLK_FIXED_RATE(virt_16800000_ck, CLK_IS_ROOT, 16800000, 0x0);

DEFINE_CLK_FIXED_RATE(virt_19200000_ck, CLK_IS_ROOT, 19200000, 0x0);

DEFINE_CLK_FIXED_RATE(virt_26000000_ck, CLK_IS_ROOT, 26000000, 0x0);

DEFINE_CLK_FIXED_RATE(virt_27000000_ck, CLK_IS_ROOT, 27000000, 0x0);

DEFINE_CLK_FIXED_RATE(virt_38400000_ck, CLK_IS_ROOT, 38400000, 0x0);

static const char *sys_clkin_parents[] = {
	"virt_12000000_ck", "virt_13000000_ck", "virt_16800000_ck",
	"virt_19200000_ck", "virt_26000000_ck", "virt_27000000_ck",
	"virt_38400000_ck",
};

DEFINE_CLK_MUX(sys_clkin, sys_clkin_parents, NULL, 0x0,
	       OMAP54XX_CM_CLKSEL_SYS, OMAP54XX_SYS_CLKSEL_SHIFT,
	       OMAP54XX_SYS_CLKSEL_WIDTH, CLK_MUX_INDEX_ONE, NULL);

DEFINE_CLK_FIXED_RATE(xclk60mhsp1, CLK_IS_ROOT, 60000000, 0x0);

DEFINE_CLK_FIXED_RATE(xclk60mhsp2, CLK_IS_ROOT, 60000000, 0x0);

/* Module clocks and DPLL outputs */

static const char *abe_dpll_bypass_clk_mux_parents[] = {
	"sys_clkin", "sys_32k_ck",
};

DEFINE_CLK_MUX(abe_dpll_bypass_clk_mux, abe_dpll_bypass_clk_mux_parents,
	       NULL, 0x0, OMAP54XX_CM_CLKSEL_WKUPAON, OMAP54XX_CLKSEL_0_0_SHIFT,
	       OMAP54XX_CLKSEL_0_0_WIDTH, 0x0, NULL);

DEFINE_CLK_MUX(abe_dpll_refclk_mux, abe_dpll_bypass_clk_mux_parents, NULL,
	       0x0, OMAP54XX_CM_CLKSEL_ABE_PLL_REF, OMAP54XX_CLKSEL_0_0_SHIFT,
	       OMAP54XX_CLKSEL_0_0_WIDTH, 0x0, NULL);

/* DPLL_ABE */
static struct dpll_data dpll_abe_dd = {
	.mult_div1_reg	= OMAP54XX_CM_CLKSEL_DPLL_ABE,
	.clk_bypass	= &abe_dpll_bypass_clk_mux,
	.clk_ref	= &abe_dpll_refclk_mux,
	.control_reg	= OMAP54XX_CM_CLKMODE_DPLL_ABE,
	.modes		= (1 << DPLL_LOW_POWER_BYPASS) | (1 << DPLL_LOCKED),
	.autoidle_reg	= OMAP54XX_CM_AUTOIDLE_DPLL_ABE,
	.idlest_reg	= OMAP54XX_CM_IDLEST_DPLL_ABE,
	.mult_mask	= OMAP54XX_DPLL_MULT_MASK,
	.div1_mask	= OMAP54XX_DPLL_DIV_MASK,
	.enable_mask	= OMAP54XX_DPLL_EN_MASK,
	.autoidle_mask	= OMAP54XX_AUTO_DPLL_MODE_MASK,
	.idlest_mask	= OMAP54XX_ST_DPLL_CLK_MASK,
	.max_multiplier	= 2047,
	.max_divider	= 128,
	.min_divider	= 1,
};


static const char *dpll_abe_ck_parents[] = {
	"abe_dpll_refclk_mux",
};

static struct clk dpll_abe_ck;

static const struct clk_ops dpll_abe_ck_ops = {
	.enable		= &omap3_noncore_dpll_enable,
	.disable	= &omap3_noncore_dpll_disable,
	.recalc_rate	= &omap4_dpll_regm4xen_recalc,
	.round_rate	= &omap4_dpll_regm4xen_round_rate,
	.set_rate	= &omap3_noncore_dpll_set_rate,
	.get_parent	= &omap2_init_dpll_parent,
};

static struct clk_hw_omap dpll_abe_ck_hw = {
	.hw = {
		.clk = &dpll_abe_ck,
	},
	.dpll_data	= &dpll_abe_dd,
	.ops		= &clkhwops_omap3_dpll,
};

DEFINE_STRUCT_CLK(dpll_abe_ck, dpll_abe_ck_parents, dpll_abe_ck_ops);

static const char *dpll_abe_x2_ck_parents[] = {
	"dpll_abe_ck",
};

static struct clk dpll_abe_x2_ck;

static const struct clk_ops dpll_abe_x2_ck_ops = {
	.recalc_rate	= &omap3_clkoutx2_recalc,
};

static struct clk_hw_omap dpll_abe_x2_ck_hw = {
	.hw = {
		.clk = &dpll_abe_x2_ck,
	},
	.flags		= CLOCK_CLKOUTX2,
	.ops		= &clkhwops_omap4_dpllmx,
};

DEFINE_STRUCT_CLK(dpll_abe_x2_ck, dpll_abe_x2_ck_parents, dpll_abe_x2_ck_ops);

static const struct clk_ops omap_hsdivider_ops = {
	.set_rate	= &omap2_clksel_set_rate,
	.recalc_rate	= &omap2_clksel_recalc,
	.round_rate	= &omap2_clksel_round_rate,
};

DEFINE_CLK_OMAP_HSDIVIDER(dpll_abe_m2x2_ck, "dpll_abe_x2_ck", &dpll_abe_x2_ck,
			  0x0, OMAP54XX_CM_DIV_M2_DPLL_ABE,
			  OMAP54XX_DIVHS_0_4_MASK);

DEFINE_CLK_FIXED_FACTOR(abe_24m_fclk, "dpll_abe_m2x2_ck", &dpll_abe_m2x2_ck,
			0x0, 1, 8);

DEFINE_CLK_DIVIDER(abe_clk, "dpll_abe_m2x2_ck", &dpll_abe_m2x2_ck, 0x0,
		   OMAP54XX_CM_CLKSEL_ABE, OMAP54XX_CLKSEL_OPP_0_1_SHIFT,
		   OMAP54XX_CLKSEL_OPP_0_1_WIDTH, CLK_DIVIDER_POWER_OF_TWO, NULL);

DEFINE_CLK_FIXED_FACTOR(abe_iclk, "abe_clk",
			&abe_clk, 0x0, 1, 2);

DEFINE_CLK_OMAP_HSDIVIDER(dpll_abe_m3x2_ck, "dpll_abe_x2_ck", &dpll_abe_x2_ck,
			  0x0, OMAP54XX_CM_DIV_M3_DPLL_ABE,
			  OMAP54XX_DIVHS_0_4_MASK);

/* DPLL_CORE */
static struct dpll_data dpll_core_dd = {
	.mult_div1_reg	= OMAP54XX_CM_CLKSEL_DPLL_CORE,
	.clk_bypass	= &dpll_abe_m3x2_ck,
	.clk_ref	= &sys_clkin,
	.control_reg	= OMAP54XX_CM_CLKMODE_DPLL_CORE,
	.modes		= (1 << DPLL_LOW_POWER_BYPASS) | (1 << DPLL_LOCKED),
	.autoidle_reg	= OMAP54XX_CM_AUTOIDLE_DPLL_CORE,
	.idlest_reg	= OMAP54XX_CM_IDLEST_DPLL_CORE,
	.mult_mask	= OMAP54XX_DPLL_MULT_MASK,
	.div1_mask	= OMAP54XX_DPLL_DIV_MASK,
	.enable_mask	= OMAP54XX_DPLL_EN_MASK,
	.autoidle_mask	= OMAP54XX_AUTO_DPLL_MODE_MASK,
	.idlest_mask	= OMAP54XX_ST_DPLL_CLK_MASK,
	.max_multiplier	= 2047,
	.max_divider	= 128,
	.min_divider	= 1,
};


static const char *dpll_core_ck_parents[] = {
	"sys_clkin",
};

static struct clk dpll_core_ck;

static const struct clk_ops dpll_core_ck_ops = {
	.recalc_rate	= &omap3_dpll_recalc,
	.get_parent	= &omap2_init_dpll_parent,
};

static struct clk_hw_omap dpll_core_ck_hw = {
	.hw = {
		.clk = &dpll_core_ck,
	},
	.dpll_data	= &dpll_core_dd,
	.ops		= &clkhwops_omap3_dpll,
};

DEFINE_STRUCT_CLK(dpll_core_ck, dpll_core_ck_parents, dpll_core_ck_ops);

static const char *dpll_core_x2_ck_parents[] = {
	"dpll_core_ck",
};

static struct clk dpll_core_x2_ck;

static struct clk_hw_omap dpll_core_x2_ck_hw = {
	.hw = {
		.clk = &dpll_core_x2_ck,
	},
};

DEFINE_STRUCT_CLK(dpll_core_x2_ck, dpll_core_x2_ck_parents, dpll_abe_x2_ck_ops);

DEFINE_CLK_OMAP5_HSDIVIDER(dpll_core_h12x2_ck, "dpll_core_x2_ck",
			  &dpll_core_x2_ck, 0x0, OMAP54XX_CM_DIV_H12_DPLL_CORE,
			  OMAP54XX_DIVHS_MASK);

DEFINE_CLK_OMAP_HSDIVIDER(dpll_core_h14x2_ck, "dpll_core_x2_ck",
			  &dpll_core_x2_ck, 0x0, OMAP54XX_CM_DIV_H14_DPLL_CORE,
			  OMAP54XX_DIVHS_MASK);

DEFINE_CLK_OMAP_HSDIVIDER(dpll_core_h22x2_ck, "dpll_core_x2_ck",
			  &dpll_core_x2_ck, 0x0, OMAP54XX_CM_DIV_H22_DPLL_CORE,
			  OMAP54XX_DIVHS_MASK);

DEFINE_CLK_OMAP_HSDIVIDER(div_iva_hs_clk, "dpll_core_h12x2_ck",
			  &dpll_core_h12x2_ck, 0x0, OMAP54XX_CM_BYPCLK_DPLL_IVA,
			  OMAP54XX_CLKSEL_0_1_MASK);

DEFINE_CLK_DIVIDER(div_mpu_hs_clk, "dpll_core_h12x2_ck", &dpll_core_h12x2_ck,
		   0x0, OMAP54XX_CM_BYPCLK_DPLL_MPU, OMAP54XX_CLKSEL_0_1_SHIFT,
		   OMAP54XX_CLKSEL_0_1_WIDTH, CLK_DIVIDER_POWER_OF_TWO, NULL);

DEFINE_CLK_OMAP5_HSDIVIDER(dpll_core_h13x2_ck, "dpll_core_x2_ck",
			  &dpll_core_x2_ck, 0x0, OMAP54XX_CM_DIV_H13_DPLL_CORE,
			  OMAP54XX_DIVHS_MASK);

/* DPLL_IVA */
static struct dpll_data dpll_iva_dd = {
	.mult_div1_reg	= OMAP54XX_CM_CLKSEL_DPLL_IVA,
	.clk_bypass	= &div_iva_hs_clk,
	.clk_ref	= &sys_clkin,
	.control_reg	= OMAP54XX_CM_CLKMODE_DPLL_IVA,
	.modes		= (1 << DPLL_LOW_POWER_BYPASS) | (1 << DPLL_LOCKED),
	.autoidle_reg	= OMAP54XX_CM_AUTOIDLE_DPLL_IVA,
	.idlest_reg	= OMAP54XX_CM_IDLEST_DPLL_IVA,
	.mult_mask	= OMAP54XX_DPLL_MULT_MASK,
	.div1_mask	= OMAP54XX_DPLL_DIV_MASK,
	.enable_mask	= OMAP54XX_DPLL_EN_MASK,
	.autoidle_mask	= OMAP54XX_AUTO_DPLL_MODE_MASK,
	.idlest_mask	= OMAP54XX_ST_DPLL_CLK_MASK,
	.max_multiplier	= 2047,
	.max_divider	= 128,
	.min_divider	= 1,
};

static struct clk dpll_iva_ck;

static struct clk_hw_omap dpll_iva_ck_hw = {
	.hw = {
		.clk = &dpll_iva_ck,
	},
	.dpll_data	= &dpll_iva_dd,
	.ops		= &clkhwops_omap3_dpll,
};

DEFINE_STRUCT_CLK(dpll_iva_ck, dpll_core_ck_parents, dpll_abe_ck_ops);

static const char *dpll_iva_x2_ck_parents[] = {
	"dpll_iva_ck",
};

static struct clk dpll_iva_x2_ck;

static struct clk_hw_omap dpll_iva_x2_ck_hw = {
	.hw = {
		.clk = &dpll_iva_x2_ck,
	},
};

DEFINE_STRUCT_CLK(dpll_iva_x2_ck, dpll_iva_x2_ck_parents, dpll_abe_x2_ck_ops);

DEFINE_CLK_OMAP5_HSDIVIDER(dpll_iva_h11x2_ck, "dpll_iva_x2_ck", &dpll_iva_x2_ck,
			  0x0, OMAP54XX_CM_DIV_H11_DPLL_IVA,
			  OMAP54XX_DIVHS_MASK);

DEFINE_CLK_OMAP_HSDIVIDER(dpll_iva_h12x2_ck, "dpll_iva_x2_ck", &dpll_iva_x2_ck,
			  0x0, OMAP54XX_CM_DIV_H12_DPLL_IVA,
			  OMAP54XX_DIVHS_MASK);

/* DPLL_MPU */
static struct dpll_data dpll_mpu_dd = {
	.mult_div1_reg	= OMAP54XX_CM_CLKSEL_DPLL_MPU,
	.clk_bypass	= &div_mpu_hs_clk,
	.clk_ref	= &sys_clkin,
	.control_reg	= OMAP54XX_CM_CLKMODE_DPLL_MPU,
	.modes		= (1 << DPLL_LOW_POWER_BYPASS) | (1 << DPLL_LOCKED),
	.autoidle_reg	= OMAP54XX_CM_AUTOIDLE_DPLL_MPU,
	.idlest_reg	= OMAP54XX_CM_IDLEST_DPLL_MPU,
	.mult_mask	= OMAP54XX_DPLL_MULT_MASK,
	.div1_mask	= OMAP54XX_DPLL_DIV_MASK,
	.enable_mask	= OMAP54XX_DPLL_EN_MASK,
	.autoidle_mask	= OMAP54XX_AUTO_DPLL_MODE_MASK,
	.idlest_mask	= OMAP54XX_ST_DPLL_CLK_MASK,
	.max_multiplier	= 2047,
	.max_divider	= 128,
	.min_divider	= 1,
};

static struct clk dpll_mpu_ck;

static struct clk_hw_omap dpll_mpu_ck_hw = {
	.hw = {
		.clk = &dpll_mpu_ck,
	},
	.dpll_data	= &dpll_mpu_dd,
	.ops		= &clkhwops_omap3_dpll,
};

DEFINE_STRUCT_CLK(dpll_mpu_ck, dpll_core_ck_parents, dpll_abe_ck_ops);

DEFINE_CLK_OMAP_HSDIVIDER(dpll_mpu_m2_ck, "dpll_mpu_ck", &dpll_mpu_ck, 0x0,
			  OMAP54XX_CM_DIV_M2_DPLL_MPU,
			  OMAP54XX_DIVHS_0_4_MASK);

DEFINE_CLK_FIXED_FACTOR(per_hs_clk_div, "dpll_abe_m3x2_ck",
			&dpll_abe_m3x2_ck, 0x0, 1, 2);

/* DPLL_PER */
static struct dpll_data dpll_per_dd = {
	.mult_div1_reg	= OMAP54XX_CM_CLKSEL_DPLL_PER,
	.clk_bypass	= &per_hs_clk_div,
	.clk_ref	= &sys_clkin,
	.control_reg	= OMAP54XX_CM_CLKMODE_DPLL_PER,
	.modes		= (1 << DPLL_LOW_POWER_BYPASS) | (1 << DPLL_LOCKED),
	.autoidle_reg	= OMAP54XX_CM_AUTOIDLE_DPLL_PER,
	.idlest_reg	= OMAP54XX_CM_IDLEST_DPLL_PER,
	.mult_mask	= OMAP54XX_DPLL_MULT_MASK,
	.div1_mask	= OMAP54XX_DPLL_DIV_MASK,
	.enable_mask	= OMAP54XX_DPLL_EN_MASK,
	.autoidle_mask	= OMAP54XX_AUTO_DPLL_MODE_MASK,
	.idlest_mask	= OMAP54XX_ST_DPLL_CLK_MASK,
	.max_multiplier	= 2047,
	.max_divider	= 128,
	.min_divider	= 1,
};


static struct clk dpll_per_ck;

static struct clk_hw_omap dpll_per_ck_hw = {
	.hw = {
		.clk = &dpll_per_ck,
	},
	.dpll_data	= &dpll_per_dd,
	.ops		= &clkhwops_omap3_dpll,
};

DEFINE_STRUCT_CLK(dpll_per_ck, dpll_core_ck_parents, dpll_abe_ck_ops);

static const char *dpll_per_x2_ck_parents[] = {
	"dpll_per_ck",
};

static struct clk dpll_per_x2_ck;

static struct clk_hw_omap dpll_per_x2_ck_hw = {
	.hw = {
		.clk = &dpll_per_x2_ck,
	},
	.flags		= CLOCK_CLKOUTX2,
	.ops		= &clkhwops_omap4_dpllmx,
};

DEFINE_STRUCT_CLK(dpll_per_x2_ck, dpll_per_x2_ck_parents, dpll_abe_x2_ck_ops);

DEFINE_CLK_OMAP_HSDIVIDER(dpll_per_h11x2_ck , "dpll_per_x2_ck", &dpll_per_x2_ck,
			  0x0, OMAP54XX_CM_DIV_H11_DPLL_PER,
			  OMAP54XX_DIVHS_MASK);

DEFINE_CLK_OMAP_HSDIVIDER(dpll_per_h12x2_ck , "dpll_per_x2_ck", &dpll_per_x2_ck,
			  0x0, OMAP54XX_CM_DIV_H12_DPLL_PER,
			  OMAP54XX_DIVHS_MASK);

DEFINE_CLK_OMAP_HSDIVIDER(dpll_per_h14x2_ck , "dpll_per_x2_ck", &dpll_per_x2_ck,
			  0x0, OMAP54XX_CM_DIV_H14_DPLL_PER,
			  OMAP54XX_DIVHS_MASK);

DEFINE_CLK_DIVIDER(dpll_per_m2_ck, "dpll_per_ck", &dpll_per_ck, 0x0,
		   OMAP54XX_CM_DIV_M2_DPLL_PER, OMAP54XX_DIVHS_0_4_SHIFT,
		   OMAP54XX_DIVHS_0_4_WIDTH, CLK_DIVIDER_ONE_BASED, NULL);

DEFINE_CLK_OMAP_HSDIVIDER(dpll_per_m2x2_ck, "dpll_per_x2_ck", &dpll_per_x2_ck,
			  0x0, OMAP54XX_CM_DIV_M2_DPLL_PER,
			  OMAP54XX_DIVHS_0_4_MASK);

/* DPLL_UNIPRO1 */

/* DPLL_UNIPRO2 */

DEFINE_CLK_FIXED_FACTOR(usb_hs_clk_div, "dpll_abe_m3x2_ck",
			&dpll_abe_m3x2_ck, 0x0, 1, 3);

/* DPLL_USB */
static struct dpll_data dpll_usb_dd = {
	.mult_div1_reg	= OMAP54XX_CM_CLKSEL_DPLL_USB,
	.clk_bypass	= &usb_hs_clk_div,
	.flags		= DPLL_J_TYPE,
	.clk_ref	= &sys_clkin,
	.control_reg	= OMAP54XX_CM_CLKMODE_DPLL_USB,
	.modes		= (1 << DPLL_LOW_POWER_BYPASS) | (1 << DPLL_LOCKED),
	.autoidle_reg	= OMAP54XX_CM_AUTOIDLE_DPLL_USB,
	.idlest_reg	= OMAP54XX_CM_IDLEST_DPLL_USB,
	.mult_mask	= OMAP54XX_DPLL_MULT_MASK,
	.div1_mask	= OMAP54XX_DPLL_DIV_MASK,
	.enable_mask	= OMAP54XX_DPLL_EN_MASK,
	.autoidle_mask	= OMAP54XX_AUTO_DPLL_MODE_MASK,
	.idlest_mask	= OMAP54XX_ST_DPLL_CLK_MASK,
	.sddiv_mask	= OMAP54XX_DPLL_SD_DIV_MASK,
	.max_multiplier	= 4095,
	.max_divider	= 256,
	.min_divider	= 1,
};

static struct clk dpll_usb_ck;

static struct clk_hw_omap dpll_usb_ck_hw = {
	.hw = {
		.clk = &dpll_usb_ck,
	},
	.dpll_data	= &dpll_usb_dd,
	.ops		= &clkhwops_omap3_dpll,
};

DEFINE_STRUCT_CLK(dpll_usb_ck, dpll_core_ck_parents, dpll_abe_ck_ops);

DEFINE_CLK_OMAP_HSDIVIDER(dpll_usb_m2_ck, "dpll_usb_ck", &dpll_usb_ck, 0x0,
			  OMAP54XX_CM_DIV_M2_DPLL_USB,
			  OMAP54XX_DIVHS_0_6_MASK);

DEFINE_CLK_FIXED_FACTOR(func_12m_fclk, "dpll_per_m2x2_ck", &dpll_per_m2x2_ck,
			0x0, 1, 16);

DEFINE_CLK_FIXED_FACTOR(func_24m_clk, "dpll_per_m2_ck", &dpll_per_m2_ck, 0x0,
			1, 4);

DEFINE_CLK_FIXED_FACTOR(func_24m_fclk, "dpll_per_m2x2_ck", &dpll_per_m2x2_ck,
			0x0, 1, 8);

DEFINE_CLK_FIXED_FACTOR(func_48m_fclk, "dpll_per_m2x2_ck", &dpll_per_m2x2_ck,
			0x0, 1, 4);

DEFINE_CLK_FIXED_FACTOR(func_48mc_fclk,	"dpll_per_m2x2_ck", &dpll_per_m2x2_ck,
			0x0, 1, 4);

DEFINE_CLK_FIXED_FACTOR(func_96m_fclk,	"dpll_per_m2x2_ck", &dpll_per_m2x2_ck,
			0x0, 1, 2);

DEFINE_CLK_DIVIDER(l3_div_ck, "dpll_core_h12x2_ck", &dpll_core_h12x2_ck, 0x0,
		   OMAP54XX_CM_CLKSEL_CORE, OMAP54XX_CLKSEL_L3_SHIFT,
		   OMAP54XX_CLKSEL_L3_WIDTH, 0x0, NULL);

DEFINE_CLK_DIVIDER(l3init_60m_fclk, "dpll_usb_m2_ck", &dpll_usb_m2_ck, 0x0,
		   OMAP54XX_CM_CLKSEL_USB_60MHZ, OMAP54XX_CLKSEL_0_0_SHIFT,
		   OMAP54XX_CLKSEL_0_0_WIDTH, 0x0, NULL);

DEFINE_CLK_DIVIDER(l4_div_ck, "l3_div_ck", &l3_div_ck, 0x0,
		   OMAP54XX_CM_CLKSEL_CORE, OMAP54XX_CLKSEL_L4_SHIFT,
		   OMAP54XX_CLKSEL_L4_WIDTH, 0x0, NULL);

DEFINE_CLK_FIXED_FACTOR(lp_clk_div, "dpll_abe_m2x2_ck", &dpll_abe_m2x2_ck,
			0x0, 1, 16);

DEFINE_CLK_FIXED_FACTOR(syc_clk_div, "sys_clkin", &sys_clkin,
			0x0, 1, 1);

static const char *wkupaon_clk_mux_parents[] = {
	"sys_clkin", "lp_clk_div",
};

DEFINE_CLK_MUX(wkupaon_clk_mux, wkupaon_clk_mux_parents, NULL, 0x0,
	       OMAP54XX_CM_CLKSEL_WKUPAON, OMAP54XX_CLKSEL_0_0_SHIFT,
	       OMAP54XX_CLKSEL_0_0_WIDTH, 0x0, NULL);


/* Leaf clocks controlled by modules */

DEFINE_CLK_GATE(usb_host_hs_hsic60m_p2_clk, "l3init_60m_fclk",
		&l3init_60m_fclk, 0x0,
		OMAP54XX_CM_L3INIT_USB_HOST_HS_CLKCTRL,
		OMAP54XX_OPTFCLKEN_HSIC60M_P2_CLK_SHIFT, 0x0, NULL);

DEFINE_CLK_GATE(usb_host_hs_hsic60m_p3_clk, "l3init_60m_fclk",
		&l3init_60m_fclk, 0x0,
		OMAP54XX_CM_L3INIT_USB_HOST_HS_CLKCTRL,
		OMAP54XX_OPTFCLKEN_HSIC60M_P3_CLK_SHIFT, 0x0, NULL);

static const char *utmi_p1_gfclk_parents[] = {
	"l3init_60m_fclk", "xclk60mhsp1",
};

DEFINE_CLK_MUX(utmi_p1_gfclk, utmi_p1_gfclk_parents, NULL, 0x0,
	       OMAP54XX_CM_L3INIT_USB_HOST_HS_CLKCTRL,
	       OMAP54XX_CLKSEL_UTMI_P1_SHIFT, OMAP54XX_CLKSEL_UTMI_P1_WIDTH,
	       0x0, NULL);

DEFINE_CLK_GATE(usb_host_hs_utmi_p1_clk, "utmi_p1_gfclk", &utmi_p1_gfclk, 0x0,
		OMAP54XX_CM_L3INIT_USB_HOST_HS_CLKCTRL,
		OMAP54XX_OPTFCLKEN_UTMI_P1_CLK_SHIFT, 0x0, NULL);

static const char *utmi_p2_gfclk_parents[] = {
	"l3init_60m_fclk", "xclk60mhsp2",
};

DEFINE_CLK_MUX(utmi_p2_gfclk, utmi_p2_gfclk_parents, NULL, 0x0,
	       OMAP54XX_CM_L3INIT_USB_HOST_HS_CLKCTRL,
	       OMAP54XX_CLKSEL_UTMI_P2_SHIFT, OMAP54XX_CLKSEL_UTMI_P2_WIDTH,
	       0x0, NULL);

DEFINE_CLK_GATE(usb_host_hs_utmi_p2_clk, "utmi_p2_gfclk", &utmi_p2_gfclk, 0x0,
		OMAP54XX_CM_L3INIT_USB_HOST_HS_CLKCTRL,
		OMAP54XX_OPTFCLKEN_UTMI_P2_CLK_SHIFT, 0x0, NULL);

DEFINE_CLK_GATE(usb_host_hs_utmi_p3_clk, "l3init_60m_fclk", &l3init_60m_fclk, 0x0,
		OMAP54XX_CM_L3INIT_USB_HOST_HS_CLKCTRL,
		OMAP54XX_OPTFCLKEN_UTMI_P3_CLK_SHIFT, 0x0, NULL);

DEFINE_CLK_GATE(usb_host_hs_hsic480m_p1_clk, "dpll_usb_m2_ck",
		&dpll_usb_m2_ck, 0x0,
		OMAP54XX_CM_L3INIT_USB_HOST_HS_CLKCTRL,
		OMAP54XX_OPTFCLKEN_HSIC480M_P1_CLK_SHIFT, 0x0, NULL);

DEFINE_CLK_GATE(usb_host_hs_hsic60m_p1_clk, "l3init_60m_fclk",
		&l3init_60m_fclk, 0x0,
		OMAP54XX_CM_L3INIT_USB_HOST_HS_CLKCTRL,
		OMAP54XX_OPTFCLKEN_HSIC60M_P1_CLK_SHIFT, 0x0, NULL);

DEFINE_CLK_GATE(usb_host_hs_hsic480m_p3_clk, "dpll_usb_m2_ck",
		&dpll_usb_m2_ck, 0x0,
		OMAP54XX_CM_L3INIT_USB_HOST_HS_CLKCTRL,
		OMAP54XX_OPTFCLKEN_HSIC480M_P3_CLK_SHIFT, 0x0, NULL);

DEFINE_CLK_GATE(usb_host_hs_hsic480m_p2_clk, "dpll_usb_m2_ck",
		&dpll_usb_m2_ck, 0x0,
		OMAP54XX_CM_L3INIT_USB_HOST_HS_CLKCTRL,
		OMAP54XX_OPTFCLKEN_HSIC480M_P2_CLK_SHIFT, 0x0, NULL);

DEFINE_CLK_GATE(sata_ref_clk, "sys_clkin", &sys_clkin, 0x0,
		OMAP54XX_CM_L3INIT_SATA_CLKCTRL,
		OMAP54XX_OPTFCLKEN_REF_CLK_SHIFT, 0x0, NULL);

DEFINE_CLK_GATE(dss_32khz_clk, "sys_32k_ck", &sys_32k_ck, 0x0,
		OMAP54XX_CM_DSS_DSS_CLKCTRL,
		OMAP54XX_OPTFCLKEN_32KHZ_CLK_SHIFT, 0x0, NULL);

DEFINE_CLK_GATE(dss_sys_clk, "syc_clk_div", &syc_clk_div, 0x0,
		OMAP54XX_CM_DSS_DSS_CLKCTRL,
		OMAP54XX_OPTFCLKEN_SYS_CLK_SHIFT, 0x0, NULL);

DEFINE_CLK_GATE(dss_48mhz_clk, "func_48m_fclk", &func_48m_fclk, 0x0,
		OMAP54XX_CM_DSS_DSS_CLKCTRL, OMAP54XX_OPTFCLKEN_48MHZ_CLK_SHIFT,
		0x0, NULL);

DEFINE_CLK_GATE(dss_dss_clk, "dpll_per_h12x2_ck", &dpll_per_h12x2_ck, 0x0,
		OMAP54XX_CM_DSS_DSS_CLKCTRL, OMAP54XX_OPTFCLKEN_DSSCLK_SHIFT,
		0x0, NULL);

DEFINE_CLK_GATE(gpio1_dbclk, "sys_32k_ck", &sys_32k_ck, 0x0,
		OMAP54XX_CM_WKUPAON_GPIO1_CLKCTRL,
		OMAP54XX_OPTFCLKEN_DBCLK_SHIFT,	0x0, NULL);

DEFINE_CLK_GATE(gpio2_dbclk, "sys_32k_ck", &sys_32k_ck, 0x0,
		OMAP54XX_CM_L4PER_GPIO2_CLKCTRL, OMAP54XX_OPTFCLKEN_DBCLK_SHIFT,
		0x0, NULL);

DEFINE_CLK_GATE(gpio3_dbclk, "sys_32k_ck", &sys_32k_ck, 0x0,
		OMAP54XX_CM_L4PER_GPIO3_CLKCTRL,
		OMAP54XX_OPTFCLKEN_DBCLK_SHIFT, 0x0, NULL);

DEFINE_CLK_GATE(gpio4_dbclk, "sys_32k_ck", &sys_32k_ck, 0x0,
		OMAP54XX_CM_L4PER_GPIO4_CLKCTRL, OMAP54XX_OPTFCLKEN_DBCLK_SHIFT,
		0x0, NULL);

DEFINE_CLK_GATE(gpio5_dbclk, "sys_32k_ck", &sys_32k_ck, 0x0,
		OMAP54XX_CM_L4PER_GPIO5_CLKCTRL, OMAP54XX_OPTFCLKEN_DBCLK_SHIFT,
		0x0, NULL);

DEFINE_CLK_GATE(gpio6_dbclk, "sys_32k_ck", &sys_32k_ck, 0x0,
		OMAP54XX_CM_L4PER_GPIO6_CLKCTRL, OMAP54XX_OPTFCLKEN_DBCLK_SHIFT,
		0x0, NULL);

DEFINE_CLK_GATE(gpio7_dbclk, "sys_32k_ck", &sys_32k_ck, 0x0,
		OMAP54XX_CM_L4PER_GPIO7_CLKCTRL, OMAP54XX_OPTFCLKEN_DBCLK_SHIFT,
		0x0, NULL);

DEFINE_CLK_GATE(gpio8_dbclk, "sys_32k_ck", &sys_32k_ck, 0x0,
		OMAP54XX_CM_L4PER_GPIO8_CLKCTRL, OMAP54XX_OPTFCLKEN_DBCLK_SHIFT,
		0x0, NULL);

/* Remaining optional clocks */
DEFINE_CLK_DIVIDER(aess_fclk, "abe_clk", &abe_clk, 0x0,
		   OMAP54XX_CM_ABE_AESS_CLKCTRL,
		   OMAP54XX_CLKSEL_AESS_FCLK_SHIFT,
		   OMAP54XX_CLKSEL_AESS_FCLK_WIDTH,
		   0x0, NULL);

static const char *dmic_sync_mux_ck_parents[] = {
	"abe_24m_fclk", "syc_clk_div_ck", "func_24m_clk",
};

DEFINE_CLK_MUX(dmic_sync_mux_ck, dmic_sync_mux_ck_parents, NULL,
	       0x0, OMAP54XX_CM_ABE_DMIC_CLKCTRL,
	       OMAP54XX_CLKSEL_INTERNAL_SOURCE_SHIFT,
	       OMAP54XX_CLKSEL_INTERNAL_SOURCE_WIDTH, 0x0, NULL);

static const struct clksel dmic_gfclk_sel[] = {
	{ .parent = &dmic_sync_mux_ck, .rates = div_1_0_rates },
	{ .parent = &pad_clks_ck, .rates = div_1_1_rates },
	{ .parent = &slimbus_clk, .rates = div_1_2_rates },
	{ .parent = NULL },
};

static const char *dmic_fck_parents[] = {
	"dmic_sync_mux_ck", "pad_clks_ck", "slimbus_clk",
};

static const struct clk_ops dmic_fck_ops = {
	.enable		= &omap2_dflt_clk_enable,
	.disable	= &omap2_dflt_clk_disable,
	.is_enabled	= &omap2_dflt_clk_is_enabled,
	.recalc_rate	= &omap2_clksel_recalc,
	.get_parent	= &omap2_clksel_find_parent_index,
	.set_parent	= &omap2_clksel_set_parent,
	.init		= &omap2_init_clk_clkdm,
};

/* Merged func_dmic_abe_gfclk into dmic */
static struct clk dmic_gfclk;

DEFINE_CLK_OMAP_MUX_GATE(dmic_gfclk, "abe_clk", dmic_gfclk_sel,
			 OMAP54XX_CM_ABE_DMIC_CLKCTRL,
			 OMAP54XX_CLKSEL_SOURCE_MASK,
			 OMAP54XX_CM_ABE_DMIC_CLKCTRL,
			 OMAP54XX_MODULEMODE_SWCTRL, NULL,
			 dmic_fck_parents, dmic_fck_ops);

DEFINE_CLK_MUX(mcasp_sync_mux_ck, dmic_sync_mux_ck_parents, NULL, 0x0,
	       OMAP54XX_CM_ABE_MCASP_CLKCTRL,
	       OMAP54XX_CLKSEL_INTERNAL_SOURCE_SHIFT,
	       OMAP54XX_CLKSEL_INTERNAL_SOURCE_WIDTH, 0x0, NULL);

static const struct clksel mcasp_gfclk_sel[] = {
	{ .parent = &mcasp_sync_mux_ck, .rates = div_1_0_rates },
	{ .parent = &pad_clks_ck, .rates = div_1_1_rates },
	{ .parent = &slimbus_clk, .rates = div_1_2_rates },
	{ .parent = NULL },
};

static const char *mcasp_fck_parents[] = {
	"mcasp_sync_mux_ck", "pad_clks_ck", "slimbus_clk",
};

/* Merged func_mcasp_abe_gfclk into mcasp */
DEFINE_CLK_OMAP_MUX_GATE(mcasp_gfclk, "abe_clk", mcasp_gfclk_sel,
			 OMAP54XX_CM_ABE_MCASP_CLKCTRL,
			 OMAP54XX_CLKSEL_SOURCE_MASK,
			 OMAP54XX_CM_ABE_MCASP_CLKCTRL,
			 OMAP54XX_MODULEMODE_SWCTRL, NULL,
			 mcasp_fck_parents, dmic_fck_ops);

DEFINE_CLK_MUX(mcbsp1_sync_mux_ck, dmic_sync_mux_ck_parents, NULL, 0x0,
	       OMAP54XX_CM_ABE_MCBSP1_CLKCTRL,
	       OMAP54XX_CLKSEL_INTERNAL_SOURCE_SHIFT,
	       OMAP54XX_CLKSEL_INTERNAL_SOURCE_WIDTH, 0x0, NULL);

static const struct clksel mcbsp1_gfclk_sel[] = {
	{ .parent = &mcbsp1_sync_mux_ck, .rates = div_1_0_rates },
	{ .parent = &pad_clks_ck, .rates = div_1_1_rates },
	{ .parent = &slimbus_clk, .rates = div_1_2_rates },
	{ .parent = NULL },
};

static const char *mcbsp1_fck_parents[] = {
	"mcbsp1_sync_mux_ck", "pad_clks_ck", "slimbus_clk",
};

/* Merged func_mcbsp1_gfclk into mcbsp1 */
DEFINE_CLK_OMAP_MUX_GATE(mcbsp1_gfclk, "abe_clk", mcbsp1_gfclk_sel,
			 OMAP54XX_CM_ABE_MCBSP1_CLKCTRL,
			 OMAP54XX_CLKSEL_SOURCE_MASK,
			 OMAP54XX_CM_ABE_MCBSP1_CLKCTRL,
			 OMAP54XX_MODULEMODE_SWCTRL, NULL,
			 mcbsp1_fck_parents, dmic_fck_ops);

DEFINE_CLK_MUX(mcbsp2_sync_mux_ck, dmic_sync_mux_ck_parents, NULL, 0x0,
	       OMAP54XX_CM_ABE_MCBSP2_CLKCTRL,
	       OMAP54XX_CLKSEL_INTERNAL_SOURCE_SHIFT,
	       OMAP54XX_CLKSEL_INTERNAL_SOURCE_WIDTH, 0x0, NULL);

static const struct clksel mcbsp2_gfclk_sel[] = {
	{ .parent = &mcbsp2_sync_mux_ck, .rates = div_1_0_rates },
	{ .parent = &pad_clks_ck, .rates = div_1_1_rates },
	{ .parent = &slimbus_clk, .rates = div_1_2_rates },
	{ .parent = NULL },
};

static const char *mcbsp2_fck_parents[] = {
	"mcbsp2_sync_mux_ck", "pad_clks_ck", "slimbus_clk",
};

/* Merged func_mcbsp2_gfclk into mcbsp2 */
DEFINE_CLK_OMAP_MUX_GATE(mcbsp2_gfclk, "abe_clk", mcbsp2_gfclk_sel,
			 OMAP54XX_CM_ABE_MCBSP2_CLKCTRL,
			 OMAP54XX_CLKSEL_SOURCE_MASK,
			 OMAP54XX_CM_ABE_MCBSP2_CLKCTRL,
			 OMAP54XX_MODULEMODE_SWCTRL, NULL,
			 mcbsp2_fck_parents, dmic_fck_ops);

DEFINE_CLK_MUX(mcbsp3_sync_mux_ck, dmic_sync_mux_ck_parents, NULL, 0x0,
	       OMAP54XX_CM_ABE_MCBSP3_CLKCTRL,
	       OMAP54XX_CLKSEL_INTERNAL_SOURCE_SHIFT,
	       OMAP54XX_CLKSEL_INTERNAL_SOURCE_WIDTH, 0x0, NULL);

static const struct clksel mcbsp3_gfclk_sel[] = {
	{ .parent = &mcbsp3_sync_mux_ck, .rates = div_1_0_rates },
	{ .parent = &pad_clks_ck, .rates = div_1_1_rates },
	{ .parent = &slimbus_clk, .rates = div_1_2_rates },
	{ .parent = NULL },
};

static const char *mcbsp3_fck_parents[] = {
	"mcbsp3_sync_mux_ck", "pad_clks_ck", "slimbus_clk",
};

/* Merged func_mcbsp3_gfclk into mcbsp3 */
DEFINE_CLK_OMAP_MUX_GATE(mcbsp3_gfclk, "abe_clk", mcbsp3_gfclk_sel,
			 OMAP54XX_CM_ABE_MCBSP3_CLKCTRL,
			 OMAP54XX_CLKSEL_SOURCE_MASK,
			 OMAP54XX_CM_ABE_MCBSP3_CLKCTRL,
			 OMAP54XX_MODULEMODE_SWCTRL, NULL,
			 mcbsp3_fck_parents, dmic_fck_ops);


static const char *gpu_core_clk_mux_parents[] = {
	"dpll_core_h14x2_ck", "dpll_per_h14x2_ck",
};

DEFINE_CLK_MUX(gpu_core_clk_mux, gpu_core_clk_mux_parents, NULL, 0x0,
	       OMAP54XX_CM_GPU_GPU_CLKCTRL, OMAP54XX_CLKSEL_GPU_CORE_GCLK_SHIFT,
	       OMAP54XX_CLKSEL_GPU_CORE_GCLK_WIDTH, 0x0, NULL);

DEFINE_CLK_MUX(gpu_hyd_clk_mux, gpu_core_clk_mux_parents, NULL, 0x0,
	       OMAP54XX_CM_GPU_GPU_CLKCTRL, OMAP54XX_CLKSEL_GPU_HYD_GCLK_SHIFT,
	       OMAP54XX_CLKSEL_GPU_HYD_GCLK_WIDTH, 0x0, NULL);

static const char *mmc1_fclk_mux_parents[] = {
	"dpll_per_h11x2_ck", "dpll_per_m2x2_ck",
};

DEFINE_CLK_MUX(mmc1_fclk_mux, mmc1_fclk_mux_parents, NULL, 0x0,
	       OMAP54XX_CM_L3INIT_MMC1_CLKCTRL, OMAP54XX_CLKSEL_SOURCE_24_24_SHIFT,
	       OMAP54XX_CLKSEL_SOURCE_24_24_WIDTH, 0x0, NULL);

DEFINE_CLK_DIVIDER(mmc1_fclk, "mmc1_fclk_mux", &mmc1_fclk_mux,
		   0x0, OMAP54XX_CM_L3INIT_MMC1_CLKCTRL, OMAP54XX_CLKSEL_DIV_SHIFT,
		   OMAP54XX_CLKSEL_DIV_WIDTH, CLK_DIVIDER_ONE_BASED, NULL);

DEFINE_CLK_MUX(mmc2_fclk_mux, mmc1_fclk_mux_parents, NULL, 0x0,
	       OMAP54XX_CM_L3INIT_MMC2_CLKCTRL, OMAP54XX_CLKSEL_SOURCE_24_24_SHIFT,
	       OMAP54XX_CLKSEL_SOURCE_24_24_WIDTH, 0x0, NULL);

DEFINE_CLK_DIVIDER(mmc2_fclk, "mmc2_fclk_mux", &mmc2_fclk_mux,
		   0x0, OMAP54XX_CM_L3INIT_MMC2_CLKCTRL, OMAP54XX_CLKSEL_DIV_SHIFT,
		   OMAP54XX_CLKSEL_DIV_WIDTH, CLK_DIVIDER_ONE_BASED, NULL);

DEFINE_CLK_MUX(timer10_clk_mux, abe_dpll_bypass_clk_mux_parents, NULL, 0x0,
	       OMAP54XX_CM_L4PER_TIMER10_CLKCTRL, OMAP54XX_CLKSEL_SHIFT,
	       OMAP54XX_CLKSEL_WIDTH, 0x0, NULL);

DEFINE_CLK_MUX(timer11_clk_mux, abe_dpll_bypass_clk_mux_parents, NULL, 0x0,
	       OMAP54XX_CM_L4PER_TIMER11_CLKCTRL, OMAP54XX_CLKSEL_SHIFT,
	       OMAP54XX_CLKSEL_WIDTH, 0x0, NULL);

DEFINE_CLK_MUX(timer1_clk_mux, abe_dpll_bypass_clk_mux_parents, NULL, 0x0,
	       OMAP54XX_CM_WKUPAON_TIMER1_CLKCTRL, OMAP54XX_CLKSEL_SHIFT,
	       OMAP54XX_CLKSEL_WIDTH, 0x0, NULL);

DEFINE_CLK_MUX(timer2_clk_mux, abe_dpll_bypass_clk_mux_parents, NULL, 0x0,
	       OMAP54XX_CM_L4PER_TIMER2_CLKCTRL, OMAP54XX_CLKSEL_SHIFT,
	       OMAP54XX_CLKSEL_WIDTH, 0x0, NULL);

DEFINE_CLK_MUX(timer3_clk_mux, abe_dpll_bypass_clk_mux_parents, NULL, 0x0,
	       OMAP54XX_CM_L4PER_TIMER3_CLKCTRL, OMAP54XX_CLKSEL_SHIFT,
	       OMAP54XX_CLKSEL_WIDTH, 0x0, NULL);

DEFINE_CLK_MUX(timer4_clk_mux, abe_dpll_bypass_clk_mux_parents, NULL, 0x0,
	       OMAP54XX_CM_L4PER_TIMER4_CLKCTRL, OMAP54XX_CLKSEL_SHIFT,
	       OMAP54XX_CLKSEL_WIDTH, 0x0, NULL);

static const char *timer5_fck_parents[] = {
	"syc_clk_div_ck", "sys_32k_ck",
};

DEFINE_CLK_MUX(timer5_sync_mux_ck, timer5_fck_parents, NULL, 0x0,
	       OMAP54XX_CM_ABE_TIMER5_CLKCTRL, OMAP54XX_CLKSEL_SHIFT,
	       OMAP54XX_CLKSEL_WIDTH, 0x0, NULL);

DEFINE_CLK_MUX(timer6_sync_mux_ck, timer5_fck_parents, NULL, 0x0,
	       OMAP54XX_CM_ABE_TIMER6_CLKCTRL, OMAP54XX_CLKSEL_SHIFT,
	       OMAP54XX_CLKSEL_WIDTH, 0x0, NULL);

DEFINE_CLK_MUX(timer7_sync_mux_ck, timer5_fck_parents, NULL, 0x0,
	       OMAP54XX_CM_ABE_TIMER7_CLKCTRL, OMAP54XX_CLKSEL_SHIFT,
	       OMAP54XX_CLKSEL_WIDTH, 0x0, NULL);

DEFINE_CLK_MUX(timer8_sync_mux_ck, timer5_fck_parents, NULL, 0x0,
	       OMAP54XX_CM_ABE_TIMER8_CLKCTRL, OMAP54XX_CLKSEL_SHIFT,
	       OMAP54XX_CLKSEL_WIDTH, 0x0, NULL);

DEFINE_CLK_MUX(timer9_clk_mux, abe_dpll_bypass_clk_mux_parents, NULL, 0x0,
	       OMAP54XX_CM_L4PER_TIMER9_CLKCTRL, OMAP54XX_CLKSEL_SHIFT,
	       OMAP54XX_CLKSEL_WIDTH, 0x0, NULL);

/*
 * clkdev
 */

static struct omap_clk omap54xx_clks[] = {
	CLK(NULL,	"pad_clks_ck",			&pad_clks_ck,	CK_54XX),
	CLK(NULL,	"pad_clks",			&pad_clks_ck,	CK_54XX),
	CLK(NULL,	"pad_slimbus_core_clks_ck",	&pad_slimbus_core_clks_ck,	CK_54XX),
	CLK(NULL,	"slimbus_clk",			&slimbus_clk,	CK_54XX),
	CLK(NULL,	"sys_32k_ck",			&sys_32k_ck,	CK_54XX),
	CLK(NULL,	"virt_12000000_ck",		&virt_12000000_ck,	CK_54XX),
	CLK(NULL,	"virt_13000000_ck",		&virt_13000000_ck,	CK_54XX),
	CLK(NULL,	"virt_16800000_ck",		&virt_16800000_ck,	CK_54XX),
	CLK(NULL,	"virt_19200000_ck",		&virt_19200000_ck,	CK_54XX),
	CLK(NULL,	"virt_26000000_ck",		&virt_26000000_ck,	CK_54XX),
	CLK(NULL,	"virt_27000000_ck",		&virt_27000000_ck,	CK_54XX),
	CLK(NULL,	"virt_38400000_ck",		&virt_38400000_ck,	CK_54XX),
	CLK(NULL,	"sys_clkin",			&sys_clkin,	CK_54XX),
	CLK(NULL,	"xclk60mhsp1",			&xclk60mhsp1,	CK_54XX),
	CLK(NULL,	"xclk60mhsp1_ck",		&xclk60mhsp1,	CK_54XX),
	CLK(NULL,	"xclk60mhsp2",			&xclk60mhsp2,	CK_54XX),
	CLK(NULL,	"xclk60mhsp2_ck",		&xclk60mhsp2,	CK_54XX),
	CLK(NULL,	"abe_dpll_bypass_clk_mux",	&abe_dpll_bypass_clk_mux,	CK_54XX),
	CLK(NULL,	"abe_dpll_refclk_mux",		&abe_dpll_refclk_mux,	CK_54XX),
	CLK(NULL,	"dpll_abe_ck",			&dpll_abe_ck,	CK_54XX),
	CLK(NULL,	"dpll_abe_x2_ck",		&dpll_abe_x2_ck,	CK_54XX),
	CLK(NULL,	"dpll_abe_m2x2_ck",		&dpll_abe_m2x2_ck,	CK_54XX),
	CLK(NULL,	"abe_24m_fclk",			&abe_24m_fclk,	CK_54XX),
	CLK(NULL,	"abe_clk",			&abe_clk,	CK_54XX),
	CLK(NULL,	"abe_iclk",			&abe_iclk,	CK_54XX),
	CLK(NULL,	"dpll_abe_m3x2_ck",		&dpll_abe_m3x2_ck,	CK_54XX),
	CLK(NULL,	"dpll_core_ck",			&dpll_core_ck,	CK_54XX),
	CLK(NULL,	"dpll_core_x2_ck",		&dpll_core_x2_ck,	CK_54XX),
	CLK(NULL,	"dpll_core_h12x2_ck",		&dpll_core_h12x2_ck,	CK_54XX),
	CLK(NULL,	"div_iva_hs_clk",		&div_iva_hs_clk,	CK_54XX),
	CLK(NULL,	"div_mpu_hs_clk",		&div_mpu_hs_clk,	CK_54XX),
	CLK(NULL,	"dpll_core_h13x2_ck",		&dpll_core_h13x2_ck,	CK_54XX),
	CLK(NULL,	"dpll_core_h22x2_ck",		&dpll_core_h22x2_ck,	CK_54XX),
	CLK(NULL,	"dpll_iva_ck",			&dpll_iva_ck,	CK_54XX),
	CLK(NULL,	"dpll_iva_x2_ck",		&dpll_iva_x2_ck,	CK_54XX),
	CLK(NULL,	"dpll_iva_h11x2_ck",		&dpll_iva_h11x2_ck,	CK_54XX),
	CLK(NULL,	"dpll_iva_h12x2_ck",		&dpll_iva_h12x2_ck,	CK_54XX),
	CLK(NULL,	"dpll_mpu_ck",			&dpll_mpu_ck,	CK_54XX),
	CLK(NULL,	"dpll_mpu_m2_ck",		&dpll_mpu_m2_ck,	CK_54XX),
	CLK(NULL,	"per_hs_clk_div",		&per_hs_clk_div,	CK_54XX),
	CLK(NULL,	"dpll_per_ck",			&dpll_per_ck,	CK_54XX),
	CLK(NULL,	"dpll_per_x2_ck",		&dpll_per_x2_ck,	CK_54XX),
	CLK(NULL,	"dpll_per_h11x2_ck",		&dpll_per_h11x2_ck,	CK_54XX),
	CLK(NULL,	"dpll_per_h12x2_ck",		&dpll_per_h12x2_ck,	CK_54XX),
	CLK(NULL,	"dpll_per_h14x2_ck",		&dpll_per_h14x2_ck,	CK_54XX),
	CLK(NULL,	"dpll_per_m2_ck",		&dpll_per_m2_ck,	CK_54XX),
	CLK(NULL,	"dpll_per_m2x2_ck",		&dpll_per_m2x2_ck,	CK_54XX),
	CLK(NULL,	"usb_hs_clk_div",		&usb_hs_clk_div,	CK_54XX),
	CLK(NULL,	"dpll_usb_ck",			&dpll_usb_ck,	CK_54XX),
	CLK(NULL,	"dpll_usb_m2_ck",		&dpll_usb_m2_ck,	CK_54XX),
	CLK(NULL,	"func_12m_fclk",		&func_12m_fclk,	CK_54XX),
	CLK(NULL,	"func_24m_clk",			&func_24m_clk,	CK_54XX),
	CLK(NULL,	"func_24m_fclk",		&func_24m_fclk,	CK_54XX),
	CLK(NULL,	"func_48m_fclk",		&func_48m_fclk,	CK_54XX),
	CLK(NULL,	"func_96m_fclk",		&func_96m_fclk,	CK_54XX),
	CLK(NULL,	"l3_div_ck",			&l3_div_ck,	CK_54XX),
	CLK(NULL,	"l3init_60m_fclk",		&l3init_60m_fclk,	CK_54XX),
	CLK(NULL,	"init_60m_fclk",		&l3init_60m_fclk,	CK_54XX),
	CLK(NULL,	"l4_div_ck",			&l4_div_ck,	CK_54XX),
	CLK(NULL,	"lp_clk_div",			&lp_clk_div,	CK_54XX),
	CLK(NULL,	"syc_clk_div",			&syc_clk_div,	CK_54XX),
	CLK(NULL,	"wkupaon_clk_mux",		&wkupaon_clk_mux,	CK_54XX),
	CLK(NULL,	"usb_host_hs_hsic60m_p2_clk",	&usb_host_hs_hsic60m_p2_clk,	CK_54XX),
	CLK(NULL,	"usb_host_hs_hsic60m_p3_clk",	&usb_host_hs_hsic60m_p3_clk,	CK_54XX),
	CLK(NULL,	"usb_host_hs_utmi_p1_clk",	&usb_host_hs_utmi_p1_clk,	CK_54XX),
	CLK(NULL,	"usb_host_hs_utmi_p2_clk",	&usb_host_hs_utmi_p2_clk,	CK_54XX),
	CLK(NULL,	"usb_host_hs_utmi_p3_clk",	&usb_host_hs_utmi_p3_clk,	CK_54XX),
	CLK(NULL,	"usb_host_hs_hsic480m_p1_clk",	&usb_host_hs_hsic480m_p1_clk,	CK_54XX),
	CLK(NULL,	"usb_host_hs_hsic60m_p1_clk",	&usb_host_hs_hsic60m_p1_clk,	CK_54XX),
	CLK(NULL,	"usb_host_hs_hsic480m_p3_clk",	&usb_host_hs_hsic480m_p3_clk,	CK_54XX),
	CLK(NULL,	"usb_host_hs_hsic480m_p2_clk",	&usb_host_hs_hsic480m_p2_clk,	CK_54XX),
	CLK(NULL,	"dss_32khz_clk",		&dss_32khz_clk,	CK_54XX),
	CLK(NULL,	"dss_48mhz_clk",		&dss_48mhz_clk,	CK_54XX),
	CLK(NULL,	"dss_dss_clk",			&dss_dss_clk,	CK_54XX),
	CLK(NULL,	"dss_sys_clk",			&dss_sys_clk,	CK_54XX),
	CLK(NULL,	"gpio1_dbclk",			&gpio1_dbclk,	CK_54XX),
	CLK(NULL,	"gpio2_dbclk",			&gpio2_dbclk,	CK_54XX),
	CLK(NULL,	"gpio3_dbclk",			&gpio3_dbclk,	CK_54XX),
	CLK(NULL,	"gpio4_dbclk",			&gpio4_dbclk,	CK_54XX),
	CLK(NULL,	"gpio5_dbclk",			&gpio5_dbclk,	CK_54XX),
	CLK(NULL,	"gpio6_dbclk",			&gpio6_dbclk,	CK_54XX),
	CLK(NULL,	"gpio7_dbclk",			&gpio7_dbclk,	CK_54XX),
	CLK(NULL,	"gpio8_dbclk",			&gpio8_dbclk,	CK_54XX),
	CLK(NULL,	"sata_ref_clk",			&sata_ref_clk,	CK_54XX),
	CLK(NULL,	"aess_fclk",			&aess_fclk,	CK_54XX),
	CLK(NULL,	"dmic_gfclk",			&dmic_gfclk,	CK_54XX),
	CLK(NULL,	"dmic_fck",			&dmic_gfclk,	CK_54XX),
	CLK(NULL,	"mcasp_sync_mux_ck",		&mcasp_sync_mux_ck,	CK_54XX),
	CLK(NULL,	"mcasp_gfclk",			&mcasp_gfclk,	CK_54XX),
	CLK(NULL,	"mcbsp1_sync_mux_ck",		&mcbsp1_sync_mux_ck,	CK_54XX),
	CLK(NULL,	"mcbsp1_gfclk",			&mcbsp1_gfclk,	CK_54XX),
	CLK(NULL,	"mcbsp2_sync_mux_ck",		&mcbsp2_sync_mux_ck,	CK_54XX),
	CLK(NULL,	"mcbsp2_gfclk",			&mcbsp2_gfclk,	CK_54XX),
	CLK(NULL,	"mcbsp3_sync_mux_ck",		&mcbsp3_sync_mux_ck,	CK_54XX),
	CLK(NULL,	"mcbsp3_gfclk",			&mcbsp3_gfclk,	CK_54XX),
	CLK(NULL,	"gpu_core_clk_mux",		&gpu_core_clk_mux,	CK_54XX),
	CLK(NULL,	"gpu_hyd_clk_mux",		&gpu_hyd_clk_mux,	CK_54XX),
	CLK(NULL,	"mmc1_fclk_mux",		&mmc1_fclk_mux,	CK_54XX),
	CLK(NULL,	"mmc1_fclk",			&mmc1_fclk,	CK_54XX),
	CLK(NULL,	"mmc2_fclk_mux",		&mmc2_fclk_mux,	CK_54XX),
	CLK(NULL,	"mmc2_fclk",			&mmc2_fclk,	CK_54XX),
	CLK(NULL,	"timer10_clk_mux",		&timer10_clk_mux,	CK_54XX),
	CLK(NULL,	"timer11_clk_mux",		&timer11_clk_mux,	CK_54XX),
	CLK(NULL,	"timer1_clk_mux",		&timer1_clk_mux,	CK_54XX),
	CLK(NULL,	"timer2_clk_mux",		&timer2_clk_mux,	CK_54XX),
	CLK(NULL,	"timer3_clk_mux",		&timer3_clk_mux,	CK_54XX),
	CLK(NULL,	"timer4_clk_mux",		&timer4_clk_mux,	CK_54XX),
	CLK(NULL,	"timer5_sync_mux_ck",		&timer5_sync_mux_ck,	CK_54XX),
	CLK(NULL,	"timer6_sync_mux_ck",		&timer6_sync_mux_ck,	CK_54XX),
	CLK(NULL,	"timer7_sync_mux_ck",		&timer7_sync_mux_ck,	CK_54XX),
	CLK(NULL,	"timer8_sync_mux_ck",		&timer8_sync_mux_ck,	CK_54XX),
	CLK(NULL,	"timer9_clk_mux",		&timer9_clk_mux,	CK_54XX),
	CLK("omap_timer.1",	"32k_ck",		&sys_32k_ck,	CK_54XX),
	CLK("omap_timer.2",	"32k_ck",		&sys_32k_ck,	CK_54XX),
	CLK("omap_timer.3",	"32k_ck",		&sys_32k_ck,	CK_54XX),
	CLK("omap_timer.4",	"32k_ck",		&sys_32k_ck,	CK_54XX),
	CLK("omap_timer.5",	"32k_ck",		&sys_32k_ck,	CK_54XX),
	CLK("omap_timer.6",	"32k_ck",		&sys_32k_ck,	CK_54XX),
	CLK("omap_timer.7",	"32k_ck",		&sys_32k_ck,	CK_54XX),
	CLK("omap_timer.8",	"32k_ck",		&sys_32k_ck,	CK_54XX),
	CLK("omap_timer.9",	"32k_ck",		&sys_32k_ck,	CK_54XX),
	CLK("omap_timer.10",	"32k_ck",		&sys_32k_ck,	CK_54XX),
	CLK("omap_timer.11",	"32k_ck",		&sys_32k_ck,	CK_54XX),
	CLK("omap_timer.1",	"sys_ck",		&sys_clkin,	CK_54XX),
	CLK("omap_timer.2",	"sys_ck",		&sys_clkin,	CK_54XX),
	CLK("omap_timer.3",	"sys_ck",		&sys_clkin,	CK_54XX),
	CLK("omap_timer.4",	"sys_ck",		&sys_clkin,	CK_54XX),
	CLK("omap_timer.9",	"sys_ck",		&sys_clkin,	CK_54XX),
	CLK("omap_timer.10",	"sys_ck",		&sys_clkin,	CK_54XX),
	CLK("omap_timer.11",	"sys_ck",		&sys_clkin,	CK_54XX),
	CLK("omap_timer.5",	"sys_ck",		&syc_clk_div,	CK_54XX),
	CLK("omap_timer.6",	"sys_ck",		&syc_clk_div,	CK_54XX),
	CLK("omap_timer.7",	"sys_ck",		&syc_clk_div,	CK_54XX),
	CLK("omap_timer.8",	"sys_ck",		&syc_clk_div,	CK_54XX),
	CLK("4ae18000.timer",	"sys_ck",		&sys_clkin,	CK_54XX),
//	CLK("48032000.timer",	"sys_ck",		&sys_clkin,	CK_54XX),
//	CLK("48034000.timer",	"sys_ck",		&sys_clkin,	CK_54XX),
//	CLK("48036000.timer",	"sys_ck",		&sys_clkin,	CK_54XX),
//	CLK("4803e000.timer",	"sys_ck",		&sys_clkin,	CK_54XX),
//	CLK("48086000.timer",	"sys_ck",		&sys_clkin,	CK_54XX),
//	CLK("48088000.timer",	"sys_ck",		&sys_clkin,	CK_54XX),
	CLK(NULL,       "sys_clkin_ck",                 &sys_clkin,     CK_54XX),
};

static const char *enable_init_clks[] = {
};

int __init omap5xxx_clk_init(void)
{
	u32 cpu_clkflg;
	struct omap_clk *c;

	if (soc_is_omap54xx()) {
		cpu_mask = RATE_IN_54XX;
		cpu_clkflg = CK_54XX;
	} else {
		return 0;
	}

	/*
	 * Must stay commented until all OMAP SoC drivers are
	 * converted to runtime PM, or drivers may start crashing
	 *
	 * omap2_clk_disable_clkdm_control();
	 */
	for (c = omap54xx_clks; c < omap54xx_clks + ARRAY_SIZE(omap54xx_clks);
									  c++) {
		if (c->cpu & cpu_clkflg) {
			clkdev_add(&c->lk);
			if (!__clk_init(NULL, c->lk.clk))
				omap2_init_clk_hw_omap_clocks(c->lk.clk);
		}
	}

	/* Disable autoidle on all clocks; let the PM code enable it later */
	omap2_clk_disable_autoidle_all();

	/*
	 * Only enable those clocks we will need, let the drivers
	 * enable other clocks as necessary
	 */
	omap2_clk_enable_init_clocks(enable_init_clks,
				     ARRAY_SIZE(enable_init_clks));

	return 0;
}
