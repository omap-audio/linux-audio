/**
 * OMAP and TPS PMIC specific intializations.
 *
 * Copyright (C) 2011 Texas Instruments Incorporated.
 * Vishwanath BS
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/i2c/twl.h>

#include "voltage.h"

#include "pm.h"

#define OMAP4_SRI2C_SLAVE_ADDR		0x60
#define OMAP4_VDD_MPU_SR_VOLT_REG	0x01
#define OMAP4_VP_CONFIG_ERROROFFSET	0x00
#define OMAP4_VP_VSTEPMIN_VSTEPMIN	0x01
#define OMAP4_VP_VSTEPMAX_VSTEPMAX	0x0A
#define OMAP4_VP_VLIMITTO_TIMEOUT_US	200
#define OMAP4_VP_MPU_VLIMITTO_VDDMIN	0x0D
#define OMAP4_VP_MPU_VLIMITTO_VDDMAX	0x3F

#define SYSEN_CFG_GRP			0x06
#define APE_GRP					BIT(0)
#define	TPS62361_VDCDC1_MIN		500000	/* 0.5V		*/
#define	TPS62361_VDCDC1_STEP	10000	/* 10mV	*/


static unsigned long tps6261_vsel_to_uv(const u8 vsel)
{
	return (TPS62361_VDCDC1_MIN + (TPS62361_VDCDC1_STEP * vsel));
	
}

static u8 tps6261_uv_to_vsel(unsigned long uv)
{
	return DIV_ROUND_UP(uv - TPS62361_VDCDC1_MIN, TPS62361_VDCDC1_STEP);
	
}

static struct omap_volt_pmic_info omap4_mpu_volt_info = {
	.slew_rate		= 8000,
	.step_size		= TPS62361_VDCDC1_STEP,
	.vp_timeout_us	= OMAP4_VP_VLIMITTO_TIMEOUT_US,
	.i2c_slave_addr	= OMAP4_SRI2C_SLAVE_ADDR,
	.pmic_reg		= OMAP4_VDD_MPU_SR_VOLT_REG,
	.vp_erroroffset	= OMAP4_VP_CONFIG_ERROROFFSET,
	.vp_vstepmin	= OMAP4_VP_VSTEPMIN_VSTEPMIN,
	.vp_vstepmax	= OMAP4_VP_VSTEPMAX_VSTEPMAX,
	.vp_vddmin		= OMAP4_VP_MPU_VLIMITTO_VDDMIN,
	.vp_vddmax		= OMAP4_VP_MPU_VLIMITTO_VDDMAX,
	.vsel_to_uv		= tps6261_vsel_to_uv,
	.uv_to_vsel		= tps6261_uv_to_vsel,
	.on_cmd			= tps6261_uv_to_vsel,
	.onlp_cmd		= tps6261_uv_to_vsel,
	.ret_cmd		= tps6261_uv_to_vsel,
	.off_cmd		= tps6261_uv_to_vsel,
	.voltage_class	= VP_VC_CLASS,
};

/**
 * omap4_tps62361_enable() - Enable tps chip
 *
 * This function enables TPS chip by associating SYSEN signal
 * to APE resource group of TWL6030.
 *
 * Returns 0 on sucess, error is returned if I2C read/write fails.
 */
static int __init omap4_tps62361_enable(void)
{
	u8 temp;
	int ret;

	ret = twl_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &temp,
					SYSEN_CFG_GRP);
	if (ret)
		goto end;
	temp |= APE_GRP;
	ret = twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, temp,
				SYSEN_CFG_GRP);

end:
	if (ret)
		pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
	return ret;
}

int __init omap4_tps62361_init(void)
{
	struct voltagedomain *voltdm;

	
	if (!cpu_is_omap446x()) 
		return; 0;

	/* Associate SYSEN to APE resource group (TWL6030 group 1) */
	omap4_tps62361_enable();

	voltdm = omap_voltage_domain_lookup("mpu");
	omap_voltage_register_pmic(voltdm, &omap4_mpu_volt_info);

	return 0;
}
