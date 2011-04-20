/*
 * Copyright (C) 2009-2010 Texas Instruments Inc.
 * Mikkel Christensen <mlc@ti.com>
 * Felipe Balbi <balbi@ti.com>
 *
 * Modified from mach-omap2/board-ldp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/i2c/twl.h>
#include <linux/mtd/nand.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <plat/common.h>
#include <plat/board.h>
#include <plat/usb.h>

#include <mach/board-zoom.h>
#include <plat/voltage.h>

#include "board-flash.h"
#include "mux.h"
#include "sdram-micron-mt46h32m32lf-6.h"
#include "sdram-hynix-h8mbx00u0mer-0em.h"

#define ZOOM3_EHCI_RESET_GPIO		64
/*
 * VOLTSETUP1 for RET & OFF
 * Setup time (in us) of the VDD1 and VDD2 regulators.
 * Number of sys_clk cycles required for VDD regulator to stabilize is
 * devided by 8 and programmed in the register field for VDD1/VDD2.
 */
#define OMAP3_VOLTSETUP_VDD1_RET		28
#define OMAP3_VOLTSETUP_VDD2_RET		26

#define OMAP3_VOLTSETUP_VDD1_OFF		55
#define OMAP3_VOLTSETUP_VDD2_OFF		49

/*
 * VOLTOFFSET for RET & OFF
 * Offset-time to de-assert sys_offmode signal while exiting the OFF mode
 * and when the OFF sequence is supervised by the Power IC.
 */
#define OMAP3_VOLTOFFSET_OFF	516

#define OMAP3_SYSREQ_CKEN_TIME	31
#define OMAP3_S2A_TIME			10041
#define OMAP3_VDD_RAMP_TIME		1250

#define OMAP3_VOLTSETUP2	(OMAP3_SYSREQ_CKEN_TIME + OMAP3_S2A_TIME + OMAP3_VDD_RAMP_TIME - OMAP3_VOLTOFFSET_OFF)

/*
 * OMAP3 CLKSETUP TIME for RET & OFF
 * Setup time of the oscillator (sys_clk), based on number of
 * 32 kHz clock cycles.
 */
#define OMAP3_CLKSETUP_RET		31
#define OMAP3_CLKSETUP_OFF		(OMAP3_VOLTOFFSET_OFF + OMAP3_VOLTSETUP2)

static void __init omap_zoom_init_early(void)
{
	omap2_init_common_infrastructure();
	if (machine_is_omap_zoom2())
		omap2_init_common_devices(mt46h32m32lf6_sdrc_params,
					  mt46h32m32lf6_sdrc_params);
	else if (machine_is_omap_zoom3())
		omap2_init_common_devices(h8mbx00u0mer0em_sdrc_params,
					  h8mbx00u0mer0em_sdrc_params);
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/* WLAN IRQ - GPIO 162 */
	OMAP3_MUX(MCBSP1_CLKX, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
	/* WLAN POWER ENABLE - GPIO 101 */
	OMAP3_MUX(CAM_D2, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	/* WLAN SDIO: MMC3 CMD */
	OMAP3_MUX(MCSPI1_CS1, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),
	/* WLAN SDIO: MMC3 CLK */
	OMAP3_MUX(ETK_CLK, OMAP_MUX_MODE2 | OMAP_PIN_INPUT_PULLUP),
	/* WLAN SDIO: MMC3 DAT[0-3] */
	OMAP3_MUX(ETK_D3, OMAP_MUX_MODE2 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(ETK_D4, OMAP_MUX_MODE2 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(ETK_D5, OMAP_MUX_MODE2 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(ETK_D6, OMAP_MUX_MODE2 | OMAP_PIN_INPUT_PULLUP),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

static struct mtd_partition zoom_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader-NAND",
		.offset		= 0,
		.size		= 4 * (64 * 2048),	/* 512KB, 0x80000 */
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot-NAND",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 10 * (64 * 2048),	/* 1.25MB, 0x140000 */
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "Boot Env-NAND",
		.offset		= MTDPART_OFS_APPEND,   /* Offset = 0x1c0000 */
		.size		= 2 * (64 * 2048),	/* 256KB, 0x40000 */
	},
	{
		.name		= "Kernel-NAND",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x0200000*/
		.size		= 240 * (64 * 2048),	/* 30M, 0x1E00000 */
	},
	{
		.name		= "system",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x2000000 */
		.size		= 3328 * (64 * 2048),	/* 416M, 0x1A000000 */
	},
	{
		.name		= "userdata",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x1C000000*/
		.size		= 256 * (64 * 2048),	/* 32M, 0x2000000 */
	},
	{
		.name		= "cache",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x1E000000*/
		.size		= 256 * (64 * 2048),	/* 32M, 0x2000000 */
	},
};

static const struct usbhs_omap_board_data usbhs_bdata __initconst = {
	.port_mode[0]		= OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[1]		= OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[2]		= OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset		= true,
	.reset_gpio_port[0]	= -EINVAL,
	.reset_gpio_port[1]	= ZOOM3_EHCI_RESET_GPIO,
	.reset_gpio_port[2]	= -EINVAL,
};

union omap_volt_board_data zoom_mpu_volt_data = {
	.omap3_board_data = {
		.vdd_setup_ret = {
			.voltsetup	= OMAP3_VOLTSETUP_VDD1_RET,
			.clksetup	= OMAP3_CLKSETUP_RET,
			.voltsetup2		= 0,
			},

		.vdd_setup_off = {
			.voltsetup	= OMAP3_VOLTSETUP_VDD1_OFF,
			.clksetup	= OMAP3_CLKSETUP_OFF,
			.voltsetup2	= OMAP3_VOLTSETUP2,
		},
		.voltoffset		= OMAP3_VOLTOFFSET_OFF
	}
};

union omap_volt_board_data zoom_core_volt_data = {
	.omap3_board_data = {
		.vdd_setup_ret = {
			.voltsetup	= OMAP3_VOLTSETUP_VDD2_RET,
			.clksetup	= OMAP3_CLKSETUP_RET,
			.voltsetup2		= 0,
		},

		.vdd_setup_off = {
			.voltsetup	= OMAP3_VOLTSETUP_VDD2_OFF,
			.clksetup	= OMAP3_CLKSETUP_OFF,
			.voltsetup2	= OMAP3_VOLTSETUP2,
		},

		.voltoffset		= OMAP3_VOLTOFFSET_OFF
	}
};


static void __init omap_zoom_init(void)
{
	struct voltagedomain *voltdm;

	if (machine_is_omap_zoom2()) {
		omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	} else if (machine_is_omap_zoom3()) {
		omap3_mux_init(board_mux, OMAP_PACKAGE_CBP);
		omap_mux_init_gpio(ZOOM3_EHCI_RESET_GPIO, OMAP_PIN_OUTPUT);
		usbhs_init(&usbhs_bdata);
	}

	board_nand_init(zoom_nand_partitions, ARRAY_SIZE(zoom_nand_partitions),
						ZOOM_NAND_CS, NAND_BUSWIDTH_16);
	zoom_debugboard_init();
	zoom_peripherals_init();
	zoom_display_init();

	voltdm = omap_voltage_domain_lookup("mpu");
	omap_voltage_register_board_params(voltdm, &zoom_mpu_volt_data);

	voltdm = omap_voltage_domain_lookup("core");
	omap_voltage_register_board_params(voltdm, &zoom_core_volt_data);

}

MACHINE_START(OMAP_ZOOM2, "OMAP Zoom2 board")
	.boot_params	= 0x80000100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap_zoom_init_early,
	.init_irq	= omap_init_irq,
	.init_machine	= omap_zoom_init,
	.timer		= &omap_timer,
MACHINE_END

MACHINE_START(OMAP_ZOOM3, "OMAP Zoom3 board")
	.boot_params	= 0x80000100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap_zoom_init_early,
	.init_irq	= omap_init_irq,
	.init_machine	= omap_zoom_init,
	.timer		= &omap_timer,
MACHINE_END

