/*
 * linux/arch/arm/mach-omap2/board-rx51.c
 *
 * Copyright (C) 2007, 2008 Nokia
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/opp.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/mcspi.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/dma.h>
#include <plat/gpmc.h>
#include <plat/usb.h>
#include <plat/omap_device.h>

#include "mux.h"
#include "pm.h"

#define RX51_GPIO_SLEEP_IND 162

struct omap_sdrc_params *rx51_get_sdram_timings(void);
extern void rx51_video_mem_init(void);
extern void rx51_camera_init(void);

static struct gpio_led gpio_leds[] = {
	{
		.name	= "sleep_ind",
		.gpio	= RX51_GPIO_SLEEP_IND,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

static struct cpuidle_params rx51_cpuidle_params[] = {
	/* C1 */
	{1, 110, 162, 5},
	/* C2 */
	{1, 106, 180, 309},
	/* C3 */
	{0, 107, 410, 46057},
	/* C4 */
	{0, 121, 3374, 46057},
	/* C5 */
	{1, 855, 1146, 46057},
	/* C6 */
	{0, 7580, 4134, 484329},
	/* C7 */
	{1, 7505, 15274, 484329},
};

static struct omap_lcd_config rx51_lcd_config = {
	.ctrl_name	= "internal",
};

static struct omap_fbmem_config rx51_fbmem0_config = {
	.size = 752 * 1024,
};

static struct omap_fbmem_config rx51_fbmem1_config = {
	.size = 752 * 1024,
};

static struct omap_fbmem_config rx51_fbmem2_config = {
	.size = 752 * 1024,
};

static struct omap_board_config_kernel rx51_config[] = {
	{ OMAP_TAG_FBMEM,	&rx51_fbmem0_config },
	{ OMAP_TAG_FBMEM,	&rx51_fbmem1_config },
	{ OMAP_TAG_FBMEM,	&rx51_fbmem2_config },
	{ OMAP_TAG_LCD,		&rx51_lcd_config },
};

static void __init rx51_init_irq(void)
{
	struct omap_sdrc_params *sdrc_params;

	omap_board_config = rx51_config;
	omap_board_config_size = ARRAY_SIZE(rx51_config);
	omap3_pm_init_cpuidle(rx51_cpuidle_params);
	sdrc_params = rx51_get_sdram_timings();
	omap2_init_common_hw(sdrc_params, sdrc_params);
	omap_init_irq();
	omap_gpio_init();
}

static void __init rx51_opp_init(void)
{
	/*
	 * Init opp tables to default values and then
	 * remove 125MHz from available cpu clock rates.
	 */

	int r = 0;

	if (r=omap3_opp_init()) {
		/*
		 * FIXME: We are now called with late_initcall() and
		 * thus omap3_opp_init() has already been run once,
		 * so we will get -EEXIST. This check should be removed
		 * when our backport of opp/dvfs code gets updated
		 * and late_initcall removed.
		 */
		if (r != -EEXIST) {
			pr_err("%s: failed to init default opp table: %d\n", __func__, r);
			return;
		}
	}

	struct omap_hwmod *mpu = omap_hwmod_lookup("mpu");
	struct device *dev;
	if (!mpu) {
		pr_err("%s: omap_hwmod_lookup failed\n", __func__);
		return;
	}
	dev = &mpu->od->pdev.dev;
	r = opp_disable(dev, 125000000);
	if (r) {
		pr_err("%s: failed to disable opp, err: %d\n", __func__, r);
	}
}
/*
 * FIXME: using late_initcall to get this working with current backported
 * version of opp/dvfs support. Currently rx51_opp_init will fail if run
 * from rx51_init due to some hwmod related inits that are not yet done.
 * Move to normal call in rx51_init when updating the backport.
 */
late_initcall(rx51_opp_init);

extern void __init rx51_peripherals_init(void);

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_PERIPHERAL,
	.power			= 0,
};

static void __init rx51_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	omap_serial_init();
	usb_musb_init(&musb_board_data);
	rx51_peripherals_init();
	rx51_camera_init();

	/* Ensure SDRC pins are mux'd for self-refresh */
	omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);

	platform_device_register(&leds_gpio);
}

static void __init rx51_map_io(void)
{
	omap2_set_globals_3xxx();
	omap34xx_map_common_io();
}

static void __init rx51_reserve(void)
{
	rx51_video_mem_init();
	omap_reserve();
}

MACHINE_START(NOKIA_RX51, "Nokia RX-51 board")
	/* Maintainer: Lauri Leukkunen <lauri.leukkunen@nokia.com> */
	.boot_params	= 0x80000100,
	.map_io		= rx51_map_io,
	.reserve	= rx51_reserve,
	.init_irq	= rx51_init_irq,
	.init_machine	= rx51_init,
	.timer		= &omap_timer,
MACHINE_END
