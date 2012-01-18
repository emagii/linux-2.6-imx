/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c/at24.h>
#include <linux/i2c/atmel_mxt_ts.h>
#include <linux/i2c/pca953x.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/hardware.h>
#include <mach/device.h>
#include <mach/pinctrl.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>

#include "device.h"
#include "mx28evk.h"

#ifdef CONFIG_MACH_MBA28
static struct at24_platform_data eeprom_info = {
	.byte_len       = (64*1024) / 8,
	.page_size      = 32,
	.flags          = AT24_FLAG_ADDR16,
};
static struct pca953x_platform_data pca9554_info1 = {
	.gpio_base      = -1,
};
static struct pca953x_platform_data pca9554_info2 = {
	.gpio_base      = -1,
};

#if	defined(CONFIG_TOUCHSCREEN_ATMEL_MXT)
/* TSP */
static u8 mxt_init_vals[] = {
	/* MXT_GEN_COMMAND(6) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* MXT_GEN_POWER(7) */
	0x20, 0xff, 0x32,
	/* MXT_GEN_ACQUIRE(8) */
	0x0a, 0x00, 0x05, 0x00, 0x00, 0x00, 0x09, 0x23,
	/* MXT_TOUCH_MULTI(9) */
	0x00, 0x00, 0x00, 0x13, 0x0b, 0x00, 0x00, 0x00, 0x02, 0x00,
	0x00, 0x01, 0x01, 0x0e, 0x0a, 0x0a, 0x0a, 0x0a, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00,
	/* MXT_TOUCH_KEYARRAY(15) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
	0x00,
	/* MXT_SPT_GPIOPWM(19) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* MXT_PROCI_GRIPFACE(20) */
	0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x28, 0x04,
	0x0f, 0x0a,
	/* MXT_PROCG_NOISE(22) */
	0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x23, 0x00,
	0x00, 0x05, 0x0f, 0x19, 0x23, 0x2d, 0x03,
	/* MXT_TOUCH_PROXIMITY(23) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	/* MXT_PROCI_ONETOUCH(24) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* MXT_SPT_SELFTEST(25) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	/* MXT_PROCI_TWOTOUCH(27) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* MXT_SPT_CTECONFIG(28) */
	0x00, 0x00, 0x02, 0x08, 0x10, 0x00,
};

static struct mxt_platform_data mxt_platform_data = {
	.config			= mxt_init_vals,
	.config_length		= ARRAY_SIZE(mxt_init_vals),

	.x_line			= 19,
	.y_line			= 11,
	.x_size			= 800,
	.y_size			= 480,
	.blen			= 0x1,
	.threshold		= 0x27,
	.voltage		= 2700000,		/* 2.8V */
	.orient			= MXT_DIAGONAL_COUNTER,
	.irqflags		= IRQF_TRIGGER_FALLING,
};
#endif

#if	defined(CONFIG_MACH_MBA28) && defined(CONFIG_TOUCHSCREEN_ATMEL_MXT)
static struct i2c_board_info __initdata mxs_i2c_device_0[] = {
	{
		I2C_BOARD_INFO("atmel_mxt_ts", 0x4a),
		.platform_data	= &mxt_platform_data,
		.irq		= MXS_PIN_TO_IRQ(3,6),
	},
};
#endif

static struct i2c_board_info __initdata mxs_i2c_device_1[] = {
	{ I2C_BOARD_INFO("24c64", 0x50), .platform_data  = &eeprom_info},
	{ I2C_BOARD_INFO("lm73", 0x4a) },
	{ I2C_BOARD_INFO("sgtl5000-i2c", 0xa), .flags = I2C_M_TEN },
	{ I2C_BOARD_INFO("pca9554", 0x20), .platform_data = &pca9554_info1 },
	{ I2C_BOARD_INFO("pca9554", 0x21), .platform_data = &pca9554_info2 },
};

static void __init i2c_device_init(void)
{
#if	defined(CONFIG_MACH_MBA28) && defined(CONFIG_TOUCHSCREEN_ATMEL_MXT)
	i2c_register_board_info(0, mxs_i2c_device_0, ARRAY_SIZE(mxs_i2c_device_0));
#endif
	i2c_register_board_info(1, mxs_i2c_device_1, ARRAY_SIZE(mxs_i2c_device_1));
}
#else
static struct i2c_board_info __initdata mxs_i2c_device[] = {
	{ I2C_BOARD_INFO("sgtl5000-i2c", 0xa), .flags = I2C_M_TEN }
};

static void __init i2c_device_init(void)
{
	i2c_register_board_info(0, mxs_i2c_device, ARRAY_SIZE(mxs_i2c_device));
}
#endif

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct flash_platform_data mx28_spi_flash_data = {
	.name = "m25p80",
	.type = "w25x80",
};
#endif

static struct spi_board_info spi_board_info[] __initdata = {
#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
	{
		/* the modalias must be the same as spi device driver name */
		.modalias = "m25p80", /* Name of spi_driver for this device */
		.max_speed_hz = 20000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 1, /* Framework bus number */
		.chip_select = 0, /* Framework chip select. */
		.platform_data = &mx28_spi_flash_data,
	},
#endif
};

static void spi_device_init(void)
{
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
}

static void __init fixup_board(struct machine_desc *desc, struct tag *tags,
			       char **cmdline, struct meminfo *mi)
{
	mx28_set_input_clk(24000000, 24000000, 32000, 50000000);
}

#if defined(CONFIG_LEDS_MXS) || defined(CONFIG_LEDS_MXS_MODULE)
static struct mxs_pwm_led  mx28evk_led_pwm[2] = {
	[0] = {
		.name = "led-pwm0",
		.pwm = 0,
		},
	[1] = {
		.name = "led-pwm1",
		.pwm = 1,
		},
};

struct mxs_pwm_leds_plat_data mx28evk_led_data = {
	.num = ARRAY_SIZE(mx28evk_led_pwm),
	.leds = mx28evk_led_pwm,
};

static struct resource mx28evk_led_res = {
	.flags = IORESOURCE_MEM,
	.start = PWM_PHYS_ADDR,
	.end   = PWM_PHYS_ADDR + 0x3FFF,
};

static void __init mx28evk_init_leds(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("mxs-leds", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;

	pdev->resource = &mx28evk_led_res;
	pdev->num_resources = 1;
	pdev->dev.platform_data = &mx28evk_led_data;
	mxs_add_device(pdev, 3);
}
#else
static void __init mx28evk_init_leds(void)
{
	;
}
#endif

static void __init mx28evk_device_init(void)
{
	/* Add mx28evk special code */
	i2c_device_init();
	spi_device_init();
	mx28evk_init_leds();
}

static void __init mx28evk_init_machine(void)
{
	mx28_pinctrl_init();
	/* Init iram allocate */
#ifdef CONFIG_VECTORS_PHY_ADDR
	/* reserve the first page for irq vector table*/
	iram_init(MX28_OCRAM_PHBASE + PAGE_SIZE, MX28_OCRAM_SIZE - PAGE_SIZE);
#else
	iram_init(MX28_OCRAM_PHBASE, MX28_OCRAM_SIZE);
#endif

	mx28_gpio_init();
	mx28evk_pins_init();
	mx28_device_init();
	mx28evk_device_init();
}

#ifdef CONFIG_MACH_MBA28
#define BOARD_NAME "TQ MBa28 board with TQMa28 module"
#else
#define BOARD_NAME "Freescale MX28EVK board"
#endif

MACHINE_START(MX28EVK, BOARD_NAME)
	.phys_io	= 0x80000000,
	.io_pg_offst	= ((0xf0000000) >> 18) & 0xfffc,
	.boot_params	= 0x40000100,
	.fixup		= fixup_board,
	.map_io		= mx28_map_io,
	.init_irq	= mx28_irq_init,
	.init_machine	= mx28evk_init_machine,
	.timer		= &mx28_timer.timer,
MACHINE_END
