/* linux/arch/arm/mach-s5pv210/mach-smdkv210.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/sysdev.h>
#ifdef CONFIG_DM9000
#include <linux/dm9000.h>
#endif
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/pwm_backlight.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>
#include <asm/mach-types.h>

#include <video/platform_lcd.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-fb.h>

#include <plat/regs-serial.h>
#include <plat/regs-srom.h>
#include <plat/gpio-cfg.h>
#include <plat/s5pv210.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/adc.h>
#include <plat/ts.h>
#include <plat/ata.h>
#include <plat/iic.h>
#include <plat/keypad.h>
#include <plat/pm.h>
#include <plat/fb.h>
#include <plat/s5p-time.h>

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define SMART210_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define SMART210_ULCON_DEFAULT	S3C2410_LCON_CS8

#define SMART210_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static __initdata struct s3c2410_uartcfg smart210_uartcfgs[]  = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= SMART210_UCON_DEFAULT,
		.ulcon		= SMART210_ULCON_DEFAULT,
		.ufcon		= SMART210_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= SMART210_UCON_DEFAULT,
		.ulcon		= SMART210_ULCON_DEFAULT,
		.ufcon		= SMART210_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= SMART210_UCON_DEFAULT,
		.ulcon		= SMART210_ULCON_DEFAULT,
		.ufcon		= SMART210_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= SMART210_UCON_DEFAULT,
		.ulcon		= SMART210_ULCON_DEFAULT,
		.ufcon		= SMART210_UFCON_DEFAULT,
	},
};

#ifdef CONFIG_DM9000
static struct resource smart210_dm9000_resources[] = {
	[0] = {
		.start	= S5PV210_PA_SROM_BANK1,
		.end	= S5PV210_PA_SROM_BANK1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= S5PV210_PA_SROM_BANK1 + 0x4,
		.end	= S5PV210_PA_SROM_BANK1 + 0x4,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.start	= IRQ_EINT(7),
		.end	= IRQ_EINT(7),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	},
};

static struct dm9000_plat_data smdkv210_dm9000_platdata = {
	.flags		= DM9000_PLATF_16BITONLY | DM9000_PLATF_NO_EEPROM,
	.dev_addr	= { 0x00, 0x09, 0xc0, 0xff, 0xec, 0x48 },
};

struct platform_device smart210_dm9000 = {
	.name		= "dm9000",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(smart210_dm9000_resources),
	.resource	= smart210_dm9000_resources,
	.dev		= {
		.platform_data	= &smdkv210_dm9000_platdata,
	},
};

static void __init smart210_dm9000_init(void)
{
	unsigned int tmp;

	gpio_request(S5PV210_MP01(1), "nCS1");
	s3c_gpio_cfgpin(S5PV210_MP01(1), S3C_GPIO_SFN(2));
	gpio_free(S5PV210_MP01(1));

	tmp = (5 << S5P_SROM_BCX__TACC__SHIFT);
	__raw_writel(tmp, S5P_SROM_BC1);

	tmp = __raw_readl(S5P_SROM_BW);
	tmp &= ~(S5P_SROM_BW__CS_MASK << S5P_SROM_BW__NCS1__SHIFT);
	tmp |= (0x3 << S5P_SROM_BW__NCS1__SHIFT);
	__raw_writel(tmp, S5P_SROM_BW);
}
#endif

#ifdef CONFIG_S3C_DEV_FB
static struct s3c_fb_pd_win smart210_fb_win0 = {
	.win_mode = {
		.left_margin	= 46,
		.right_margin	= 210,
		.upper_margin	= 23,
		.lower_margin	= 22,
		.hsync_len	= 1,
		.vsync_len	= 1,
		.xres		= 800,
		.yres		= 480,
	},
	.max_bpp	= 32,
	.default_bpp	= 24,
};

static __initdata struct s3c_fb_platdata smart210_lcd0_pdata  = {
	.win[0]		= &smart210_fb_win0,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
	.vidcon1	= VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC,
	.setup_gpio	= s5pv210_fb_gpio_setup_24bpp,
};

static struct platform_device s3c_device_1wire = {
	.name			= "smart210_1wire",
	.id				= -1,
	.num_resources	= 0,
};

#endif

static __initdata struct platform_device *smart210_devices[]  = {
#ifdef CONFIG_DM9000
	&smart210_dm9000,
#endif
#ifdef CONFIG_S3C_DEV_RTC
    &s3c_device_rtc,
#endif
#ifdef CONFIG_S3C_DEV_FB
    &s3c_device_fb,
    &s3c_device_1wire,
#endif
};

static void __init smart210_map_io(void)
{
	s5p_init_io(NULL, 0, S5P_VA_CHIPID);
	s3c24xx_init_clocks(24000000);

	s3c24xx_init_uarts(smart210_uartcfgs, ARRAY_SIZE(smart210_uartcfgs));
	s5p_set_timer_source(S5P_PWM2, S5P_PWM4);
}

static void __init smart210_machine_init(void)
{
#ifdef CONFIG_DM9000
    smart210_dm9000_init();
#endif

#ifdef CONFIG_S3C_DEV_FB
    s3c_fb_set_platdata(&smart210_lcd0_pdata);
#endif

	platform_add_devices(smart210_devices, ARRAY_SIZE(smart210_devices));
}

MACHINE_START(SMART210, "SMART210")
	/* Maintainer: Kukjin Kim <kgene.kim@samsung.com> */
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.init_irq	= s5pv210_init_irq,
	.map_io		= smart210_map_io,
	.init_machine	= smart210_machine_init,
	.timer		= &s5p_timer,
MACHINE_END
