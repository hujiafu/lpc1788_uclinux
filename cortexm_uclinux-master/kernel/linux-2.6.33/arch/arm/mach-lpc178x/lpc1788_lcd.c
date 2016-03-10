/*
 * (C) Copyright 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/dma-mapping.h>
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>
#include <linux/delay.h>

#include <mach/lpc178x.h>
#include <mach/fb.h>
#include <mach/power.h>
#include <mach/platform.h>

/*
 * "Interrupt ID" in Table 43 in the LPC178x/7x User Manual (page 70)
 */
#define LPC178X_LCD_IRQ		37

/*
 * LCD panel-specific configuration.
 *
 * Note that left_margin/right_margin and upper_margin/lower_margin are swapped.
 * This is so because these variables are mistakenly swapped
 * in `clcdfb_decode()` in `include/linux/amba/clcd.h`.
 */
#define CONFIG_LCD_INDEX0
#ifdef CONFIG_LCD_INDEX0
int	lcd_index = 0;
#endif


#define _LCD_DECLARE(_clock,_xres,margin_left,margin_right,hsync, \
                         _yres,margin_top,margin_bottom,vsync, refresh) \
         .width = _xres, \
         .xres = _xres, \
         .height = _yres, \
         .yres = _yres, \
         .left_margin    = margin_left,  \
         .right_margin   = margin_right, \
         .upper_margin   = margin_top,   \
         .lower_margin   = margin_bottom,        \
         .hsync_len      = hsync,        \
         .vsync_len      = vsync,        \
         .pixclock       = ((_clock*100000000000LL) /    \
                            ((refresh) * \
                            (hsync + margin_left + _xres + margin_right) * \
                            (vsync + margin_top + _yres + margin_bottom))), \
         .bpp            = 16,\
         .type           = (LPC178X_LCD_TFT |\
                            LPC178X_LCD_16BPP_565)

static struct lpc1788fb_display lpc1788_lcd_cfg[] __initdata = {
         [0] = { /* mini2440 + 3.5" TFT + touchscreen */
                 _LCD_DECLARE(
                         7,                      /* The 3.5 is quite fast */
                         480, 5, 40, 20,         /* x timing */
                         272, 8, 8, 10,           /* y timing */
                         60),                    /* refresh rate */
				 .max_fre		= 12000000,
				 .min_fre		= 5000000,	 
				 .lcdctrl		= (LPC178X_LCD_TFT | LPC178X_LCD_16BPP_565),
                 .lcdpol        = (LPC178X_LCD_FP_LOW |
                                    LPC178X_LCD_LP_LOW |
                                    LPC178X_LCD_DATA_FALL_EAGE |
                                    LPC178X_LCD_CLK_BYPASS),
         },              
         [1] = { /* mini2440 + 7" TFT + touchscreen */
                 _LCD_DECLARE(      
                         10,                     /* the 7" runs slower */
                         800, 40, 40, 48,        /* x timing */
                         480, 29, 3, 3,          /* y timing */
                         50),                    /* refresh rate */
				 .lcdctrl		= (LPC178X_LCD_TFT | LPC178X_LCD_16BPP_565),
                 .lcdpol        = (LPC178X_LCD_FP_LOW |
                                    LPC178X_LCD_LP_LOW |
                                    LPC178X_LCD_DATA_FALL_EAGE |
                                    LPC178X_LCD_CLK_BYPASS),
         },
         [2] = {
                 _LCD_DECLARE(
                         10,
                         1024, 1, 2, 2,          /* y timing */
                         768, 200, 16, 16,       /* x timing */
                         24),    /* refresh rate, maximum stable,
                                  tested with the FPGA shield */
				 .lcdctrl		= (LPC178X_LCD_TFT | LPC178X_LCD_16BPP_565),
                 .lcdpol        = (LPC178X_LCD_FP_LOW |
                                    LPC178X_LCD_LP_LOW |
                                    LPC178X_LCD_DATA_FALL_EAGE |
                                    LPC178X_LCD_CLK_BYPASS),
         },
};

static struct lpc1788fb_mach_info lpc1788_fb_info __initdata = {
         .displays        = &lpc1788_lcd_cfg[0], /* not constant! see init */
         .num_displays    = 1,
         .default_display = 0,
};


static struct resource lpc1788_lcd_resource[] = {
	[0]	=	{
		.start	=	LPC178X_LCD_BASE,
		.end	=	LPC178X_LCD_BASE + SZ_16K - 1,
		.flags	=	IORESOURCE_MEM,
	},
	[1]	=	{
		.start	=	LPC178X_LCD_IRQ,
		.end	=	LPC178X_LCD_IRQ,
		.flags	=	IORESOURCE_IRQ,
	}
};

struct platform_device lpc1788_device_lcd = {
	.name	= "dev:clcd",
	.id		=	-1,
	.num_resources	=	ARRAY_SIZE(lpc1788_lcd_resource),
	.resource	=	lpc1788_lcd_resource,
	.dev	=	{
		.dma_mask	=	~0,
		.coherent_dma_mask	=	~0,
	}
};


void __init lpc1788_fb_set_platdata(struct lpc1788fb_mach_info *pd)
{               
         struct lpc1788fb_mach_info *npd;
                 
         npd = kmalloc(sizeof(*npd), GFP_KERNEL);
         if (npd) {
                 memcpy(npd, pd, sizeof(*npd));
                 lpc1788_device_lcd.dev.platform_data = npd;
         } else {        
                 printk(KERN_ERR "no memory for LCD platform data\n");
         }                               
}

void __init lpc178x_fb_init(void)
{
	int i;
	
	lpc1788_fb_info.displays = &lpc1788_lcd_cfg[lcd_index];

	for(i=0; i<ARRAY_SIZE(lpc1788_lcd_cfg); i++){
		if (i == lcd_index){
			printk(" [%d:%dx%d]", i,
                                         lpc1788_lcd_cfg[i].width,
                                         lpc1788_lcd_cfg[i].height);

		}else{
			printk(" %d:%dx%d", i,
                                         lpc1788_lcd_cfg[i].width,
                                         lpc1788_lcd_cfg[i].height);
		}
	}

	lpc1788_fb_set_platdata(&lpc1788_fb_info);
	platform_device_register(&lpc1788_device_lcd);
}
