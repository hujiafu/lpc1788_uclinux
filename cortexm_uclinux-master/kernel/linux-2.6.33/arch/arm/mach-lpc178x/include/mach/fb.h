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

#ifndef _MACH_LPC178X_FB_H_
#define _MACH_LPC178X_FB_H_

#include <linux/init.h>

/*
 * LCD controller registers base
 */
#define LPC178X_LCD_BASE	(LPC178X_AHB_PERIPH_BASE + 0x00008000)

#define LPC178X_LCD_TIMH		0x0
#define LPC178X_LCD_TIMV		0x4
#define LPC178X_LCD_POL		0x8
#define LPC178X_LCD_UPBASE		0x10
#define LPC178X_LCD_LPBASE		0x14
#define LPC178X_LCD_CTRL	0x18
#define LPC178X_LCD_CRSR_CTRL	0xC00


#define LPC178X_LCD_TFT		(0x1<<5)
#define LPC178X_LCD_RGB_SWAP	(0x1<<8)
#define LPC178X_LCD_RGB_BEBO	(0x1<<9)
#define LPC178X_LCD_16BPP_565	(0x6<<1)
#define LPC178X_LCD_FMT_MASK	(0x7<<1)

//lcd_pol
#define LPC178X_LCD_FP_LOW		(0x1<<11)
#define LPC178X_LCD_LP_LOW		(0x1<<12)
#define LPC178X_LCD_DATA_FALL_EAGE	(0x1<<13)
#define LPC178X_LCD_CLK_BYPASS		(0x1<<26)
#define LPC178X_LCD_CPL_MASK		(0x3ff<<16)


#define LPC178X_LCD_CLK_FRE		(54000000)

void __init lpc178x_fb_init(void);

struct lpc178xfb_hw {

	u32 lcd_timh;
	u32 lcd_timv;
	u32 lcd_pol;
	u32 lcd_le;
	u32 lcd_upbase;
	u32 lcd_lpbase;
	u32 lcd_ctrl;
	u32 lcd_intmsk;
	u32 lcd_intraw;
	u32 lcd_intstat;
	u32 lcd_intclr;
	u32 lcd_upcurr;
	u32 lcd_lpcurr;
	u32 lcd_rcv1[115];
	u32 lcd_pal[128];
	u32 lcd_rcv2[256];
	u32 crsr_img[256];
	u32 crsr_ctrl;
	u32 crsr_cfg;
	u32 crsr_pal0;
	u32 crsr_pal1;
	u32 crsr_xy;
	u32 crsr_clip;
	u32 crsr_rcv1[2];
	u32 crsr_intmsk;
	u32 crsr_intclr;
	u32 crsr_intraw;
	u32 crsr_intstat;
};


struct lpc1788fb_display {
	/* LCD type */
	unsigned type;

	/* Screen size */
	unsigned short width;
	unsigned short height;

	/* Screen info */
	unsigned short xres;
	unsigned short yres;
	unsigned short bpp;

	unsigned pixclock;		/* pixclock in picoseconds */
	unsigned long max_fre;
	unsigned long min_fre;
	unsigned short left_margin;  /* value in pixels (TFT) or HCLKs (STN) */
	unsigned short right_margin; /* value in pixels (TFT) or HCLKs (STN) */
	unsigned short hsync_len;    /* value in pixels (TFT) or HCLKs (STN) */
	unsigned short upper_margin;	/* value in lines (TFT) or 0 (STN) */
	unsigned short lower_margin;	/* value in lines (TFT) or 0 (STN) */
	unsigned short vsync_len;	/* value in lines (TFT) or 0 (STN) */

	unsigned long lcdctrl;
	unsigned long lcdpol;
	unsigned long lcdtimeh;
	unsigned long lcdtimev;
	
	struct clk *clk
};

struct lpc1788fb_mach_info {
         
        struct lpc1788fb_display *displays;     /* attached diplays info */
        unsigned num_displays;                  /* number of defined displays */
        unsigned default_display;
 
};


#endif /* _MACH_LPC178X_FB_H_ */
