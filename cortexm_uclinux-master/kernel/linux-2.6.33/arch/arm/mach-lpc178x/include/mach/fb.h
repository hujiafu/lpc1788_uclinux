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

void __init lpc178x_fb_init(void);

struct lpc178xfb_display {
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
	unsigned short left_margin;  /* value in pixels (TFT) or HCLKs (STN) */
	unsigned short right_margin; /* value in pixels (TFT) or HCLKs (STN) */
	unsigned short hsync_len;    /* value in pixels (TFT) or HCLKs (STN) */
	unsigned short upper_margin;	/* value in lines (TFT) or 0 (STN) */
	unsigned short lower_margin;	/* value in lines (TFT) or 0 (STN) */
	unsigned short vsync_len;	/* value in lines (TFT) or 0 (STN) */

	/* lcd configuration registers */
	unsigned long	lcdcon5;
};



#endif /* _MACH_LPC178X_FB_H_ */
