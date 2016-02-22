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

#ifndef _MACH_LPC178X_TOUCH_H_
#define _MACH_LPC178X_TOUCH_H_

#include <mach/lpc178x.h>

struct lpc178x_eint_regs {
	unsigned int extint;
	unsigned int rsv0;
	unsigned int extmode;
	unsigned int extpolar;
};


struct touch_platdata {
	int (*io_setup)(void);
	int (*eint_clear)(void);
	unsigned int mem_start;
	unsigned int mem_size;
	unsigned int irq;
};


void __init lpc178x_touch_init(void);

static int lpc178x_touch_iosetup(void);
static int lpc178x_eint_clear(void);

#endif /* _MACH_LPC178X_TOUCH_H_ */
