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

#ifndef _MACH_LPC178X_PWM_H_
#define _MACH_LPC178X_PWM_H_

#include <linux/init.h>

/*
 * PWM controller registers base
 */

struct lpc178x_pwm {

	u32 pwm_ir;		//0x0
	u32 pwm_tcr;
	u32 pwm_tc;
	u32 pwm_pr;
	u32 pwm_pc;		//0x10
	u32 pwm_mcr;
	u32 pwm_mr0;
	u32 pwm_mr1;
	u32 pwm_mr2;	//0x20
	u32 pwm_mr3;
	u32 pwm_ccr;
	u32 pwm_cr0;
	u32 pwm_cr1;	//0x30
	u32	rcv0[3]
	u32 pwm_mr4;	//0x40
	u32 pwm_mr5;	
	u32 pwm_mr6;	//0x48
	u32 pwm_pcr;	//0x4c
	u32 pwm_ler;	//0x50
	u32 rcv1[7];
	u32 pwm_ctcr;	//0x70
};

struct lpc178x_pwm_mach_info {
         
	unsigned int pwm_nr;
	unsigned long max_frequence; 
};



#endif /* _MACH_LPC178X_PWM_H_ */
