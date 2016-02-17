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
#include <linux/spi.h>

#include <mach/spi.h>
#include <mach/power.h>
#include <mach/platform.h>

/*
 * SPI interfaces' registers bases
 */
#define LPC178X_SPI0_BASE	(LPC178X_APB_PERIPH_BASE + 0x00088000)
#define LPC178X_SPI1_BASE	(LPC178X_APB_PERIPH_BASE + 0x00030000)
#define LPC178X_SPI2_BASE	(LPC178X_APB_PERIPH_BASE + 0x000AC000)
/*
 * "Interrupt ID" in Table 43 in the LPC178x/7x User Manual (page 70)
 */
#define LPC178X_SPI0_IRQ	14
#define LPC178X_SPI1_IRQ	15
#define LPC178X_SPI2_IRQ	36

/*
 * SPI platform devices and resources they use
 */
#define SPI_PLAT_DEVICE(uid)						\
static struct lpc1788_spi_mach_info spi## uid ##_data[] = {		       \
	{								       \
		.spiclk		= 0,	       \
	},								       \
	{  },								       \
};									       \
static struct resource lpc178x_spi## uid ##_resources[] = {		\
        {								\
				.start	= LPC178X_SPI## uid ##_BASE,			\
                .end	= LPC178X_SPI## uid ##_BASE + SZ_4K - 1,	\
                .flags	= IORESOURCE_MEM,				\
        },								\
	{								\
                .start	= LPC178X_SPI## uid ##_IRQ,			\
                .flags	= IORESOURCE_IRQ,				\
        },								\
};									\
struct platform_device lpc178x_spi## uid ##_device = {			\
	.name           = "lpc2k-spi",					\
	.id             = uid,						\
	.dev.platform_data	= spi## uid ##_data,	
	.num_resources  = ARRAY_SIZE(lpc178x_spi## uid ##_resources),	\
	.resource       = lpc178x_spi## uid ##_resources,		\
};



/*
 * Enable power on SPI, initialize clock rate and register platform device
 */
#define spi_init_and_register(uid) do {				       \
	lpc178x_periph_enable(LPC178X_SCC_PCONP_PCSSP## uid ##_MSK, 1);       \
	spi## uid ##_data[0].spiclk = lpc178x_clock_get(CLOCK_PCLK);		\
	(void) platform_device_register(&lpc178x_spi## uid ##_device);		       \
} while (0)


/*
 * Declare 3 platform devices
 */
#if defined(CONFIG_LPC178X_SPI0)
SPI_PLAT_DEVICE(0);
#endif

#if defined(CONFIG_LPC178X_SPI1)
SPI_PLAT_DEVICE(1);
#endif

#if defined(CONFIG_LPC178X_SPI2)
SPI_PLAT_DEVICE(2);
#endif

void __init lpc178x_spi_init(void)
{
	/*
	 * Register platform devices
	 */
#if defined(CONFIG_LPC178X_SPI0)
	spi_init_and_register(0);
#endif

#if defined(CONFIG_LPC178X_SPI1)
	spi_init_and_register(1);
#endif

#if defined(CONFIG_LPC178X_SPI2)
	spi_init_and_register(2);
#endif

}
