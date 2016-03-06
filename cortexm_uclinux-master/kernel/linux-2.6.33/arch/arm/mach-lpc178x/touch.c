#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/spi/spi.h>
#include <linux/spi/tsc2046.h>
#include <mach/lpc178x.h>
#include <mach/clock.h>
#include <mach/touch.h>
#include <mach/power.h>
#include <mach/gpio.h>



#define LPC178X_SYSCON_BASE		(LPC178X_APB_PERIPH_BASE + 0x000FC000)
#define LPC178X_EINT_BASE		(LPC178X_SYSCON_BASE + 0x140)

#define LPC178X_EINT0_IRQ	18
#define LPC178X_EINT1_IRQ	19
#define LPC178X_EINT2_IRQ	20
#define LPC178X_EINT3_IRQ	21

#define LPC178X_EINT_0_FLAG		(0x1<<0)
#define LPC178X_EINT_1_FLAG		(0x1<<1)
#define LPC178X_EINT_2_FLAG		(0x1<<2)
#define LPC178X_EINT_3_FALG		(0x1<<3)

#define LPC178X_EINT_0_EAGE		(0x1<<0)
#define LPC178X_EINT_1_EAGE		(0x1<<1)
#define LPC178X_EINT_2_EAGE		(0x1<<2)
#define LPC178X_EINT_3_EAGE		(0x1<<3)

#define LPC178X_EINT_0_FALL		~(0x1<<0)
#define LPC178X_EINT_1_FALL		~(0x1<<1)
#define LPC178X_EINT_2_FALL		~(0x1<<2)
#define LPC178X_EINT_3_FALL		~(0x1<<3)

#define LPC178X_EINT			((volatile struct lpc178x_eint_regs *)LPC178X_EINT_BASE)


#define TOUCH_PLAT_DEVICE(uid)						\
static struct tsc2046_platform_data tsc2046_pldata = {		\
	.swap_xy = 0,		\
	.vref_mv = 2500,	\
	.model = 2046,		\
	.vref_delay_usecs = 100,	\
	.x_plate_ohms = 400,		\
	.pressure_max = 15000,		\
	.debounce_max = 2,		\
	.debounce_rep = 1,		\
	.debounce_tol = ~(0),		\
	.penirq_recheck_delay_usecs = 100,	\
	.settle_delay_usecs = 150,	\
	.keep_vref_on = 1,		\
	.gpio_pendown = LPC178X_GPIO_MKPIN(2,11),	\
	.io_setup = lpc178x_touch_iosetup,				\
	.eint_clear = lpc178x_eint_clear,				\
};									\
static struct spi_board_info __initdata lpc178x_spi_devs[] = {			\
	{												\
		.modalias           = "tsc2046",					\
		.bus_num             = 0,\
		.chip_select	= 0, \
		.irq = LPC178X_EINT## uid ##_IRQ,	\
		.mode		= SPI_MODE_3,	\
		.max_speed_hz	= 100000,	\
		.platform_data  = &tsc2046_pldata,	\
	},												\
}

#if defined(CONFIG_LPC178X_TOUCH0)
TOUCH_PLAT_DEVICE(0);
#endif

#if defined(CONFIG_LPC178X_TOUCH1)
TOUCH_PLAT_DEVICE(1);
#endif

#if defined(CONFIG_LPC178X_TOUCH2)
TOUCH_PLAT_DEVICE(2);
#endif

#if defined(CONFIG_LPC178X_TOUCH3)
TOUCH_PLAT_DEVICE(3);
#endif

static int lpc178x_touch_iosetup(void)
{

#if defined(CONFIG_LPC178X_TOUCH0)
	LPC178X_EINT->extmode |= LPC178X_EINT_0_EAGE;	
	LPC178X_EINT->extpolar &= LPC178X_EINT_0_FALL;	
#endif	

#if defined(CONFIG_LPC178X_TOUCH1)
	LPC178X_EINT->extmode |= LPC178X_EINT_1_EAGE;	
	LPC178X_EINT->extpolar &= LPC178X_EINT_1_FALL;	
#endif	

#if defined(CONFIG_LPC178X_TOUCH2)
	LPC178X_EINT->extmode |= LPC178X_EINT_2_EAGE;	
	LPC178X_EINT->extpolar &= LPC178X_EINT_2_FALL;	
#endif	

#if defined(CONFIG_LPC178X_TOUCH3)
	LPC178X_EINT->extmode |= LPC178X_EINT_3_EAGE;	
	LPC178X_EINT->extpolar &= LPC178X_EINT_3_FALL;	
#endif	

}

static int lpc178x_eint_clear(void)
{

#if defined(CONFIG_LPC178X_TOUCH0)
	LPC178X_EINT->extint = LPC178X_EINT_0_FLAG;	
#endif	

#if defined(CONFIG_LPC178X_TOUCH1)
	LPC178X_EINT->extint = LPC178X_EINT_1_FLAG;	
#endif	

#if defined(CONFIG_LPC178X_TOUCH2)
	LPC178X_EINT->extint = LPC178X_EINT_2_FLAG;	
#endif	

#if defined(CONFIG_LPC178X_TOUCH3)
	LPC178X_EINT->extint = LPC178X_EINT_3_FLAG;	
#endif	
}

void __init lpc178x_touch_init(void)
{
	spi_register_board_info(lpc178x_spi_devs, ARRAY_SIZE(lpc178x_spi_devs));
}



