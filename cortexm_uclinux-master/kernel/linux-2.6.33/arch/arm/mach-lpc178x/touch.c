#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/io.h>
#include <linux/spi.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <mach/lpc178x.h>
#include <mach/clock.h>
#include <mach/touch.h>
#include <mach/power.h>



#define LPC178X_SYSCON_BASE		(LPC178X_APB_PERIPH_BASE + 0x000FC000)
#define LPC178X_EINT_BASE		(LPC178X_SYSCON_BASE + 0x140)

#define LPC178X_EINT1_IRQ	19
#define LPC178X_EINT2_IRQ	20
#define LPC178X_EINT3_IRQ	21



#define TOUCH_PLAT_DEVICE(uid)						\
static struct touch_platdata tsc2046_config = {		\
	.mem_start = LPC178X_SYSCON_BASE,				\
	.mem_size = SZ_4K,								\
	.irq = LPC178X_EINT## uid ##_IRQ,				\
};									\
static struct spi_board_info __initdata lpc178x_spi_devs[] = {			\
	{												\
		.modalias           = "ads7846",					\
		.bus_num             = 0,\
		.chip_select	= 0, \
		.mode		= SPI_MODE_3,	\
		.max_speed_hz	= 100000,	\
		.platform_data  = &tsc2046_config,	\
	},												\
}

#if defined(CONFIG_LPC178X_TOUCH_1)
TOUCH_PLAT_DEVICE(1);
#endif

#if defined(CONFIG_LPC178X_TOUCH_2)
TOUCH_PLAT_DEVICE(2);
#endif

#if defined(CONFIG_LPC178X_TOUCH_3)
TOUCH_PLAT_DEVICE(3);
#endif

void __init lpc178x_touch_init(void)
{
	spi_register_board_info(lpc178x_spi_devs, ARRAY_SIZE(lpc178x_spi_devs));
}



