#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/io.h>
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
static struct resource lpc178x_touch_resources[] = {		\
        {								\
                .start	= LPC178X_SYSCON_BASE,			\
                .end	= LPC178X_SYSCON_BASE + SZ_4K - 1,	\
                .flags	= IORESOURCE_MEM,				\
        },								\
	{								\
                .start	= LPC178X_EINT## uid ##_IRQ,			\
                .flags	= IORESOURCE_IRQ,				\
        },								\
};									\
struct platform_device lpc178x_touch_device = {			\
	.name           = "ads7846",					\
	.id             = uid,						\
	.num_resources  = ARRAY_SIZE(lpc178x_touch_resources),	\
	.resource       = lpc178x_touch_resources,		\
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
	platform_device_register(&lpc178x_touch_device);	
}



