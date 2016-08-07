
#include <linux/init.h>
#include <linux/platform_device.h>

#include <mach/lpc178x.h>
#include <mach/eint-gpio.h>
#include <mach/platform.h>

#define LPC178X_EINT_BASE (LPC178X_APB_PERIPH_BASE + 0xFC140)
#define LPC178X_EINT0_IRQ	18
#define LPC178X_EINT0_ACTIVE	1
#define LPC178X_EINT1_ACTIVE	1
#define LPC178X_EINT2_ACTIVE	0
#define LPC178X_EINT3_ACTIVE	0

static struct plat_eint_data eint_data[] = {
	{
		.port0_active = LPC178X_EINT0_ACTIVE,
		.port1_active = LPC178X_EINT1_ACTIVE,
		.port2_active = LPC178X_EINT2_ACTIVE,
		.port3_active = LPC178X_EINT3_ACTIVE,
	},
	{ },
};

static struct resource lpc178x_eint_resources[] = {
	{
		.start = LPC178X_EINT_BASE,
		.end = LPC178X_EINT_BASE + 0x20 - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = LPC178X_EINT0_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device lpc178x_eint_device = {
	.name	= "lpc178x-eint",
	.id		= 0,
	.dev.platform_data = eint_data,
	.num_resources = ARRAY_SIZE(lpc178x_eint_resources),
	.resource	= lpc178x_eint_resources,
};

void __init lpc178x_eint_gpio_init(void)
{

	//LPC178X_EINT->pconp |= pconp_mask;
	platform_device_register(&lpc178x_eint_device);
}


