#include <linux/init.h>
#include <linux/platform_device.h>

#include <mach/lpc178x.h>
#include <mach/power.h>
#include <mach/platform.h>

#define LPC178X_UART2_BASE	(LPC178X_APB_PERIPH_BASE + 0x00098000)
#define LPC178X_UART2_IRQ	7	


static struct resource lpc178x_uart2_resources[] = {
        {
                .start  = LPC178X_UART2_BASE,
                .end    = LPC178X_UART2_BASE + 0x60 - 1,
                .flags  = IORESOURCE_MEM,
        },
        {
                .start  = LPC178X_UART2_IRQ,
                .flags  = IORESOURCE_IRQ,
        },
};


static struct platform_device lpc178x_ts_device = {
	.name = "ts-lpc178x",
	.id = 0,
	.num_resources  = ARRAY_SIZE(lpc178x_uart2_resources),
	.resource = lpc178x_uart2_resources,
};

void __init lpc178x_ts_init(void)
{

	platform_device_register(&lpc178x_ts_device);

}

