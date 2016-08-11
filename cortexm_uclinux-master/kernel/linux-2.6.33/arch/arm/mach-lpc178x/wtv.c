#include <linux/init.h>
#include <linux/platform_device.h>

#include <mach/key.h>
#include <mach/power.h>
#include <mach/platform.h>
#include <mach/gpio.h>
#include <mach/wtv.h>


static struct wtv_platform_data wtv_pldata = {
	.cs = LPC178X_GPIO_MKPIN(1,2),
	.reset = LPC178X_GPIO_MKPIN(5,4),
	.clock = LPC178X_GPIO_MKPIN(1,3),	
	.data = LPC178X_GPIO_MKPIN(5,0),
};


struct platform_device lpc178x_wtv_device = {			
	.name           = "lpc178x-wtv",					
	.id             = 0,
	.dev			= {
		.platform_data  = &wtv_pldata,
	}
};


void __init lpc178x_wtv_init(void)
{
	platform_device_register(&lpc178x_wtv_device);

}

