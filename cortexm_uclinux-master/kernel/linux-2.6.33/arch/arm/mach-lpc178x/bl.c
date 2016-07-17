#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/pwm_backlight.h>

#include <mach/pwm.h>
#include <mach/power.h>
#include <mach/platform.h>

static int lpc178x_backlight_init(struct device *dev)
{
	return 0;
}

static struct platform_pwm_backlight_data lpc178x_backlight_data = {
	.pwm_id	= 1,
	.max_brightness = 100,
	.dft_brightness = 80,
	.pwm_period_ns	= 5000,
	//.pwm_period_ns	= 3300000,
	.init = lpc178x_backlight_init,
};

static struct platform_device lpc178x_bl_device = {
	.name = "pwm-backlight",
	.dev = {
		.platform_data = &lpc178x_backlight_data,
	},
	.id = 0,
};

void __init lpc178x_bl_init(void)
{

	platform_device_register(&lpc178x_bl_device);

}

