#include <linux/init.h>
#include <linux/platform_device.h>

#include <mach/pwm.h>
#include <mach/power.h>
#include <mach/platform.h>



#define LPC178X_PWM0_BASE	(LPC178X_APB_PERIPH_BASE + 0x00014000)
#define LPC178X_PWM1_BASE	(LPC178X_APB_PERIPH_BASE + 0x00018000)


#define PWM_PLAT_DEVICE(uid)	\
static struct lpc178x_pwm_mach_info lpc178x_pwm## uid ##_info __initdata = { \
         .pwm_nr    = 0, \
}; \
static struct resource lpc178x_pwm## uid ##_resources[] = {		\	
        {								\
                .start	= LPC178X_PWM## uid ##_BASE,	\			
                .end	= LPC178X_PWM## uid ##_BASE + 0x80,	\	
                .flags	= IORESOURCE_MEM,				\
        },								\
};									\
struct platform_device lpc178x_pwm## uid ##_device = {	\			
	.name           = "lpc178x-pwm",				\	
	.id             = uid,						\
	.num_resources  = ARRAY_SIZE(lpc178x_pwm## uid ##_resources),\	
	.resource       = lpc178x_pwm## uid ##_resources,		\
}

#if defined(CONFIG_LPC178X_PWM0)
PWM_PLAT_DEVICE(0);
#endif

#if defined(CONFIG_LPC178X_PWM1)
PWM_PLAT_DEVICE(1);
#endif

void __init lpc178x_pwm_init(void)
{

#if defined(CONFIG_LPC178X_PWM0)
	lpc178x_pwm0_info.pwm_nr = 2;
	lpc178x_pwm0_info.max_frequence = 1000;
	lpc178x_pwm0_device.dev.platform_data = &lpc178x_pwm0_info;
	platform_device_register(&lpc178x_pwm0_device);
#endif

#if defined(CONFIG_LPC178X_PWM1)
	lpc178x_pwm1_info.pwm_nr = 0;
	lpc178x_pwm1_info.max_frequence = 1000;
	lpc178x_pwm1_device.dev.platform_data = &lpc178x_pwm1_info;
	platform_device_register(&lpc178x_pwm1_device);
#endif

}

