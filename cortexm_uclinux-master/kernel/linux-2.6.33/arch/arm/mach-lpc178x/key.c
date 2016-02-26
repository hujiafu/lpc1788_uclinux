#include <linux/init.h>
#include <linux/platform_device.h>

#include <mach/key.h>
#include <mach/power.h>
#include <mach/platform.h>


#define LPC178X_GEINT_BASE	(LPC178X_APB_PERIPH_BASE + 0x00028080)
#define LPC178X_GEINT_IRQ	38	


static struct resource lpc178x_geint_resources[] = {		
        {								
                .start	= LPC178X_GEINT_BASE,			
                .end	= LPC178X_GEINT_BASE + 0x30 - 1,	
                .flags	= IORESOURCE_MEM,				
        },								
	{								
                .start	= LPC178X_GEINT_IRQ,			
                .flags	= IORESOURCE_IRQ,				
        },								
};									
struct platform_device lpc178x_geint_device = {			
	.name           = "lpc178x-key",					
	.id             = 0,						
	.num_resources  = ARRAY_SIZE(lpc178x_geint_resources),	
	.resource       = lpc178x_geint_resources,		
};


void __init lpc178x_key_init(void)
{
	platform_device_register(&lpc178x_geint_device);

}

