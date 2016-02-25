#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/i2c/at24.h>
#include <mach/lpc178x.h>
#include <mach/clock.h>
#include <mach/power.h>
#include <mach/gpio.h>
#include <mach/i2c.h>


static struct at24_platform_data board_eeprom = {
	.byte_len = 8192,
	.page_size = 16,
	.flags = AT24_FLAG_ADDR16,
};

static struct i2c_board_info at24C08_i2c_devices[] = {
    {
		I2C_BOARD_INFO("24c08", 0x50), /* A0=0, A1=0, A2=0 */
		.platform_data = &board_eeprom,
	},
};

void __init lpc178x_at24C08_init(void)
{
	i2c_register_board_info(0, at24C08_i2c_devices, ARRAY_SIZE(at24C08_i2c_devices));
}

