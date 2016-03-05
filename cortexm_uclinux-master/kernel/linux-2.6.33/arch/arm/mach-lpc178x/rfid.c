#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/i2c/rfid.h>
#include <mach/lpc178x.h>
#include <mach/rfid.h>
#include <mach/clock.h>
#include <mach/power.h>
#include <mach/gpio.h>
#include <mach/i2c.h>

static struct rfid_platform_data board_info_rfid0 = {
	.dev_id = 0,
};	

static struct rfid_platform_data board_info_rfid1 = {
	.dev_id = 1,
};	

static struct i2c_board_info rfid_i2c_devices[] = {
    {
		I2C_BOARD_INFO("lpc1788_rfid1", 0xA0), /* A0=0, A1=0, A2=0 */
		.platform_data = &board_info_rfid0,
	},
    {
		I2C_BOARD_INFO("lpc1788_rfid2", 0xB0), /* A0=0, A1=0, A2=0 */
		.platform_data = &board_info_rfid1,
	},
};

void __init lpc178x_rfid_init(void)
{
	i2c_register_board_info(0, rfid_i2c_devices, ARRAY_SIZE(rfid_i2c_devices));
}

