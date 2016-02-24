

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


i2c_register_board_info(0, at24C08_i2c_devices, ARRAY_SIZE(at24C08_i2c_devices));


