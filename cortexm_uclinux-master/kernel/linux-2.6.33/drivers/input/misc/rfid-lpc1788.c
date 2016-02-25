
struct rfid_data {
        struct rfid_platform_data chip;
        bool use_smbus;

        /*  
         * Lock protects against activities from other Linux tasks,
         * but not from changes by other I2C masters.
         */
        struct mutex lock;

        u8 *writebuf;
        unsigned write_max;
        unsigned num_addresses;

        /*  
         * Some chips tie up multiple I2C addresses; dummy devices reserve
         * them for us, and we'll use them with SMBus calls.
         */
        struct i2c_client *client[];
};




static int rfid_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct rfid_platform_data chip;
	struct rfid_data *rfid;
	int err;

	if (client->dev.platform_data) {
		chip = *(struct rfid_platform_data *)client->dev.platform_data;
	} else {
		err = -ENODEV;
		return err;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -EPFNOSUPPORT;
		return err;
	}

	rfid = kzalloc(sizeof(struct rfid_data) +
                1 * sizeof(struct i2c_client *), GFP_KERNEL);
        if (!rfid) {
                err = -ENOMEM;
                goto err_out;
        }


}





static struct i2c_driver rfid_driver = {
	.driver = {
		.name = "rfid-lpc1788",
		.owner = THIS_MODULE,
	},
	.probe = rfid_probe,
	.remove = __devexit_p(rfid_remove),
	.id_table = rfid_ids,
};

static int __init rfid_init(void)
{
	return i2c_add_driver(&rfid_driver);
}
module_init(rfid_init);

static void __exit rfid_exit(void)
{
	i2c_del_driver(&rfid_driver);
}
module_exit(rfid_exit);

MODULE_DESCRIPTION("Driver for RFID I2C DEVICE");
MODULE_AUTHOR("Hujiafu");
MODULE_LICENSE("GPL");
