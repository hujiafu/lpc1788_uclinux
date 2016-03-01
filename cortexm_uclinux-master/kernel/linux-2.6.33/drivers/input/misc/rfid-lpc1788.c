

static int rfid_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct rfid_platform_data chip;
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
