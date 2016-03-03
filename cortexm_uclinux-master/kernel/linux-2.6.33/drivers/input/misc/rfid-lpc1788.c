
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


static struct file_operations dev_fops = {
	.owner = THIS_MODULE,
	.open = lpc178x_rfid_open,
	.release = lpc178x_rfid_close,
	.read = lpc178x_rfid_read,
	.poll = lpc178x_rfid_poll,
};

static struct miscdevice misc = {
	.minor = MISC_DYNAMIC_MINOR, 
	.name = "lpc178x_rfid",
	.fops = &dev_fops,
};

static int rfid_pin_count = 0;
static int rfid_pin_flag[10];
static int rfid_pin_no[10];


static irqreturn_t rfid_lpc1788_handler(int this_irq, void *dev_id)
{
	struct rfid_data *rfid = (struct rfid_data *) dev_id;
	unsigned int data;
	int i = 0;

	for(i=0; i<rfid_pin_count; i++){
		if(rfid_pin_flag[i] == 1){ //fall eage
			if(LPC178X_GPIO_GETPORT(rfid_pin_no[i]) == 0){
				data = btn_readl(rfid->chip.reg_base + LPC178X_INT_STATF0);
				if(data & (1<<LPC178X_GPIO_GETPIN(rfid_pin_no[i]))){
				
				}
			}
			if(LPC178X_GPIO_GETPORT(rfid_pin_no[i]) == 2){
				data = btn_readl(rfid->chip.reg_base + LPC178X_INT_STATF2)
				if(data & (1<<LPC178X_GPIO_GETPIN(rfid_pin_no[i]))){
				
				}
			}
		}else{	//rising eage
			if(LPC178X_GPIO_GETPORT(rfid_pin_no[i]) == 0){
				data = btn_readl(rfid->chip.reg_base + LPC178X_INT_STATR0);
				if(data & (1<<LPC178X_GPIO_GETPIN(rfid_pin_no[i]))){
				
				}
			}
			if(LPC178X_GPIO_GETPORT(rfid_pin_no[i]) == 2){
				data = btn_readl(rfid->chip.reg_base + LPC178X_INT_STATR2)
				if(data & (1<<LPC178X_GPIO_GETPIN(rfid_pin_no[i]))){
				
				}
			}
			
		}
	
	}
	if(LPC178X_GPIO_GETPORT(rfid->chip.irq_pin) == 2){
		data = btn_readl(rfid->chip.reg_base + LPC178X_INT_STATF2)
	}



}


static int rfid_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct rfid_platform_data chip;
	struct rfid_data *rfid;
	unsigned char id_num[4];
	bool use_smbus = false;
	int err;
	unsigned int i, num_addresses;

	if (client->dev.platform_data) {
		chip = *(struct rfid_platform_data *)client->dev.platform_data;
	} else {
		err = -ENODEV;
		goto err_out;
	}

	num_addresses = chip.num_addresses;
	if(num_address < 1){
		err = -ENODEV;
		goto err_out;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -EPFNOSUPPORT;
		goto err_out;
	}

	rfid = kzalloc(sizeof(struct rfid_data) +
                 num_addresses * sizeof(struct i2c_client *), GFP_KERNEL);
    if (!rfid) {
		err = -ENOMEM;
        goto err_out;
    }
	mutex_init(&rfid->lock);
	rfid->use_smbus = use_smbus;
	rfid->chip = chip;
	rfid->num_addresses = num_addresses;
	rfid->client[0] = client;

	/* use dummy devices for multiple-address chips */
	for (i = 1; i < num_addresses; i++) {
		rfid->client[i] = i2c_new_dummy(client->adapter,
					client->addr + i);
		if (!rfid->client[i]) {
			dev_err(&client->dev, "address 0x%02x unavailable\n",
					client->addr + i);
			err = -EADDRINUSE;
			goto err_clients;
		}
	}

	chip.reg_base = ioremap(chip.start_mem, chip.end_mem - chip.start_mem + 1);
	if (!chip.reg_base) {
		dev_err(&client->dev, "unable to map registers for "
			"gpio controller base=%08x\n", chip.start_mem);
		ret = -EINVAL;
		goto err_out;
	}


	ret = request_irq(chip.irq, rfid_lpc1788_handler, IRQF_DISABLED | IRQF_SHARED,
		"lpc178x-rfid", rfid);
	if (ret){
		err = -ENOMEM;
		goto err_out;
	}


	if(LPC178X_GPIO_GETPORT(chip.irq_pin) == 0){
		if(chip.irq_falleage & 1){
			rfid_writel(1<<LPC178X_GPIO_GETPIN(chip.irq_pin), chip->reg_base + LPC178X_INT_ENF0);
		}else{
			rfid_writel(1<<LPC178X_GPIO_GETPIN(chip.irq_pin), chip->reg_base + LPC178X_INT_ENR0);
		}
	}
	if(LPC178X_GPIO_GETPORT(chip.irq_pin) == 2){
		if(chip.irq_falleage & 1){
			rfid_writel(1<<LPC178X_GPIO_GETPIN(chip.irq_pin), chip->reg_base + LPC178X_INT_ENF2);
		}else{
			rfid_writel(1<<LPC178X_GPIO_GETPIN(chip.irq_pin), chip->reg_base + LPC178X_INT_ENR2);
		}
	}


	i2c_set_clientdata(client, rfid);

	sprintf(id_num, "%d", chip.id_num);
	strcat(misc.name, id_num);
	
	ret = misc_register(&misc);
	if (ret)
		goto err_out;



err_clients:
	for (i = 1; i < num_addresses; i++)
		if (rfid->client[i])
			i2c_unregister_device(rfid->client[i]);
err_out:
	dev_dbg(&client->dev, "probe error %d\n", err);
	return err;


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
