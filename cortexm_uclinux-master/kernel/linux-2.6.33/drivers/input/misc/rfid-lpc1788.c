
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/mod_devicetable.h>
#include <linux/log2.h>
#include <linux/bitops.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/i2c/rfid.h>
#include <linux/uaccess.h>

#define DEV_NAME	"rfid"
#define RFID_MAX_NUM	4

static struct i2c_device_id rfid_ids[]=
{
		{"lpc1788_rfid1",0xA0},
		{"lpc1788_rfid2",0xB0},
			{},
};



static struct class *rfid_dev_class;
static struct i2c_client *my_client[RFID_MAX_NUM];
static int client_count = 0;
static unsigned char mifarecode[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

#if 0
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
#endif


static ssize_t lpc178x_rfid_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	struct i2c_client *client= (struct i2c_client *)file->private_data;
	struct i2c_adapter *adap;
	struct i2c_msg msg[2];
	unsigned char write_buf[11];
	unsigned char read_buf[20];
	int xor;
	int i;
	int ret = 0;

	if(count < 19){
		printk(KERN_ERR "rfid read buf less 19\n");
		goto out;
	}

	write_buf[0] = 0xA;
	write_buf[1] = 0x21;
	write_buf[2] = 0x0;
	write_buf[3] = 0x0;
	write_buf[4] = mifarecode[0];
	write_buf[5] = mifarecode[1];
	write_buf[6] = mifarecode[2];
	write_buf[7] = mifarecode[3];
	write_buf[8] = mifarecode[4];
	write_buf[9] = mifarecode[5];
	xor = write_buf[0];
	for(i=1; i<write_buf[0]; i++){
		xor ^= write_buf[i];
	}
	write_buf[i] = xor;

	if(client == NULL){
		printk(KERN_ERR "rfid i2c_client is NULL\n");
		goto out;
	}

	adap = client->adapter;

	msg[0].addr = (client->addr) >> 1;
	msg[0].len = write_buf[0] + 1;
	msg[0].buf = write_buf;
	printk("client->addr %x\n", msg[0].addr);

	//msg[1].addr = client->addr;
	//msg[1].flags = I2C_M_RD | I2C_M_RECV_LEN;
	//msg[1].len = 19;
	//msg[1].buf = read_buf;//read data

	ret = i2c_transfer(adap, msg, 1);
	printk(KERN_ERR "i2c_transfer:ret=%d\n",ret);
	if (ret != 0){
		printk(KERN_ERR "rfid i2c_transfer error\n");
		goto out;
	}
	
	msg[0].addr = (client->addr) >> 1;
	msg[0].flags = I2C_M_RD | I2C_M_RECV_LEN;
	msg[0].len = 19;
	msg[0].buf = read_buf;//read data
	ret = i2c_transfer(adap, msg, 1);
	printk(KERN_ERR "read i2c_transfer:ret=%d\n",ret);

	if (ret != 0){
		printk(KERN_ERR "rfid i2c_transfer error\n");
		goto out;
	}
	ret = copy_to_user(buf, read_buf, msg[0].len);
	for(i=0; i<msg[0].len; i++){
		printk("%x ", read_buf[i]);
	}
	printk("\n");
	printk(KERN_ERR "copy_to_user:ret=%d\n",ret);
	if(ret)
		goto out;
	return count;
;out:
	return -1;
}


static int lpc178x_rfid_open(struct inode *inode, struct file *file)
{
	int minor = MINOR(inode->i_rdev);

	printk(KERN_ERR "i2cdev_open %d\n", minor);


	if(my_client[minor] != NULL){	
		file->private_data = my_client[minor];
	}else{
		return -1;
	}

	return 0;
}


static int lpc178x_rfid_close(struct inode *inode, struct file *file)
{
	printk(KERN_ERR "lpc178x_rfid_close\n");
	file->private_data = NULL;
	return 0;
}


static struct file_operations dev_fops = {
	.owner = THIS_MODULE,
	.open = lpc178x_rfid_open,
	.release = lpc178x_rfid_close,
	.read = lpc178x_rfid_read,
};
#if 0
static struct miscdevice misc = {
	.minor = MISC_DYNAMIC_MINOR, 
	.name = "lpc178x_rfid",
	.fops = &dev_fops,
};
#endif
#if 0
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
#endif

static int rfid_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct rfid_platform_data chip;
	struct device *dev;
	unsigned char id_num[4];
	bool use_smbus = false;
	int err;
	unsigned int i, num_addresses;

	printk(KERN_ERR "probe:name = %s,flag =%d,addr = %x,adapter = %d,driver = %s\n",client->name,  
        client->flags,client->addr,client->adapter->nr,client->driver->driver.name );	
	
	if (client->dev.platform_data) {
		chip = *(struct rfid_platform_data *)client->dev.platform_data;
	} else {
		err = -ENODEV;
		goto err_out;
	}
	
	my_client[client_count] = client;
	client_count++;

	dev = device_create(rfid_dev_class, &client->dev,
						 MKDEV(RFID_MAJOR, chip.dev_id), NULL,
						 "rfid-%d", chip.dev_id);
	if (IS_ERR(dev)) {
		err = PTR_ERR(dev);
        goto err_out;
    }

	
	printk(KERN_ERR "%s: rfid_probe successful\n", __FILE__);
	return 0;
 


#if 0

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
#endif	

err_out:
	dev_dbg(&client->dev, "probe error %d\n", err);
	return err;
}


static int rfid_remove(struct i2c_client *client)
{
	struct rfid_platform_data chip;
	
	printk(KERN_ERR "rfid_remove\n");
		
	if (client->dev.platform_data) {
		chip = *(struct rfid_platform_data *)client->dev.platform_data;
		device_destroy(rfid_dev_class, MKDEV(RFID_MAJOR, chip.dev_id));	
	}
	return 0;

}


static struct i2c_driver rfid_driver = {
	.driver = {
		.name = "rfid-lpc1788",
	},
	.probe = rfid_probe,
	.remove =  __devexit_p(rfid_remove),
	.id_table = rfid_ids,
};

static int __init rfid_init(void)
{
	int res;

	res = register_chrdev(RFID_MAJOR, DEV_NAME, &dev_fops);
	if (res){
		printk(KERN_ERR "%s: register_chrdev failed\n", __FILE__);
		goto out;
	}
	
	rfid_dev_class = class_create(THIS_MODULE, DEV_NAME);
	if (IS_ERR(rfid_dev_class)) {
		res = PTR_ERR(rfid_dev_class);
		printk(KERN_ERR "%s: class_create failed\n", __FILE__);
		goto out_unreg_chrdev;
	}

	res = i2c_add_driver(&rfid_driver);
	if (res){
		printk(KERN_ERR "%s: i2c_add_driver failed\n", __FILE__);
		goto out_unreg_class;
	}

	return res;
out_unreg_class:
	class_destroy(rfid_dev_class);
out_unreg_chrdev:
	unregister_chrdev(RFID_MAJOR, DEV_NAME);
out:
	printk(KERN_ERR "%s: Driver Initialisation failed\n", __FILE__);
	return res;

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
