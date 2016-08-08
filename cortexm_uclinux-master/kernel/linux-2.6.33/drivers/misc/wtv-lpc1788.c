
#include <linux/hwmon.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <mach/touch.h>
#include <mach/wtv.h>
#include <asm/irq.h>


static struct  wtv_platform_data *wtv_pdata;

/*
 * Driver verbosity level: 0->silent; >0->verbose (1 to 4, growing verbosity)
 */
static int wtv_lpc1788_debug = 4;

#define WTV_LPC1788_DEBUG 1

#if defined(WTV_LPC1788_DEBUG)

#define wtv_printk(level, fmt, args...)					\
	if (wtv_lpc1788_debug >= level) printk(KERN_INFO "%s: " fmt,	\
				       	   __func__, ## args)

#else

#define wtv_printk(level, fmt, args...)

#endif


static int lpc178x_wtv_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int lpc178x_wtv_close(struct inode *inode, struct file *file)
{
	return 0;
}

static int lpc178x_wtv_write(struct file *file, const char __user *buff, size_t count, loff_t *offset)
{
	char data = *buff;
	int i = 0, j = 0;

	//printk("lpc178x_wtv_wirte start %x %d\n", data, count);
	//if(count != 1){
	//	wtv_printk(1, "wtv only support 1 byte write\n");
	//	return -EFAULT;
	//}
	//gpio_set_value(wtv_pdata->reset, 0);
	//mdelay(6); //2ms < tsc < 10ms
	//gpio_set_value(wtv_pdata->reset, 1);
	//mdelay(70); //2ms < tsc < 10ms

	gpio_set_value(wtv_pdata->cs, 0);
	gpio_set_value(wtv_pdata->clock, 1);

	mdelay(5); //2ms < tsc < 10ms

	for(j=0; j<count; j++){
		data = buff[j];
		for(i=0; i<8; i++){
			gpio_set_value(wtv_pdata->clock, 0);
			if(data & 0x1){
				gpio_set_value(wtv_pdata->data, 1);
			}else{
				gpio_set_value(wtv_pdata->data, 0);
			}
			data = data >> 1;
			udelay(500); // 100us < tdh < 1000us
			gpio_set_value(wtv_pdata->clock, 1);
			udelay(500); // 100us < tsckw < 1000us
		}
		gpio_set_value(wtv_pdata->data, 1);
		mdelay(6); //tch > 20us
	}
	//udelay(50);
	mdelay(1); //tch > 20us
	gpio_set_value(wtv_pdata->cs, 1);
	printk("lpc178x_wtv_wirte finish\n");

	return 1;
}

static int lpc178x_wtv_read(struct file *file, char __user *buff, size_t count, loff_t *offset)
{
	return 0;
}


static struct file_operations dev_fops = {
	.owner = THIS_MODULE,
	.open = lpc178x_wtv_open,
	.release = lpc178x_wtv_close,
	.read = lpc178x_wtv_read,
	.write = lpc178x_wtv_write,
};

static struct miscdevice misc = {
	.minor = MISC_DYNAMIC_MINOR, 
	.name = "lpc178x_wtv",
	.fops = &dev_fops,
};


static __devinit wtv_probe(struct platform_device *pdev)
{
	struct lpc178x_gpio *gpio;
	struct resource *regs;
	struct wtv_platform_data	*pdata = pdev->dev.platform_data;
	int ret = 0;
	int i = 0;


	wtv_pdata = pdata;
	if(wtv_pdata == NULL){
		wtv_printk(1, "Error get platform data\n");
		ret = -EINVAL;
		goto Error_release_nothing;
	}

	wtv_printk(1, "cs pin = p(%d, %d)\n", LPC178X_GPIO_GETPORT(wtv_pdata->cs), LPC178X_GPIO_GETPIN(wtv_pdata->cs));
	wtv_printk(1, "reset pin = p(%d, %d)\n", LPC178X_GPIO_GETPORT(wtv_pdata->reset), LPC178X_GPIO_GETPIN(wtv_pdata->reset));
	wtv_printk(1, "clock pin = p(%d, %d)\n", LPC178X_GPIO_GETPORT(wtv_pdata->clock), LPC178X_GPIO_GETPIN(wtv_pdata->clock));
	wtv_printk(1, "data pin = p(%d, %d)\n", LPC178X_GPIO_GETPORT(wtv_pdata->data), LPC178X_GPIO_GETPIN(wtv_pdata->data));

	platform_set_drvdata(pdev, gpio);

	gpio_direction_output(pdata->cs, 1);
	gpio_direction_output(pdata->reset, 1);
	gpio_direction_output(pdata->clock, 1);
	gpio_direction_output(pdata->data, 1);

	gpio_set_value(pdata->reset, 0);
	msleep(6);
	gpio_set_value(pdata->reset, 1);

	ret = misc_register(&misc);
	if (ret){
		wtv_printk(1, "misc_register failed\n");
		goto Error_release_nothing;
	}

	
	wtv_printk(1, "wtv probe successful\n");
	return 0;

Error_release_nothing:
Done:
	return ret;

}

static int wtv_remove(struct platform_device *dev)
{
	wtv_printk(1, "remove\n");
	misc_deregister(&misc);
	return 0;
}



static struct platform_driver wtv_drv = {
	.probe = wtv_probe,
	.remove = wtv_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "lpc178x-wtv",
	},
};

static int __init platform_wtv_init(void)
{
	int ret;

	ret = platform_driver_register(&wtv_drv);

	return ret;
}

static void __exit platform_wtv_exit(void)
{
	platform_driver_unregister(&wtv_drv);
}

module_init(platform_wtv_init);
module_exit(platform_wtv_exit);

MODULE_AUTHOR("hujiafu");
MODULE_LICENSE("GPL");


