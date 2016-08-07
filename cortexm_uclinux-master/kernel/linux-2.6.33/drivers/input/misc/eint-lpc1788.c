
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
#include <mach/eint-gpio.h>
#include <asm/irq.h>


//#define BTN_DEBOUNCE_DELAY   (20 * 1000000)   /* ns delay before the first sample */

#define LPC178X_EXTINT		0x0
#define LPC178X_MODE		0x8
#define LPC178X_POLAR		0xC


#define LPC178X_GPIO_RISING	0x1
#define LPC178X_GPIO_FALLING 0x2

#define RFID_0_PIN	LPC178X_GPIO_MKPIN(0,29)
#define RFID_1_PIN	LPC178X_GPIO_MKPIN(0,30)

#define RFID_0_MSK LPC178X_GPIO_FALLING 
#define RFID_1_MSK LPC178X_GPIO_FALLING 

//#define BTN_NUM	4

static DECLARE_WAIT_QUEUE_HEAD(eint_waitq);
static volatile int ev_press = 0;
static volatile int eint_value;

struct eint_lpc1788 {
	unsigned int port;
	unsigned int pin;
	unsigned int pin_flag;
	unsigned int pin_num;
};

static struct eint_lpc1788 eint_data[] = {
	{
		.port = LPC178X_GPIO_GETPORT(RFID_0_PIN),
		.pin = LPC178X_GPIO_GETPIN(RFID_0_PIN),
		.pin_flag = RFID_0_MSK,
		.pin_num = RFID_0_PIN,
	},
	{
		.port = LPC178X_GPIO_GETPORT(RFID_1_PIN),
		.pin = LPC178X_GPIO_GETPIN(RFID_1_PIN),
		.pin_flag = RFID_1_MSK,
		.pin_num = RFID_1_PIN,
	},
};
/*
 * Driver verbosity level: 0->silent; >0->verbose (1 to 4, growing verbosity)
 */
static int eint_lpc1788_debug = 4;

#define EINT_LPC1788_DEBUG 1

#if defined(EINT_LPC1788_DEBUG)

#define eint_printk(level, fmt, args...)					\
	if (eint_lpc1788_debug >= level) printk(KERN_INFO "%s: " fmt,	\
				       	   __func__, ## args)

#else

#define eint_printk(level, fmt, args...)

#endif

struct lpc178x_eint {

	void __iomem		*reg_base;
	int			irq;
	int			status0;
	int			status2;
	spinlock_t              lock;
	struct hrtimer          timer;
};



static int lpc178x_eint_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int lpc178x_eint_close(struct inode *inode, struct file *file)
{
	return 0;
}

static int lpc178x_eint_read(struct file *file, char __user *buff, size_t count, loff_t *offset)
{
	unsigned long err;

	if(!ev_press){
		if(file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		else
			wait_event_interruptible(eint_waitq, ev_press);
	}

	ev_press = 0;

	err = copy_to_user(buff, (const void*)eint_value, min(sizeof(eint_value), count));
	return err ? -EFAULT : min(sizeof(eint_value), count);
}

static int lpc178x_eint_poll(struct file *file, struct poll_table_struct *wait)
{
	return 0;
}

static struct file_operations dev_fops = {
	.owner = THIS_MODULE,
	.open = lpc178x_eint_open,
	.release = lpc178x_eint_close,
	.read = lpc178x_eint_read,
	.poll = lpc178x_eint_poll,
};

static struct miscdevice misc = {
	.minor = MISC_DYNAMIC_MINOR, 
	.name = "lpc178x_eint",
	.fops = &dev_fops,
};

static inline unsigned long eint_readl(void __iomem *reg)
{
	return __raw_readl(reg);
}

static inline void eint_writel(unsigned long val, void __iomem *reg)
{
	__raw_writel(val, reg);
}

static enum hrtimer_restart eint_timer(struct hrtimer *handle)
{
#if 0
	struct lpc178x_gpio  *gpio = container_of(handle, struct lpc178x_gpio, timer);
	int i = 0;
	unsigned int data = 0;
	spin_lock(&gpio->lock);
	
	for(i=0; i<BTN_NUM; i++){
		if((btn_data[i].port) == 0){
			data = !gpio_get_value(btn_data[i].pin_num);
			if((data<<i) & gpio->status0){
				//report key down
				ev_press = 1;
				if(BTN_NUM < 32){
					key_value[0] |= 1<<i;
				}
				if(BTN_NUM >= 32){
					key_value[1] |= 1<<i;
				}
			}else{
				if(BTN_NUM < 32){
					key_value[0] &= ~(1<<i);
				}
				if(BTN_NUM >= 32){
					key_value[1] &= ~(1<<i);
				}
				gpio->status0 &= ~(1<<i);
			}
		}
		if((btn_data[i].port) == 2){
			data = !gpio_get_value(btn_data[i].pin_num);
			if((data<<i) & gpio->status2){
				ev_press = 1;
				
if(BTN_NUM < 32){
					key_value[0] |= 1<<i;
				}
				if(BTN_NUM >= 32){
					key_value[1] |= 1<<i;
				}
				//report key down
			}else{
				if(BTN_NUM < 32){
					key_value[0] &= ~(1<<i);
				}
				if(BTN_NUM >= 32){
					key_value[1] &= ~(1<<i);
				}
				gpio->status0 &= ~(1<<i);
			}
		}
	}	

	if(ev_press == 1){
		wake_up_interruptible(&eint_waitq);
	}

	spin_unlock(&gpio->lock);
	return HRTIMER_NORESTART;

#endif
}

static irqreturn_t eint3_lpc1788_handler(int this_irq, void *dev_id)
{
	struct lpc178x_eint *eint = (struct lpc178x_eint *) dev_id;
	int i = 0;
	int data = 0;

	printk("eint3_lpc1788_handler\n");

	data = eint_readl(eint->reg_base + LPC178X_EXTINT);
		
	if(data & 0x8){
		printk("eint3 flag set\n");
		eint_writel(0x8, eint->reg_base + LPC178X_EXTINT);
		eint_value = 0x8;
	}
	wake_up_interruptible(&eint_waitq);
	return IRQ_HANDLED;
}

static irqreturn_t eint2_lpc1788_handler(int this_irq, void *dev_id)
{
	struct lpc178x_eint *eint = (struct lpc178x_eint *) dev_id;
	int i = 0;
	int data = 0;

	printk("eint2_lpc1788_handler\n");

	data = eint_readl(eint->reg_base + LPC178X_EXTINT);
		
	if(data & 0x4){
		printk("eint2 flag set\n");
		eint_writel(0x4, eint->reg_base + LPC178X_EXTINT);
		eint_value = 0x4;
	}
	wake_up_interruptible(&eint_waitq);
	return IRQ_HANDLED;
}

static irqreturn_t eint1_lpc1788_handler(int this_irq, void *dev_id)
{
	struct lpc178x_eint *eint = (struct lpc178x_eint *) dev_id;
	int i = 0;
	int data = 0;

	printk("eint1_lpc1788_handler\n");

	data = eint_readl(eint->reg_base + LPC178X_EXTINT);
		
	if(data & 0x2){
		printk("eint1 flag set\n");
		eint_writel(0x2, eint->reg_base + LPC178X_EXTINT);
		eint_value = 0x2;
	}
	wake_up_interruptible(&eint_waitq);
	return IRQ_HANDLED;
}

static irqreturn_t eint0_lpc1788_handler(int this_irq, void *dev_id)
{
	struct lpc178x_eint *eint = (struct lpc178x_eint *) dev_id;
	int i = 0;
	int data = 0;

		printk("eint0_lpc1788_handler\n");

	data = eint_readl(eint->reg_base + LPC178X_EXTINT);
		
	if(data & 0x1){
		printk("eint0 flag set\n");
		eint_writel(0x1, eint->reg_base + LPC178X_EXTINT);
		eint_value = 0x1;
	}
	
	wake_up_interruptible(&eint_waitq);
#if 0
		for(i=0; i<BTN_NUM; i++){ 
			if((btn_data[i].port) == 0){
				//port 0
				if(btn_data[i].pin_flag & LPC178X_GPIO_FALLING){	
					data = btn_readl(gpio->reg_base + LPC178X_INT_STATF0);
					//TODO
					if(data & (1<<btn_data[i].pin)){
						//report key
						gpio->status0 |= (1<<i);
						btn_writel(1<<i, gpio->reg_base + LPC178X_INT_CLR0);
					}
				}	
				if(btn_data[i].pin_flag & LPC178X_GPIO_RISING){	
					data = btn_readl(gpio->reg_base + LPC178X_INT_STATR0);
					//TODO
					if(data & (1<<btn_data[i].pin)){
						gpio->status0 &= ~(1<<i);
						btn_writel(data, gpio->reg_base + LPC178X_INT_CLR0);
					}
				}	
			}	
			if((btn_data[i].port) == 2){
				//port 2
				if(btn_data[i].pin_flag & LPC178X_GPIO_FALLING){	
					data = btn_readl(gpio->reg_base + LPC178X_INT_STATF2);
					//TODO
					if(data & (1<<btn_data[i].pin)){
						gpio->status2 |= (1<<i);
						btn_writel(data, gpio->reg_base + LPC178X_INT_CLR2);
					}
				}	
				if(btn_data[i].pin_flag & LPC178X_GPIO_RISING){	
					data = btn_readl(gpio->reg_base + LPC178X_INT_STATR2);
					//TODO
					if(data & (1<<btn_data[i].pin)){
						gpio->status2 &= ~(1<<i);
						btn_writel(data, gpio->reg_base + LPC178X_INT_CLR2);
					}
				}	
			}	
		}

		if(gpio->status0 || gpio->status2){
			hrtimer_start(&gpio->timer, ktime_set(0, BTN_DEBOUNCE_DELAY), HRTIMER_MODE_REL);

		}
#endif		
		return IRQ_HANDLED;

}


static __devinit eint_probe(struct platform_device *pdev)
{
	struct lpc178x_eint *eint;
	struct plat_eint_data * eint_dev;
	struct resource *regs;
	int ret = 0;
	int i = 0;

	eint = kzalloc(sizeof(struct lpc178x_eint), GFP_KERNEL);
	if (!eint) {
		eint_printk(1, "Error allocating memory\n");
		ret = -ENOMEM;
		goto Error_release_nothing;
	}

	eint_dev = (struct plat_eint_data *)pdev->dev.platform_data;
	if(eint_dev == NULL){
		eint_printk(1, "no platform_data for eint controller %d\n", pdev->id);
		ret = -ENXIO;
                goto Error_release_nothing;
	}

	eint->irq = platform_get_irq(pdev, 0);
	if (eint->irq < 0) {
		eint_printk(1, "invalid IRQ %d for gpio contoller\n", eint->irq);
		ret = -ENXIO;
		goto Error_release_nothing;
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (! regs) {
		eint_printk(1, "no register base for gpio contoller\n");
		ret = -ENXIO;
		goto Error_release_nothing;
	}

	eint->reg_base = ioremap(regs->start, regs->end - regs->start + 1);
	if (!eint->reg_base) {
		eint_printk(1, "unable to map register for gpio controller base=%08x\n", regs->start);
		ret = -EINVAL;
		goto Error_release_nothing;
	}

	platform_set_drvdata(pdev, eint);

	eint_writel(0xf, eint->reg_base + LPC178X_MODE);
	eint_writel(0x0, eint->reg_base + LPC178X_POLAR);
	//ret = request_irq(gpio->irq, btn_lpc1788_handler, IRQF_DISABLED | IRQF_SHARED, "lpc178x-gpio", gpio);
	if(eint_dev->port0_active == 1){
		ret = request_irq(eint->irq, eint0_lpc1788_handler, IRQF_DISABLED, "lpc178x-eint", eint);
		if (ret){
			eint_printk(1, "request eint0 irq failed\n");
			goto Error_release_nothing;
		}
	}
	if(eint_dev->port1_active == 1){
		ret = request_irq(eint->irq + 1, eint1_lpc1788_handler, IRQF_DISABLED, "lpc178x-eint", eint);
		if (ret){
			eint_printk(1, "request eint1 irq failed\n");
			goto Error_release_nothing;
		}
	}
	if(eint_dev->port2_active == 1){
		ret = request_irq(eint->irq + 1, eint2_lpc1788_handler, IRQF_DISABLED, "lpc178x-eint", eint);
		if (ret){
			eint_printk(1, "request eint2 irq failed\n");
			goto Error_release_nothing;
		}
	}
	if(eint_dev->port3_active == 1){
		ret = request_irq(eint->irq + 1, eint3_lpc1788_handler, IRQF_DISABLED, "lpc178x-eint", eint);
		if (ret){
			eint_printk(1, "request eint3 irq failed\n");
			goto Error_release_nothing;
		}
	}

	ret = misc_register(&misc);
	if (ret){
		eint_printk(1, "misc_register failed\n");
		goto Error_release_nothing;
	}

#if 0
	for(i=0; i<BTN_NUM; i++){
		if((btn_data[i].port) == 0){
			if(btn_data[i].pin_flag & LPC178X_GPIO_RISING){
				btn_writel(1<<i, gpio->reg_base + LPC178X_INT_ENR0);
			}
			if(btn_data[i].pin_flag & LPC178X_GPIO_FALLING){
				btn_writel(1<<i, gpio->reg_base + LPC178X_INT_ENF0);
			}
		}
		if((btn_data[i].port) == 2){
			if(btn_data[i].pin_flag & LPC178X_GPIO_RISING){
				btn_writel(1<<i, gpio->reg_base + LPC178X_INT_ENR2);
			}
			if(btn_data[i].pin_flag & LPC178X_GPIO_FALLING){
				btn_writel(1<<i, gpio->reg_base + LPC178X_INT_ENF2);
			}
		}
	}
#endif

	hrtimer_init(&eint->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	eint->timer.function = eint_timer;

	spin_lock_init(&eint->lock);
	
	eint_printk(1, "eint probe successful\n");
	return 0;

Error_release_nothing:
Done:
	return ret;

}

static int eint_remove(struct platform_device *dev)
{
	eint_printk(1, "remove\n");
	misc_deregister(&misc);
	return 0;
}



static struct platform_driver eint_drv = {
	.probe = eint_probe,
	.remove = eint_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "lpc178x-eint",
	},
};

static int __init platform_eint_init(void)
{
	int ret;

	ret = platform_driver_register(&eint_drv);

	return ret;
}

static void __exit platform_eint_exit(void)
{
	platform_driver_unregister(&eint_drv);
}

module_init(platform_eint_init);
module_exit(platform_eint_exit);

MODULE_AUTHOR("hujiafu");
MODULE_LICENSE("GPL");


