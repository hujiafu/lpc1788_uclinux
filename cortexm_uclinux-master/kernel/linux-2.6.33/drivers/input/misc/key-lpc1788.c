
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
#include <asm/irq.h>


#define BTN_DEBOUNCE_DELAY   (20 * 1000000)   /* ns delay before the first sample */


#define LPC178X_INT_STATUS	0x0	
#define LPC178X_INT_STATR0	0x4	
#define LPC178X_INT_STATF0	0x8	
#define LPC178X_INT_CLR0	0xC	
#define LPC178X_INT_ENR0	0x10	
#define LPC178X_INT_ENF0	0x14
#define LPC178X_INT_STATR2	0x24	
#define LPC178X_INT_STATF2	0x28	
#define LPC178X_INT_CLR2	0x2C	
#define LPC178X_INT_ENR2	0x30	
#define LPC178X_INT_ENF2	0x34

#define LPC178X_GPIO_RISING	0x1
#define LPC178X_GPIO_FALLING 0x2

#define BTN_0_PIN	LPC178X_GPIO_MKPIN(0,1)
#define BTN_1_PIN	LPC178X_GPIO_MKPIN(2,4)
#define RFID_0_PIN	LPC178X_GPIO_MKPIN(2,2)
#define RFID_1_PIN	LPC178X_GPIO_MKPIN(2,3)

#define BTN_0_MSK LPC178X_GPIO_FALLING 
#define BTN_1_MSK LPC178X_GPIO_FALLING 
#define RFID_0_MSK LPC178X_GPIO_FALLING 
#define RFID_1_MSK LPC178X_GPIO_FALLING 

#define BTN_NUM	4

static DECLARE_WAIT_QUEUE_HEAD(button_waitq);
static volatile int ev_press = 0;
static volatile int key_value[2];

struct btn_lpc1788 {
	unsigned int port;
	unsigned int pin;
	unsigned int pin_flag;
	unsigned int pin_num;
};

static struct btn_lpc1788 btn_data[] = {
	{
		.port = LPC178X_GPIO_GETPORT(BTN_0_PIN),
		.pin = LPC178X_GPIO_GETPIN(BTN_0_PIN),
		.pin_flag = BTN_0_MSK,
		.pin_num = BTN_0_PIN,
	},
	{
		.port = LPC178X_GPIO_GETPORT(BTN_1_PIN),
		.pin = LPC178X_GPIO_GETPIN(BTN_1_PIN),
		.pin_flag = BTN_1_MSK,
		.pin_num = BTN_1_PIN,
	},
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
static int btn_lpc1788_debug = 4;

#define BTN_LPC1788_DEBUG 1

#if defined(BTN_LPC1788_DEBUG)

#define btn_printk(level, fmt, args...)					\
	if (btn_lpc1788_debug >= level) printk(KERN_INFO "%s: " fmt,	\
				       	   __func__, ## args)

#else

#define btn_printk(level, fmt, args...)

#endif

struct lpc178x_gpio {

	void __iomem		*reg_base;
	int			irq;
	int			status0;
	int			status2;
	spinlock_t              lock;
	struct hrtimer          timer;
};



static int lpc178x_btn_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int lpc178x_btn_close(struct inode *inode, struct file *file)
{
	return 0;
}

static int lpc178x_btn_read(struct file *file, char __user *buff, size_t count, loff_t *offset)
{
	unsigned long err;

	if(!ev_press){
		if(file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		else
			wait_event_interruptible(button_waitq, ev_press);
	}

	ev_press = 0;

	err = copy_to_user(buff, (const void*)key_value, min(sizeof(key_value), count));
	return err ? -EFAULT : min(sizeof(key_value), count);
}

static int lpc178x_btn_poll(struct file *file, struct poll_table_struct *wait)
{
	return 0;
}

static struct file_operations dev_fops = {
	.owner = THIS_MODULE,
	.open = lpc178x_btn_open,
	.release = lpc178x_btn_close,
	.read = lpc178x_btn_read,
	.poll = lpc178x_btn_poll,
};

static struct miscdevice misc = {
	.minor = MISC_DYNAMIC_MINOR, 
	.name = "lpc178x_btn",
	.fops = &dev_fops,
};

static inline unsigned long btn_readl(void __iomem *reg)
{
	return __raw_readl(reg);
}

static inline void btn_writel(unsigned long val, void __iomem *reg)
{
	__raw_writel(val, reg);
}

static enum hrtimer_restart btn_timer(struct hrtimer *handle)
{
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
		wake_up_interruptible(&button_waitq);
	}

	spin_unlock(&gpio->lock);
	return HRTIMER_NORESTART;

}


static irqreturn_t btn_lpc1788_handler(int this_irq, void *dev_id)
{
	struct lpc178x_gpio *gpio = (struct lpc178x_gpio *) dev_id;
	int i = 0;
	int data = 0;

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
		
		return IRQ_HANDLED;

}


static __devinit button_probe(struct platform_device *pdev)
{
	struct lpc178x_gpio *gpio;
	struct resource *regs;
	int ret = 0;
	int i = 0;

	gpio = kzalloc(sizeof(struct lpc178x_gpio), GFP_KERNEL);
	if (!gpio) {
		btn_printk(1, "Error allocating memory\n");
		ret = -ENOMEM;
		goto Error_release_nothing;
	}

	gpio->irq = platform_get_irq(pdev, 0);
	if (gpio->irq < 0) {
		btn_printk(1, "invalid IRQ %d for gpio contoller\n", gpio->irq);
		ret = -ENXIO;
		goto Error_release_nothing;
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (! regs) {
		btn_printk(1, "no register base for gpio contoller\n");
		ret = -ENXIO;
		goto Error_release_nothing;
	}

	gpio->reg_base = ioremap(regs->start, regs->end - regs->start + 1);
	if (!gpio->reg_base) {
		btn_printk(1, "unable to map register for gpio controller base=%08x\n", regs->start);
		ret = -EINVAL;
		goto Error_release_nothing;
	}

	platform_set_drvdata(pdev, gpio);

	//ret = request_irq(gpio->irq, btn_lpc1788_handler, IRQF_DISABLED | IRQF_SHARED, "lpc178x-gpio", gpio);
	ret = request_irq(gpio->irq, btn_lpc1788_handler, IRQF_DISABLED, "lpc178x-gpio", gpio);
	if (ret){
		btn_printk(1, "request irq failed\n");
		goto Error_release_nothing;
	}

	ret = misc_register(&misc);
	if (ret){
		btn_printk(1, "misc_register failed\n");
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

	hrtimer_init(&gpio->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	gpio->timer.function = btn_timer;

	spin_lock_init(&gpio->lock);
	
	btn_printk(1, "key probe successful\n");
	return 0;

Error_release_nothing:
Done:
	return ret;

}

static int button_remove(struct platform_device *dev)
{
	btn_printk(1, "remove\n");
	misc_deregister(&misc);
	return 0;
}



static struct platform_driver button_drv = {
	.probe = button_probe,
	.remove = button_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "lpc178x-key",
	},
};

static int __init platform_button_init(void)
{
	int ret;

	ret = platform_driver_register(&button_drv);

	return ret;
}

static void __exit platform_button_exit(void)
{
	platform_driver_unregister(&button_drv);
}

module_init(platform_button_init);
module_exit(platform_button_exit);

MODULE_AUTHOR("hujiafu");
MODULE_LICENSE("GPL");


