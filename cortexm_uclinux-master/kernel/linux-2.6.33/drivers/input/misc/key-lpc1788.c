
/*
 * Driver verbosity level: 0->silent; >0->verbose (1 to 4, growing verbosity)
 */
static int btn_lpc1788_debug = 4;


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
};



static int lpc178x_btn_open(sttic inode *inode, struct file *file)
{
	return 0;
}

static int lpc178x_btn_close(sttic inode *inode, struct file *file)
{
	return 0;
}

static int lpc178x_btn_read(sttic file *file, char __user *buff, size_t count, loff_t *offset)
{
	return 0;
}

static int lpc178x_btn_poll(sttic file *file, struct poll_table_struct *wait)
{
	return 0;
}

static struct file_operations dev_fops = {
	.owner = THIS_MODULE;
	.open = lpc178x_btn_open,
	.release = lpc178x_btn_close,
	.read = lpc178x_btn_read,
	.poll = lpc178x_btn_poll,
};

static struct miscdevice misc = {
	.minor = MISC_DYNAMIC_MINOR; 
	.name = "lpc178x_btn";
	.fops = &dev_fops,
};


static int button_probe(struct platform_device *pdev)
{
	struct lpc178x_gpio *gpio;
	struct resource *regs;
	int ret = 0;

	gpio = kzalloc(sizeof(struct lpc178x_gpio), GFP_KERNEL);
	if (!gpio) {
		dev_err(&pdev->dev, "Error allocating memory!\n");
		ret = -ENOMEM;
		goto Error_release_nothing;
	}

	gpio->irq = platform_get_irq(pdev, 0);
	if (gpio->irq < 0) {
		dev_err(&pdev->dev, "invalid IRQ %d for gpio contoller %d\n",
			irq, bus);
		ret = irq;
		goto Error_release_nothing;
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (! regs) {
		dev_err(&pdev->dev, "no register base for gpio controller %d\n",
                        bus);
		ret = -ENXIO;
		goto Error_release_nothing;
	}

	gpio->reg_base = ioremap(regs->start, regs->end - regs->start + 1);
	if (!gpio->reg_base) {
		dev_err(&pdev->dev, "unable to map registers for "
			"gpio controller base=%08x\n", regs->start);
		ret = -EINVAL;
		goto Error_release_nothing;
	}

	platform_set_drvdata(pdev, gpio);

Error_release_nothing:
Done:
	return ret;

}

static int button_remove(struct platform_device *dev)
{
	btn_printk("remove\n");
	misc_deregister(&misc);
	return 0;
}



struct platform_driver button_drv = {
	.probe = button_probe,
	.remove = button_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "lpc178x-key"
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


