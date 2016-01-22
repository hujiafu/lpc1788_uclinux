
static char driver_name[] = "lpc1788fb";

static int __init lpc1788fb_probe(struct platform_device *pdev,
				  enum s3c_drv_type drv_type){

	struct lpc1788fb_display *display;
	struct fb_info *fbinfo;
	struct resource *res;
	int irq;
	int size;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq for device\n");
		return -ENOENT;
	}

	fbinfo = framebuffer_alloc(sizeof(struct lpc1788fb_info), &pdev->dev);
	if (!fbinfo)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get memory registers\n");
		ret = -ENXIO;
		goto dealloc_fb;
	}

	size = (res->end - res->start) + 1;
	info->mem = request_mem_region(res->start, size, pdev->name);
	if (info->mem == NULL) {
		dev_err(&pdev->dev, "failed to get memory region\n");
		ret = -ENOENT;
		goto dealloc_fb;
	}

	strcpy(fbinfo->fix.id, driver_name);



}


static struct platform_driver lpc1788fb_driver = {
	.probe		= lpc1788fb_probe,
	.remove		= lpc1788fb_remove,
	.suspend	= lpc1788fb_suspend,
	.resume		= lpc1788fb_resume,
	.driver		= {
		.name	= "lpc1788-lcd",
		.owner	= THIS_MODULE,
	},
};

int __init lpc1788fb_init(void){

	int ret = platform_driver_register(&lpc1788fb_driver);
}

static void __exit lpc1788fb_cleanup(void){
	platform_driver_unregister(&lpc1788fb_driver);
}

module_init(lpc1788fb_init);
module_exit(lpc1788fb_cleanup);
