
static int lpc1788fb_check_var(struct fb_var_screeninfo *var,
                                struct fb_info *info)
{

	printk("check_var(var=%p, info=%p)\n", var, info);

}


static struct fb_ops lpc1788fb_ops = {
         .owner          = THIS_MODULE,
         .fb_check_var   = lpc1788fb_check_var,
         .fb_set_par     = s3c2410fb_set_par,
         .fb_blank       = s3c2410fb_blank,
         .fb_setcolreg   = s3c2410fb_setcolreg,
         .fb_fillrect    = cfb_fillrect,
         .fb_copyarea    = cfb_copyarea,
         .fb_imageblit   = cfb_imageblit,
};




static char driver_name[] = "lpc1788fb";

static int __init lpc1788fb_map_video_memory(struct fb_info *info)
{
        struct lpc1788fb_info *fbi = info->par;
        dma_addr_t map_dma;
        unsigned map_size = PAGE_ALIGN(info->fix.smem_len);

        dprintk("map_video_memory(fbi=%p) map_size %u\n", fbi, map_size);

        info->screen_base = dma_alloc_writecombine(fbi->dev, map_size,
                                                   &map_dma, GFP_KERNEL);

        if (info->screen_base) {
                /* prevent initial garbage on screen */
                dprintk("map_video_memory: clear %p:%08x\n",
                        info->screen_base, map_size);
                memset(info->screen_base, 0x00, map_size);

                info->fix.smem_start = map_dma;

                dprintk("map_video_memory: dma=%08lx cpu=%p size=%08x\n",
                        info->fix.smem_start, info->screen_base, map_size);
        }

        return info->screen_base ? 0 : -ENOMEM;
}

static inline void lpc1788fb_unmap_video_memory(struct fb_info *info)
{
        struct lpc1788fb_info *fbi = info->par;

        dma_free_writecombine(fbi->dev, PAGE_ALIGN(info->fix.smem_len),
                              info->screen_base, info->fix.smem_start);
}


static int lpc1788fb_init_registers(struct fb_info *info)
{


}


static irqreturn_t lpc1788fb_irq(int irq, void *dev_id)
{
        struct lpc1788fb_info *fbi = dev_id;
        void __iomem *irq_base = fbi->irq_base;
        unsigned long lcdirq = readl(irq_base + S3C24XX_LCDINTPND);
 
        if (lcdirq & S3C2410_LCDINT_FRSYNC) {
                if (fbi->palette_ready)
                        s3c2410fb_write_palette(fbi);
 
                writel(S3C2410_LCDINT_FRSYNC, irq_base + S3C24XX_LCDINTPND);
                writel(S3C2410_LCDINT_FRSYNC, irq_base + S3C24XX_LCDSRCPND);
        }
 
        return IRQ_HANDLED;
}


static int __init lpc1788fb_probe(struct platform_device *pdev,
				  enum s3c_drv_type drv_type){

	struct lp1788fb_info *info;
	struct lpc1788fb_display *display;
	struct fb_info *fbinfo;
	struct lpc1788fb_mach_info *mach_info;
	struct resource *res;
	int irq;
	int size;

	mach_info = pdev->dev.platform_data;
	if (mach_info == NULL) {
                 dev_err(&pdev->dev,
                         "no platform data for lcd, cannot attach\n");
                 return -EINVAL;
        }

	 if (mach_info->default_display >= mach_info->num_displays) {
                 dev_err(&pdev->dev, "default is %d but only %d displays\n",
                         mach_info->default_display, mach_info->num_displays);
                 return -EINVAL;
        }

	display = mach_info->displays + mach_info->default_display;

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

	info->io = ioremap(res->start, size);
	if (info->io == NULL) {
		dev_err(&pdev->dev, "ioremap() of registers failed\n");
		ret = -ENXIO;
		goto release_mem;
	}
#if LPC1788_LCD_HW
	display->clk = clk_get(&pdev->dev, NULL);
        if (!info->clk || IS_ERR(info->clk)) {
                printk(KERN_ERR "failed to get lcd clock source\n");
                ret = -ENOENT;
                goto release_irq;
        }

	clk_enable(display->clk); //enable LCD POWER
	msleep(1);
#endif
	info->irq_base = info->io;

	strcpy(fbinfo->fix.id, driver_name);

	//disable LCD 
	lcdcon1 = readl(info->io + LPC178X_LCD_CTRL);
	writel(lcdcon1 & ~(0x1<<11), info->io + LPC178X_LCD_CTRL);	
	writel(lcdcon1 & ~(0x1<<0), info->io + LPC178X_LCD_CTRL);	

	fbinfo->fix.type            = FB_TYPE_PACKED_PIXELS;
	fbinfo->fix.type_aux        = 0;
	fbinfo->fix.xpanstep        = 0;
	fbinfo->fix.ypanstep        = 0;
	fbinfo->fix.ywrapstep       = 0;
	fbinfo->fix.accel           = FB_ACCEL_NONE;

	fbinfo->var.nonstd          = 0;
	fbinfo->var.activate        = FB_ACTIVATE_NOW;
	fbinfo->var.accel_flags     = 0;
	fbinfo->var.vmode           = FB_VMODE_NONINTERLACED;
	
	fbinfo->fbops               = &lpc1788fb_ops;
	fbinfo->flags               = FBINFO_FLAG_DEFAULT;
	fbinfo->pseudo_palette      = &info->pseudo_pal;	

	for (i = 0; i < 256; i++)
                info->palette_buffer[i] = PALETTE_BUFF_CLEAR;

        ret = request_irq(irq, lpc1788fb_irq, IRQF_DISABLED, pdev->name, info);
        if (ret) {
                dev_err(&pdev->dev, "cannot get irq %d - err %d\n", irq, ret);
                ret = -EBUSY;
                goto release_regs;
        }

	info->clk_rate = clk_get_rate(info->clk);

	        /* find maximum required memory size for display */
        for (i = 0; i < mach_info->num_displays; i++) {
                unsigned long smem_len = mach_info->displays[i].xres;

                smem_len *= mach_info->displays[i].yres;
                smem_len *= mach_info->displays[i].bpp;
                smem_len >>= 3;
                if (fbinfo->fix.smem_len < smem_len)
                        fbinfo->fix.smem_len = smem_len;
        }

	ret = lpc1788fb_map_video_memory(fbinfo);

	dprintk("got video memory\n");

	fbinfo->var.xres = display->xres;
        fbinfo->var.yres = display->yres;
        fbinfo->var.bits_per_pixel = display->bpp;

	lpc1788fb_init_registers(fbinfo);

	lpc1788fb_check_var(&fbinfo->var, fbinfo);

        ret = register_framebuffer(fbinfo);
        if (ret < 0) {
                printk(KERN_ERR "Failed to register framebuffer device: %d\n",
                        ret);
                goto free_cpufreq;
        }
	
	return 0;

free_video_memory:
        lpc1788fb_unmap_video_memory(fbinfo);
release_clock:
        clk_disable(info->clk);
        clk_put(info->clk);
release_irq:
        free_irq(irq, info);
release_regs:
        iounmap(info->io);
release_mem:
        release_resource(info->mem);
        kfree(info->mem);
dealloc_fb:
        platform_set_drvdata(pdev, NULL);
        framebuffer_release(fbinfo);
        return ret;

}


static struct platform_driver lpc1788fb_driver = {
	.probe		= lpc1788fb_probe,
	.remove		= lpc1788fb_remove,
	.suspend	= lpc1788fb_suspend,
	.resume		= lpc1788fb_resume,
	.driver		= {
		.name	= "dev:clcd",
		.owner	= THIS_MODULE,
	},
	pr_debug("IO address start     :0x%08x\n", (u32) pldat->net_region_start);
	pr_debug("IO address size      :%d\n", (u32) pldat->net_region_size);
	pr_debug("IO address (mapped)  :0x%08x\n", (u32) pldat->net_base);
	pr_debug("IRQ number           :%d\n", ndev->irq);
	pr_debug("DMA buffer size      :%d\n", pldat->dma_buff_size);
	pr_debug("DMA buffer P address :0x%08x\n", pldat->dma_buff_base_p);
	pr_debug("DMA buffer V address :0x%08x\n", pldat->dma_buff_base_v);
};

int __init lpc1788fb_init(void){

	int ret = platform_driver_register(&lpc1788fb_driver);
}

static void __exit lpc1788fb_cleanup(void){
	platform_driver_unregister(&lpc1788fb_driver);
}

module_init(lpc1788fb_init);
module_exit(lpc1788fb_cleanup);
