
static int lpc1788fb_setcolreg(unsigned regno,
                               unsigned red, unsigned green, unsigned blue,
                               unsigned transp, struct fb_info *info)
{
        struct lpc1788fb_info *fbi = info->par;
        void __iomem *regs = fbi->io;
        unsigned int val;

        /* dprintk("setcol: regno=%d, rgb=%d,%d,%d\n",
                   regno, red, green, blue); */
 
        switch (info->fix.visual) {
        case FB_VISUAL_TRUECOLOR:
                /* true-colour, use pseudo-palette */

                if (regno < 16) {
                        u32 *pal = info->pseudo_palette;

                        val  = chan_to_field(red,   &info->var.red);
                        val |= chan_to_field(green, &info->var.green);
                        val |= chan_to_field(blue,  &info->var.blue);

                        pal[regno] = val;
                }
                break;

        case FB_VISUAL_PSEUDOCOLOR:
                if (regno < 256) {
                        /* currently assume RGB 5-6-5 mode */

                        val  = (red   >>  0) & 0xf800;
                        val |= (green >>  5) & 0x07e0;
                        val |= (blue  >> 11) & 0x001f;

                        //writel(val, regs + S3C2410_TFTPAL(regno));
                        //schedule_palette_update(fbi, regno, val);
                }

                break;

        default:
                return 1;       /* unknown type */
        }

        return 0;
}



static void lpc1788fb_lcd_enable(struct lpc1788fb_info *info, int enable)
{
        unsigned long flags;
	unsigned long lcdctrl;
	volatile int delay;

        local_irq_save(flags);

        if (enable){
		lcdctrl = readl(info->io + LPC178X_LCD_CTRL);
		writel(lcdctrl | (0x1<<0), info->io + LPC178X_LCD_CTRL);
		for(delay=0; delay<10000; delay++);
		writel(lcdctrl | (0x1<<11), info->io + LPC178X_LCD_CTRL);	
        }
	else{
		lcdctrl = readl(info->io + LPC178X_LCD_CTRL);
		writel(lcdctrl & ~(0x1<<11), info->io + LPC178X_LCD_CTRL);	
		for(delay=0; delay<10000; delay++);
		writel(lcdctrl & ~(0x1<<0), info->io + LPC178X_LCD_CTRL);	
	}

        local_irq_restore(flags);
}



static int lpc1788fb_blank(int blank_mode, struct fb_info *info)
{
        struct lpc1788fb_info *fbi = info->par;

        dprintk("blank(mode=%d, info=%p)\n", blank_mode, info);

        if (blank_mode == FB_BLANK_POWERDOWN) {
                lpc1788fb_lcd_enable(fbi, 0);
        } else {
                lpc1788fb_lcd_enable(fbi, 1);
        }

        return 0;
}



static int lpc1788fb_check_var(struct fb_var_screeninfo *var,
                                struct fb_info *info)
{
	struct lpc1788fb_info *fbi = info->par;
	struct lpc1788fb_mach_info *mach_info = fbi->dev->platform_data;
	struct lpc1788fb_display *display = NULL;
	struct lpc1788fb_display *default_display = mach_info->displays +
						    mach_info->default_display;
	int type = default_display->type;
	unsigned i;

	printk("check_var(var=%p, info=%p)\n", var, info);
	
	if (var->yres == default_display->yres &&
	    var->xres == default_display->xres &&
	    var->bits_per_pixel == default_display->bpp)
		display = default_display;
	else
		for (i = 0; i < mach_info->num_displays; i++)
			if (type == mach_info->displays[i].type &&
			    var->yres == mach_info->displays[i].yres &&
			    var->xres == mach_info->displays[i].xres &&
			    var->bits_per_pixel == mach_info->displays[i].bpp) {
				display = mach_info->displays + i;
				break;
			}

	if (!display) {
		dprintk("wrong resolution or depth %dx%d at %d bpp\n",
			var->xres, var->yres, var->bits_per_pixel);
		return -EINVAL;
	}

	/* it is always the size as the display */
	var->xres_virtual = display->xres;
	var->yres_virtual = display->yres;
	var->height = display->height;
	var->width = display->width;

	/* copy lcd settings */
	var->pixclock = display->pixclock;
	var->left_margin = display->left_margin;
	var->right_margin = display->right_margin;
	var->upper_margin = display->upper_margin;
	var->lower_margin = display->lower_margin;
	var->vsync_len = display->vsync_len;
	var->hsync_len = display->hsync_len;

	fbi->regs.lcd_ctrl = display->lcdctrl;
	fbi->regs.lcd_pol = display->lcdpol;

	var->transp.offset = 0;
	var->transp.length = 0;

	/* set r/g/b positions */
	switch (var->bits_per_pixel) {
	case 1:
	case 2:
	case 4:
		var->red.offset	= 0;
		var->red.length	= var->bits_per_pixel;
		var->green	= var->red;
		var->blue	= var->red;
		break;
	case 8:
		if (display->type != S3C2410_LCDCON1_TFT) {
			/* 8 bpp 332 */
			var->red.length		= 3;
			var->red.offset		= 5;
			var->green.length	= 3;
			var->green.offset	= 2;
			var->blue.length	= 2;
			var->blue.offset	= 0;
		} else {
			var->red.offset		= 0;
			var->red.length		= 8;
			var->green		= var->red;
			var->blue		= var->red;
		}
		break;
	case 12:
		/* 12 bpp 444 */
		var->red.length		= 4;
		var->red.offset		= 8;
		var->green.length	= 4;
		var->green.offset	= 4;
		var->blue.length	= 4;
		var->blue.offset	= 0;
		break;

	default:
	case 16:
		if (display->lcdpol & LPC178X_LCD_16BPP_565) {
			/* 16 bpp, 565 format */
			var->red.offset		= 11;
			var->green.offset	= 5;
			var->blue.offset	= 0;
			var->red.length		= 5;
			var->green.length	= 6;
			var->blue.length	= 5;
		} else {
			/* 16 bpp, 5551 format */
			var->red.offset		= 11;
			var->green.offset	= 6;
			var->blue.offset	= 1;
			var->red.length		= 5;
			var->green.length	= 5;
			var->blue.length	= 5;
		}
		break;
	case 32:
		/* 24 bpp 888 and 8 dummy */
		var->red.length		= 8;
		var->red.offset		= 16;
		var->green.length	= 8;
		var->green.offset	= 8;
		var->blue.length	= 8;
		var->blue.offset	= 0;
		break;
	}
	return 0;

}

static void lpc1788fb_calculate_tft_lcd_regs(const struct fb_info *info,
					     struct lpc178xfb_hw *regs)
{
	const struct lpc1788fb_info *fbi = info->par;
	void __iomem *regs = fbi->io;
	const struct fb_var_screeninfo *var = &info->var;
	long len;

	switch (var->bits_per_pixel) {
	case 16:
		readl(fbi->regs.lcd_ctrl, regs + LPC178X_LCD_CTRL);
		fbi->regs.lcd_ctrl &= ~LPC178X_LCD_FMT_CLEAR;
		fbi->regs.lcd_ctrl |= LPC178X_LCD_16BPP_565;
		writel(fbi->regs.lcd_ctrl, regs + LPC178X_LCD_CTRL);

		break;
	default:
		/* invalid pixel depth */
		dev_err(fbi->dev, "invalid bpp %d\n",
			var->bits_per_pixel);
	}

	len = var->xres * var
	/* update X/Y info */
	dprintk("setting vert: up=%d, low=%d, sync=%d\n",
		var->upper_margin, var->lower_margin, var->vsync_len);

	dprintk("setting horz: lft=%d, rt=%d, sync=%d\n",
		var->left_margin, var->right_margin, var->hsync_len);

	fbi->regs.lcd_timh = 0;
	fbi->regs.lcd_timh |= (var->left_margin - 1) << 24;
	fbi->regs.lcd_timh |= (var->right_margin - 1) << 16;
	fbi->regs.lcd_timh |= (var->hsync_len - 1) << 8;
	fbi->regs.lcd_timh |= ((var->xres)/16 - 1) << 2;
	writel(fbi->regs.lcd_timh, regs + LPC178X_LCD_TIMH);

	fbi->regs.lcd_timv = 0;
	fbi->regs.lcd_timv |= (var->upper_margin - 1) << 24;
	fbi->regs.lcd_timv |= (var->lower_margin - 1) << 16;
	fbi->regs.lcd_timv |= (var->vsync_len - 1) << 8;
	fbi->regs.lcd_timv |= ((var->yres) - 1);
	writel(fbi->regs.lcd_timv, regs + LPC178X_LCD_TIMV);
	
}




static void lpc1788fb_activate_var(struct fb_info *info)
{
	struct lpc1788fb_info *fbi = info->par;
	void __iomem *regs = fbi->io;
	int type = fbi->regs.lcdctrl & LPC178X_LCD_TFT;
	struct fb_var_screeninfo *var = &info->var;
	int clkdiv;
	volatile int delay;
	unsigned long lcdctrl;

	clkdiv = LPC178X_LCD_CLK_FRE / (var->pixclock);

	dprintk("%s: var->xres  = %d\n", __func__, var->xres);
	dprintk("%s: var->yres  = %d\n", __func__, var->yres);
	dprintk("%s: var->bpp   = %d\n", __func__, var->bits_per_pixel);

	if (type == LPC178X_LCD_TFT) {
		lpc1788fb_calculate_tft_lcd_regs(info, &fbi->regs);
		--clkdiv;
		if (clkdiv < 0)
			clkdiv = 0;
	} else {
		printk("not LPC178X_LCD_TFT\n");
	}

	LPC178X_SCC->lcd_cfg = clk_div;

	/* write new registers */

	dprintk("new register set:\n");


	fbi->regs.lcd_pol &= ~(LPC178X_LCD_CPL_MASK);
	fbi->regs.lcd_pol |= (var->xres - 1)<<16;
	writel(fbi->regs.lcd_pol, regs + LPC178X_LCD_POL);

	/* set lcd address pointers */
	writel(info->fix.smem_start, regs + LPC178X_LCD_UPBASE);
	writel(info->fix.smem_start, regs + LPC178X_LCD_LPBASE);

	/* enable lcd control */
	lcdctrl = readl(regs + LPC178X_LCD_CTRL);
	writel(lcdctrl | (0x1<<0), regs + LPC178X_LCD_CTRL);
	for(delay=0; delay<10000; delay++);
	writel(lcdctrl | (0x1<<11), regs + LPC178X_LCD_CTRL);	
	
}

static int lpc1788fb_set_par(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;

	switch (var->bits_per_pixel) {
	case 32:
	case 16:
	case 12:
		info->fix.visual = FB_VISUAL_TRUECOLOR;
		break;
	case 1:
		info->fix.visual = FB_VISUAL_MONO01;
		break;
	default:
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
		break;
	}

	info->fix.line_length = (var->xres_virtual * var->bits_per_pixel) / 8;

	/* activate this new configuration */

	lpc1788fb_activate_var(info);
	return 0;
}



static struct fb_ops lpc1788fb_ops = {
         .owner          = THIS_MODULE,
         .fb_check_var   = lpc1788fb_check_var,
         .fb_set_par     = lpc1788fb_set_par,
         .fb_blank       = lpc1788fb_blank,
         .fb_setcolreg   = lpc1788fb_setcolreg,
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

	struct lpc1788fb_info *info;
	struct lpc1788fb_display *display;
	struct fb_info *fbinfo;
	struct lpc1788fb_mach_info *mach_info;
	struct resource *res;
	int irq;
	int size;
	unsigned long lcdctrl;
	volatile int delay;

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

	platform_set_drvdata(pdev, fbinfo);

	info = fbinfo->par;
	info->dev = &pdev->dev;
	info->drv_type = drv_type;

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
	
	display->clk = clk_get(&pdev->dev, NULL);
        if (!info->clk || IS_ERR(info->clk)) {
                printk(KERN_ERR "failed to get lcd clock source\n");
                ret = -ENOENT;
                goto release_irq;
        }

	clk_enable(display->clk); //enable LCD POWER
	msleep(1);
	
	info->irq_base = info->io;

	strcpy(fbinfo->fix.id, driver_name);

	//disable LCD 
	lcdctrl = readl(info->io + LPC178X_LCD_CTRL);
	writel(lcdctrl & ~(0x1<<11), info->io + LPC178X_LCD_CTRL);	
	for(delay=0; delay<10000; delay++);
	writel(lcdctrl & ~(0x1<<0), info->io + LPC178X_LCD_CTRL);	

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
