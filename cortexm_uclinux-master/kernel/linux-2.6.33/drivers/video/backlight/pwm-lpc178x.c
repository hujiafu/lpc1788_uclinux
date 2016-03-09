/*
 *
 * Copyright (c) 2007 Ben Dooks
 * Copyright (c) 2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>, <ben-linux@fluff.org>
 *
 * LPC178X series PWM device core
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/pwm.h>

#include <mach/irqs.h>
#include <mach/pwm.h>

#define LPC1788_PWM_IR		0x0
#define LPC1788_PWM_TCR		0x4
#define LPC1788_PWM_PR		0xc
#define LPC1788_PWM_CTCR	0x70
#define LPC1788_PWM_MCR		0x14
#define LPC1788_PWM_MR0		0x18
#define LPC1788_PWM_MR2		0x20
#define LPC1788_PWM_CCR		0x28
#define LPC1788_PWM_PCR		0x4C
#define LPC1788_PWM_LER		0x50


struct pwm_device {
	struct list_head	 list;
	struct platform_device	*pdev;

	struct clk		*clk_div;
	struct clk		*clk;
	const char		*label;

	unsigned long		 period_ns;
	unsigned long		 duty_ns;

	unsigned char		 tcon_base;
	unsigned char		 running;
	unsigned char		 use_count;
	unsigned char		 pwm_id;

	void __iomem		*reg_base;
	unsigned long       clk_rate;
	unsigned long		max_rate;
	unsigned int		pr_cnt;
	unsigned int		tcnt;
	unsigned int		tcmp;
	int					pwm_nr;
};

#define pwm_dbg(_pwm, msg...) dev_dbg(&(_pwm)->pdev->dev, msg)

static struct clk *clk_scaler[2];

/* since we already have an static mapping for the timer, we do not
 * bother setting any IO resource for the base.
 */

static inline unsigned long pwm_readl(void __iomem *reg)
{
        return __raw_readl(reg);
}

static inline void pwm_writel(unsigned long val, void __iomem *reg)
{
        __raw_writel(val, reg);
}



static DEFINE_MUTEX(pwm_lock);
static LIST_HEAD(pwm_list);

struct pwm_device *pwm_request(int pwm_id, const char *label)
{
	struct pwm_device *pwm;
	int found = 0;

	mutex_lock(&pwm_lock);

	list_for_each_entry(pwm, &pwm_list, list) {
		if (pwm->pwm_id == pwm_id) {
			found = 1;
			break;
		}
	}

	if (found) {
		if (pwm->use_count == 0) {
			pwm->use_count = 1;
			pwm->label = label;
		} else
			pwm = ERR_PTR(-EBUSY);
	} else
		pwm = ERR_PTR(-ENOENT);

	mutex_unlock(&pwm_lock);
	return pwm;
}

EXPORT_SYMBOL(pwm_request);


void pwm_free(struct pwm_device *pwm)
{
	mutex_lock(&pwm_lock);

	if (pwm->use_count) {
		pwm->use_count--;
		pwm->label = NULL;
	} else
		printk(KERN_ERR "PWM%d device already freed\n", pwm->pwm_id);

	mutex_unlock(&pwm_lock);
}

EXPORT_SYMBOL(pwm_free);


int pwm_enable(struct pwm_device *pwm)
{
	unsigned long flags;
	unsigned long tcon;

	local_irq_save(flags);
	
	if(pwm->pwm_nr == 2){
		pwm_writel(0x1<<10, pwm->reg_base + LPC1788_PWM_PCR); //pwm2 enable out
	}
	pwm_writel((0x1<<0 | 0x1<<3), pwm->reg_base + LPC1788_PWM_TCR); //pwm counter enable, pwm enable

	local_irq_restore(flags);

	pwm->running = 1;

	printk("pwm_enable\n");
	printk("pwm_pcr = 0x%x\n", pwm_readl(pwm->reg_base + LPC1788_PWM_PCR));
	printk("pwm_tcr = 0x%x\n", pwm_readl(pwm->reg_base + LPC1788_PWM_TCR));
	
	return 0;
}

EXPORT_SYMBOL(pwm_enable);

void pwm_disable(struct pwm_device *pwm)
{
	unsigned long flags;
	unsigned long tcon;

	local_irq_save(flags);

	pwm_writel(0x0, pwm->reg_base + LPC1788_PWM_PCR); //pwm out disable
	pwm_writel(0x0, pwm->reg_base + LPC1788_PWM_TCR); //pwm counter disable, pwm disable

	local_irq_restore(flags);

	pwm->running = 0;
	
	printk("pwm_disable\n");
	printk("pwm_pcr = 0x%x\n", pwm_readl(pwm->reg_base + LPC1788_PWM_PCR));
	printk("pwm_tcr = 0x%x\n", pwm_readl(pwm->reg_base + LPC1788_PWM_TCR));

}

EXPORT_SYMBOL(pwm_disable);

static unsigned long pwm_calc_tin(struct pwm_device *pwm, unsigned long freq)
{
	int i, j;
	unsigned long pr, mr;
	
	for(i=1; i<16; i+=1){
		pr = pwm->clk_rate >> i;
		for(j=0; j<16; j+=1){
			mr = pr >> j;
			if(mr <= freq){
				//PWMSEL2 = 0
				pwm->pr_cnt = 0x1<<i;
				pwm->tcnt = 0x1<<j;
				pwm->tcmp = pwm->tcnt >> 1;
				printk("pr_cnt = 0x%x\n", pwm->pr_cnt);
				printk("tcnt = 0x%x\n", pwm->tcnt);
				printk("tcmp = 0x%x\n", pwm->tcmp);
				return 0;
			}
		}
	}

	return -1;
}

#define NS_IN_HZ (1000000000UL)

int pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns)
{
	unsigned long tin_rate;
	unsigned long tin_ns;
	unsigned long period;
	unsigned long flags;
	unsigned long tcon;
	unsigned long tcnt;
	long tcmp;
	int percent;
	int ret = 0;

	/* We currently avoid using 64bit arithmetic by using the
	 * fact that anything faster than 1Hz is easily representable
	 * by 32bits. */

	if (period_ns > NS_IN_HZ || duty_ns > NS_IN_HZ)
		return -ERANGE;

	if (duty_ns > period_ns)
		return -EINVAL;

	if (period_ns == pwm->period_ns &&
	    duty_ns == pwm->duty_ns)
		return 0;



	period = NS_IN_HZ / period_ns;

	pwm_dbg(pwm, "duty_ns=%d, period_ns=%d (%lu)\n",
		duty_ns, period_ns, period);

	/* Check to see if we are changing the clock rate of the PWM */

	if (pwm->period_ns != period_ns) {
		ret = pwm_calc_tin(pwm, period);
		if(ret != 0){
			printk("pwm_calc_tin failed\n");
			return -1;
		}

		pwm->period_ns = period_ns;
		pwm->duty_ns = duty_ns;
		percent = (duty_ns * 100) / period_ns;
		pwm->tcmp = (pwm->tcnt * percent) / 100;
		printk("percent = %d\n", percent);
		printk("pwm_tcmp = 0x%x\n", pwm->tcmp);
	}else{
	
		if(pwm->duty_ns != duty_ns){
			pwm->duty_ns = duty_ns;
			percent = (duty_ns * 100) / period_ns;
			pwm->tcmp = (pwm->tcnt * percent) / 100;
			printk("percent = %d\n", percent);
			printk("pwm_tcmp = 0x%x\n", pwm->tcmp);
			if(pwm->pwm_nr == 2){
				pwm_writel(pwm->tcmp - 1, pwm->reg_base + LPC1788_PWM_MR2);
				pwm_writel((0x1<<2), pwm->reg_base + LPC1788_PWM_LER); //updata MR2 value
				printk("pwm_mr2 = 0x%x\n", pwm_readl(pwm->reg_base + LPC1788_PWM_MR2));
				printk("pwm_ler = 0x%x\n", pwm_readl(pwm->reg_base + LPC1788_PWM_LER));
			}

		}
		
	}

	local_irq_save(flags);

	pwm_writel(0x0, pwm->reg_base + LPC1788_PWM_CTCR); //timeing mode
	pwm_writel(0x0, pwm->reg_base + LPC1788_PWM_PCR); //sigle eage mode, PWMSEL2 = 0
	pwm_writel(pwm->pr_cnt - 1, pwm->reg_base + LPC1788_PWM_PR);
	pwm_writel(pwm->tcnt - 1, pwm->reg_base + LPC1788_PWM_MR0);
	pwm_writel((0x1<<0), pwm->reg_base + LPC1788_PWM_LER); //updata MR0 value
	if(pwm->pwm_nr == 2){
		pwm_writel(0x1<<1, pwm->reg_base + LPC1788_PWM_MCR); // PWMTC reset when MCR0 Matched
		pwm_writel(pwm->tcmp - 1, pwm->reg_base + LPC1788_PWM_MR2);
		pwm_writel((0x1<<2), pwm->reg_base + LPC1788_PWM_LER); //updata MR2 value
	}
	
	local_irq_restore(flags);

	printk("pwm_pr = 0x%x\n", pwm_readl(pwm->reg_base + LPC1788_PWM_PR));
	printk("pwm_mcr = 0x%x\n", pwm_readl(pwm->reg_base + LPC1788_PWM_MCR));
	printk("pwm_mr0 = 0x%x\n", pwm_readl(pwm->reg_base + LPC1788_PWM_MR0));
	printk("pwm_mr2 = 0x%x\n", pwm_readl(pwm->reg_base + LPC1788_PWM_MR2));
	printk("pwm_ler = 0x%x\n", pwm_readl(pwm->reg_base + LPC1788_PWM_LER));
	printk("pwm_pcr = 0x%x\n", pwm_readl(pwm->reg_base + LPC1788_PWM_PCR));
	printk("pwm_ctcr = 0x%x\n", pwm_readl(pwm->reg_base + LPC1788_PWM_CTCR));
	
	return 0;
}

EXPORT_SYMBOL(pwm_config);

static int pwm_register(struct pwm_device *pwm)
{
	pwm->duty_ns = -1;
	pwm->period_ns = -1;

	mutex_lock(&pwm_lock);
	list_add_tail(&pwm->list, &pwm_list);
	mutex_unlock(&pwm_lock);

	return 0;
}

static int lpc178x_pwm_defconfig(struct pwm_device *pwm)
{
	int ret;

	ret = pwm_config(pwm, 500000, 1000000);
	if(ret != 0){
		printk("pwm_config failed\n");
		return -1;
	}

	ret = pwm_enable(pwm);
	if(ret != 0){
		printk("pwm_enable failed\n");
		return -1;
	}
	return 0;
}

static int lpc178x_pwm_probe(struct platform_device *pdev)
{
	struct pwm_device *pwm;
	struct lpc178x_pwm_mach_info *pwm_info;
	struct resource *res;
	unsigned long flags;
	unsigned long tcon;
	unsigned int id = pdev->id;
	int ret;


	printk("lpc178x_pwm_probe\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (! res) {
		dev_err(&pdev->dev, "no register base for pwm controller %d\n",
                        pdev->id);
		return -ENODEV;
	}

	pwm_info = (struct lpc178x_pwm_mach_info *)pdev->dev.platform_data;
	if(pwm_info == NULL){
		dev_err(&pdev->dev, "no platform_data for pwm controller %d\n",
                        pdev->id);
		return -ENODEV;
	}

	pwm = kzalloc(sizeof(struct pwm_device), GFP_KERNEL);
	if (pwm == NULL) {
		dev_err(&pdev->dev, "failed to allocate pwm_device\n");
		return -ENOMEM;
	}

	pwm->pdev = pdev;
	pwm->pwm_id = id;
	pwm->pwm_nr = pwm_info->pwm_nr;
	
	pwm->reg_base = ioremap(res->start, res->end - res->start);
	if (!pwm->reg_base) {
		dev_err(&pdev->dev, "Error mapping memory!\n");
		return -EIO;
	}
	
	pwm->clk = clk_get(&pdev->dev, NULL);
	if (!pwm->clk || IS_ERR(pwm->clk)) {
		dev_err(&pdev->dev, "failed to get pwm tin clk\n");
		ret = PTR_ERR(pwm->clk);
		goto err_alloc;
	}

	clk_enable(pwm->clk);

	pwm->clk_rate = clk_get_rate(pwm->clk);

	pwm_writel(0xff, pwm->reg_base + LPC1788_PWM_IR);
	pwm_writel(0x0, pwm->reg_base + LPC1788_PWM_TCR);
	pwm_writel(0x0, pwm->reg_base + LPC1788_PWM_CTCR);
	pwm_writel(0x0, pwm->reg_base + LPC1788_PWM_MCR);
	pwm_writel(0x0, pwm->reg_base + LPC1788_PWM_CCR);
	pwm_writel(0x0, pwm->reg_base + LPC1788_PWM_PCR);
	pwm_writel(0x0, pwm->reg_base + LPC1788_PWM_LER);

	ret = lpc178x_pwm_defconfig(pwm);
	if(ret){
		dev_err(&pdev->dev, "lpc178x_pwm_defconfig failed\n");
		goto err_alloc;
	}

	ret = pwm_register(pwm);
	if (ret) {
		dev_err(&pdev->dev, "failed to register pwm\n");
		goto err_alloc;
	}


	platform_set_drvdata(pdev, pwm);

	printk("lpc178x_pwm_probe finish\n");
	
	return 0;



 err_alloc:
	kfree(pwm);
	return ret;
}

static int __devexit lpc178x_pwm_remove(struct platform_device *pdev)
{
	struct pwm_device *pwm = platform_get_drvdata(pdev);

	clk_put(pwm->clk_div);
	clk_put(pwm->clk);
	kfree(pwm);

	return 0;
}

static struct platform_driver lpc178x_pwm_driver = {
	.driver		= {
		.name	= "lpc178x-pwm",
		.owner	= THIS_MODULE,
	},
	.probe		= lpc178x_pwm_probe,
	.remove		= __devexit_p(lpc178x_pwm_remove),
};

static int __init pwm_init(void)
{
	int ret;

	ret = platform_driver_register(&lpc178x_pwm_driver);
	if (ret)
		printk(KERN_ERR "%s: failed to add pwm driver\n", __func__);

	return ret;
}

arch_initcall(pwm_init);
