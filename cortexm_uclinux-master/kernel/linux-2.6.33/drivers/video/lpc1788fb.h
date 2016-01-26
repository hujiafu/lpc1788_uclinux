
#ifndef __LPC1788FB_H
#define __LPC1788FB_H


struct lpc1788fb_info { 

	struct device           *dev;
	struct clk              *clk;
 
	struct resource         *mem;
    void __iomem            *io;
    void __iomem            *irq_base;
 
    struct lpc178xfb_hw     regs;
 
    unsigned long           clk_rate;
    unsigned int            palette_ready;
 
 
    /* keep these registers in case we need to re-write palette */
    u32                     palette_buffer[256];
    u32                     pseudo_pal[16];
};       

#define PALETTE_BUFF_CLEAR (0x80000000)	/* entry is clear/invalid */



#endif
