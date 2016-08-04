

#include <linux/hwmon.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/irq.h>


#define LPC1788_UART_RX			0x0
#define LPC1788_UART_TX			0x0
#define LPC1788_UART_DLL		0x0
#define LPC1788_UART_DLM		0x4
#define LPC1788_UART_IER		0x4
#define LPC1788_UART_IIR		0x8
#define LPC1788_UART_FCR		0x8
#define LPC1788_UART_LCR		0xC
#define LPC1788_UART_LSR		0x14
#define LPC1788_UART_SCR		0x1C
#define LPC1788_UART_ACR		0x20
#define LPC1788_UART_FDR		0x28
#define LPC1788_UART_TER		0x30
#define LPC1788_UART_CTRL		0x4c

#define LPC1788_LCR_DLAB		0x80 /* Divisor latch access bit */
#define LPC1788_LCR_SBC			0x40 /* Set break control */
#define LPC1788_LCR_SPAR		0x20 /* Stick parity (?) */
#define LPC1788_LCR_EPAR		0x10 /* Even parity select */
#define LPC1788_LCR_PARITY		0x08 /* Parity Enable */
#define LPC1788_LCR_STOP		0x04 /* Stop bits: 0=1 bit, 1=2 bits */
#define LPC1788_LCR_WLEN5		0x00 /* Wordlength: 5 bits */
#define LPC1788_LCR_WLEN6		0x01 /* Wordlength: 6 bits */
#define LPC1788_LCR_WLEN7		0x02 /* Wordlength: 7 bits */
#define LPC1788_LCR_WLEN8		0x03 /* Wordlength: 8 bits */


#define MAX_12BIT 4096

static int ts_lpc1788_debug = 4;

#define TS_LPC1788_DEBUG 1

#if defined(TS_LPC1788_DEBUG)

#define ts_printk(level, fmt, args...)					\
	if (ts_lpc1788_debug >= level) printk(KERN_INFO "%s: " fmt,	\
				       	   __func__, ## args)

#else

#define ts_printk(level, fmt, args...)

#endif

struct irq_info {
        struct                  hlist_node node;
        int                     irq;
        spinlock_t              lock;   /* Protects list not the hash */
        struct list_head        *head;
};


struct ts_event {
	/* For portability, we can't read 12 bit values using SPI (which
	 * would make the controller deliver them as native byteorder u16
	 * with msbs zeroed).  Instead, we read them as two 8-bit values,
	 * *** WHICH NEED BYTESWAPPING *** and range adjustment.
	 */
	u16 status;
	u16	x;
	u16	y;
	u16	z1, z2;
	int	ignore;
};

struct lpc178x_packet {
	u8			read_x, read_y, read_z1, read_z2, pwrdown;
	u16			dummy;		/* for the pwrdown read */
	struct ts_event		tc;
};

struct lpc178x_ts_packet {
	u8	status;
	u8	param[4];
};

struct lpc178x_ts {
	struct input_dev	*input;
	char			phys[32];
	char			name[32];

	u16			model;
	u16			vref_mv;
	u16			vref_delay_usecs;
	u16			x_plate_ohms;
	u16			pressure_max;

	bool			swap_xy;

	struct lpc178x_packet	*packet;

	int			msg_idx;
	int			read_cnt;
	int			read_rep;
	int			last_read;

	u16			debounce_max;
	u16			debounce_tol;
	u16			debounce_rep;

	u16			penirq_recheck_delay_usecs;

	spinlock_t		lock;
	struct hrtimer		timer;
	unsigned		pendown:1;	/* P: lock */
	unsigned		pending:1;	/* P: lock */
// FIXME remove "irq_disabled"
	unsigned		irq_disabled:1;	/* P: lock */
	unsigned		disabled:1;
	unsigned		is_suspended:1;

	int			(*filter)(void *data, int data_idx, int *val);
	void			*filter_data;
	void			(*filter_cleanup)(void *data);
	int			(*get_pendown_state)(void);
	int			gpio_pendown;

	void			(*wait_for_sync)(void);
};



struct lpc178x_uart {

	void __iomem		*reg_base;
	int			irq;
	spinlock_t              lock;
	struct hrtimer          timer;
};
	
struct lpc178x_ts			*ts;
struct lpc178x_uart *uart;

static inline unsigned long serial_readl(void __iomem *reg)
{
	return __raw_readl(reg);
}

static inline void serial_writel(unsigned long val, void __iomem *reg)
{
        __raw_writel(val, reg);
}

static inline void _serial_set_dlba(struct lpc178x_uart *up)
{
	unsigned long val;

	val = serial_readl(up->reg_base + LPC1788_UART_LCR);
	val |= (0x1<<7);  //DLBA bit
	serial_writel(val, up->reg_base + LPC1788_UART_LCR);
	printk("LCR 0x%x\n", serial_readl(up->reg_base + LPC1788_UART_LCR)); 

}

static inline void _serial_reset_dlba(struct lpc178x_uart *up)
{
	unsigned long val;

	val = serial_readl(up->reg_base + LPC1788_UART_LCR);
	val &= ~(0x1<<7);  //DLBA bit
	serial_writel(val, up->reg_base + LPC1788_UART_LCR); 
	printk("LCR 0x%x\n", serial_readl(up->reg_base + LPC1788_UART_LCR)); 

}

static inline void _serial_dl_write(struct lpc178x_uart *up, int value)
{
	_serial_set_dlba(up);
	serial_writel(value & 0xff, up->reg_base + LPC1788_UART_DLL); 
	serial_writel((value >> 8) & 0xff, up->reg_base + LPC1788_UART_DLM);
	_serial_reset_dlba(up);
}

static inline void _serial_dl_read(struct lpc178x_uart *up)
{
	unsigned long val;

	_serial_set_dlba(up);
	val = serial_readl(up->reg_base + LPC1788_UART_DLL);
	printk("read DLL 0x%x\n", val);
	val = serial_readl(up->reg_base + LPC1788_UART_DLM);
	printk("read DLM 0x%x\n", val);
	_serial_reset_dlba(up);
}

static inline void _serial_ier_write(struct lpc178x_uart *up, int value)
{
	unsigned long val;
	serial_writel(value, up->reg_base + LPC1788_UART_IER); 
	val = serial_readl(up->reg_base + LPC1788_UART_IER); 
	printk("read IER 0x%x\n", val);
}

static inline void _serial_lcr_write(struct lpc178x_uart *up, int value)
{
	unsigned long val;
	serial_writel(value, up->reg_base + LPC1788_UART_LCR); 
	val = serial_readl(up->reg_base + LPC1788_UART_LCR); 
	printk("read LCR 0x%x\n", val);
}

static inline void _serial_fcr_write(struct lpc178x_uart *up, int value)
{
	serial_writel(value, up->reg_base + LPC1788_UART_FCR); 
}

unsigned long ts_calc_uart_baud(unsigned int baud)
{
	unsigned long val;
	val = (54000000 / (16 * baud)) - 1;
	printk("cal dll 0x%x\n", val);
	return val;
}

static void ts_lpc178x_rx()
{
	struct lpc178x_packet	*packet = ts->packet;

	u16			x, y, z1, z2, status;

	/* ads7846_rx_val() did in-place conversion (including byteswap) from
	 * on-the-wire format as part of debouncing to get stable readings.
	 */
	x = packet->tc.x;
	y = packet->tc.y;
	//z1 = packet->tc.z1;
	//z2 = packet->tc.z2;
	status = packet->tc.status;

	if (x == MAX_12BIT)
		x = 0;

	struct input_dev *input = ts->input;

	if(status == 0x81){
		printk("p %d %d\n", x, y);
		input_report_key(input, BTN_TOUCH, 1);
		input_report_abs(input, ABS_X, x);
		input_report_abs(input, ABS_Y, y);
		input_report_abs(input, ABS_PRESSURE, 50);
	}else{
		printk("u %d %d\n", x, y);
		input_report_key(input, BTN_TOUCH, 0);
		input_report_abs(input, ABS_PRESSURE, 0);
		input_sync(input);
	}
	input_sync(input);
}

struct lpc178x_ts_packet ts_data;
int ts_byte_count = 0;

static void uart_receive_chars(struct lpc178x_uart *up, unsigned int *status)
{

	int i;
	unsigned char ch, lsr = *status;
	struct lpc178x_packet	*packet = ts->packet;
	
	do {
		if (likely(lsr & 0x1)){
			ch = serial_readl(uart->reg_base + LPC1788_UART_RX); 
			if(ts_byte_count >= 1){
				ts_byte_count++;
			}
		}else{
			ch = 0;
		}
		if (unlikely(lsr & 0x1E)) {
			if (lsr & 0x10) {
				//up->port.icount.brk++;
			}
			else if (lsr & 0x4){
				//up->port.icount.parity++;
			}else if (lsr & 0x8){
				//up->port.icount.frame++;
			}
			if (lsr & 0x2){
				//up->port.icount.overrun++;
			}
		}
		if(ch == 0x81 || ch == 0x80){
			ts_byte_count = 1;
			ts_data.status = ch;

		}
		if(ts_byte_count > 1 && ts_byte_count <= 5){
			ts_data.param[ts_byte_count - 1] = ch;
		}
		if(ts_byte_count >= 5){

			//for(i=1; i<5; i++){
			//	printk("0x%x ", ts_data.param[i]);
			//}
			//printk("\n");
			
			ts_byte_count = 0;
			if(ts_data.status == 0x81 || ts_data.status == 0x80){
				packet->tc.y = ((ts_data.param[1] >> 1) << 8) | (((ts_data.param[1] & 0x1) << 7) | (ts_data.param[2]));
				packet->tc.x = ((ts_data.param[3] >> 1) << 8) | (((ts_data.param[3] & 0x1) << 7) | (ts_data.param[4]));
				packet->tc.status = ts_data.status;
				ts_lpc178x_rx();
			}
		}
ignore_char:
		lsr = serial_readl(up->reg_base + LPC1788_UART_LSR);
	}while ((lsr & (0x1 | 0x10)));

}

static irqreturn_t ts_lpc1788_handler(int this_irq, void *dev_id)
{
	unsigned long iir;
	unsigned long status;
	struct irq_info *i = dev_id;

	//printk("ts_lpc1788_handler\n");

	spin_lock(&i->lock);

	iir = serial_readl(uart->reg_base + LPC1788_UART_IIR); 

	if (!(iir & 0x1)) {
		status = serial_readl(uart->reg_base + LPC1788_UART_LSR); 
		if (status & (0x1 | 0x10))
		  uart_receive_chars(uart, &status);
		//check_modem_status(up);
		//if (status & UART_LSR_THRE)
                	//transmit_chars(up);
	}

	spin_unlock(&i->lock);
}

static void ts_lpc178x_start(struct lpc178x_uart *up)
{
	unsigned long baud;
	unsigned long dll;

	baud = 9600;
	dll = ts_calc_uart_baud(baud);
	dll = 0x16f;
	_serial_dl_write(up, dll);

	_serial_dl_read(up);
	_serial_lcr_write(up, 0x3); //8bit 1 stop
	_serial_fcr_write(up, 0x1); //reset fifo and enable fifo
	_serial_ier_write(up, 0x5); //rbr and rx line interrupt enable


}

static __devinit ts_lpc178x_probe(struct platform_device *pdev)
{
	struct lpc178x_packet		*packet;
	struct resource *regs;
	struct input_dev		*input_dev;
	struct irq_info	*i;
	int ret;
	unsigned int *pconp;

	i = kzalloc(sizeof(struct irq_info), GFP_KERNEL);
	if(i == NULL) {
		ret = -ENOMEM;
		goto Error_release_nothing;
	}

	uart = kzalloc(sizeof(struct lpc178x_uart), GFP_KERNEL);
	if (!uart) {
		ts_printk(1, "Error allocating memory\n");
		ret = -ENOMEM;
		goto Error_release_nothing;
	}


	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (! regs) {
		ts_printk(1, "no register base for gpio contoller\n");
		ret = -ENXIO;
		goto Error_release_nothing;
	}

	uart->reg_base = ioremap(regs->start, regs->end - regs->start + 1);
	if (!uart->reg_base) {
		ts_printk(1, "unable to map register for gpio controller base=%08x\n", regs->start);
		ret = -EINVAL;
		goto Error_release_nothing;
	}

	uart->irq = platform_get_irq(pdev, 0);
	if ((uart->irq < 0) || (uart->irq >= NR_IRQS)) {
		ts_printk(1, "error getting irq\n");
		ret = -ENXIO;
		goto Error_release_nothing;
	}
	
	pconp = (unsigned int *)0x400fc0c4;	
	printk("PCONP 0x%x\n", *pconp);
	*pconp |= (0x1<<24);
	
	_serial_ier_write(uart, 0x0); //rbr and rx line interrupt enable

	printk("uart2 irq %d\n", uart->irq);
	ret = request_irq(uart->irq, ts_lpc1788_handler, IRQF_DISABLED, "lpc178x-ts", i);
	if (ret){
		ts_printk(1, "request irq failed\n");
		ret = -ENXIO;
		goto Error_release_nothing;
	}

	ts = kzalloc(sizeof(struct lpc178x_ts), GFP_KERNEL);
	packet = kzalloc(sizeof(struct lpc178x_packet), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !packet || !input_dev) {
		ret = -ENOMEM;
		goto Error_release_nothing;
	}

	ts->packet = packet;
	ts->input = input_dev;

	spin_lock_init(&ts->lock);

	ts->model = 7777;
	ts->vref_delay_usecs =100;
	ts->x_plate_ohms = 400;
	ts->pressure_max = ~0;

	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", "lpc178x");
	snprintf(ts->name, sizeof(ts->name), "ADS%d Touchscreen", ts->model);
	
	input_dev->name = ts->name;
	input_dev->phys = ts->phys;
	//input_dev->dev.parent = &spi->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(input_dev, ABS_X,
			0,
			MAX_12BIT,
			0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			0,
			MAX_12BIT,
			0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE,
			0, 100, 0, 0);
	
	ret = input_register_device(input_dev);
	if (ret){
		goto Error_release_nothing;
	}

	ts_lpc178x_start(uart);

	printk("ts-lpc178x probe finish\n");
	printk("iir = 0x%x\n", serial_readl(uart->reg_base + LPC1788_UART_IIR));
	printk("lsr = 0x%x\n", serial_readl(uart->reg_base + LPC1788_UART_LSR));
	printk("fdr = 0x%x\n", serial_readl(uart->reg_base + LPC1788_UART_FDR));
	printk("ctrl = 0x%x\n", serial_readl(uart->reg_base + LPC1788_UART_CTRL));
	printk("ter = 0x%x\n", serial_readl(uart->reg_base + LPC1788_UART_TER));
Error_release_nothing:
Done:
	return ret;


}

static int ts_lpc178x_remove(struct platform_device *dev)
{

}

static struct platform_driver ts_lpc178x_driver = {
	.probe = ts_lpc178x_probe,
	.remove = ts_lpc178x_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "ts-lpc178x",
	},
};


static int __init ts_lpc178x_init(void)
{
	return platform_driver_register(&ts_lpc178x_driver);
}
module_init(ts_lpc178x_init);

static void __exit ts_lpc178x_exit(void)
{
	platform_driver_unregister(&ts_lpc178x_driver);
}
module_exit(ts_lpc178x_exit);

MODULE_DESCRIPTION("LPC178X TouchScreen Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("lpc178x:touchscreen");
