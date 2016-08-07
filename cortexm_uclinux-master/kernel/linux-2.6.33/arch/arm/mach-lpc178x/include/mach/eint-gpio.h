#ifndef __LPC178X_EINT_GPIO_H__
#define __LPC178X_EINT_GPIO_H__



struct plat_eint_data {

	unsigned char port0_active;
	unsigned char port1_active;
	unsigned char port2_active;
	unsigned char port3_active;

};


void __init lpc178x_eint_gpio_init(void);













#endif
