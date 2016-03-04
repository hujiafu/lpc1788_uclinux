#ifndef _LINUX_RFID_H
#define _LINUX_RFID_H

#include <linux/types.h>
#include <linux/memory.h>

/*
 * As seen through Linux I2C, differences between the most common types of I2C
 * memory include:
 * - How much memory is available (usually specified in bit)?
 * - What write page size does it support?
 * - Special flags (16 bit addresses, read_only, world readable...)?
 *
 * If you set up a custom eeprom type, please double-check the parameters.
 * Especially page_size needs extra care, as you risk data loss if your value
 * is bigger than what the chip actually supports!
 */
#define RFID_MAJOR	112

struct rfid_platform_data {
	u32		byte_len;		/* size (sum of all addr) */
	u16		page_size;		/* for writes */
	u8		flags;

	void __iomem		*reg_base;
	int		start_mem;
	int		end_mem;
	int		irq;
	int		irq_pin;
	int		irq_falleage;
	int		dev_id;
	unsigned int num_addresses;


	void		(*setup)(struct memory_accessor *, void *context);
	void		*context;
};

#endif /* _LINUX_RFID_H */
