/*
 * TSC2046 based touchscreen and sensor driver
 *
 * Copyright (c) 2005 David Brownell
 * Copyright (c) 2006 Nokia Corporation
 * Various changes: Imre Deak <imre.deak@nokia.com>
 *
 * Using code from:
 *  - corgi_ts.c
 *	Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], tsc2046.h, ts_osk.c
 *	Copyright (C) 2002 MontaVista Software
 *	Copyright (C) 2004 Texas Instruments
 *	Copyright (C) 2005 Dirk Behme
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/hwmon.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/tsc2046.h>
#include <mach/touch.h>
#include <asm/irq.h>

/*
 * This code has been heavily tested on a Nokia 770, and lightly
 * tested on other tsc2046 devices (OSK/Mistral, Lubbock, Spitz).
 * TSC2046 is just newer tsc2046 silicon.
 * Support for ads7843 tested on Atmel at91sam926x-EK.
 * Support for ads7845 has only been stubbed in.
 *
 * IRQ handling needs a workaround because of a shortcoming in handling
 * edge triggered IRQs on some platforms like the OMAP1/2. These
 * platforms don't handle the ARM lazy IRQ disabling properly, thus we
 * have to maintain our own SW IRQ disabled status. This should be
 * removed as soon as the affected platform's IRQ handling is fixed.
 *
 * App note sbaa036 talks in more detail about accurate sampling...
 * that ought to help in situations like LCDs inducing noise (which
 * can also be helped by using synch signals) and more generally.
 * This driver tries to utilize the measures described in the app
 * note. The strength of filtering can be set in the board-* specific
 * files.
 */

#define TS_POLL_DELAY	(1 * 1000000)	/* ns delay before the first sample */
#define TS_POLL_PERIOD	(5 * 1000000)	/* ns delay between samples */

/* this driver doesn't aim at the peak continuous sample rate */
#define	SAMPLE_BITS	(8 /*cmd*/ + 16 /*sample*/ + 2 /* before, after */)

struct ts_event {
	/* For portability, we can't read 12 bit values using SPI (which
	 * would make the controller deliver them as native byteorder u16
	 * with msbs zeroed).  Instead, we read them as two 8-bit values,
	 * *** WHICH NEED BYTESWAPPING *** and range adjustment.
	 */
	u16	x;
	u16	y;
	u16	z1, z2;
	int	ignore;
};

/*
 * We allocate this separately to avoid cache line sharing issues when
 * driver is used with DMA-based SPI controllers (like atmel_spi) on
 * systems where main memory is not DMA-coherent (most non-x86 boards).
 */
struct tsc2046_packet {
	u8			read_x, read_y, read_z1, read_z2, pwrdown;
	u16			dummy;		/* for the pwrdown read */
	struct ts_event		tc;
};

struct tsc2046 {
	struct input_dev	*input;
	char			phys[32];
	char			name[32];

	struct spi_device	*spi;

#if defined(CONFIG_HWMON) || defined(CONFIG_HWMON_MODULE)
	struct attribute_group	*attr_group;
	struct device		*hwmon;
#endif

	u16			model;
	u16			vref_mv;
	u16			vref_delay_usecs;
	u16			x_plate_ohms;
	u16			pressure_max;

	bool			swap_xy;

	struct tsc2046_packet	*packet;

	struct spi_transfer	xfer[18];
	struct spi_message	msg[5];
	struct spi_message	*last_msg;
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
	void		(*penirq_clear)(void);
};

/* leave chip selected when we're done, for quicker re-select? */
#if	0
#define	CS_CHANGE(xfer)	((xfer).cs_change = 1)
#else
#define	CS_CHANGE(xfer)	((xfer).cs_change = 0)
#endif

/*--------------------------------------------------------------------------*/

/* The TSC2046 has touchscreen and other sensors.
 * Earlier ads784x chips are somewhat compatible.
 */
#define	ADS_START		(1 << 7)
#define	ADS_A2A1A0_d_y		(1 << 4)	/* differential */
#define	ADS_A2A1A0_d_z1		(3 << 4)	/* differential */
#define	ADS_A2A1A0_d_z2		(4 << 4)	/* differential */
#define	ADS_A2A1A0_d_x		(5 << 4)	/* differential */
#define	ADS_A2A1A0_temp0	(0 << 4)	/* non-differential */
#define	ADS_A2A1A0_vbatt	(2 << 4)	/* non-differential */
#define	ADS_A2A1A0_vaux		(6 << 4)	/* non-differential */
#define	ADS_A2A1A0_temp1	(7 << 4)	/* non-differential */
#define	ADS_8_BIT		(1 << 3)
#define	ADS_12_BIT		(0 << 3)
#define	ADS_SER			(1 << 2)	/* non-differential */
#define	ADS_DFR			(0 << 2)	/* differential */
#define	ADS_PD10_PDOWN		(0 << 0)	/* lowpower mode + penirq */
#define	ADS_PD10_ADC_ON		(1 << 0)	/* ADC on */
#define	ADS_PD10_REF_ON		(2 << 0)	/* vREF on + penirq */
#define	ADS_PD10_ALL_ON		(3 << 0)	/* ADC + vREF on */

#define	MAX_12BIT	((1<<12)-1)

/* leave ADC powered up (disables penirq) between differential samples */
#define	READ_12BIT_DFR(x, adc, vref) (ADS_START | ADS_A2A1A0_d_ ## x \
	| ADS_12_BIT | ADS_DFR | \
	(adc ? ADS_PD10_ADC_ON : 0) | (vref ? ADS_PD10_REF_ON : 0))

#define	READ_Y(vref)	(READ_12BIT_DFR(y,  1, vref))
#define	READ_Z1(vref)	(READ_12BIT_DFR(z1, 1, vref))
#define	READ_Z2(vref)	(READ_12BIT_DFR(z2, 1, vref))

#define	READ_X(vref)	(READ_12BIT_DFR(x,  1, vref))
#define	PWRDOWN		(READ_12BIT_DFR(y,  0, 0))	/* LAST */

/* single-ended samples need to first power up reference voltage;
 * we leave both ADC and VREF powered
 */
#define	READ_12BIT_SER(x) (ADS_START | ADS_A2A1A0_ ## x \
	| ADS_12_BIT | ADS_SER)

#define	REF_ON	(READ_12BIT_DFR(x, 1, 1))
#define	REF_OFF	(READ_12BIT_DFR(y, 0, 0))

/*--------------------------------------------------------------------------*/

/*
 * Non-touchscreen sensors only use single-ended conversions.
 * The range is GND..vREF. The ads7843 and ads7835 must use external vREF;
 * tsc2046 lets that pin be unconnected, to use internal vREF.
 */

struct ser_req {
	u8			ref_on;
	u8			command;
	u8			ref_off;
	u16			scratch;
	__be16			sample;
	struct spi_message	msg;
	struct spi_transfer	xfer[6];
};

static void tsc2046_enable(struct tsc2046 *ts);
static void tsc2046_disable(struct tsc2046 *ts);

static int device_suspended(struct device *dev)
{
	struct tsc2046 *ts = dev_get_drvdata(dev);
	return ts->is_suspended || ts->disabled;
}

static int tsc2046_read12_ser(struct device *dev, unsigned command)
{
	struct spi_device	*spi = to_spi_device(dev);
	struct tsc2046		*ts = dev_get_drvdata(dev);
	struct ser_req		*req = kzalloc(sizeof *req, GFP_KERNEL);
	int			status;
	int			use_internal;

	if (!req)
		return -ENOMEM;

	spi_message_init(&req->msg);

	/* FIXME boards with tsc2046 might use external vref instead ... */
	use_internal = (ts->model == 2046);

	/* maybe turn on internal vREF, and let it settle */
	if (use_internal) {
		req->ref_on = REF_ON;
		req->xfer[0].tx_buf = &req->ref_on;
		req->xfer[0].len = 1;
		spi_message_add_tail(&req->xfer[0], &req->msg);

		req->xfer[1].rx_buf = &req->scratch;
		req->xfer[1].len = 2;

		/* for 1uF, settle for 800 usec; no cap, 100 usec.  */
		req->xfer[1].delay_usecs = ts->vref_delay_usecs;
		spi_message_add_tail(&req->xfer[1], &req->msg);
	}

	/* take sample */
	req->command = (u8) command;
	req->xfer[2].tx_buf = &req->command;
	req->xfer[2].len = 1;
	spi_message_add_tail(&req->xfer[2], &req->msg);

	req->xfer[3].rx_buf = &req->sample;
	req->xfer[3].len = 2;
	spi_message_add_tail(&req->xfer[3], &req->msg);

	/* REVISIT:  take a few more samples, and compare ... */

	/* converter in low power mode & enable PENIRQ */
	req->ref_off = PWRDOWN;
	req->xfer[4].tx_buf = &req->ref_off;
	req->xfer[4].len = 1;
	spi_message_add_tail(&req->xfer[4], &req->msg);

	req->xfer[5].rx_buf = &req->scratch;
	req->xfer[5].len = 2;
	CS_CHANGE(req->xfer[5]);
	spi_message_add_tail(&req->xfer[5], &req->msg);

	ts->irq_disabled = 1;
	disable_irq(spi->irq);
	status = spi_sync(spi, &req->msg);
	ts->irq_disabled = 0;
	enable_irq(spi->irq);

	if (status == 0) {
		/* on-wire is a must-ignore bit, a BE12 value, then padding */
		status = be16_to_cpu(req->sample);
		status = status >> 3;
		status &= 0x0fff;
	}

	kfree(req);
	return status;
}

#if defined(CONFIG_HWMON) || defined(CONFIG_HWMON_MODULE)

#define SHOW(name, var, adjust) static ssize_t \
name ## _show(struct device *dev, struct device_attribute *attr, char *buf) \
{ \
	struct tsc2046 *ts = dev_get_drvdata(dev); \
	ssize_t v = tsc2046_read12_ser(dev, \
			READ_12BIT_SER(var) | ADS_PD10_ALL_ON); \
	if (v < 0) \
		return v; \
	return sprintf(buf, "%u\n", adjust(ts, v)); \
} \
static DEVICE_ATTR(name, S_IRUGO, name ## _show, NULL);


/* Sysfs conventions report temperatures in millidegrees Celsius.
 * TSC2046 could use the low-accuracy two-sample scheme, but can't do the high
 * accuracy scheme without calibration data.  For now we won't try either;
 * userspace sees raw sensor values, and must scale/calibrate appropriately.
 */
static inline unsigned null_adjust(struct tsc2046 *ts, ssize_t v)
{
	return v;
}

SHOW(temp0, temp0, null_adjust)		/* temp1_input */
SHOW(temp1, temp1, null_adjust)		/* temp2_input */


/* sysfs conventions report voltages in millivolts.  We can convert voltages
 * if we know vREF.  userspace may need to scale vAUX to match the board's
 * external resistors; we assume that vBATT only uses the internal ones.
 */
static inline unsigned vaux_adjust(struct tsc2046 *ts, ssize_t v)
{
	unsigned retval = v;

	/* external resistors may scale vAUX into 0..vREF */
	retval *= ts->vref_mv;
	retval = retval >> 12;
	return retval;
}

static inline unsigned vbatt_adjust(struct tsc2046 *ts, ssize_t v)
{
	unsigned retval = vaux_adjust(ts, v);

	/* tsc2046 has a resistor ladder to scale this signal down */
	if (ts->model == 2046)
		retval *= 4;
	return retval;
}

SHOW(in0_input, vaux, vaux_adjust)
SHOW(in1_input, vbatt, vbatt_adjust)


static struct attribute *tsc2046_attributes[] = {
	&dev_attr_temp0.attr,
	&dev_attr_temp1.attr,
	&dev_attr_in0_input.attr,
	&dev_attr_in1_input.attr,
	NULL,
};

static struct attribute_group tsc2046_attr_group = {
	.attrs = tsc2046_attributes,
};

static struct attribute *ads7843_attributes[] = {
	&dev_attr_in0_input.attr,
	&dev_attr_in1_input.attr,
	NULL,
};

static struct attribute_group ads7843_attr_group = {
	.attrs = ads7843_attributes,
};

static struct attribute *ads7845_attributes[] = {
	&dev_attr_in0_input.attr,
	NULL,
};

static struct attribute_group ads7845_attr_group = {
	.attrs = ads7845_attributes,
};

static int ads784x_hwmon_register(struct spi_device *spi, struct tsc2046 *ts)
{
	struct device *hwmon;
	int err;

	/* hwmon sensors need a reference voltage */
	switch (ts->model) {
	case 2046:
		if (!ts->vref_mv) {
			dev_dbg(&spi->dev, "assuming 2.5V internal vREF\n");
			ts->vref_mv = 2500;
		}
		break;
	case 7845:
	case 7843:
		if (!ts->vref_mv) {
			dev_warn(&spi->dev,
				"external vREF for ADS%d not specified\n",
				ts->model);
			return 0;
		}
		break;
	}

	/* different chips have different sensor groups */
	switch (ts->model) {
	case 2046:
		ts->attr_group = &tsc2046_attr_group;
		break;
	case 7845:
		ts->attr_group = &ads7845_attr_group;
		break;
	case 7843:
		ts->attr_group = &ads7843_attr_group;
		break;
	default:
		dev_dbg(&spi->dev, "ADS%d not recognized\n", ts->model);
		return 0;
	}

	err = sysfs_create_group(&spi->dev.kobj, ts->attr_group);
	if (err)
		return err;

	hwmon = hwmon_device_register(&spi->dev);
	if (IS_ERR(hwmon)) {
		sysfs_remove_group(&spi->dev.kobj, ts->attr_group);
		return PTR_ERR(hwmon);
	}

	ts->hwmon = hwmon;
	return 0;
}

static void ads784x_hwmon_unregister(struct spi_device *spi,
				     struct tsc2046 *ts)
{
	if (ts->hwmon) {
		sysfs_remove_group(&spi->dev.kobj, ts->attr_group);
		hwmon_device_unregister(ts->hwmon);
	}
}

#else
static inline int ads784x_hwmon_register(struct spi_device *spi,
					 struct tsc2046 *ts)
{
	return 0;
}

static inline void ads784x_hwmon_unregister(struct spi_device *spi,
					    struct tsc2046 *ts)
{
}
#endif

static int is_pen_down(struct device *dev)
{
	struct tsc2046	*ts = dev_get_drvdata(dev);

	return ts->pendown;
}

static ssize_t tsc2046_pen_down_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", is_pen_down(dev));
}

static DEVICE_ATTR(pen_down, S_IRUGO, tsc2046_pen_down_show, NULL);

static ssize_t tsc2046_disable_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct tsc2046	*ts = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ts->disabled);
}

static ssize_t tsc2046_disable_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct tsc2046 *ts = dev_get_drvdata(dev);
	unsigned long i;

	if (strict_strtoul(buf, 10, &i))
		return -EINVAL;

	spin_lock_irq(&ts->lock);

	if (i)
		tsc2046_disable(ts);
	else
		tsc2046_enable(ts);

	spin_unlock_irq(&ts->lock);

	return count;
}

static DEVICE_ATTR(disable, 0664, tsc2046_disable_show, tsc2046_disable_store);

static struct attribute *ads784x_attributes[] = {
	&dev_attr_pen_down.attr,
	&dev_attr_disable.attr,
	NULL,
};

static struct attribute_group ads784x_attr_group = {
	.attrs = ads784x_attributes,
};

/*--------------------------------------------------------------------------*/

static int get_pendown_state(struct tsc2046 *ts)
{
	if (ts->get_pendown_state)
		return ts->get_pendown_state();

	return !gpio_get_value(ts->gpio_pendown);
}

static void null_wait_for_sync(void)
{
}

/*
 * PENIRQ only kicks the timer.  The timer only reissues the SPI transfer,
 * to retrieve touchscreen status.
 *
 * The SPI transfer completion callback does the real work.  It reports
 * touchscreen events and reactivates the timer (or IRQ) as appropriate.
 */

static void tsc2046_rx(void *ads)
{
	struct tsc2046		*ts = ads;
	struct tsc2046_packet	*packet = ts->packet;
	unsigned		Rt;
	u16			x, y, z1, z2;

	/* tsc2046_rx_val() did in-place conversion (including byteswap) from
	 * on-the-wire format as part of debouncing to get stable readings.
	 */
	x = packet->tc.x;
	y = packet->tc.y;
	z1 = packet->tc.z1;
	z2 = packet->tc.z2;

	/* range filtering */
	if (x == MAX_12BIT)
		x = 0;

	if (ts->model == 7843) {
		Rt = ts->pressure_max / 2;
	} else if (likely(x && z1)) {
		/* compute touch pressure resistance using equation #2 */
		Rt = z2;
		Rt -= z1;
		Rt *= x;
		Rt *= ts->x_plate_ohms;
		Rt /= z1;
		Rt = (Rt + 2047) >> 12;
	} else {
		Rt = 0;
	}

	/* Sample found inconsistent by debouncing or pressure is beyond
	 * the maximum. Don't report it to user space, repeat at least
	 * once more the measurement
	 */
	if (packet->tc.ignore || Rt > ts->pressure_max) {
		dev_vdbg(&ts->spi->dev, "ignored %d pressure %d\n",
			 packet->tc.ignore, Rt);
		hrtimer_start(&ts->timer, ktime_set(0, TS_POLL_PERIOD),
			      HRTIMER_MODE_REL);
		return;
	}

	/* Maybe check the pendown state before reporting. This discards
	 * false readings when the pen is lifted.
	 */
	if (ts->penirq_recheck_delay_usecs) {
		udelay(ts->penirq_recheck_delay_usecs);
		if (!get_pendown_state(ts))
			Rt = 0;
	}

	/* NOTE: We can't rely on the pressure to determine the pen down
	 * state, even this controller has a pressure sensor.  The pressure
	 * value can fluctuate for quite a while after lifting the pen and
	 * in some cases may not even settle at the expected value.
	 *
	 * The only safe way to check for the pen up condition is in the
	 * timer by reading the pen signal state (it's a GPIO _and_ IRQ).
	 */
	if (Rt) {
		struct input_dev *input = ts->input;

		if (!ts->pendown) {
			input_report_key(input, BTN_TOUCH, 1);
			ts->pendown = 1;
			dev_vdbg(&ts->spi->dev, "DOWN\n");
		}

		if (ts->swap_xy)
			swap(x, y);

		input_report_abs(input, ABS_X, x);
		input_report_abs(input, ABS_Y, y);
		input_report_abs(input, ABS_PRESSURE, ts->pressure_max - Rt);

		input_sync(input);
		dev_vdbg(&ts->spi->dev, "%4d/%4d/%4d\n", x, y, Rt);
	}

	hrtimer_start(&ts->timer, ktime_set(0, TS_POLL_PERIOD),
			HRTIMER_MODE_REL);
}

static int tsc2046_debounce(void *ads, int data_idx, int *val)
{
	struct tsc2046		*ts = ads;

	if (!ts->read_cnt || (abs(ts->last_read - *val) > ts->debounce_tol)) {
		/* Start over collecting consistent readings. */
		ts->read_rep = 0;
		/* Repeat it, if this was the first read or the read
		 * wasn't consistent enough. */
		if (ts->read_cnt < ts->debounce_max) {
			ts->last_read = *val;
			ts->read_cnt++;
			return TSC2046_FILTER_REPEAT;
		} else {
			/* Maximum number of debouncing reached and still
			 * not enough number of consistent readings. Abort
			 * the whole sample, repeat it in the next sampling
			 * period.
			 */
			ts->read_cnt = 0;
			return TSC2046_FILTER_IGNORE;
		}
	} else {
		if (++ts->read_rep > ts->debounce_rep) {
			/* Got a good reading for this coordinate,
			 * go for the next one. */
			ts->read_cnt = 0;
			ts->read_rep = 0;
			return TSC2046_FILTER_OK;
		} else {
			/* Read more values that are consistent. */
			ts->read_cnt++;
			return TSC2046_FILTER_REPEAT;
		}
	}
}

static int tsc2046_no_filter(void *ads, int data_idx, int *val)
{
	return TSC2046_FILTER_OK;
}

static void tsc2046_rx_val(void *ads)
{
	struct tsc2046 *ts = ads;
	struct tsc2046_packet *packet = ts->packet;
	struct spi_message *m;
	struct spi_transfer *t;
	int val;
	int action;
	int status;

	m = &ts->msg[ts->msg_idx];
	t = list_entry(m->transfers.prev, struct spi_transfer, transfer_list);

	/* adjust:  on-wire is a must-ignore bit, a BE12 value, then padding;
	 * built from two 8 bit values written msb-first.
	 */
	val = be16_to_cpup((__be16 *)t->rx_buf) >> 3;

	action = ts->filter(ts->filter_data, ts->msg_idx, &val);
	switch (action) {
	case TSC2046_FILTER_REPEAT:
		break;
	case TSC2046_FILTER_IGNORE:
		packet->tc.ignore = 1;
		/* Last message will contain tsc2046_rx() as the
		 * completion function.
		 */
		m = ts->last_msg;
		break;
	case TSC2046_FILTER_OK:
		*(u16 *)t->rx_buf = val;
		packet->tc.ignore = 0;
		m = &ts->msg[++ts->msg_idx];
		break;
	default:
		BUG();
	}
	ts->wait_for_sync();
	status = spi_async(ts->spi, m);
	if (status)
		dev_err(&ts->spi->dev, "spi_async --> %d\n",
				status);
}

static enum hrtimer_restart tsc2046_timer(struct hrtimer *handle)
{
	struct tsc2046	*ts = container_of(handle, struct tsc2046, timer);
	int		status = 0;

	spin_lock(&ts->lock);

	if (unlikely(!get_pendown_state(ts) ||
		     device_suspended(&ts->spi->dev))) {
		if (ts->pendown) {
			struct input_dev *input = ts->input;

			input_report_key(input, BTN_TOUCH, 0);
			input_report_abs(input, ABS_PRESSURE, 0);
			input_sync(input);

			ts->pendown = 0;
			dev_vdbg(&ts->spi->dev, "UP\n");
		}

		/* measurement cycle ended */
		if (!device_suspended(&ts->spi->dev)) {
			ts->irq_disabled = 0;
			enable_irq(ts->spi->irq);
		}
		ts->pending = 0;
	} else {
		/* pen is still down, continue with the measurement */
		ts->msg_idx = 0;
		ts->wait_for_sync();
		status = spi_async(ts->spi, &ts->msg[0]);
		if (status)
			dev_err(&ts->spi->dev, "spi_async --> %d\n", status);
	}

	spin_unlock(&ts->lock);
	return HRTIMER_NORESTART;
}

static irqreturn_t tsc2046_irq(int irq, void *handle)
{
	struct tsc2046 *ts = handle;
	unsigned long flags;

	ts->penirq_clear();

	spin_lock_irqsave(&ts->lock, flags);
	if (likely(get_pendown_state(ts))) {
		if (!ts->irq_disabled) {
			/* The ARM do_simple_IRQ() dispatcher doesn't act
			 * like the other dispatchers:  it will report IRQs
			 * even after they've been disabled.  We work around
			 * that here.  (The "generic irq" framework may help...)
			 */
			ts->irq_disabled = 1;
			disable_irq_nosync(ts->spi->irq);
			ts->pending = 1;
			hrtimer_start(&ts->timer, ktime_set(0, TS_POLL_DELAY),
					HRTIMER_MODE_REL);
		}
	}
	spin_unlock_irqrestore(&ts->lock, flags);

	return IRQ_HANDLED;
}

/*--------------------------------------------------------------------------*/

/* Must be called with ts->lock held */
static void tsc2046_disable(struct tsc2046 *ts)
{
	if (ts->disabled)
		return;

	ts->disabled = 1;

	/* are we waiting for IRQ, or polling? */
	if (!ts->pending) {
		ts->irq_disabled = 1;
		disable_irq(ts->spi->irq);
	} else {
		/* the timer will run at least once more, and
		 * leave everything in a clean state, IRQ disabled
		 */
		while (ts->pending) {
			spin_unlock_irq(&ts->lock);
			msleep(1);
			spin_lock_irq(&ts->lock);
		}
	}

	/* we know the chip's in lowpower mode since we always
	 * leave it that way after every request
	 */
}

/* Must be called with ts->lock held */
static void tsc2046_enable(struct tsc2046 *ts)
{
	if (!ts->disabled)
		return;

	ts->disabled = 0;
	ts->irq_disabled = 0;
	enable_irq(ts->spi->irq);
}

static int tsc2046_suspend(struct spi_device *spi, pm_message_t message)
{
	struct tsc2046 *ts = dev_get_drvdata(&spi->dev);

	spin_lock_irq(&ts->lock);

	ts->is_suspended = 1;
	tsc2046_disable(ts);

	spin_unlock_irq(&ts->lock);

	return 0;

}

static int tsc2046_resume(struct spi_device *spi)
{
	struct tsc2046 *ts = dev_get_drvdata(&spi->dev);

	spin_lock_irq(&ts->lock);

	ts->is_suspended = 0;
	tsc2046_enable(ts);

	spin_unlock_irq(&ts->lock);

	return 0;
}

static int __devinit setup_pendown(struct spi_device *spi, struct tsc2046 *ts)
{
	struct tsc2046_platform_data *pdata = spi->dev.platform_data;
	int err;

	/* REVISIT when the irq can be triggered active-low, or if for some
	 * reason the touchscreen isn't hooked up, we don't need to access
	 * the pendown state.
	 */
	if (!pdata->get_pendown_state && !gpio_is_valid(pdata->gpio_pendown)) {
		dev_err(&spi->dev, "no get_pendown_state nor gpio_pendown?\n");
		return -EINVAL;
	}

	if (pdata->get_pendown_state) {
		ts->get_pendown_state = pdata->get_pendown_state;
		return 0;
	}

	err = gpio_request(pdata->gpio_pendown, "tsc2046_pendown");
	if (err) {
		dev_err(&spi->dev, "failed to request pendown GPIO%d\n",
				pdata->gpio_pendown);
		return err;
	}
	
	gpio_direction_input(pdata->gpio_pendown);

	ts->gpio_pendown = pdata->gpio_pendown;
	return 0;
}

static int __devinit tsc2046_probe(struct spi_device *spi)
{
	struct tsc2046			*ts;
	struct tsc2046_packet		*packet;
	struct input_dev		*input_dev;
	struct tsc2046_platform_data	*pdata = spi->dev.platform_data;
	struct spi_message		*m;
	struct spi_transfer		*x;
	int				vref;
	int				err;

	if (!spi->irq) {
		dev_dbg(&spi->dev, "no IRQ?\n");
		return -ENODEV;
	}

	if (!pdata) {
		dev_dbg(&spi->dev, "no platform data?\n");
		return -ENODEV;
	}

	if(pdata->io_setup != NULL){
		pdata->io_setup();
	}

	/* don't exceed max specified sample rate */
	if (spi->max_speed_hz > (125000 * SAMPLE_BITS)) {
		dev_dbg(&spi->dev, "f(sample) %d KHz?\n",
				(spi->max_speed_hz/SAMPLE_BITS)/1000);
		return -EINVAL;
	}

	/* We'd set TX wordsize 8 bits and RX wordsize to 13 bits ... except
	 * that even if the hardware can do that, the SPI controller driver
	 * may not.  So we stick to very-portable 8 bit words, both RX and TX.
	 */
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	err = spi_setup(spi);
	if (err < 0)
		return err;

	ts = kzalloc(sizeof(struct tsc2046), GFP_KERNEL);
	packet = kzalloc(sizeof(struct tsc2046_packet), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !packet || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	dev_set_drvdata(&spi->dev, ts);

	ts->packet = packet;
	ts->spi = spi;
	ts->input = input_dev;
	ts->vref_mv = pdata->vref_mv;
	ts->swap_xy = pdata->swap_xy;

	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = tsc2046_timer;

	spin_lock_init(&ts->lock);

	ts->model = pdata->model ? : 2046;
	ts->vref_delay_usecs = pdata->vref_delay_usecs ? : 100;
	ts->x_plate_ohms = pdata->x_plate_ohms ? : 400;
	ts->pressure_max = pdata->pressure_max ? : ~0;

	if (pdata->filter != NULL) {
		if (pdata->filter_init != NULL) {
			err = pdata->filter_init(pdata, &ts->filter_data);
			if (err < 0)
				goto err_free_mem;
		}
		ts->filter = pdata->filter;
		ts->filter_cleanup = pdata->filter_cleanup;
	} else if (pdata->debounce_max) {
		ts->debounce_max = pdata->debounce_max;
		if (ts->debounce_max < 2)
			ts->debounce_max = 2;
		ts->debounce_tol = pdata->debounce_tol;
		ts->debounce_rep = pdata->debounce_rep;
		ts->filter = tsc2046_debounce;
		ts->filter_data = ts;
	} else
		ts->filter = tsc2046_no_filter;

	ts->penirq_clear = pdata->eint_clear;

	err = setup_pendown(spi, ts);
	if (err)
		goto err_cleanup_filter;

	if (pdata->penirq_recheck_delay_usecs)
		ts->penirq_recheck_delay_usecs =
				pdata->penirq_recheck_delay_usecs;

	ts->wait_for_sync = pdata->wait_for_sync ? : null_wait_for_sync;

	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(&spi->dev));
	snprintf(ts->name, sizeof(ts->name), "ADS%d Touchscreen", ts->model);

	input_dev->name = ts->name;
	input_dev->phys = ts->phys;
	input_dev->dev.parent = &spi->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(input_dev, ABS_X,
			pdata->x_min ? : 0,
			pdata->x_max ? : MAX_12BIT,
			0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			pdata->y_min ? : 0,
			pdata->y_max ? : MAX_12BIT,
			0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE,
			pdata->pressure_min, pdata->pressure_max, 0, 0);

	vref = pdata->keep_vref_on;

	/* set up the transfers to read touchscreen state; this assumes we
	 * use formula #2 for pressure, not #3.
	 */
	m = &ts->msg[0];
	x = ts->xfer;

	spi_message_init(m);

	/* y- still on; turn on only y+ (and ADC) */
	packet->read_y = READ_Y(vref);
	x->tx_buf = &packet->read_y;
	x->len = 1;
	spi_message_add_tail(x, m);

	x++;
	x->rx_buf = &packet->tc.y;
	x->len = 2;
	spi_message_add_tail(x, m);

	/* the first sample after switching drivers can be low quality;
	 * optionally discard it, using a second one after the signals
	 * have had enough time to stabilize.
	 */
	if (pdata->settle_delay_usecs) {
		x->delay_usecs = pdata->settle_delay_usecs;

		x++;
		x->tx_buf = &packet->read_y;
		x->len = 1;
		spi_message_add_tail(x, m);

		x++;
		x->rx_buf = &packet->tc.y;
		x->len = 2;
		spi_message_add_tail(x, m);
	}

	m->complete = tsc2046_rx_val;
	m->context = ts;

	m++;
	spi_message_init(m);

	/* turn y- off, x+ on, then leave in lowpower */
	x++;
	packet->read_x = READ_X(vref);
	x->tx_buf = &packet->read_x;
	x->len = 1;
	spi_message_add_tail(x, m);

	x++;
	x->rx_buf = &packet->tc.x;
	x->len = 2;
	spi_message_add_tail(x, m);

	/* ... maybe discard first sample ... */
	if (pdata->settle_delay_usecs) {
		x->delay_usecs = pdata->settle_delay_usecs;

		x++;
		x->tx_buf = &packet->read_x;
		x->len = 1;
		spi_message_add_tail(x, m);

		x++;
		x->rx_buf = &packet->tc.x;
		x->len = 2;
		spi_message_add_tail(x, m);
	}

	m->complete = tsc2046_rx_val;
	m->context = ts;

	/* turn y+ off, x- on; we'll use formula #2 */
	if (ts->model == 2046) {
		m++;
		spi_message_init(m);

		x++;
		packet->read_z1 = READ_Z1(vref);
		x->tx_buf = &packet->read_z1;
		x->len = 1;
		spi_message_add_tail(x, m);

		x++;
		x->rx_buf = &packet->tc.z1;
		x->len = 2;
		spi_message_add_tail(x, m);

		/* ... maybe discard first sample ... */
		if (pdata->settle_delay_usecs) {
			x->delay_usecs = pdata->settle_delay_usecs;

			x++;
			x->tx_buf = &packet->read_z1;
			x->len = 1;
			spi_message_add_tail(x, m);

			x++;
			x->rx_buf = &packet->tc.z1;
			x->len = 2;
			spi_message_add_tail(x, m);
		}

		m->complete = tsc2046_rx_val;
		m->context = ts;

		m++;
		spi_message_init(m);

		x++;
		packet->read_z2 = READ_Z2(vref);
		x->tx_buf = &packet->read_z2;
		x->len = 1;
		spi_message_add_tail(x, m);

		x++;
		x->rx_buf = &packet->tc.z2;
		x->len = 2;
		spi_message_add_tail(x, m);

		/* ... maybe discard first sample ... */
		if (pdata->settle_delay_usecs) {
			x->delay_usecs = pdata->settle_delay_usecs;

			x++;
			x->tx_buf = &packet->read_z2;
			x->len = 1;
			spi_message_add_tail(x, m);

			x++;
			x->rx_buf = &packet->tc.z2;
			x->len = 2;
			spi_message_add_tail(x, m);
		}

		m->complete = tsc2046_rx_val;
		m->context = ts;
	}

	/* power down */
	m++;
	spi_message_init(m);

	x++;
	packet->pwrdown = PWRDOWN;
	x->tx_buf = &packet->pwrdown;
	x->len = 1;
	spi_message_add_tail(x, m);

	x++;
	x->rx_buf = &packet->dummy;
	x->len = 2;
	CS_CHANGE(*x);
	spi_message_add_tail(x, m);

	m->complete = tsc2046_rx;
	m->context = ts;

	ts->last_msg = m;

	if (request_irq(spi->irq, tsc2046_irq, IRQF_TRIGGER_FALLING,
			spi->dev.driver->name, ts)) {
		dev_info(&spi->dev,
			"trying pin change workaround on irq %d\n", spi->irq);
		err = request_irq(spi->irq, tsc2046_irq,
				  IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				  spi->dev.driver->name, ts);
		if (err) {
			dev_dbg(&spi->dev, "irq %d busy?\n", spi->irq);
			goto err_free_gpio;
		}
	}

	err = ads784x_hwmon_register(spi, ts);
	if (err)
		goto err_free_irq;

	dev_info(&spi->dev, "touchscreen, irq %d\n", spi->irq);

	/* take a first sample, leaving nPENIRQ active and vREF off; avoid
	 * the touchscreen, in case it's not connected.
	 */
	(void) tsc2046_read12_ser(&spi->dev,
			  READ_12BIT_SER(vaux) | ADS_PD10_ALL_ON);

	err = sysfs_create_group(&spi->dev.kobj, &ads784x_attr_group);
	if (err)
		goto err_remove_hwmon;

	err = input_register_device(input_dev);
	if (err)
		goto err_remove_attr_group;

	return 0;

 err_remove_attr_group:
	sysfs_remove_group(&spi->dev.kobj, &ads784x_attr_group);
 err_remove_hwmon:
	ads784x_hwmon_unregister(spi, ts);
 err_free_irq:
	free_irq(spi->irq, ts);
 err_free_gpio:
	if (ts->gpio_pendown != -1)
		gpio_free(ts->gpio_pendown);
 err_cleanup_filter:
	if (ts->filter_cleanup)
		ts->filter_cleanup(ts->filter_data);
 err_free_mem:
	input_free_device(input_dev);
	kfree(packet);
	kfree(ts);
	return err;
}

static int __devexit tsc2046_remove(struct spi_device *spi)
{
	struct tsc2046		*ts = dev_get_drvdata(&spi->dev);

	ads784x_hwmon_unregister(spi, ts);
	input_unregister_device(ts->input);

	tsc2046_suspend(spi, PMSG_SUSPEND);

	sysfs_remove_group(&spi->dev.kobj, &ads784x_attr_group);

	free_irq(ts->spi->irq, ts);
	/* suspend left the IRQ disabled */
	enable_irq(ts->spi->irq);

	if (ts->gpio_pendown != -1)
		gpio_free(ts->gpio_pendown);

	if (ts->filter_cleanup)
		ts->filter_cleanup(ts->filter_data);

	kfree(ts->packet);
	kfree(ts);

	dev_dbg(&spi->dev, "unregistered touchscreen\n");
	return 0;
}

static struct spi_driver tsc2046_driver = {
	.driver = {
		.name	= "tsc2046",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= tsc2046_probe,
	.remove		= __devexit_p(tsc2046_remove),
	.suspend	= tsc2046_suspend,
	.resume		= tsc2046_resume,
};

static int __init tsc2046_init(void)
{
	return spi_register_driver(&tsc2046_driver);
}
module_init(tsc2046_init);

static void __exit tsc2046_exit(void)
{
	spi_unregister_driver(&tsc2046_driver);
}
module_exit(tsc2046_exit);

MODULE_DESCRIPTION("TSC2046 TouchScreen Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:tsc2046");
