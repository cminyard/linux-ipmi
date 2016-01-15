/*
 * I2C multiplexer driver for PCA9541 bus master selector
 *
 * Copyright (c) 2010 Ericsson AB.
 *
 * Author: Guenter Roeck <linux@roeck-us.net>
 *
 * Derived from:
 *  pca954x.c
 *
 *  Copyright (c) 2008-2009 Rodolfo Giometti <giometti@linux.it>
 *  Copyright (c) 2008-2009 Eurotech S.p.A. <info@eurotech.it>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>

#include <linux/i2c/pca954x.h>

/*
 * The PCA9541 is a bus master selector. It supports two I2C masters connected
 * to a single slave bus.
 *
 * Before each bus transaction, a master has to acquire bus ownership. After the
 * transaction is complete, bus ownership has to be released. This fits well
 * into the I2C multiplexer framework, which provides select and release
 * functions for this purpose. For this reason, this driver is modeled as
 * single-channel I2C bus multiplexer.
 *
 * This driver assumes that the two bus masters are controlled by two different
 * hosts. If a single host controls both masters, platform code has to ensure
 * that only one of the masters is instantiated at any given time.
 */

#define PCA9541_CONTROL		0x01
#define PCA9541_ISTAT		0x02

#define PCA9541_CTL_MYBUS	(1 << 0)
#define PCA9541_CTL_NMYBUS	(1 << 1)
#define PCA9541_CTL_BUSON	(1 << 2)
#define PCA9541_CTL_NBUSON	(1 << 3)
#define PCA9541_CTL_BUSINIT	(1 << 4)
#define PCA9541_CTL_TESTON	(1 << 6)
#define PCA9541_CTL_NTESTON	(1 << 7)

#define PCA9541_ISTAT_INTIN	(1 << 0)
#define PCA9541_ISTAT_BUSINIT	(1 << 1)
#define PCA9541_ISTAT_BUSOK	(1 << 2)
#define PCA9541_ISTAT_BUSLOST	(1 << 3)
#define PCA9541_ISTAT_MYTEST	(1 << 6)
#define PCA9541_ISTAT_NMYTEST	(1 << 7)

#define BUSON		(PCA9541_CTL_BUSON | PCA9541_CTL_NBUSON)
#define MYBUS		(PCA9541_CTL_MYBUS | PCA9541_CTL_NMYBUS)
#define mybus(x)	(!((x) & MYBUS) || ((x) & MYBUS) == MYBUS)
#define busoff(x)	(!((x) & BUSON) || ((x) & BUSON) == BUSON)

/* arbitration timeouts, in jiffies */
#define ARB_TIMEOUT	(HZ / 8)	/* 125 ms until forcing bus ownership */
#define ARB2_TIMEOUT	(HZ / 4)	/* 250 ms until acquisition failure */

/* arbitration retry delays, in us */
#define SELECT_DELAY_SHORT		50
#define SELECT_DELAY_SHORT_JIFFIES	(SELECT_DELAY_SHORT * 1000 / TICK_NSEC)
#define SELECT_DELAY_LONG		1000
#define SELECT_DELAY_LONG_JIFFIES	(SELECT_DELAY_LONG * 1000 / TICK_NSEC)

enum pca9541_state {
	pca9541_idle,
	pca9541_read_control,
	pca9541_read_istat,
	pca9541_take_bus,
	pca9541_own_bus,
	pca9541_force_bus,
	pca9541_request_bus,
	pca9541_waiting,
	pca9541_release_read_control,
	pca9541_release_write_control
};


struct pca9541 {
	struct i2c_client *client;
	struct i2c_adapter *mux_adap;
	unsigned long select_timeout;
	unsigned long arb_timeout;
	unsigned long timeout;

	enum pca9541_state state;
	uint8_t last_control_read;

	struct i2c_op_q_entry op_e;
	union i2c_smbus_data op_data;
	struct timer_list timer;
	
	bool in_select;
	u8 pending_chan;
	void *cb_data;
	void (*cb)(void *, int);
};

static const struct i2c_device_id pca9541_id[] = {
	{"pca9541", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, pca9541_id);

/*
 * Write to chip register.
 * We already hold the lock, so use a special call.
 */
static int pca9541_reg_write(struct i2c_client *client, u8 command, u8 val)
{
	struct i2c_adapter *adap = client->adapter;
	union i2c_smbus_data data;

	data.byte = val;
	return i2c_smbus_xfer_nolock(adap, client->addr, client->flags,
				     I2C_SMBUS_WRITE, command,
				     I2C_SMBUS_BYTE_DATA, &data);
}

static int pca9541_d_reg_write(struct pca9541 *data, u8 command, u8 val)
{
	struct i2c_client *client = data->client;

	data->op_e.smbus.read_write = I2C_SMBUS_WRITE;
	data->op_e.smbus.command = command;
	data->op_data.byte = val;
		
	return i2c_non_blocking_op_head(client, &data->op_e);
}

/*
 * Read from chip register.
 * We already hold the lock, so use a special call.
 */
static int pca9541_reg_read(struct i2c_client *client, u8 command)
{
	struct i2c_adapter *adap = client->adapter;
	union i2c_smbus_data data;
	int ret;

	ret = i2c_smbus_xfer_nolock(adap, client->addr, client->flags,
				    I2C_SMBUS_READ, command,
				    I2C_SMBUS_BYTE_DATA, &data);
	if (!ret)
		ret = data.byte;

	return ret;
}

static int pca9541_d_reg_read(struct pca9541 *data, u8 command)
{
	struct i2c_client *client = data->client;

	data->op_e.smbus.read_write = I2C_SMBUS_READ;
	data->op_e.smbus.command = command;

	return i2c_non_blocking_op_head(client, &data->op_e);
}

/*
 * Arbitration management functions
 */

/* Release bus. Also reset NTESTON and BUSINIT if it was set. */
static void pca9541_release_bus(struct i2c_client *client)
{
	int reg;

	reg = pca9541_reg_read(client, PCA9541_CONTROL);
	if (reg >= 0 && !busoff(reg) && mybus(reg))
		pca9541_reg_write(client, PCA9541_CONTROL,
				  (reg & PCA9541_CTL_NBUSON) >> 1);
}

/*
 * Arbitration is defined as a two-step process. A bus master can only activate
 * the slave bus if it owns it; otherwise it has to request ownership first.
 * This multi-step process ensures that access contention is resolved
 * gracefully.
 *
 * Bus	Ownership	Other master	Action
 * state		requested access
 * ----------------------------------------------------
 * off	-		yes		wait for arbitration timeout or
 *					for other master to drop request
 * off	no		no		take ownership
 * off	yes		no		turn on bus
 * on	yes		-		done
 * on	no		-		wait for arbitration timeout or
 *					for other master to release bus
 *
 * The main contention point occurs if the slave bus is off and both masters
 * request ownership at the same time. In this case, one master will turn on
 * the slave bus, believing that it owns it. The other master will request
 * bus ownership. Result is that the bus is turned on, and master which did
 * _not_ own the slave bus before ends up owning it.
 */

/* Control commands per PCA9541 datasheet */
static const u8 pca9541_control[16] = {
	4, 0, 1, 5, 4, 4, 5, 5, 0, 0, 1, 1, 0, 4, 5, 1
};

/*
 * Channel arbitration
 *
 * Return values:
 *  <0: error
 *  0 : bus not acquired
 *  1 : bus acquired
 */
static int pca9541_arbitrate(struct i2c_client *client)
{
	struct pca9541 *data = i2c_get_clientdata(client);
	int reg;

	reg = pca9541_reg_read(client, PCA9541_CONTROL);
	if (reg < 0)
		return reg;

	if (busoff(reg)) {
		int istat;
		/*
		 * Bus is off. Request ownership or turn it on unless
		 * other master requested ownership.
		 */
		istat = pca9541_reg_read(client, PCA9541_ISTAT);
		if (!(istat & PCA9541_ISTAT_NMYTEST)
		    || time_is_before_eq_jiffies(data->arb_timeout)) {
			/*
			 * Other master did not request ownership,
			 * or arbitration timeout expired. Take the bus.
			 */
			pca9541_reg_write(client,
					  PCA9541_CONTROL,
					  pca9541_control[reg & 0x0f]
					  | PCA9541_CTL_NTESTON);
			data->select_timeout = SELECT_DELAY_SHORT;
		} else {
			/*
			 * Other master requested ownership.
			 * Set extra long timeout to give it time to acquire it.
			 */
			data->select_timeout = SELECT_DELAY_LONG * 2;
		}
	} else if (mybus(reg)) {
		/*
		 * Bus is on, and we own it. We are done with acquisition.
		 * Reset NTESTON and BUSINIT, then return success.
		 */
		if (reg & (PCA9541_CTL_NTESTON | PCA9541_CTL_BUSINIT))
			pca9541_reg_write(client,
					  PCA9541_CONTROL,
					  reg & ~(PCA9541_CTL_NTESTON
						  | PCA9541_CTL_BUSINIT));
		return 1;
	} else {
		/*
		 * Other master owns the bus.
		 * If arbitration timeout has expired, force ownership.
		 * Otherwise request it.
		 */
		data->select_timeout = SELECT_DELAY_LONG;
		if (time_is_before_eq_jiffies(data->arb_timeout)) {
			/* Time is up, take the bus and reset it. */
			pca9541_reg_write(client,
					  PCA9541_CONTROL,
					  pca9541_control[reg & 0x0f]
					  | PCA9541_CTL_BUSINIT
					  | PCA9541_CTL_NTESTON);
		} else {
			/* Request bus ownership if needed */
			if (!(reg & PCA9541_CTL_NTESTON))
				pca9541_reg_write(client,
						  PCA9541_CONTROL,
						  reg | PCA9541_CTL_NTESTON);
		}
	}
	return 0;
}

static int pca9541_select_chan(struct i2c_adapter *adap, void *client, u32 chan)
{
	struct pca9541 *data = i2c_get_clientdata(client);
	int ret;

	data->timeout = jiffies + ARB2_TIMEOUT;
		/* give up after this time */

	data->arb_timeout = jiffies + ARB_TIMEOUT;
		/* force bus ownership after this time */

	do {
		ret = pca9541_arbitrate(client);
		if (ret)
			return ret < 0 ? ret : 0;

		if (data->select_timeout == SELECT_DELAY_SHORT)
			udelay(data->select_timeout);
		else
			msleep(data->select_timeout / 1000);
	} while (time_is_after_eq_jiffies(data->timeout));

	return -ETIMEDOUT;
}

static int pca9541_release_chan(struct i2c_adapter *adap,
				void *client, u32 chan)
{
	pca9541_release_bus(client);
	return 0;
}

static int pca9541_start_arb(struct pca9541 *data)
{
	data->state = pca9541_read_control;
	return pca9541_d_reg_read(data, PCA9541_CONTROL);
}

static int pca9541_d_select_chan(struct i2c_adapter *adap, void *client,
				 u32 chan,
				 void *cb_data, void (cb)(void *, int))
{
	struct pca9541 *data = i2c_get_clientdata(client);
	int ret;

	data->cb = cb;
	data->cb_data = cb_data;

	data->timeout = jiffies + ARB2_TIMEOUT;
		/* give up after this time */

	data->arb_timeout = jiffies + ARB_TIMEOUT;
		/* force bus ownership after this time */

	BUG_ON(data->state != pca9541_idle);

	ret = pca9541_start_arb(data);
	if (ret)
		data->state = pca9541_idle;
	return ret;
}

static int pca9541_d_release_chan(struct i2c_adapter *adap,
				  void *client, u32 chan,
				  void *cb_data, void (cb)(void *, int))
{
	struct pca9541 *data = i2c_get_clientdata(client);
	int ret;

	BUG_ON(data->state != pca9541_idle);

	data->cb = cb;
	data->cb_data = cb_data;

	data->state = pca9541_release_read_control;
	ret = pca9541_d_reg_read(data, PCA9541_CONTROL);
	if (ret < 0)
		data->state = pca9541_idle;
	return ret;
}

static void pca9541_d_complete(struct pca9541 *data, int result)
{
	data->state = pca9541_idle;
	data->cb(data->cb_data, result);
}

static void pca9541_op_done(struct i2c_op_q_entry *entry)
{
	struct pca9541 *data = entry->handler_data;
	u8 reg = data->op_data.byte;
	int ret = 0;
	int write_val = -1;
	int timeout = -1;

	if (entry->result < 0) {
		pca9541_d_complete(data, entry->result);
		return;
	}
	
	switch (data->state) {
	case pca9541_idle:
	case pca9541_waiting:
		BUG();

	case pca9541_read_control:
		if (busoff(reg)) {
			/*
			 * Bus is off. Request ownership or turn it on unless
			 * other master requested ownership.
			 */
			data->state = pca9541_read_istat;
			data->last_control_read = reg;
			ret = pca9541_d_reg_read(data, PCA9541_ISTAT);
		} else if (mybus(reg)) {
			/*
			 * Bus is on, and we own it. We are done with
			 * acquisition.  Reset NTESTON and BUSINIT,
			 * then return success.
			 */
			data->state = pca9541_own_bus;
			write_val = reg & ~(PCA9541_CTL_NTESTON
					    | PCA9541_CTL_BUSINIT);
		} else {			
			/*
			 * Other master owns the bus.
			 * If arbitration timeout has expired, force ownership.
			 * Otherwise request it.
			 */
			if (time_is_before_eq_jiffies(data->arb_timeout)) {
				/* Time is up, take the bus and reset it. */
				data->state = pca9541_force_bus;
				write_val = (pca9541_control[reg & 0x0f]
					     | PCA9541_CTL_BUSINIT
					     | PCA9541_CTL_NTESTON);
			} else {
				if (!(reg & PCA9541_CTL_NTESTON)) {
					data->state = pca9541_request_bus;
					write_val = reg | PCA9541_CTL_NTESTON;
				} else {
					data->state = pca9541_waiting;
					timeout = SELECT_DELAY_LONG_JIFFIES;
				}
			}
		}
		break;
						  
	case pca9541_read_istat:
		if (!(reg & PCA9541_ISTAT_NMYTEST)
		    || time_is_before_eq_jiffies(data->arb_timeout)) {
			uint8_t oreg = data->last_control_read;

			/*
			 * Other master did not request ownership,
			 * or arbitration timeout expired. Take the bus.
			 */
			data->state = pca9541_take_bus;
			write_val = (pca9541_control[oreg & 0x0f]
				     | PCA9541_CTL_NTESTON);
		} else {
			/*
			 * Other master requested ownership.
			 * Set extra long timeout to give it time to acquire it.
			 */
			data->state = pca9541_waiting;
			timeout = SELECT_DELAY_LONG_JIFFIES * 2;
		}
		break;
		
	case pca9541_take_bus:
		data->state = pca9541_waiting;
		timeout = SELECT_DELAY_SHORT_JIFFIES;
		break;
		
	case pca9541_force_bus:
	case pca9541_request_bus:
		data->state = pca9541_waiting;
		timeout = SELECT_DELAY_LONG_JIFFIES;
		break;

	case pca9541_own_bus:
		pca9541_d_complete(data, 0);
		break;

	case pca9541_release_read_control:
		if (reg >= 0 && !busoff(reg) && mybus(reg))
			write_val = (reg & PCA9541_CTL_NBUSON) >> 1;
		else
			pca9541_d_complete(data, 0);
		break;

	case pca9541_release_write_control:
		pca9541_d_complete(data, 0);
		break;
	}

	if (write_val >= 0) {
		ret = pca9541_d_reg_write(data,
					  PCA9541_CONTROL, write_val);
		if (ret)
			pca9541_d_complete(data, ret);
	}
	if (timeout >= 0)
		mod_timer(&data->timer, jiffies + timeout);
}

static void pca9541_timeout(unsigned long tdata)
{
	struct pca9541 *data = (struct pca9541 *) tdata;
	int ret;

	BUG_ON(data->state != pca9541_waiting);

	if (!time_is_after_eq_jiffies(data->timeout)) {
		pca9541_d_complete(data, -ETIMEDOUT);
		return;
	}

	ret = pca9541_start_arb(data);
	if (ret)
		pca9541_d_complete(data, ret);
}

/*
 * I2C init/probing/exit functions
 */
static int pca9541_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct i2c_adapter *adap = client->adapter;
	struct pca954x_platform_data *pdata = dev_get_platdata(&client->dev);
	struct pca9541 *data;
	int force;
	int ret = -ENODEV;

	if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_BYTE_DATA))
		goto err;

	data = kzalloc(sizeof(struct pca9541), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto err;
	}

	i2c_set_clientdata(client, data);

	data->client = client;
	data->op_e.xfer_type = I2C_OP_SMBUS;
	data->op_e.handler = pca9541_op_done;
	data->op_e.handler_data = data;
	data->op_e.smbus.addr = client->addr;
	data->op_e.smbus.flags = client->flags;
	data->op_e.smbus.data = &data->op_data;
	data->op_e.smbus.size = I2C_SMBUS_BYTE_DATA;

	init_timer(&data->timer);
	data->timer.data = (unsigned long) data;
	data->timer.function = pca9541_timeout;

	/*
	 * I2C accesses are unprotected here.
	 * We have to lock the adapter before releasing the bus.
	 */
	i2c_lock_adapter(adap);
	pca9541_release_bus(client);
	i2c_unlock_adapter(adap);

	/* Create mux adapter */

	force = 0;
	if (pdata)
		force = pdata->modes[0].adap_id;
	if (adap->algo->master_start || adap->algo->smbus_start)
		data->mux_adap = i2c_add_mux_adapter_delayed_select(
						     adap, &client->dev,
						     client, force, 0, 0,
						     pca9541_d_select_chan,
						     pca9541_d_release_chan);
	else
		data->mux_adap = i2c_add_mux_adapter(adap, &client->dev,
						     client, force, 0, 0,
						     pca9541_select_chan,
						     pca9541_release_chan);

	if (data->mux_adap == NULL) {
		dev_err(&client->dev, "failed to register master selector\n");
		goto exit_free;
	}

	dev_info(&client->dev, "registered master selector for I2C %s\n",
		 client->name);

	return 0;

exit_free:
	kfree(data);
err:
	return ret;
}

static int pca9541_remove(struct i2c_client *client)
{
	struct pca9541 *data = i2c_get_clientdata(client);

	i2c_del_mux_adapter(data->mux_adap);

	kfree(data);
	return 0;
}

static struct i2c_driver pca9541_driver = {
	.driver = {
		   .name = "pca9541",
		   .owner = THIS_MODULE,
		   },
	.probe = pca9541_probe,
	.remove = pca9541_remove,
	.id_table = pca9541_id,
};

module_i2c_driver(pca9541_driver);

MODULE_AUTHOR("Guenter Roeck <linux@roeck-us.net>");
MODULE_DESCRIPTION("PCA9541 I2C master selector driver");
MODULE_LICENSE("GPL v2");
