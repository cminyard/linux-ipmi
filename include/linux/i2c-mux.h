/*
 *
 * i2c-mux.h - functions for the i2c-bus mux support
 *
 * Copyright (c) 2008-2009 Rodolfo Giometti <giometti@linux.it>
 * Copyright (c) 2008-2009 Eurotech S.p.A. <info@eurotech.it>
 * Michael Lawnick <michael.lawnick.ext@nsn.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301 USA.
 */

#ifndef _LINUX_I2C_MUX_H
#define _LINUX_I2C_MUX_H

#ifdef __KERNEL__

/*
 * Called to create a i2c bus on a multiplexed bus segment.
 * The mux_dev and chan_id parameters are passed to the select
 * and deselect callback functions to perform hardware-specific
 * mux control.
 */
typedef int (*i2c_mux_select_cb)(struct i2c_adapter *,
				 void *mux_priv, u32 chan_id);
struct i2c_adapter *i2c_add_mux_adapter(struct i2c_adapter *parent,
				struct device *mux_dev,
				void *mux_priv, u32 force_nr, u32 chan_id,
				unsigned int class,
				i2c_mux_select_cb select,
				i2c_mux_select_cb deselect);

/*
 * Delayed select, like above, but the select functions do not
 * operate immediately, they (may) require some time to operate and
 * tell the mux layer when they are done through a callback.
 *
 * If the callback returns 1, the operation occured immediately
 * and no callback will be called.
 */
typedef int (*i2c_mux_delayed_select_cb)(struct i2c_adapter *,
					 void *mux_priv, u32 chan_id,
					 void *cb_data,
					 void (*cb)(void *, int));
struct i2c_adapter *i2c_add_mux_adapter_delayed_select(
				struct i2c_adapter *parent,
				struct device *mux_dev,
				void *mux_priv, u32 force_nr, u32 chan_id,
				unsigned int class,
				i2c_mux_delayed_select_cb select,
				i2c_mux_delayed_select_cb deselect);

void i2c_del_mux_adapter(struct i2c_adapter *adap);

#endif /* __KERNEL__ */

#endif /* _LINUX_I2C_MUX_H */
