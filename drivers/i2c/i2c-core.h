/*
 * i2c-core.h - interfaces internal to the I2C framework
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

#include <linux/rwsem.h>

struct i2c_devinfo {
	struct list_head	list;
	int			busnum;
	struct i2c_board_info	board_info;
};

/* board_lock protects board_list and first_dynamic_bus_num.
 * only i2c core components are allowed to use these symbols.
 */
extern struct rw_semaphore	__i2c_board_lock;
extern struct list_head	__i2c_board_list;
extern int		__i2c_first_dynamic_bus_num;

/*
 * Start the timer for the given entry.  The entry must be on top of
 * the queue and running.  This is for use by muxes, they run an
 * operation on top of the queue to start the operation to set the
 * mux.  But then when the original operation starts up, it will not
 * have a timer running.  Allow the mux to start the timer.
 * The mux must to an extra get on the entry, otherwise it may go
 * away after the start operation.
 */
extern void i2c_start_timer(struct i2c_op_q_entry *entry);

/*
 * Increment the usecount of the entry.
 */
extern void i2c_entry_use(struct i2c_op_q_entry *entry);
