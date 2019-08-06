// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2019 MontaVista Software, LLC
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/watchdog.h>

#include "watchdog_pretimeout.h"

/**
 * pretimeout_read_data - Cause a read to return on the watchdog device.
 * @wdd - watchdog_device
 */
static void pretimeout_read_data(struct watchdog_device *wdd)
{
	unsigned long flags;

	spin_lock_irqsave(&wdd->readlock, flags);
	wdd->data_to_read = 1;
	wake_up_interruptible(&wdd->read_q);
	kill_fasync(&wdd->fasync_q, SIGIO, POLL_IN);
	spin_unlock_irqrestore(&wdd->readlock, flags);
}

static struct watchdog_governor watchdog_gov_read_data = {
	.name		= "read_data",
	.pretimeout	= pretimeout_read_data,
};

static int __init watchdog_gov_read_data_register(void)
{
	return watchdog_register_governor(&watchdog_gov_read_data);
}

static void __exit watchdog_gov_read_data_unregister(void)
{
	watchdog_unregister_governor(&watchdog_gov_read_data);
}
module_init(watchdog_gov_read_data_register);
module_exit(watchdog_gov_read_data_unregister);

MODULE_AUTHOR("Corey Minyard <cminyard@mvista.com>");
MODULE_DESCRIPTION("Read data watchdog pretimeout governor");
MODULE_LICENSE("GPL");
