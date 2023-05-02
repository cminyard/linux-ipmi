// SPDX-License-Identifier: GPL-2.0+
/*
 * ipmi_watchdog.c
 *
 * A watchdog timer based upon the IPMI interface.
 *
 * Author: MontaVista Software, Inc.
 *         Corey Minyard <minyard@mvista.com>
 *         source@mvista.com
 *
 * Copyright 2002 MontaVista Software Inc.
 */

#define pr_fmt(fmt) "IPMI Watchdog: " fmt

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/ipmi.h>
#include <linux/ipmi_smi.h>
#include <linux/mutex.h>
#include <linux/watchdog.h>
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/kdebug.h>
#include <linux/kstrtox.h>
#include <linux/rwsem.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/notifier.h>
#include <linux/nmi.h>
#include <linux/reboot.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include <linux/sched/signal.h>

#ifdef CONFIG_X86
/*
 * This is ugly, but I've determined that x86 is the only architecture
 * that can reasonably support the IPMI NMI watchdog timeout at this
 * time.  If another architecture adds this capability somehow, it
 * will have to be a somewhat different mechanism and I have no idea
 * how it will work.  So in the unlikely event that another
 * architecture supports this, we can figure out a good generic
 * mechanism for it at that time.
 */
#include <linux/nmi.h>
#define HAVE_DIE_NMI
#endif

/*
 * The IPMI command/response information for the watchdog timer.
 */

/* values for byte 1 of the set command, byte 2 of the get response. */
#define WDOG_DONT_LOG		(1 << 7)
#define WDOG_DONT_STOP_ON_SET	(1 << 6)
#define WDOG_SET_TIMER_USE(byte, use) \
	(((byte) & 0xf8) | ((use) & 0x7))
#define WDOG_GET_TIMER_USE(byte) ((byte) & 0x7)
#define WDOG_TIMER_USE_BIOS_FRB2	1
#define WDOG_TIMER_USE_BIOS_POST	2
#define WDOG_TIMER_USE_OS_LOAD		3
#define WDOG_TIMER_USE_SMS_OS		4
#define WDOG_TIMER_USE_OEM		5

/* values for byte 2 of the set command, byte 3 of the get response. */
#define WDOG_SET_PRETIMEOUT_ACT(byte, use) \
	(((byte) & 0x8f) | (((use) & 0x7) << 4))
#define WDOG_GET_PRETIMEOUT_ACT(byte) (((byte) >> 4) & 0x7)
#define WDOG_PRETIMEOUT_NONE		0
#define WDOG_PRETIMEOUT_SMI		1
#define WDOG_PRETIMEOUT_NMI		2
#define WDOG_PRETIMEOUT_MSG_INT		3

/* Actions to perform on a full timeout. */
#define WDOG_SET_TIMEOUT_ACT(byte, use) \
	(((byte) & 0xf8) | ((use) & 0x7))
#define WDOG_GET_TIMEOUT_ACT(byte) ((byte) & 0x7)
#define WDOG_TIMEOUT_NONE		0
#define WDOG_TIMEOUT_RESET		1
#define WDOG_TIMEOUT_POWER_DOWN		2
#define WDOG_TIMEOUT_POWER_CYCLE	3

/*
 * Byte 3 of the get command, byte 4 of the get response is the
 * pre-timeout in seconds.
 */

/* Bits for setting byte 4 of the set command, byte 5 of the get response. */
#define WDOG_EXPIRE_CLEAR_BIOS_FRB2	(1 << 1)
#define WDOG_EXPIRE_CLEAR_BIOS_POST	(1 << 2)
#define WDOG_EXPIRE_CLEAR_OS_LOAD	(1 << 3)
#define WDOG_EXPIRE_CLEAR_SMS_OS	(1 << 4)
#define WDOG_EXPIRE_CLEAR_OEM		(1 << 5)

/*
 * Setting/getting the watchdog timer value.  This is for bytes 5 and
 * 6 (the timeout time) of the set command, and bytes 6 and 7 (the
 * timeout time) and 8 and 9 (the current countdown value) of the
 * response.  The timeout value is given in seconds (in the command it
 * is 100ms intervals).
 */
static inline void wdog_set_timeout_val(unsigned char *val, int timeout)
{
	val[0] = (timeout * 10) & 0xff;
	val[1] = (timeout * 10) >> 8;
}
#define WDOG_GET_TIMEOUT(byte1, byte2) (((byte1) | ((byte2) << 8)) / 10)

#define IPMI_WDOG_RESET_TIMER		0x22
#define IPMI_WDOG_SET_TIMER		0x24
#define IPMI_WDOG_GET_TIMER		0x25

#define IPMI_WDOG_TIMER_NOT_INIT_RESP	0x80

static DEFINE_MUTEX(ipmi_watchdog_mutex);

static struct ipmi_user *watchdog_user;
static int watchdog_ifnum;

/* Default the timeout to 10 seconds. */
static int timeout = 10;

/* The pre-timeout is disabled by default. */
static int pretimeout;

/* Whether the timer can be stopped or not. */
static bool nowayout = WATCHDOG_NOWAYOUT;

/* Default timeout to set on panic, in seconds. */
static int panic_wdt_timeout = 255;

/* Default action is to reset the board on a timeout. */
static unsigned char action_val = WDOG_TIMEOUT_RESET;

/* Default state of the timer. */
static unsigned char ipmi_watchdog_state = WDOG_TIMEOUT_NONE;

/* Default pretimeout is disabled until set_pretimeout is called. */
static unsigned char pretimeout_state = WDOG_PRETIMEOUT_NONE;

/* Test the NMI pretimeout at startup. */
static bool test_pretimeout;

static atomic_t pretimeout_since_last_heartbeat;

static int ifnum_to_use = -1;

/* Parameters to ipmi_set_timeout */
#define IPMI_SET_TIMEOUT_NO_HB			0
#define IPMI_SET_TIMEOUT_HB_IF_NECESSARY	1
#define IPMI_SET_TIMEOUT_FORCE_HB		2

static void ipmi_register_watchdog(int ipmi_intf);
static void ipmi_unregister_watchdog(int ipmi_intf);

static int set_param_wdog_ifnum(const char *val, const struct kernel_param *kp)
{
	int rv = param_set_int(val, kp);

	if (rv)
		return rv;
	if ((ifnum_to_use < 0) || (ifnum_to_use == watchdog_ifnum))
		return 0;

	ipmi_unregister_watchdog(watchdog_ifnum);
	ipmi_register_watchdog(ifnum_to_use);
	return 0;
}

static const struct kernel_param_ops param_ops_wdog_ifnum = {
	.set = set_param_wdog_ifnum,
	.get = param_get_int,
};

#define param_check_wdog_ifnum param_check_int

module_param(ifnum_to_use, wdog_ifnum, 0644);
MODULE_PARM_DESC(ifnum_to_use, "The interface number to use for the watchdog timer.  Setting to -1 defaults to the first registered interface");

module_param(timeout, int, 0644);
MODULE_PARM_DESC(timeout, "Timeout value in seconds.");

module_param(pretimeout, int, 0644);
MODULE_PARM_DESC(pretimeout, "Pretimeout value in seconds.");

module_param(nowayout, bool, 0644);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default=CONFIG_WATCHDOG_NOWAYOUT)");

module_param(panic_wdt_timeout, int, 0644);
MODULE_PARM_DESC(panic_wdt_timeout, "Timeout value on kernel panic in seconds.");

module_param(test_pretimeout, bool, 0444);
MODULE_PARM_DESC(test_pretimeout, "Test that the NMI pretimeout works");

/* IPMI version of the BMC. */
static unsigned char ipmi_version_major;
static unsigned char ipmi_version_minor;

#ifdef HAVE_DIE_NMI
/* If a pretimeout occurs, this is used to allow only one panic to happen. */
static atomic_t preop_panic_excl = ATOMIC_INIT(-1);

static int testing_nmi;
static int nmi_handler_registered;
#endif

static int __ipmi_heartbeat(void);

/*
 * We use a mutex to make sure that only one thing can send a set a
 * message at one time.  The mutex is claimed when a message is sent
 * and freed when both the send and receive messages are free.
 */
static atomic_t msg_tofree = ATOMIC_INIT(0);
static DECLARE_COMPLETION(msg_wait);
static void msg_free_smi(struct ipmi_smi_msg *msg)
{
	if (atomic_dec_and_test(&msg_tofree)) {
		if (!oops_in_progress)
			complete(&msg_wait);
	}
}
static void msg_free_recv(struct ipmi_recv_msg *msg)
{
	if (atomic_dec_and_test(&msg_tofree)) {
		if (!oops_in_progress)
			complete(&msg_wait);
	}
}
static struct ipmi_smi_msg smi_msg = INIT_IPMI_SMI_MSG(msg_free_smi);
static struct ipmi_recv_msg recv_msg = INIT_IPMI_RECV_MSG(msg_free_recv);

static int __ipmi_set_timeout(struct ipmi_smi_msg  *smi_msg,
			      struct ipmi_recv_msg *recv_msg,
			      int                  *send_heartbeat_now)
{
	struct kernel_ipmi_msg            msg;
	unsigned char                     data[6];
	int                               rv;
	struct ipmi_system_interface_addr addr;
	int                               hbnow = 0;


	data[0] = 0;
	data[0] = WDOG_SET_TIMER_USE(data[0], WDOG_TIMER_USE_SMS_OS);

	if (ipmi_watchdog_state != WDOG_TIMEOUT_NONE) {
		if ((ipmi_version_major > 1) ||
		    ((ipmi_version_major == 1) && (ipmi_version_minor >= 5))) {
			/* This is an IPMI 1.5-only feature. */
			data[0] |= WDOG_DONT_STOP_ON_SET;
		} else {
			/*
			 * In ipmi 1.0, setting the timer stops the watchdog, we
			 * need to start it back up again.
			 */
			hbnow = 1;
		}
	}

	data[1] = 0;
	data[1] = WDOG_SET_TIMEOUT_ACT(data[1], ipmi_watchdog_state);
	if ((pretimeout > 0) && (ipmi_watchdog_state != WDOG_TIMEOUT_NONE)) {
		data[1] = WDOG_SET_PRETIMEOUT_ACT(data[1], pretimeout_state);
		data[2] = pretimeout;
	} else {
		data[1] = WDOG_SET_PRETIMEOUT_ACT(data[1],
						  WDOG_PRETIMEOUT_NONE);
		data[2] = 0; /* No pretimeout. */
	}
	data[3] = 0;
	wdog_set_timeout_val(data + 4, timeout);

	addr.addr_type = IPMI_SYSTEM_INTERFACE_ADDR_TYPE;
	addr.channel = IPMI_BMC_CHANNEL;
	addr.lun = 0;

	msg.netfn = 0x06;
	msg.cmd = IPMI_WDOG_SET_TIMER;
	msg.data = data;
	msg.data_len = sizeof(data);
	rv = ipmi_request_supply_msgs(watchdog_user,
				      (struct ipmi_addr *) &addr,
				      0,
				      &msg,
				      NULL,
				      smi_msg,
				      recv_msg,
				      1);
	if (rv)
		pr_warn("set timeout error: %d\n", rv);
	else if (send_heartbeat_now)
		*send_heartbeat_now = hbnow;

	return rv;
}

static int _ipmi_set_timeout(int do_heartbeat)
{
	int send_heartbeat_now;
	int rv;

	if (!watchdog_user)
		return -ENODEV;

	atomic_set(&msg_tofree, 2);

	rv = __ipmi_set_timeout(&smi_msg,
				&recv_msg,
				&send_heartbeat_now);
	if (rv) {
		atomic_set(&msg_tofree, 0);
		return rv;
	}

	wait_for_completion(&msg_wait);

	if ((do_heartbeat == IPMI_SET_TIMEOUT_FORCE_HB)
		|| ((send_heartbeat_now)
		    && (do_heartbeat == IPMI_SET_TIMEOUT_HB_IF_NECESSARY)))
		rv = __ipmi_heartbeat();

	return rv;
}

static atomic_t panic_done_count = ATOMIC_INIT(0);

static void panic_smi_free(struct ipmi_smi_msg *msg)
{
	atomic_dec(&panic_done_count);
}
static void panic_recv_free(struct ipmi_recv_msg *msg)
{
	atomic_dec(&panic_done_count);
}

static struct ipmi_smi_msg panic_halt_heartbeat_smi_msg =
	INIT_IPMI_SMI_MSG(panic_smi_free);
static struct ipmi_recv_msg panic_halt_heartbeat_recv_msg =
	INIT_IPMI_RECV_MSG(panic_recv_free);

static void panic_halt_ipmi_heartbeat(void)
{
	struct kernel_ipmi_msg             msg;
	struct ipmi_system_interface_addr addr;
	int rv;

	/*
	 * Don't reset the timer if we have the timer turned off, that
	 * re-enables the watchdog.
	 */
	if (ipmi_watchdog_state == WDOG_TIMEOUT_NONE)
		return;

	addr.addr_type = IPMI_SYSTEM_INTERFACE_ADDR_TYPE;
	addr.channel = IPMI_BMC_CHANNEL;
	addr.lun = 0;

	msg.netfn = 0x06;
	msg.cmd = IPMI_WDOG_RESET_TIMER;
	msg.data = NULL;
	msg.data_len = 0;
	atomic_add(2, &panic_done_count);
	rv = ipmi_request_supply_msgs(watchdog_user,
				      (struct ipmi_addr *) &addr,
				      0,
				      &msg,
				      NULL,
				      &panic_halt_heartbeat_smi_msg,
				      &panic_halt_heartbeat_recv_msg,
				      1);
	if (rv)
		atomic_sub(2, &panic_done_count);
}

static struct ipmi_smi_msg panic_halt_smi_msg =
	INIT_IPMI_SMI_MSG(panic_smi_free);
static struct ipmi_recv_msg panic_halt_recv_msg =
	INIT_IPMI_RECV_MSG(panic_recv_free);

/*
 * Special call, doesn't claim any locks.  This is only to be called
 * at panic or halt time, in run-to-completion mode, when the caller
 * is the only CPU and the only thing that will be going is these IPMI
 * calls.
 */
static void panic_halt_ipmi_set_timeout(void)
{
	int send_heartbeat_now;
	int rv;

	/* Wait for the messages to be free. */
	while (atomic_read(&panic_done_count) != 0)
		ipmi_poll_interface(watchdog_user);
	atomic_add(2, &panic_done_count);
	rv = __ipmi_set_timeout(&panic_halt_smi_msg,
				&panic_halt_recv_msg,
				&send_heartbeat_now);
	if (rv) {
		atomic_sub(2, &panic_done_count);
		pr_warn("Unable to extend the watchdog timeout\n");
	} else {
		if (send_heartbeat_now)
			panic_halt_ipmi_heartbeat();
	}
	while (atomic_read(&panic_done_count) != 0)
		ipmi_poll_interface(watchdog_user);
}

static int __ipmi_heartbeat(void)
{
	struct kernel_ipmi_msg msg;
	int rv;
	struct ipmi_system_interface_addr addr;
	int timeout_retries = 0;

restart:
	/*
	 * Don't reset the timer if we have the timer turned off, that
	 * re-enables the watchdog.
	 */
	if (ipmi_watchdog_state == WDOG_TIMEOUT_NONE)
		return 0;

	atomic_set(&msg_tofree, 2);

	addr.addr_type = IPMI_SYSTEM_INTERFACE_ADDR_TYPE;
	addr.channel = IPMI_BMC_CHANNEL;
	addr.lun = 0;

	msg.netfn = 0x06;
	msg.cmd = IPMI_WDOG_RESET_TIMER;
	msg.data = NULL;
	msg.data_len = 0;
	rv = ipmi_request_supply_msgs(watchdog_user,
				      (struct ipmi_addr *) &addr,
				      0,
				      &msg,
				      NULL,
				      &smi_msg,
				      &recv_msg,
				      1);
	if (rv) {
		atomic_set(&msg_tofree, 0);
		pr_warn("heartbeat send failure: %d\n", rv);
		return rv;
	}

	/* Wait for the heartbeat to be sent. */
	wait_for_completion(&msg_wait);

	if (recv_msg.msg.data[0] == IPMI_WDOG_TIMER_NOT_INIT_RESP)  {
		timeout_retries++;
		if (timeout_retries > 3) {
			pr_err("Unable to restore the IPMI watchdog's settings, giving up\n");
			rv = -EIO;
			goto out;
		}

		/*
		 * The timer was not initialized, that means the BMC was
		 * probably reset and lost the watchdog information.  Attempt
		 * to restore the timer's info.  Note that we still hold
		 * the heartbeat lock, to keep a heartbeat from happening
		 * in this process, so must say no heartbeat to avoid a
		 * deadlock on this mutex
		 */
		rv = _ipmi_set_timeout(IPMI_SET_TIMEOUT_NO_HB);
		if (rv) {
			pr_err("Unable to send the command to set the watchdog's settings, giving up\n");
			goto out;
		}

		/* Might need a heartbeat send, go ahead and do it. */
		goto restart;
	} else if (recv_msg.msg.data[0] != 0) {
		/*
		 * Got an error in the heartbeat response.  It was already
		 * reported in ipmi_wdog_msg_handler, but we should return
		 * an error here.
		 */
		rv = -EINVAL;
	}

out:
	return rv;
}

static int _ipmi_heartbeat(void)
{
	int rv;

	if (!watchdog_user)
		return -ENODEV;

	if (atomic_cmpxchg(&pretimeout_since_last_heartbeat, 1, 0)) {
		/*
		 * A pretimeout occurred, make sure we set the timeout.
		 * We don't want to set the action, though, we want to
		 * leave that alone (thus it can't be combined with the
		 * above operation.
		 */
		rv = _ipmi_set_timeout(IPMI_SET_TIMEOUT_HB_IF_NECESSARY);
	} else {
		rv = __ipmi_heartbeat();
	}

	return rv;
}

static int ipmi_wdt_start(struct watchdog_device *wd)
{
	int rv;

	mutex_lock(&ipmi_watchdog_mutex);
	ipmi_watchdog_state = action_val;
	rv = _ipmi_set_timeout(IPMI_SET_TIMEOUT_FORCE_HB);
	if (rv)
		ipmi_watchdog_state = WDOG_TIMEOUT_NONE;
	mutex_unlock(&ipmi_watchdog_mutex);

	return rv;
}

static int ipmi_wdt_stop(struct watchdog_device *wd)
{
	mutex_lock(&ipmi_watchdog_mutex);
	ipmi_watchdog_state = WDOG_TIMEOUT_NONE;
	_ipmi_set_timeout(IPMI_SET_TIMEOUT_NO_HB);
	mutex_unlock(&ipmi_watchdog_mutex);

	return 0;
}

static int ipmi_wdt_ping(struct watchdog_device *wd)
{
	int rv;

	mutex_lock(&ipmi_watchdog_mutex);
	rv = _ipmi_heartbeat();
	mutex_unlock(&ipmi_watchdog_mutex);

	return rv;
}

static int ipmi_wdt_set_timeout(struct watchdog_device *wd, unsigned int to)
{
	int rv;

	mutex_lock(&ipmi_watchdog_mutex);
	timeout = to;
	rv = _ipmi_set_timeout(IPMI_SET_TIMEOUT_HB_IF_NECESSARY);
	mutex_unlock(&ipmi_watchdog_mutex);

	return rv;
}

#ifdef HAVE_DIE_NMI
static int ipmi_wdt_set_pretimeout(struct watchdog_device *wd, unsigned int pto)
{
	int rv;

	mutex_lock(&ipmi_watchdog_mutex);
	pretimeout = pto;
	if (pto)
		pretimeout_state = WDOG_PRETIMEOUT_NMI;
	else
		pretimeout_state = WDOG_PRETIMEOUT_NONE;

	rv = _ipmi_set_timeout(IPMI_SET_TIMEOUT_HB_IF_NECESSARY);
	mutex_unlock(&ipmi_watchdog_mutex);

	return rv;
}
#endif

static const struct watchdog_info ipmi_wdt_info = {
	.options = (WDIOF_SETTIMEOUT |
		    WDIOF_MAGICCLOSE |
#ifdef HAVE_DIE_NMI
		    WDIOF_PRETIMEOUT |
#endif
		    WDIOF_KEEPALIVEPING),
	.identity = "IPMI",
};

static const struct watchdog_ops ipmi_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= ipmi_wdt_start,
	.stop		= ipmi_wdt_stop,
	.ping		= ipmi_wdt_ping,
	.set_timeout	= ipmi_wdt_set_timeout,
#ifdef HAVE_DIE_NMI
	.set_pretimeout	= ipmi_wdt_set_pretimeout,
#endif
};

static struct watchdog_device ipmi_wdt = {
	.info		= &ipmi_wdt_info,
	.ops		= &ipmi_wdt_ops,
	.min_timeout	= 1,
	.max_timeout	= 255,
};

static void ipmi_wdog_msg_handler(struct ipmi_recv_msg *msg,
				  void                 *handler_data)
{
	if (msg->msg.cmd == IPMI_WDOG_RESET_TIMER &&
			msg->msg.data[0] == IPMI_WDOG_TIMER_NOT_INIT_RESP)
		pr_info("response: The IPMI controller appears to have been reset, will attempt to reinitialize the watchdog timer\n");
	else if (msg->msg.data[0] != 0)
		pr_err("response: Error %x on cmd %x\n",
		       msg->msg.data[0],
		       msg->msg.cmd);

	ipmi_free_recv_msg(msg);
}

static void ipmi_wdog_panic_handler(void *user_data)
{
	static int panic_event_handled;

	/*
	 * On a panic, if we have a panic timeout, make sure to extend
	 * the watchdog timer to a reasonable value to complete the
	 * panic, if the watchdog timer is running.  Plus the
	 * pretimeout is meaningless at panic time.
	 */
	if (watchdog_user && !panic_event_handled &&
	    ipmi_watchdog_state != WDOG_TIMEOUT_NONE) {
		/* Make sure we do this only once. */
		panic_event_handled = 1;

		timeout = panic_wdt_timeout;
		pretimeout = 0;
		panic_halt_ipmi_set_timeout();
	}
}

static const struct ipmi_user_hndl ipmi_hndlrs = {
	.ipmi_recv_hndl           = ipmi_wdog_msg_handler,
	.ipmi_panic_handler       = ipmi_wdog_panic_handler
};

static void ipmi_register_watchdog(int ipmi_intf)
{
	int rv;

	if (watchdog_user)
		return;

	if ((ifnum_to_use >= 0) && (ifnum_to_use != ipmi_intf))
		return;

	watchdog_ifnum = ipmi_intf;

	rv = ipmi_create_user(ipmi_intf, &ipmi_hndlrs, NULL, &watchdog_user);
	if (rv < 0) {
		pr_crit("Unable to register with ipmi\n");
		return;
	}

	rv = ipmi_get_version(watchdog_user,
			      &ipmi_version_major,
			      &ipmi_version_minor);
	if (rv) {
		pr_warn("Unable to get IPMI version, assuming 1.0\n");
		ipmi_version_major = 1;
		ipmi_version_minor = 0;
	}

#ifdef HAVE_DIE_NMI
	if (test_pretimeout && nmi_handler_registered) {
		int old_pretimeout = pretimeout;
		int old_timeout = timeout;

		/*
		 * Set the pretimeout to go off in a second and give
		 * ourselves plenty of time to stop the timer.
		 */
		ipmi_watchdog_state = WDOG_TIMEOUT_RESET;
		pretimeout_state = WDOG_PRETIMEOUT_NMI;
		pretimeout = 99;
		timeout = 100;

		testing_nmi = 1;

		rv = _ipmi_set_timeout(IPMI_SET_TIMEOUT_FORCE_HB);
		if (rv) {
			pr_warn("Error starting timer to test NMI: 0x%x.  The NMI pretimeout will likely not work\n",
				rv);
			rv = 0;
			goto out_restore;
		}

		msleep(1500);

		if (testing_nmi != 2)
			pr_warn("IPMI NMI didn't seem to occur.  The NMI pretimeout will likely not work\n");
		else
			pr_info("IPMI NMI is functional\n");

out_restore:
		testing_nmi = 0;
		ipmi_watchdog_state = WDOG_TIMEOUT_NONE;
		pretimeout_state = WDOG_PRETIMEOUT_NONE;
		pretimeout = old_pretimeout;
		timeout = old_timeout;
		_ipmi_set_timeout(IPMI_SET_TIMEOUT_NO_HB);
	}
#endif

	ipmi_wdt.timeout = timeout;
	ipmi_wdt.pretimeout = pretimeout;
	watchdog_set_nowayout(&ipmi_wdt, nowayout);

	rv = watchdog_register_device(&ipmi_wdt);
	if (rv < 0) {
		ipmi_destroy_user(watchdog_user);
		watchdog_user = NULL;
		pr_crit("Unable to register watchdog handler\n");
		return;
	}
}

static void ipmi_unregister_watchdog(int ipmi_intf)
{
	int rv;
	struct ipmi_user *loc_user = watchdog_user;

	if (!loc_user)
		return;

	if (watchdog_ifnum != ipmi_intf)
		return;

	/* Make sure no one can call us any more. */
	watchdog_unregister_device(&ipmi_wdt);

	watchdog_user = NULL;

	/*
	 * Wait to make sure the message makes it out.  The lower layer has
	 * pointers to our buffers, we want to make sure they are done before
	 * we release our memory.
	 */
	while (atomic_read(&msg_tofree))
		msg_free_smi(NULL);

	mutex_lock(&ipmi_watchdog_mutex);

	/* Disconnect from IPMI. */
	rv = ipmi_destroy_user(loc_user);
	if (rv)
		pr_warn("error unlinking from IPMI: %d\n",  rv);

	mutex_unlock(&ipmi_watchdog_mutex);
}

#ifdef HAVE_DIE_NMI
static int ipmi_nmi(unsigned int val, struct pt_regs *regs)
{
	/*
	 * If we get here, it's an NMI that's not a memory or I/O
	 * error.  We can't truly tell if it's from IPMI or not
	 * without sending a message, and sending a message is almost
	 * impossible because of locking.
	 */

	if (testing_nmi) {
		testing_nmi = 2;
		return NMI_HANDLED;
	}

	/* If we are not expecting a timeout, ignore it. */
	if (ipmi_watchdog_state == WDOG_TIMEOUT_NONE)
		return NMI_DONE;

	if (pretimeout_state != WDOG_PRETIMEOUT_NMI)
		return NMI_DONE;

	/*
	 * If no one else handled the NMI, we assume it was the IPMI
	 * watchdog.
	 */

	/*
	 * On some machines, the heartbeat will give an error and not
	 * work unless we re-enable the timer.  So do so.
	 */
	atomic_set(&pretimeout_since_last_heartbeat, 1);

	/*
	 * Don't call watchdog_notify_pretimeout() here.  You really
	 * need to call nmi_panic from an NMI, and that code doesn't
	 * do that.
	 */
	if (atomic_inc_and_test(&preop_panic_excl))
		nmi_panic(regs, "ipmi wdt pre-timeout");

	return NMI_HANDLED;
}
#endif

static int wdog_reboot_handler(struct notifier_block *this,
			       unsigned long         code,
			       void                  *unused)
{
	static int reboot_event_handled;

	mutex_lock(&ipmi_watchdog_mutex);
	if ((watchdog_user) && (!reboot_event_handled)) {
		/* Make sure we only do this once. */
		reboot_event_handled = 1;

		if (code == SYS_POWER_OFF || code == SYS_HALT) {
			/* Disable the WDT if we are shutting down. */
			ipmi_watchdog_state = WDOG_TIMEOUT_NONE;
			_ipmi_set_timeout(IPMI_SET_TIMEOUT_NO_HB);
		} else if (ipmi_watchdog_state != WDOG_TIMEOUT_NONE) {
			/*
			 * Set a long timer to let the reboot happen or
			 * reset if it hangs, but only if the watchdog
			 * timer was already running.
			 */
			if (timeout < 120)
				timeout = 120;
			pretimeout = 0;
			ipmi_watchdog_state = WDOG_TIMEOUT_RESET;
			_ipmi_set_timeout(IPMI_SET_TIMEOUT_NO_HB);
		}
	}
	mutex_unlock(&ipmi_watchdog_mutex);
	return NOTIFY_OK;
}

static struct notifier_block wdog_reboot_notifier = {
	.notifier_call	= wdog_reboot_handler,
	.next		= NULL,
	.priority	= 0
};

static void ipmi_new_smi(int if_num, struct device *device)
{
	ipmi_register_watchdog(if_num);
}

static void ipmi_smi_gone(int if_num)
{
	ipmi_unregister_watchdog(if_num);
}

static struct ipmi_smi_watcher smi_watcher = {
	.owner    = THIS_MODULE,
	.new_smi  = ipmi_new_smi,
	.smi_gone = ipmi_smi_gone
};

static int __init ipmi_wdog_init(void)
{
	int rv;

#ifdef HAVE_DIE_NMI
	rv = register_nmi_handler(NMI_UNKNOWN, ipmi_nmi, 0, "ipmi");
	if (rv)
		pr_warn("Can't register nmi handler: %d\n", rv);
	else
		nmi_handler_registered = 1;
#endif

	register_reboot_notifier(&wdog_reboot_notifier);

	rv = ipmi_smi_watcher_register(&smi_watcher);
	if (rv) {
#ifdef HAVE_DIE_NMI
		if (nmi_handler_registered)
			unregister_nmi_handler(NMI_UNKNOWN, "ipmi");
#endif
		unregister_reboot_notifier(&wdog_reboot_notifier);
		pr_warn("can't register smi watcher\n");
		return rv;
	}

	return 0;
}

static void __exit ipmi_wdog_exit(void)
{
	ipmi_smi_watcher_unregister(&smi_watcher);
	ipmi_unregister_watchdog(watchdog_ifnum);

#ifdef HAVE_DIE_NMI
	if (nmi_handler_registered)
		unregister_nmi_handler(NMI_UNKNOWN, "ipmi");
#endif

	unregister_reboot_notifier(&wdog_reboot_notifier);
}
module_exit(ipmi_wdog_exit);
module_init(ipmi_wdog_init);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Corey Minyard <minyard@mvista.com>");
MODULE_DESCRIPTION("watchdog timer based upon the IPMI interface.");
