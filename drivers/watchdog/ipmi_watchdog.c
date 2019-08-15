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
#include <linux/rwsem.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/notifier.h>
#include <linux/nmi.h>
#include <linux/reboot.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include <linux/sched/signal.h>
#include <linux/rcupdate.h>

#include "watchdog_pretimeout.h"

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
#include <asm/kdebug.h>
#include <asm/nmi.h>
#define HAVE_DIE_NMI
#endif

/*
 * The IPMI command/response information for the watchdog timer.
 */

/* values for byte 1 of the set command, byte 2 of the get response. */
#define WDOG_DONT_LOG		(1 << 7)
#define WDOG_DONT_STOP_ON_SET	(1 << 6)
#define WDOG_SET_TIMER_USE(byte, use) \
	byte = ((byte) & 0xf8) | ((use) & 0x7)
#define WDOG_GET_TIMER_USE(byte) ((byte) & 0x7)
#define WDOG_TIMER_USE_BIOS_FRB2	1
#define WDOG_TIMER_USE_BIOS_POST	2
#define WDOG_TIMER_USE_OS_LOAD		3
#define WDOG_TIMER_USE_SMS_OS		4
#define WDOG_TIMER_USE_OEM		5

/* values for byte 2 of the set command, byte 3 of the get response. */
#define WDOG_SET_PRETIMEOUT_ACT(byte, use) \
	byte = ((byte) & 0x8f) | (((use) & 0x7) << 4)
#define WDOG_GET_PRETIMEOUT_ACT(byte) (((byte) >> 4) & 0x7)
#define WDOG_PRETIMEOUT_NONE		0
#define WDOG_PRETIMEOUT_SMI		1
#define WDOG_PRETIMEOUT_NMI		2
#define WDOG_PRETIMEOUT_MSG_INT		3

/* Actions to perform on a full timeout. */
#define WDOG_SET_TIMEOUT_ACT(byte, use) \
	byte = ((byte) & 0xf8) | ((use) & 0x7)
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
#define WDOG_SET_TIMEOUT(byte1, byte2, val) \
	(byte1) = (((val) * 10) & 0xff), (byte2) = (((val) * 10) >> 8)
#define WDOG_GET_TIMEOUT(byte1, byte2) \
	(((byte1) | ((byte2) << 8)) / 10)

#define IPMI_WDOG_RESET_TIMER		0x22
#define IPMI_WDOG_SET_TIMER		0x24
#define IPMI_WDOG_GET_TIMER		0x25

#define IPMI_WDOG_TIMER_NOT_INIT_RESP	0x80

#define IPMI_MAX_TIMEOUT	6553	/* 16 bit value in 1/10 of a second */
#define IPMI_MIN_TIMEOUT	0

struct ipmi_wdt {
	struct watchdog_device wdd;
	struct watchdog_info info;
	struct ipmi_user *user;
	int ifnum;		/* IPMI interface number. */
	u8 ipmi_version_major;
	u8 ipmi_version_minor;

	struct mutex lock;

	struct completion msg_wait;
	atomic_t msg_tofree;
	struct ipmi_smi_msg smi_msg;
	struct ipmi_recv_msg recv_msg;

	int state; /* One of WDOG_TIMEOUT_xxx, NONE if disabled. */
	int prestate;  /* One of WDOG_PRETIMEOUT_xxx. */

	bool nmi_works;

	atomic_t pretimeout_since_last_heartbeat;
	bool panic_event_handled;
};

#define wdd_to_ipmi_wdt(wdd) container_of(wdd, struct ipmi_wdt, wdd)

/* Parameters to ipmi_set_timeout(), _ipmi_update_timeout() */
#define IPMI_SET_TIMEOUT_NO_HB			0
#define IPMI_SET_TIMEOUT_HB_IF_NECESSARY	1
#define IPMI_SET_TIMEOUT_FORCE_HB		2

static int _ipmi_update_timeout(struct ipmi_wdt *iwd, int do_heartbeat,
				bool do_poll);
static int ipmi_set_timeout(struct ipmi_wdt *iwd, int timeout,
			    int do_heartbeat);
static int ipmi_set_pretimeout(struct ipmi_wdt *iwd, int timeout,
			       int do_heartbeat);
static int ipmi_set_action(struct ipmi_wdt *iwd, unsigned int action);
static int ipmi_set_preaction(struct ipmi_wdt *iwd, unsigned int action);
static void ipmi_register_watchdog(int ipmi_intf, struct device *dev);
static void _ipmi_unregister_watchdog(struct ipmi_wdt *iwd);
static void ipmi_unregister_watchdog(int ipmi_intf);

/*
 * Only used if HAVE_DIE_NMI is set, but pretimeout will not be delivered
 * if this is set.
 * 0 = not testing
 * 1 = test running, no NMI
 * 2 = test running, got an NMI
 */
static int testing_nmi;


/* Protects the following values. */
static DEFINE_MUTEX(ipmi_wdt_data_mutex);
static int ifnum_to_use = -1;
static struct ipmi_wdt *ipmi_wdt;

static bool nowayout = WATCHDOG_NOWAYOUT;

static int timeout = 10; /* Default timeout. */
static int pretimeout; /* Default pretimeout. */
static int panic_wdt_timeout = 255; /* Default timeout to set on panic */

#define WDOG_PREOP_NONE		0
#define WDOG_PREOP_PANIC	1
#define WDOG_PREOP_GIVE_DATA	2

/* Default action is to reset the board on a timeout. */
static unsigned char def_action_val = WDIOA_RESET;
static unsigned char def_preaction_val = WDIOP_NONE;
static unsigned char def_preop_val = WDOG_PREOP_NONE;

/*
 * If true, the driver will start running as soon as it is configured
 * and ready.
 */
static int start_now;

static int action_op(const char *inval, char *outval)
{
	int rv = 0;
	unsigned int new_val;

	mutex_lock(&ipmi_wdt_data_mutex);
	if (outval) {
		switch (def_action_val) {
		case WDIOA_RESET:
			strcpy(outval, "reset");
			break;
		case WDIOA_POWER_OFF:
			strcpy(outval, "power_off");
			break;
		case WDIOA_POWER_CYCLE:
			strcpy(outval, "power_cycle");
			break;
		default:
			strcpy(outval, "?");
			break;
		}
	}

	if (!inval)
		goto out_unlock;

	if (strcmp(inval, "reset") == 0)
		new_val = WDIOA_RESET;
	else if (strcmp(inval, "power_cycle") == 0)
		new_val = WDIOA_POWER_CYCLE;
	else if (strcmp(inval, "power_off") == 0)
		new_val = WDIOA_POWER_OFF;
	else
		rv = -EINVAL;
	if (!rv && ipmi_wdt)
		rv = ipmi_set_action(ipmi_wdt, new_val);
	if (!rv)
		def_action_val = new_val;
out_unlock:
	mutex_unlock(&ipmi_wdt_data_mutex);

	return rv;
}

static int preaction_op(const char *inval, char *outval)
{
	int rv = 0;
	unsigned int new_val;

	mutex_lock(&ipmi_wdt_data_mutex);
	if (outval) {
		switch (def_preaction_val) {
		case WDIOP_NONE:
			strcpy(outval, "pre_none");
			break;
		case WDIOP_NMI:
			strcpy(outval, "pre_nmi");
			break;
		case WDIOP_SMI:
			strcpy(outval, "pre_smi");
			break;
		case WDIOP_INTERRUPT:
			strcpy(outval, "pre_int");
			break;
		default:
			strcpy(outval, "?");
			break;
		}
	}

	if (!inval)
		goto out_unlock;

	if (strcmp(inval, "pre_none") == 0) {
		new_val = WDIOP_NONE;
	} else if (strcmp(inval, "pre_smi") == 0) {
		new_val = WDIOP_SMI;
#ifdef HAVE_DIE_NMI
	} else if (strcmp(inval, "pre_nmi") == 0) {
		new_val = WDIOP_NMI;
#endif
	} else if (strcmp(inval, "pre_int") == 0) {
		new_val = WDIOP_INTERRUPT;
	} else {
		rv = -EINVAL;
	}
	if (!rv && ipmi_wdt)
		rv = ipmi_set_preaction(ipmi_wdt, new_val);
	if (!rv)
		def_preaction_val = new_val;
out_unlock:
	mutex_unlock(&ipmi_wdt_data_mutex);

	return rv;
}

static const char *preop_to_governor(unsigned int preop)
{
	switch (preop) {
	case WDOG_PREOP_NONE:
		return "noop";
	case WDOG_PREOP_PANIC:
		return "panic";
	case WDOG_PREOP_GIVE_DATA:
		return "read_data";
	default:
		return NULL;
	}
}
static int preop_op(const char *inval, char *outval)
{
	int rv = 0;
	const char *gov = NULL;
	unsigned int new_val;

	mutex_lock(&ipmi_wdt_data_mutex);
	if (outval) {
		switch (def_preop_val) {
		case WDOG_PREOP_NONE:
			strcpy(outval, "preop_none");
			break;
		case WDOG_PREOP_PANIC:
			strcpy(outval, "preop_panic");
			break;
		case WDOG_PREOP_GIVE_DATA:
			strcpy(outval, "preop_give_data");
			break;
		default:
			strcpy(outval, "?");
			break;
		}
	}

	if (!inval)
		goto out_unlock;

	if (strcmp(inval, "preop_none") == 0) {
		new_val = WDOG_PREOP_NONE;
	} else if (strcmp(inval, "preop_panic") == 0) {
		new_val = WDOG_PREOP_PANIC;
	} else if (strcmp(inval, "preop_give_data") == 0) {
		new_val = WDOG_PREOP_GIVE_DATA;
	} else {
		rv = -EINVAL;
	}
	if (!rv && ipmi_wdt) {
		gov = preop_to_governor(new_val);
		if (!gov)
			rv = -EINVAL;
		if (!rv) {
			rv = watchdog_pretimeout_governor_set(&ipmi_wdt->wdd,
							      gov);
		}
	}
	if (!rv)
		def_preop_val = new_val;
out_unlock:
	mutex_unlock(&ipmi_wdt_data_mutex);

	return rv;
}

static struct ipmi_smi_watcher smi_watcher = {
	.owner    = THIS_MODULE,
	.new_smi  = ipmi_register_watchdog,
	.smi_gone = ipmi_unregister_watchdog
};

static int set_param_timeout(const char *val, const struct kernel_param *kp)
{
	unsigned long l;
	int  rv = 0;

	if (!val)
		return -EINVAL;
	rv = kstrtoul(val, 0, &l);
	if (rv)
		return rv;

	if (l < IPMI_MIN_TIMEOUT || l > IPMI_MAX_TIMEOUT)
		return -EINVAL;

	mutex_lock(&ipmi_wdt_data_mutex);
	*((int *)kp->arg) = l;
	if (!ipmi_wdt)
		goto out_unlock;
	if (kp->arg == &timeout)
		rv = ipmi_set_timeout(ipmi_wdt, l,
				      IPMI_SET_TIMEOUT_HB_IF_NECESSARY);
	else if (kp->arg == &pretimeout)
		rv = ipmi_set_pretimeout(ipmi_wdt, l,
					 IPMI_SET_TIMEOUT_HB_IF_NECESSARY);

out_unlock:
	mutex_unlock(&ipmi_wdt_data_mutex);

	return rv;
}

static const struct kernel_param_ops param_ops_timeout = {
	.set = set_param_timeout,
	.get = param_get_int,
};
#define param_check_timeout param_check_int

typedef int (*action_fn)(const char *intval, char *outval);

static int set_param_str(const char *val, const struct kernel_param *kp)
{
	action_fn  fn = (action_fn) kp->arg;
	char       valcp[16];
	char       *s;

	strncpy(valcp, val, 15);
	valcp[15] = '\0';

	s = strstrip(valcp);

	return fn(s, NULL);
}

static int get_param_str(char *buffer, const struct kernel_param *kp)
{
	action_fn fn = (action_fn) kp->arg;
	int       rv;

	rv = fn(NULL, buffer);
	if (rv)
		return rv;
	return strlen(buffer);
}

static int set_param_wdog_ifnum(const char *val, const struct kernel_param *kp)
{
	int rv;
	unsigned long l;
	struct ipmi_wdt *old_iwd = NULL;

	if (!val)
		return 0;

	rv = kstrtoul(val, 0, &l);
	if (rv)
		return rv;

	mutex_lock(&ipmi_wdt_data_mutex);
	if (l == ifnum_to_use) {
		mutex_unlock(&ipmi_wdt_data_mutex);
		return 0;
	}

	if (ipmi_wdt) {
		old_iwd = ipmi_wdt;
		ipmi_wdt = NULL;
	}
	mutex_unlock(&ipmi_wdt_data_mutex);

	if (old_iwd)
		_ipmi_unregister_watchdog(old_iwd);

	/*
	 * Re-register the smi watcher to run through all the IPMI
	 * interfaces again.
	 */
	ipmi_smi_watcher_unregister(&smi_watcher);
	rv = ipmi_smi_watcher_register(&smi_watcher);
	if (rv) {
		pr_warn("can't register smi watcher\n");
		return rv;
	}

	return 0;
}

static const struct kernel_param_ops param_ops_wdog_ifnum = {
	.set = set_param_wdog_ifnum,
	.get = param_get_int,
};

#define param_check_wdog_ifnum param_check_int

static const struct kernel_param_ops param_ops_str = {
	.set = set_param_str,
	.get = get_param_str,
};

module_param(ifnum_to_use, wdog_ifnum, 0644);
MODULE_PARM_DESC(ifnum_to_use, "The interface number to use for the watchdog "
		 "timer.  Setting to -1 defaults to the first registered "
		 "interface");

module_param(timeout, timeout, 0644);
MODULE_PARM_DESC(timeout, "Timeout value in seconds.");

module_param(pretimeout, timeout, 0644);
MODULE_PARM_DESC(pretimeout, "Pretimeout value in seconds.");

module_param(panic_wdt_timeout, timeout, 0644);
MODULE_PARM_DESC(panic_wdt_timeout, "Timeout value on kernel panic in seconds.");

module_param_cb(action, &param_ops_str, action_op, 0644);
MODULE_PARM_DESC(action, "Timeout action. One of: "
		 "reset, none, power_cycle, power_off.");

module_param_cb(preaction, &param_ops_str, preaction_op, 0644);
MODULE_PARM_DESC(preaction, "Pretimeout action.  One of: "
		 "pre_none, pre_smi, pre_nmi, pre_int.");

module_param_cb(preop, &param_ops_str, preop_op, 0644);
MODULE_PARM_DESC(preop, "Pretimeout driver operation.  One of: "
		 "preop_none, preop_panic, preop_give_data.");

module_param(start_now, int, 0444);
MODULE_PARM_DESC(start_now, "Set to 1 to start the watchdog as"
		 "soon as the driver is loaded.");

module_param(nowayout, bool, 0644);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started "
		 "(default=CONFIG_WATCHDOG_NOWAYOUT)");

static int __ipmi_heartbeat(struct ipmi_wdt *iwd, bool do_poll);

/*
 * We use a mutex to make sure that only one thing can send a set a
 * message at one time.  The mutex is claimed when a message is sent
 * and freed when both the send and receive messages are free.
 */
static void msg_free_smi(struct ipmi_smi_msg *msg)
{
	struct ipmi_wdt *iwd = container_of(msg, struct ipmi_wdt, smi_msg);

	if (atomic_dec_and_test(&iwd->msg_tofree))
		complete(&iwd->msg_wait);
}
static void msg_free_recv(struct ipmi_recv_msg *msg)
{
	struct ipmi_wdt *iwd = container_of(msg, struct ipmi_wdt, recv_msg);

	if (atomic_dec_and_test(&iwd->msg_tofree))
		complete(&iwd->msg_wait);
}

static void wait_msg_done(struct ipmi_wdt *iwd, bool do_poll)
{
	if (do_poll) {
		while (atomic_read(&iwd->msg_tofree) != 0)
			ipmi_poll_interface(iwd->user);
	} else {
		wait_for_completion(&iwd->msg_wait);
	}
}

static int __ipmi_update_timeout(struct ipmi_wdt *iwd, int *send_heartbeat_now)
{
	struct kernel_ipmi_msg            msg;
	unsigned char                     data[6];
	int                               rv;
	struct ipmi_system_interface_addr addr;
	int                               hbnow = 0;

	data[0] = 0;
	WDOG_SET_TIMER_USE(data[0], WDOG_TIMER_USE_SMS_OS);

	/* If timer is running, keep it running. */
	if (iwd->state != WDOG_TIMEOUT_NONE) {
		if ((iwd->ipmi_version_major > 1)
		    || ((iwd->ipmi_version_major == 1) &&
			(iwd->ipmi_version_minor >= 5)))
			/* This is an IPMI 1.5-only feature. */
			data[0] |= WDOG_DONT_STOP_ON_SET;
		else
		/*
		 * In ipmi 1.0, setting the timer stops the watchdog, we
		 * need to start it back up again.
		 */
		hbnow = 1;
	}

	data[1] = 0;
	WDOG_SET_TIMEOUT_ACT(data[1], iwd->state);
	if ((iwd->wdd.pretimeout > 0) &&
	    (iwd->state != WDOG_TIMEOUT_NONE)) {
		WDOG_SET_PRETIMEOUT_ACT(data[1], iwd->prestate);
		data[2] = iwd->wdd.pretimeout;
	} else {
		WDOG_SET_PRETIMEOUT_ACT(data[1], WDOG_PRETIMEOUT_NONE);
		data[2] = 0; /* No pretimeout. */
	}
	data[3] = 0;
	WDOG_SET_TIMEOUT(data[4], data[5], iwd->wdd.timeout);

	addr.addr_type = IPMI_SYSTEM_INTERFACE_ADDR_TYPE;
	addr.channel = IPMI_BMC_CHANNEL;
	addr.lun = 0;

	msg.netfn = 0x06;
	msg.cmd = IPMI_WDOG_SET_TIMER;
	msg.data = data;
	msg.data_len = sizeof(data);
	rv = ipmi_request_supply_msgs(iwd->user,
				      (struct ipmi_addr *) &addr,
				      0,
				      &msg,
				      NULL,
				      &iwd->smi_msg,
				      &iwd->recv_msg,
				      1);
	if (rv)
		pr_warn("set timeout error: %d\n", rv);
	else if (send_heartbeat_now)
		*send_heartbeat_now = hbnow;

	return rv;
}

static int _ipmi_update_timeout(struct ipmi_wdt *iwd, int do_heartbeat,
				bool do_poll)
{
	int send_heartbeat_now;
	int rv;

	atomic_set(&iwd->msg_tofree, 2);

	rv = __ipmi_update_timeout(iwd, &send_heartbeat_now);
	if (rv) {
		atomic_set(&iwd->msg_tofree, 0);
		return rv;
	}

	wait_msg_done(iwd, do_poll);

	if ((do_heartbeat == IPMI_SET_TIMEOUT_FORCE_HB)
		|| ((send_heartbeat_now)
		    && (do_heartbeat == IPMI_SET_TIMEOUT_HB_IF_NECESSARY)))
		rv = __ipmi_heartbeat(iwd, do_poll);

	return rv;
}

static int ipmi_set_timeout(struct ipmi_wdt *iwd, int val, int do_heartbeat)
{
	int rv;

	mutex_lock(&iwd->lock);
	if (val <= iwd->wdd.pretimeout) {
		pr_warn("pretimeout set >= timeout, pretimeout disabled\n");
		iwd->wdd.pretimeout = 0;
	}
	iwd->wdd.timeout = val;
	rv = _ipmi_update_timeout(iwd, do_heartbeat, false);
	mutex_unlock(&iwd->lock);

	return rv;
}

static int ipmi_set_pretimeout(struct ipmi_wdt *iwd, int val, int do_heartbeat)
{
	int rv;

	mutex_lock(&iwd->lock);
	if (val >= iwd->wdd.timeout) {
		pr_warn("pretimeout set >= timeout, pretimeout disabled\n");
		val = 0;
	}
	iwd->wdd.pretimeout = val;
	rv = _ipmi_update_timeout(iwd, do_heartbeat, false);
	mutex_unlock(&iwd->lock);

	return rv;
}

/*
 * Special call, doesn't claim any locks.  This is only to be called
 * at panic or halt time, in run-to-completion mode, when the caller
 * is the only CPU and the only thing that will be going is these IPMI
 * calls.
 */
static void panic_halt_ipmi_update_timeout(struct ipmi_wdt *iwd)
{
	int rv;

	wait_msg_done(iwd, true);
	rv = _ipmi_update_timeout(iwd, IPMI_SET_TIMEOUT_HB_IF_NECESSARY, true);
	if (rv)
		pr_warn("Unable to extend the watchdog timeout\n");
}

static int __ipmi_heartbeat(struct ipmi_wdt *iwd, bool do_poll)
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
	if (iwd->state == WDOG_TIMEOUT_NONE)
		return 0;

	atomic_set(&iwd->msg_tofree, 2);

	addr.addr_type = IPMI_SYSTEM_INTERFACE_ADDR_TYPE;
	addr.channel = IPMI_BMC_CHANNEL;
	addr.lun = 0;

	msg.netfn = 0x06;
	msg.cmd = IPMI_WDOG_RESET_TIMER;
	msg.data = NULL;
	msg.data_len = 0;
	rv = ipmi_request_supply_msgs(iwd->user,
				      (struct ipmi_addr *) &addr,
				      0,
				      &msg,
				      NULL,
				      &iwd->smi_msg,
				      &iwd->recv_msg,
				      1);
	if (rv) {
		atomic_set(&iwd->msg_tofree, 0);
		pr_warn("heartbeat send failure: %d\n", rv);
		return rv;
	}

	/* Wait for the heartbeat to be sent. */
	wait_msg_done(iwd, do_poll);

	if (iwd->recv_msg.msg.data[0] == IPMI_WDOG_TIMER_NOT_INIT_RESP)  {
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
		rv = _ipmi_update_timeout(iwd, IPMI_SET_TIMEOUT_NO_HB,
					  do_poll);
		if (rv) {
			pr_err("Unable to send the command to set the watchdog's settings, giving up\n");
			goto out;
		}

		/* Might need a heartbeat send, go ahead and do it. */
		goto restart;
	} else if (iwd->recv_msg.msg.data[0] != 0) {
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

static int _ipmi_heartbeat(struct ipmi_wdt *iwd)
{
	int rv;

	if (atomic_cmpxchg(&iwd->pretimeout_since_last_heartbeat, 1, 0)) {
		/*
		 * A pretimeout occurred, make sure we set the timeout.
		 * We don't want to set the action, though, we want to
		 * leave that alone (thus it can't be combined with the
		 * above operation.
		 */
		rv = _ipmi_update_timeout(iwd,
					  IPMI_SET_TIMEOUT_HB_IF_NECESSARY,
			false);
	} else {
		rv = __ipmi_heartbeat(iwd, false);
	}

	return rv;
}

static unsigned int ipmi_wdt_get_timeleft(struct watchdog_device *wdd)
{
	struct ipmi_wdt *iwd = wdd_to_ipmi_wdt(wdd);
	struct kernel_ipmi_msg msg;
	int rv = 0;
	struct ipmi_system_interface_addr addr;

	mutex_lock(&iwd->lock);
	if (iwd->state == WDOG_TIMEOUT_NONE)
		goto out_unlock;

	addr.addr_type = IPMI_SYSTEM_INTERFACE_ADDR_TYPE;
	addr.channel = IPMI_BMC_CHANNEL;
	addr.lun = 0;

	atomic_set(&iwd->msg_tofree, 2);

	msg.netfn = 0x06;
	msg.cmd = IPMI_WDOG_GET_TIMER;
	msg.data = NULL;
	msg.data_len = 0;
	rv = ipmi_request_supply_msgs(iwd->user,
				      (struct ipmi_addr *) &addr,
				      0,
				      &msg,
				      NULL,
				      &iwd->smi_msg,
				      &iwd->recv_msg,
				      1);
	if (rv) {
		pr_warn("get timeout error: %d\n", rv);
		goto out_unlock;
	}

	wait_msg_done(iwd, false);

	if (iwd->recv_msg.msg.data[0] != 0)  {
		pr_warn("get timeout IPMI error: %d\n",
			iwd->recv_msg.msg.data[0]);
		goto out_unlock;
	}

	if (iwd->recv_msg.msg.data_len < 9) {
		pr_warn("get timeout IPMI response too short: %d\n",
			iwd->recv_msg.msg.data_len);
		goto out_unlock;
	}

	rv = iwd->recv_msg.msg.data[8] << 8 | iwd->recv_msg.msg.data[7];
	rv /= 10; /* IPMI time is in 100s of milliseconds. */

out_unlock:
	mutex_unlock(&iwd->lock);
	return rv;
}

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

static void ipmi_wdog_pretimeout_handler(void *handler_data)
{
	struct ipmi_wdt *iwd = handler_data;

	/*
	 * On some machines, the heartbeat will give an error and not
	 * work unless we re-enable the timer.  So do so.  Do this
	 * before the notify because it may panic.
	 */
	atomic_set(&iwd->pretimeout_since_last_heartbeat, 1);

	if (!testing_nmi)
		watchdog_notify_pretimeout(&iwd->wdd);
}

static void ipmi_wdog_panic_handler(void *user_data)
{
	struct ipmi_wdt *iwd = user_data;

	/*
	 * On a panic, if we have a panic timeout, make sure to extend
	 * the watchdog timer to a reasonable value to complete the
	 * panic, if the watchdog timer is running.  Plus the
	 * pretimeout is meaningless at panic time.
	 */
	if (!iwd->panic_event_handled &&
	    iwd->state != WDOG_TIMEOUT_NONE) {
		/* Make sure we do this only once. */
		iwd->panic_event_handled = true;

		iwd->wdd.timeout = panic_wdt_timeout;
		iwd->wdd.pretimeout = 0;
		panic_halt_ipmi_update_timeout(iwd);
	}
}

static int _ipmi_wdt_start(struct ipmi_wdt *iwd)
{
	int rv = 0;

	switch (iwd->wdd.action) {
	case WDIOA_RESET:
		iwd->state = WDOG_TIMEOUT_RESET;
		break;
	case WDIOA_POWER_OFF:
		iwd->state = WDOG_TIMEOUT_RESET;
		break;
	case WDIOA_POWER_CYCLE:
		iwd->state = WDOG_TIMEOUT_RESET;
		break;
	default:
		rv = -EINVAL;
		break;
	}
	if (!rv)
		rv = _ipmi_update_timeout(iwd, IPMI_SET_TIMEOUT_FORCE_HB,
					  false);

	return rv;
}

static int ipmi_wdt_start(struct watchdog_device *wdd)
{
	return _ipmi_wdt_start(wdd_to_ipmi_wdt(wdd));
}

static int ipmi_wdt_stop(struct watchdog_device *wdd)
{
	struct ipmi_wdt *iwd = wdd_to_ipmi_wdt(wdd);
	int rv;

	mutex_lock(&iwd->lock);
	iwd->state = WDOG_TIMEOUT_NONE;
	rv = _ipmi_update_timeout(iwd, IPMI_SET_TIMEOUT_NO_HB, false);
	mutex_unlock(&iwd->lock);

	return rv;
}

static int ipmi_wdt_ping(struct watchdog_device *wdd)
{
	struct ipmi_wdt *iwd = wdd_to_ipmi_wdt(wdd);
	int rv;

	mutex_lock(&iwd->lock);
	rv = _ipmi_heartbeat(iwd);
	mutex_unlock(&iwd->lock);

	return rv;
}

static int ipmi_wdt_set_timeout(struct watchdog_device *wdd, unsigned int val)
{
	return ipmi_set_timeout(wdd_to_ipmi_wdt(wdd), val,
				IPMI_SET_TIMEOUT_HB_IF_NECESSARY);
}

static int ipmi_wdt_set_pretimeout(struct watchdog_device *wdd,
				   unsigned int val)
{
	struct ipmi_wdt *iwd = wdd_to_ipmi_wdt(wdd);

	return ipmi_set_pretimeout(iwd, val, IPMI_SET_TIMEOUT_HB_IF_NECESSARY);
}

static int ipmi_set_action(struct ipmi_wdt *iwd, unsigned int val)
{
	int rv = 0;

	mutex_lock(&iwd->lock);
	if (val == iwd->wdd.action)
		goto out_unlock;
	switch (val) {
	case WDIOA_RESET:
	case WDIOA_POWER_OFF:
	case WDIOA_POWER_CYCLE:
		iwd->wdd.action = val;
		break;
	default:
		rv = -EINVAL;
	}
	if (!rv)
		rv = _ipmi_wdt_start(iwd);
out_unlock:
	mutex_unlock(&iwd->lock);

	return rv;
}

static int ipmi_wdt_set_action(struct watchdog_device *wdd,
			       unsigned int val)
{
	return ipmi_set_action(wdd_to_ipmi_wdt(wdd), val);
}

static int ipmi_set_preaction(struct ipmi_wdt *iwd, unsigned int val)
{
	int rv = 0;
	int new_state = 0;

	mutex_lock(&iwd->lock);
	if (val == iwd->wdd.preaction)
		goto out_unlock;
	switch (val) {
	case WDIOP_NONE:
		new_state = WDOG_PRETIMEOUT_NONE;
		break;
	case WDIOP_SMI:
		new_state = WDOG_PRETIMEOUT_SMI;
		break;
	case WDIOP_INTERRUPT:
		new_state = WDOG_PRETIMEOUT_MSG_INT;
		break;
#ifdef HAVE_DIE_NMI
	case WDIOP_NMI:
		if (!iwd->nmi_works)
			rv = -EINVAL;
		else
			new_state = WDOG_PRETIMEOUT_NMI;
		break;
#endif
	default:
		rv = -EINVAL;
	}
	if (!rv) {
		int old_preaction = iwd->wdd.preaction;
		int old_prestate = iwd->prestate;

		iwd->wdd.preaction = val;
		iwd->prestate = new_state;
		rv = _ipmi_wdt_start(iwd);
		if (rv) {
			iwd->wdd.preaction = old_preaction;
			iwd->prestate = old_prestate;
		}
	}
out_unlock:
	mutex_unlock(&iwd->lock);

	return rv;
}

static int ipmi_wdt_set_preaction(struct watchdog_device *wdd,
				  unsigned int val)
{
	return ipmi_set_preaction(wdd_to_ipmi_wdt(wdd), val);
}

static const struct watchdog_info ipmi_wdt_ident = {
	.options	= WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT |
			  WDIOF_PRETIMEOUT | WDIOF_MAGICCLOSE,
	.firmware_version = 1,
	.identity	= "IPMI"
};

static const struct watchdog_ops ipmi_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= ipmi_wdt_start,
	.stop		= ipmi_wdt_stop,
	.ping		= ipmi_wdt_ping,
	.set_timeout	= ipmi_wdt_set_timeout,
	.set_pretimeout = ipmi_wdt_set_pretimeout,
	.get_timeleft	= ipmi_wdt_get_timeleft,
	.set_action	= ipmi_wdt_set_action,
	.set_preaction	= ipmi_wdt_set_preaction,
};

static const struct ipmi_user_hndl ipmi_hndlrs = {
	.ipmi_recv_hndl           = ipmi_wdog_msg_handler,
	.ipmi_watchdog_pretimeout = ipmi_wdog_pretimeout_handler,
	.ipmi_panic_handler       = ipmi_wdog_panic_handler
};

#ifdef HAVE_DIE_NMI
static DEFINE_MUTEX(ipmi_nmi_test_mutex);
static bool nmi_handler_registered;

/* If a pretimeout occurs, this is used to allow only one panic to happen. */
static atomic_t preop_panic_excl = ATOMIC_INIT(-1);

static int
ipmi_nmi(unsigned int val, struct pt_regs *regs)
{
	struct ipmi_wdt *iwd = READ_ONCE(ipmi_wdt);

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

	if (!iwd)
		return NMI_DONE;

	/* If we are not expecting a timeout, ignore it. */
	if (iwd->state == WDOG_TIMEOUT_NONE)
		return NMI_DONE;

	if (iwd->wdd.preaction != WDIOP_NMI)
		return NMI_DONE;

	/*
	 * If no one else handled the NMI, we assume it was the IPMI
	 * watchdog.
	 */
	if (iwd->wdd.gov && strcmp(iwd->wdd.gov->name, "panic") == 0) {
		/*
		 * On some machines, the heartbeat will give an error
		 * and not work unless we re-enable the timer.  So
		 * make it do so on the next heartbeat.
		 */
		atomic_set(&iwd->pretimeout_since_last_heartbeat, 1);
		if (atomic_inc_and_test(&preop_panic_excl))
			nmi_panic(regs, "pre-timeout");
	}

	return NMI_HANDLED;
}

static void nmi_check(struct ipmi_wdt *iwd)
{
	int old_state = iwd->state;
	int old_prestate = iwd->prestate;
	int old_pretimeout = iwd->wdd.pretimeout;
	int old_timeout = iwd->wdd.timeout;
	int rv;
	unsigned int count;

	if (iwd->nmi_works)
		return;

	mutex_lock(&ipmi_nmi_test_mutex);
	if (!nmi_handler_registered) {
		rv = register_nmi_handler(NMI_UNKNOWN, ipmi_nmi, 0,
					  "ipmi");
		if (rv) {
			iwd->nmi_works = false;
			pr_warn("Can't register nmi handler\n");
			goto out_restore;
		} else {
			nmi_handler_registered = true;
		}
	}

	/*
	 * Set the pretimeout to go off in a second and give
	 * ourselves 99 seconds to stop the timer.
	 */
	iwd->state = WDOG_TIMEOUT_RESET;
	iwd->prestate = WDOG_PRETIMEOUT_NMI;
	iwd->wdd.pretimeout = 99;
	iwd->wdd.timeout = 100;

	testing_nmi = 1;

	rv = _ipmi_update_timeout(iwd, IPMI_SET_TIMEOUT_FORCE_HB, false);
	if (rv) {
		pr_warn("Error starting timer to test NMI: 0x%x.  The NMI pretimeout will likely not work\n",
			rv);
		iwd->nmi_works = false;
		goto out_restore;
	}

	if (iwd->recv_msg.msg.data[0] != 0)  {
		pr_warn("Error in IPMI NMI pretimeout set: 0x%x.  The NMI pretimeout will likely not work\n",
			iwd->recv_msg.msg.data[0]);
		iwd->nmi_works = false;
		goto out_restore;
	}

	for (count = 0; count < 50; count++) {
		if (testing_nmi == 2)
			break;
		msleep(20);
	}

	if (testing_nmi == 2) {
		iwd->nmi_works = true;
	} else {
		iwd->nmi_works = false;
		pr_warn("IPMI NMI didn't seem to occur.  The NMI pretimeout will likely not work\n");
	}
 out_restore:
	testing_nmi = 0;
	mutex_unlock(&ipmi_nmi_test_mutex);
	iwd->wdd.pretimeout = old_pretimeout;
	iwd->wdd.timeout = old_timeout;
	iwd->prestate = old_prestate;
	iwd->state = old_state;
	rv = _ipmi_update_timeout(iwd, IPMI_SET_TIMEOUT_FORCE_HB, false);
	if (rv)
		pr_warn("Unable to stop timer after NMI test: 0x%x.\n", rv);
}
#else
static void nmi_check(struct ipmi_wdt *iwd)
{
	iwd->nmi_works = false;
}
#endif

static void ipmi_register_watchdog(int ipmi_intf, struct device *dev)
{
	struct ipmi_wdt *iwd = NULL;
	int rv = -EBUSY;
	const char *gov;

	mutex_lock(&ipmi_wdt_data_mutex);
	if ((ifnum_to_use != -1) && (ifnum_to_use != ipmi_intf))
		goto out;
	/* -1 means allow the first interface. */
	if (ifnum_to_use == -1 && ipmi_wdt)
		goto out;

	iwd = kzalloc(sizeof(*iwd), GFP_KERNEL);
	if (!iwd) {
		rv = -ENOMEM;
		goto out;
	}
	iwd->ifnum = ipmi_intf;
	init_completion(&iwd->msg_wait);
	iwd->smi_msg.done = msg_free_smi;
	iwd->recv_msg.done = msg_free_recv;
	iwd->state = WDOG_TIMEOUT_NONE;
	mutex_init(&iwd->lock);

	rv = ipmi_create_user(ipmi_intf, &ipmi_hndlrs, iwd, &iwd->user);
	if (rv < 0) {
		pr_crit("Unable to register with ipmi\n");
		goto out;
	}

	rv = ipmi_get_version(iwd->user,
			      &iwd->ipmi_version_major,
			      &iwd->ipmi_version_minor);
	if (rv) {
		pr_warn("Unable to get IPMI version, assuming 1.0\n");
		iwd->ipmi_version_major = 1;
		iwd->ipmi_version_minor = 0;
	}

	iwd->wdd.ops = &ipmi_wdt_ops;
	iwd->info = ipmi_wdt_ident;
	iwd->info.firmware_version = (iwd->ipmi_version_major << 8 |
				      iwd->ipmi_version_minor);
	iwd->wdd.info = &iwd->info;
	iwd->wdd.max_timeout = IPMI_MAX_TIMEOUT;
	iwd->wdd.min_timeout = IPMI_MIN_TIMEOUT;
	watchdog_stop_on_unregister(&iwd->wdd);
	watchdog_set_nowayout(&iwd->wdd, nowayout);
	watchdog_init_timeout(&iwd->wdd, timeout, NULL);
	iwd->wdd.pretimeout = pretimeout;
	iwd->wdd.parent = dev;
	iwd->wdd.action = def_action_val;
	iwd->wdd.preaction = def_preaction_val;

	rv = watchdog_register_device(&iwd->wdd);
	if (rv) {
		ipmi_destroy_user(iwd->user);
		pr_crit("Unable to register IPMI watchdog device\n");
		goto out;
	}

	nmi_check(iwd);

	if (iwd->wdd.preaction == WDIOP_NMI && !iwd->nmi_works) {
		pr_warn("NMI pretimeout test failed, disabling pretimeout.\n");
		iwd->wdd.preaction = WDIOP_NONE;
	}

	gov = preop_to_governor(def_preop_val);
	if (!gov) {
		pr_warn("Invalid preop value: %d\n", def_preop_val);
	} else {
		rv = watchdog_pretimeout_governor_set(&iwd->wdd, gov);
		if (rv)
			pr_warn("Setting governor to %s failed: %d\n",
				gov, rv);
	}

	ipmi_wdt = iwd;
	rv = 0;

 out:
	mutex_unlock(&ipmi_wdt_data_mutex);
	if (start_now && rv == 0) {
		/* Run from startup, so start the timer now. */
		start_now = 0; /* Disable this function after first startup. */
		if (_ipmi_wdt_start(iwd))
			pr_warn("Starting now failed!\n");
		else
			pr_info("Starting now!\n");
	} else if (iwd && rv) {
		kfree(iwd);
	}
}

static void _ipmi_unregister_watchdog(struct ipmi_wdt *iwd)
{
	int rv;

	/*
	 * This function should make sure that the misc device has no
	 * outstanding calls, thus we should have no messages being
	 * used after this.
	 */
	watchdog_unregister_device(&iwd->wdd);

	/* Make sure no NMIs or interrupts are running. */
	synchronize_rcu();

	/* Disconnect from IPMI. */
	rv = ipmi_destroy_user(iwd->user);
	if (rv)
		pr_warn("error unlinking from IPMI: %d\n",  rv);

	kfree(iwd);
}

static void ipmi_unregister_watchdog(int ipmi_intf)
{
	struct ipmi_wdt *iwd = NULL;

	mutex_lock(&ipmi_wdt_data_mutex);
	if (ipmi_wdt->ifnum == ipmi_intf) {
		iwd = ipmi_wdt;
		ipmi_wdt = NULL;
	}
	mutex_unlock(&ipmi_wdt_data_mutex);
	if (!iwd)
		return;

	_ipmi_unregister_watchdog(iwd);
}

static int wdog_reboot_handler(struct notifier_block *this,
			       unsigned long         code,
			       void                  *unused)
{
	static int reboot_event_handled;
	struct ipmi_wdt *iwd = READ_ONCE(ipmi_wdt);

	if (iwd && !reboot_event_handled) {
		/* Make sure we only do this once. */
		reboot_event_handled = 1;

		if (code == SYS_POWER_OFF || code == SYS_HALT) {
			/* Disable the WDT if we are shutting down. */
			iwd->state = WDOG_TIMEOUT_NONE;
			_ipmi_update_timeout(iwd, IPMI_SET_TIMEOUT_NO_HB,
					     false);
		} else if (iwd->state != WDOG_TIMEOUT_NONE) {
			/* Set a long timer to let the reboot happen or
			   reset if it hangs, but only if the watchdog
			   timer was already running. */
			if (iwd->wdd.timeout < panic_wdt_timeout)
				iwd->wdd.timeout = panic_wdt_timeout;
			iwd->wdd.pretimeout = 0;
			_ipmi_update_timeout(iwd, IPMI_SET_TIMEOUT_NO_HB,
					     false);
		}
	}
	return NOTIFY_OK;
}

static struct notifier_block wdog_reboot_notifier = {
	.notifier_call	= wdog_reboot_handler,
	.next		= NULL,
	.priority	= 0
};

static int __init ipmi_wdog_init(void)
{
	int rv;

	register_reboot_notifier(&wdog_reboot_notifier);

	rv = ipmi_smi_watcher_register(&smi_watcher);
	if (rv) {
		unregister_reboot_notifier(&wdog_reboot_notifier);
		pr_warn("can't register smi watcher\n");
		return rv;
	}

	pr_info("driver initialized\n");

	return 0;
}

static void __exit ipmi_wdog_exit(void)
{
	ipmi_smi_watcher_unregister(&smi_watcher);
	if (ipmi_wdt)
		ipmi_unregister_watchdog(ipmi_wdt->ifnum);

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
