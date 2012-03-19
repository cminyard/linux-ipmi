/*
 * ipmi_serial.c
 *
 * The interface to the IPMI driver for the serial system interface
 *
 * Author: MontaVista Software, Inc.
 *         David Griego <dgriego@mvista.com>
 *         Corey Minyard <cminyard@mvista.com>
 *         source@mvista.com
 *
 * Copyright 2005,2006,2007 MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or (at your
 *  option) any later version.
 *
 *
 *  THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/rcupdate.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/ctype.h>
#include <linux/ipmi_smi.h>
#include <linux/ipmi_serial_sm.h>
#include <linux/init.h>
#include <linux/serial_core.h>

#define PFX "ipmi_serial(%s%d): " /* %s%d is for info->name, info->line */
#define NPFX "ipmi_serial: "

#define MAX_SERIAL_SETUP_STR 100
static char setup_str[MAX_SERIAL_SETUP_STR] = CONFIG_SERIAL_IPMI_SETUP;
module_param_string(port_info, setup_str, MAX_SERIAL_SETUP_STR, 0444);
MODULE_PARM_DESC(port_info, "Defines the perameters for the serial interface"
		 " i.e. port_info=ttyS1,38400,Direct:ttyS2,9600n81r,"
		 "TerminalMode,pp");

static int hotmod_handler(const char *val, struct kernel_param *kp);

module_param_call(hotmod, hotmod_handler, NULL, NULL, 0200);
MODULE_PARM_DESC(hotmod, "Add and remove interfaces.  See"
		 " Documentation/IPMI.txt in the kernel sources for the"
		 " gory details.");

static int unload_when_empty;
module_param(unload_when_empty, int, 0);
MODULE_PARM_DESC(unload_when_empty, "Unload the module if no interfaces are"
		 " specified or found, default is false (0).  Setting to 1"
		 " is useful for not loading the module if nothing is"
		 " found.");

#define IPMI_SER_DEBUG_STATE		1
#define IPMI_SER_DEBUG_MSG		2
#define IPMI_SER_DEBUG_DATA		4
#define IPMI_SER_DEBUG_TIMING		8
#define IPMI_SER_DEBUG_CHAR_TIMING	16
static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Set bit 0 to enable state debugging, bit 1 to"
		 " enable data debugging, bit 2 to enable message debugging,"
		 " bit 3 to enable message timing, bit 4 to enable"
		 " character timing.");

/* List of registered codecs */
static LIST_HEAD(codec_list);

/* List of interfaces that are configured. */
static LIST_HEAD(info_list);

/* Lock for the above two lists. */
static DEFINE_MUTEX(list_lock);

/* Call every 10 ms. */
#define IPMI_SERIAL_TIMEOUT_TIME_USEC	10000
#define IPMI_SERIAL_USEC_PER_JIFFY	(1000000 / HZ)
#define IPMI_SERIAL_TIMEOUT_JIFFIES	(IPMI_SERIAL_TIMEOUT_TIME_USEC /      \
					 IPMI_SERIAL_USEC_PER_JIFFY)


/* Timeouts in microseconds. */
#define IPMI_SERIAL_INIT_TIMEOUT	1000000
#define IPMI_SERIAL_INIT_RETRIES	10
#define IPMI_SERIAL_RETRY_TIMEOUT 	1000000
#define IPMI_SERIAL_MAX_ERROR_RETRIES 	10

/*
 * Polling for flags.  Note that event if the interface doesn't
 * request polling, we still poll once a second.
 */
#define NO_POLLING_FLAG_TIMEOUT		1000000
#define POLLING_FLAG_TIMEOUT		100000

enum ipmi_serial_state {
	SERIAL_EMPTY,
	SERIAL_HANDLING_MSG,
	SERIAL_GETTING_FLAGS,
	SERIAL_GETTING_EVENTS,
	SERIAL_CLEARING_FLAGS,
	SERIAL_GETTING_MESSAGES,
	/* FIXME - add watchdog stuff. */
};

enum ipmi_serial_states {
	SERIAL_IDLE,		/* The serial interface is currently
				   doing nothing. */
	SERIAL_WRITE,		/* We are writing bytes to the
				   interface. */
	SERIAL_READ,		/* We are waiting to read data from
				   the interface. */
	SERIAL_ASYNC_MSG,	/* Getting async message from BMC */
	SERIAL_COMPLETE,	/* Allows get message to complete
				   correctly */
	SERIAL_ERROR0,		/* State to transition to the error
				   handler, this was added to the
				   state machine in the spec to be
				   sure IBF was there. */
	SERIAL_HOSED		/* The hardware failed to follow the
				   state machine. */
};

#define TTY_NAME_LEN	8

/*
 * Indexes into stats[] in smb_info below.
 */
enum serial_stat_indexes {
	/* Number of watchdog pretimeouts */
	SERIAL_STAT_watchdog_pretimeouts = 0,

	/* Number of asyncronous messages received. */
	SERIAL_STAT_incoming_messages,

	/* Number of events received. */
	SERIAL_STAT_events,

	/* Number of completed requests. */
	SERIAL_STAT_complete_transactions,

	/* Number of times flags were fetched. */
	SERIAL_STAT_flag_fetches,

	/* Number of ATTN characters receved. */
	SERIAL_STAT_attentions,

	/* Number of times the timer went off. */
	SERIAL_STAT_timer_ticks,

	/* Number of transmitted characters. */
	SERIAL_STAT_xmit_chars,

	/* Number of received characters. */
	SERIAL_STAT_recv_chars,

	/* Number of times the protocol was violated. */
	SERIAL_STAT_protocol_violations,

	/* Number of times the checksum was incorrect. */
	SERIAL_STAT_checksum_errors,

	/* Number of times a transaction timed out */
	SERIAL_STAT_timeouts,

	/* Number of times more data than allowed was received in a message. */
	SERIAL_STAT_overruns,


	/* This *must* remain last, add new values above this. */
	SERIAL_NUM_STATS,
};

struct ipmi_serial_info {
	ipmi_smi_t		intf;
	struct list_head	link;

	spinlock_t		lock;

	struct list_head	xmit_msgs;
	struct list_head	hp_xmit_msgs;
	struct ipmi_smi_msg	*curr_msg;
	struct ipmi_smi_msg	*to_send;

	int                     msg_timeout;
	int                     retries;
	unsigned long           last_timeout_jiffies;
	struct timer_list	timer;

	enum ipmi_serial_state	state;

	bool upper_layer_ready;

	unsigned char slave_addr;

	/*
	 * Used to deliver things to the upper layer when processing
	 * is complete.  Instead of delivering immediately, we queue
	 * things up and deliver them when all processing is done.
	 * Makes locking a lot easier.
	 */
	int              watchdog_pretimeouts_to_deliver;
	int              msg_delivery_in_progress;
	struct list_head msgs_to_deliver;

	/*
	 * Sequence number for send messages.  Note that we reserve
	 * zero for special messages (getting flags, etc.); it is
	 * never used for normal messages.
	 */
	unsigned int     send_seq;

	/*
	 * If set to true, this will request events the next time the
	 * state machine is idle.  Only do this if do_event_request is
	 * true.  Otherwise the codec doesn't support this capability.
	 */
	bool req_events;
	bool do_event_request;
	bool has_event_buffer;
	int global_enable_valid;

	/*
	 * Used to handle automatic timing of getting flags.
	 */
	int flag_timeout;
	bool req_flags;
	int flag_timer;
	bool supports_flags;

	/*
	 * If true, run the state machine to completion on every send
	 * call.  Generally used after a panic to make sure stuff goes
	 * out.
	 */
	bool run_to_completion;

	/*
	 * The driver is shutting down, don't start anything new.
	 */
	bool stop_operation;

	/* Flags from the last GET_MSG_FLAGS command, used when an ATTN
	   is set to hold the flags until we are done handling everything
	   from the flags. */
#define RECEIVE_MSG_AVAIL	0x01
#define EVENT_MSG_BUFFER_FULL	0x02
#define WDT_PRE_TIMEOUT_INT	0x08
#define OEM0_DATA_AVAIL     0x20
#define OEM1_DATA_AVAIL     0x40
#define OEM2_DATA_AVAIL     0x80
#define OEM_DATA_AVAIL      (OEM0_DATA_AVAIL | \
			     OEM1_DATA_AVAIL | \
			     OEM2_DATA_AVAIL)
	unsigned char msg_flags;

	/* Per-OEM handler, called from handle_flags().
	   Returns 1 when handle_flags() needs to be re-run
	   or 0 indicating it set si_state itself.
	*/
	int (*oem_data_avail_handler)(struct ipmi_serial_info *info);

	/*
	 * CODEC information.
	 */
	struct ipmi_serial_codec *codec;
	struct ipmi_serial_codec_data *codec_data;
	struct ipmi_serial_codec_data *alloc_codec_data;

	/* Holds the parameter string. */
	char             *keepstr;

	/* Serial port information. */
	char		 name[TTY_NAME_LEN];	/* Name of the serial driver
						   we are looking for. */
	int		 line;			/* Serial line we are looking
						   for. */
	char		 *options;		/* Options for serial port  */
	char		 *codec_name;		/* Name of codec to try     */
	char		 *codec_options;	/* Options for the codec    */
	struct uart_direct direct;
	unsigned long	   poll_state;
	char               uart_buffer[UART_XMIT_SIZE];
	struct uart_port *port;
	struct ktermios  termios, old_termios;

	/* Buffer for incoming serial data. */
#define SBUF_NEXT(v) ((v + 1) % 128)
	unsigned char sbuf[128];
	unsigned int  sbuf_start, sbuf_next;
	int           delivering_char;

	/* From the get device id response... */
	int device_id_valid;
	struct ipmi_device_id device_id;

	/* Used to return intialization status. */
	int init_status;

	/* If this is set, returned responses will go to this function. */
	void (*internal_msg_handler)(struct ipmi_serial_info *info,
				     const unsigned char     *data,
				     unsigned int            len);

	atomic_t stats[SERIAL_NUM_STATS];
};

#define serial_inc_stat(serial, stat) \
	atomic_inc(&(serial)->stats[SERIAL_STAT_ ## stat])
#define serial_add_stat(serial, stat, val) \
	atomic_add(val, &(serial)->stats[SERIAL_STAT_ ## stat])
#define serial_get_stat(serial, stat) \
	((unsigned int) atomic_read(&(serial)->stats[SERIAL_STAT_ ## stat]))

static void ipmi_serial_cleanup_one(struct ipmi_serial_info *to_clean);

struct baud_rates {
	unsigned int rate;
	unsigned int cflag;
	char         *name;
};

static struct baud_rates baud_rates[] = {
	{ 921600, B921600, "921600" },
	{ 500000, B500000, "500000" },
	{ 460800, B460800, "460800" },
	{ 230400, B230400, "230400" },
	{ 115200, B115200, "115200" },
	{  57600, B57600,  "57600"  },
	{  38400, B38400,  "38400"  },
	{  19200, B19200,  "19200"  },
	{   9600, B9600,   "9600"   },
	{   4800, B4800,   "4800"   },
	{   2400, B2400,   "2400"   },
	{   1200, B1200,   "1200"   },
	{      0, B38400,  "?"      }
};

/*
 * If run_to_completion mode is on, return NULL to know the lock wasn't
 * taken.  Otherwise lock info->lock and return the flags.
 */
static unsigned long *ipmi_serial_lock_cond(struct ipmi_serial_info *info,
					    unsigned long *flags)
{
	if (info->run_to_completion)
		return NULL;
	spin_lock_irqsave(&info->lock, *flags);
	return flags;
}

static void ipmi_serial_unlock_cond(struct ipmi_serial_info *info,
				    unsigned long *flags)
{
	if (!flags)
		return;
	spin_unlock_irqrestore(&info->lock, *flags);
}

static void queue_return_msg(struct ipmi_serial_info *info)
{
	if (debug & IPMI_SER_DEBUG_TIMING) {
		struct timeval t;
		do_gettimeofday(&t);
		printk(KERN_DEBUG PFX "Recv msg at %ld.%6.6ld\n",
		       info->name, info->line, t.tv_sec, t.tv_usec);
	}
	if (debug & IPMI_SER_DEBUG_MSG) {
		int i;
		unsigned char *buf = info->curr_msg->rsp;
		printk(KERN_DEBUG PFX "Received msg:", info->name, info->line);
		for (i = 0; i < info->curr_msg->rsp_size; i++)
			printk(" %2.2x(%c)", buf[i],
			       isprint(buf[i]) ? buf[i] : ' ');
		printk("\n");
	}

	/* Queue the message for delivery when we release the lock. */
	list_add_tail(&info->curr_msg->link, &info->msgs_to_deliver);
	info->curr_msg = NULL;
}

static void queue_async_msg(struct ipmi_serial_info *info,
			    struct ipmi_smi_msg *msg)
{
	if (debug & IPMI_SER_DEBUG_TIMING) {
		struct timeval t;
		do_gettimeofday(&t);
		printk(KERN_DEBUG PFX "Recv async msg at %ld.%6.6ld\n",
		       info->name, info->line, t.tv_sec, t.tv_usec);
	}
	if (debug & IPMI_SER_DEBUG_MSG) {
		int i;
		unsigned char *buf = msg->rsp;
		printk(KERN_DEBUG PFX "Received async msg:",
		       info->name, info->line);
		for (i = 0; i < msg->rsp_size; i++)
			printk(" %2.2x(%c)", buf[i],
			       isprint(buf[i]) ? buf[i] : ' ');
		printk("\n");
	}

	/* Queue the message for delivery when we release the lock. */
	list_add_tail(&msg->link, &info->msgs_to_deliver);
}

static void queue_return_err_msg(struct ipmi_serial_info *info, int err)
{
	struct ipmi_smi_msg *msg = info->curr_msg;

	/* Make it a reponse */
	msg->rsp[0] = msg->data[0] | 4;
	msg->rsp[1] = msg->data[1];
	msg->rsp[2] = err;
	msg->rsp_size = 3;
	queue_return_msg(info);
}

static int send_curr_msg(struct ipmi_serial_info *info)
{
	if (!info->port)
		return -ENODEV;

	WARN_ON(info->to_send);
	if (info->to_send)
		return -EBUSY;

	if (debug & IPMI_SER_DEBUG_TIMING) {
		struct timeval t;
		do_gettimeofday(&t);
		printk(KERN_DEBUG PFX "Send msg at %ld.%6.6ld\n",
		       info->name, info->line, t.tv_sec, t.tv_usec);
	}
	if (debug & IPMI_SER_DEBUG_MSG) {
		int i;
		unsigned char *buf = info->curr_msg->data;
		printk(KERN_DEBUG PFX "Sent msg:", info->name, info->line);
		for (i = 0; i < info->curr_msg->data_size; i++)
			printk(" %2.2x(%c)", buf[i],
			       isprint(buf[i]) ? buf[i] : ' ');
		printk("\n");
	}
	info->to_send = info->curr_msg;
	return 0;
}

static int start_clear_flags(struct ipmi_serial_info *info)
{
	struct ipmi_smi_msg *msg;
	int                 rv;

	WARN_ON(info->curr_msg);
	if (info->curr_msg)
		return -EBUSY;

	msg = ipmi_alloc_smi_msg();
	if (!msg)
		return -ENOMEM;
	msg->data[0] = (IPMI_NETFN_APP_REQUEST << 2);
	msg->data[1] = IPMI_CLEAR_MSG_FLAGS_CMD;
	msg->data[2] = WDT_PRE_TIMEOUT_INT;
	msg->data_size = 3;
	info->curr_msg = msg;
	rv = send_curr_msg(info);
	if (!rv)
		info->state = SERIAL_CLEARING_FLAGS;
	else {
		msg->done(msg);
		info->curr_msg = NULL;
	}
	return rv;
}

static int start_flag_fetch(struct ipmi_serial_info *info)
{
	struct ipmi_smi_msg *msg;
	int                 rv;

	WARN_ON(info->curr_msg);
	if (info->curr_msg)
		return -EBUSY;

	msg = ipmi_alloc_smi_msg();
	if (!msg)
		return -ENOMEM;

	msg->data[0] = (IPMI_NETFN_APP_REQUEST << 2);
	msg->data[1] = IPMI_GET_MSG_FLAGS_CMD;
	msg->data_size = 2;
	info->curr_msg = msg;
	rv = send_curr_msg(info);
	if (!rv)
		info->state = SERIAL_GETTING_FLAGS;
	else {
		msg->done(msg);
		info->curr_msg = NULL;
	}
	return rv;
}

static int start_event_fetch(struct ipmi_serial_info *info)
{
	struct ipmi_smi_msg *msg;
	int                 rv;

	WARN_ON(info->curr_msg);
	if (info->curr_msg)
		return -EBUSY;

	msg = ipmi_alloc_smi_msg();
	if (!msg)
		return -ENOMEM;

	msg->data[0] = (IPMI_NETFN_APP_REQUEST << 2);
	msg->data[1] = IPMI_READ_EVENT_MSG_BUFFER_CMD;
	msg->data_size = 2;
	info->curr_msg = msg;
	rv = send_curr_msg(info);
	if (rv) {
		msg->done(msg);
		info->curr_msg = NULL;
	} else
		info->state = SERIAL_GETTING_EVENTS;

	return rv;
}

static int start_msg_fetch(struct ipmi_serial_info *info)
{
	struct ipmi_smi_msg *msg;
	int                 rv;

	WARN_ON(info->curr_msg);
	if (info->curr_msg)
		return -EBUSY;

	msg = ipmi_alloc_smi_msg();
	if (!msg)
		return -ENOMEM;

	msg->data[0] = (IPMI_NETFN_APP_REQUEST << 2);
	msg->data[1] = IPMI_GET_MSG_CMD;
	msg->data_size = 2;
	info->curr_msg = msg;
	rv = send_curr_msg(info);
	if (rv) {
		info->curr_msg->done(info->curr_msg);
		info->curr_msg = NULL;
	} else
		info->state = SERIAL_GETTING_MESSAGES;

	return rv;
}

static void handle_flags(struct ipmi_serial_info *info)
{
	int rv;

 retry:
	if (info->msg_flags & WDT_PRE_TIMEOUT_INT) {
		/* Watchdog pre-timeout */
		serial_inc_stat(info, watchdog_pretimeouts);

		info->watchdog_pretimeouts_to_deliver++;
		start_clear_flags(info);
		info->msg_flags &= ~WDT_PRE_TIMEOUT_INT;
	} else if (info->msg_flags & RECEIVE_MSG_AVAIL) {
		/*
		 * Messages available. If for some reason we fail,
		 * just give up for now.  It will be retried.
		 */
		info->msg_flags &= ~RECEIVE_MSG_AVAIL;
		rv = start_msg_fetch(info);
		if (rv)
			goto retry;
	} else if (info->msg_flags & EVENT_MSG_BUFFER_FULL) {
		/*
		 * Messages available. If for some reason we fail,
		 * just give up for now.  It will be retried.
		 */
		info->msg_flags &= ~EVENT_MSG_BUFFER_FULL;
		rv = start_event_fetch(info);
		if (rv)
			goto retry;
	} else if (info->msg_flags & OEM_DATA_AVAIL
				&& info->oem_data_avail_handler) {
		if (info->oem_data_avail_handler(info))
			goto retry;
	} else {
		info->state = SERIAL_EMPTY;
	}
}

/*
 * This routine starts the next thing to be processed.  Called with
 * the lock, and releases the lock.
 *
 * We delay handling of flags and transmissions of new messages here.
 * We also delay the deliver of message to the upper and lower layer
 * in a single-threaded but unlocked section below, so that we don't
 * have worries of deadlocks calling back into this layer.
 */
static void start_next_msg(struct ipmi_serial_info *info, unsigned long *flags)
{
	struct list_head    to_deliver;
	int                 wdog_pretimeout_count;
	int                 rv;
	struct ipmi_smi_msg *msg, *s;
	unsigned long       oflags;

 restart:
	if (debug & IPMI_SER_DEBUG_STATE)
		printk(KERN_DEBUG PFX "start_next_message state = %d\n",
		       info->name, info->line, info->state);

	if (info->state == SERIAL_EMPTY) {
		if (info->msg_flags) {
			/* Handle flags we have first. */
			handle_flags(info);
			if (info->state == SERIAL_EMPTY) {
				info->msg_flags = 0;
				goto restart;
			}
			info->msg_timeout = IPMI_SERIAL_RETRY_TIMEOUT;
			info->retries = IPMI_SERIAL_MAX_ERROR_RETRIES;
		} else if (info->req_events) {
			/* We prefer fetching events over new messages. */
			info->req_events = false;
			rv = start_event_fetch(info);
			if (rv)
				goto restart;
			info->msg_timeout = IPMI_SERIAL_RETRY_TIMEOUT;
			info->retries = IPMI_SERIAL_MAX_ERROR_RETRIES;
		} else if (likely(info->upper_layer_ready) && info->req_flags) {
			/* We prefer fetching flags over new messages. */
			info->req_flags = false;
			rv = start_flag_fetch(info);
			if (rv)
				goto restart;
			info->msg_timeout = IPMI_SERIAL_RETRY_TIMEOUT;
			info->retries = IPMI_SERIAL_MAX_ERROR_RETRIES;
		} else {
			/* Look for messages to transmit. */
			if (!list_empty(&info->hp_xmit_msgs)) {
				msg = list_entry(info->hp_xmit_msgs.next,
						 struct ipmi_smi_msg,
						 link);
			} else if (!list_empty(&info->xmit_msgs)) {
				msg = list_entry(info->xmit_msgs.next,
						 struct ipmi_smi_msg,
						 link);
			} else
				msg = NULL;

			info->curr_msg = msg;
			if (msg) {
				if (debug & IPMI_SER_DEBUG_TIMING) {
					struct timeval t;
					do_gettimeofday(&t);
					printk(KERN_DEBUG PFX
					       "Start send at %ld.%6.6ld\n",
					       info->name, info->line,
					       t.tv_sec, t.tv_usec);
				}
				info->state = SERIAL_HANDLING_MSG;
				list_del(&msg->link);
				rv = send_curr_msg(info);
				if (rv) {
					queue_return_err_msg(info,
						       IPMI_ERR_UNSPECIFIED);
					goto restart;
				}
				info->msg_timeout = IPMI_SERIAL_RETRY_TIMEOUT;
				info->retries = IPMI_SERIAL_MAX_ERROR_RETRIES;
			}
		}
	}

 restart_delivery:
	if (info->watchdog_pretimeouts_to_deliver
	    || !list_empty(&info->msgs_to_deliver)
	    || info->to_send) {
		if (info->msg_delivery_in_progress) {
			/*
			 * Another thread is already delivering, tell
			 * it there are more things to do and
			 * leave.
			 */
			info->msg_delivery_in_progress = 2;
			goto out_unlock;
		}

		/* Pull the data we need to deliver. */
		wdog_pretimeout_count = info->watchdog_pretimeouts_to_deliver;
		info->watchdog_pretimeouts_to_deliver = 0;

		INIT_LIST_HEAD(&to_deliver);
		if (!list_empty(&info->msgs_to_deliver))
			list_splice_init(&info->msgs_to_deliver, &to_deliver);

		info->msg_delivery_in_progress = 1;
		ipmi_serial_unlock_cond(info, flags);
		while (wdog_pretimeout_count > 0) {
			ipmi_smi_watchdog_pretimeout(info->intf);
			wdog_pretimeout_count--;
		}

		list_for_each_entry_safe(msg, s, &to_deliver, link) {
			list_del(&msg->link);
			ipmi_smi_msg_received(info->intf, msg);
		}
		if (info->to_send) {
			if (debug & IPMI_SER_DEBUG_TIMING) {
				struct timeval t;
				do_gettimeofday(&t);
				printk(KERN_DEBUG PFX
				       "send to codec at %ld.%6.6ld\n",
				       info->name, info->line,
				       t.tv_sec, t.tv_usec);
			}
			info->send_seq++;
			if (info->send_seq == 0)
				info->send_seq++;
			msg = info->to_send;
			info->to_send = NULL;
			rv = info->codec->send_msg(info->codec_data,
						   msg->data,
						   msg->data_size,
						   info->send_seq);
			if (rv)
				printk(KERN_WARNING PFX
				       "Error from codec send_msg: %d\n",
				       info->name, info->line, rv);
		}
		flags = ipmi_serial_lock_cond(info, &oflags);
		if (info->msg_delivery_in_progress > 1) {
			/* Another thread put things in to be delivered, go
			 * ahead and do the delivery since it couldn't. */
			info->msg_delivery_in_progress = 0;
			goto restart_delivery;
		}
		info->msg_delivery_in_progress = 0;
	}
 out_unlock:
	ipmi_serial_unlock_cond(info, flags);
}

unsigned int ipmi_serial_ll_xmit(struct ipmi_serial_info *info,
				 const unsigned char     *buf,
				 unsigned int            count)
{
	struct uart_port *port;
	unsigned long    oflags, *flags;
	unsigned int     rv;

	if (debug & IPMI_SER_DEBUG_CHAR_TIMING) {
		struct timeval t;
		do_gettimeofday(&t);
		printk(KERN_DEBUG PFX "xmit chars at %ld.%6.6ld\n",
		       info->name, info->line, t.tv_sec, t.tv_usec);
	}

	flags = ipmi_serial_lock_cond(info, &oflags);
	port = info->port;
	ipmi_serial_unlock_cond(info, flags);
	if (!port)
		return 0;

	rv = uart_direct_write(port, buf, count, flags != NULL);
	if (rv > 0)
		serial_add_stat(info, xmit_chars, rv);

	if (debug & IPMI_SER_DEBUG_DATA) {
		int i;
		printk(KERN_DEBUG PFX "Outgoing data:",
		       info->name, info->line);
		for (i = 0; i < rv; i++)
			printk(" %2.2x(%c)", buf[i],
			       isprint(buf[i]) ? buf[i] : ' ');
		printk("\n");
	}
	return rv;
}
EXPORT_SYMBOL(ipmi_serial_ll_xmit);

void ipmi_serial_ll_attn(struct ipmi_serial_info *info)
{
	unsigned long oflags, *flags;

	if (info->stop_operation)
		return;

	flags = ipmi_serial_lock_cond(info, &oflags);
	serial_inc_stat(info, attentions);
	if (info->port)
		info->req_flags = true;
	start_next_msg(info, flags);
}
EXPORT_SYMBOL(ipmi_serial_ll_attn);

void ipmi_serial_ll_recv(struct ipmi_serial_info *info,
			 const unsigned char     *msg,
			 unsigned int            len,
			 unsigned int            seq)
{
	struct ipmi_smi_msg *rmsg;
	int                 truncated = 0;
	unsigned long       oflags, *flags;

	if (info->internal_msg_handler) {
		info->internal_msg_handler(info, msg, len);
		return;
	}

	flags = ipmi_serial_lock_cond(info, &oflags);
	if (!info->intf) {
		ipmi_serial_unlock_cond(info, flags);
		return;
	}
	if (seq && (seq != info->send_seq)) {
		/* Sequence doesn't match and is non-zero, just ignore. */
		ipmi_serial_unlock_cond(info, flags);
		return;
	}
	if (debug & IPMI_SER_DEBUG_STATE)
		printk(KERN_DEBUG PFX "ll_recv state = %d\n",
		       info->name, info->line, info->state);
	switch (info->state) {
	case SERIAL_EMPTY:
		break;

	case SERIAL_HANDLING_MSG:
		serial_inc_stat(info, complete_transactions);
		rmsg = info->curr_msg;
		if (len > IPMI_MAX_MSG_LENGTH) {
			rmsg->rsp_size = IPMI_MAX_MSG_LENGTH;
			truncated = 1;
		} else
			rmsg->rsp_size = len;
		memcpy(rmsg->rsp, msg, rmsg->rsp_size);
		if (truncated && (msg[2] == 0))
			rmsg->rsp[2] = IPMI_ERR_MSG_TRUNCATED;
		queue_return_msg(info);
		break;

	case SERIAL_GETTING_FLAGS:
		info->curr_msg->done(info->curr_msg);
		info->curr_msg = NULL;
		/* We got the flags from the SMI, now fetch them. */
		if ((msg[2] == 0) && (len >= 4)) {
			serial_inc_stat(info, flag_fetches);
			info->msg_flags = msg[3];
		}
		break;

	case SERIAL_GETTING_EVENTS:
		rmsg = info->curr_msg;
		if (len > IPMI_MAX_MSG_LENGTH) {
			printk(KERN_WARNING PFX "got event response "
			       "that was too long: %d\n", info->name,
			       info->line, len);
			break;
		}
		rmsg->rsp_size = len;
		memcpy(rmsg->rsp, msg, rmsg->rsp_size);
		if (rmsg->rsp[2] != 0) {
			/* Error getting event, probably done. */
			rmsg->done(rmsg);
			info->curr_msg = NULL;
		} else {
			serial_inc_stat(info, events);

			queue_return_msg(info);
		}
		break;

	case SERIAL_CLEARING_FLAGS:
		/* We cleared the flags. */
		info->curr_msg->done(info->curr_msg);
		info->curr_msg = NULL;
		if (msg[2] != 0) {
			/* Error clearing flags */
			printk(KERN_WARNING
			       PFX "Error clearing flags: %2.2x\n",
			       info->name, info->line, msg[2]);
		}
		info->state = SERIAL_EMPTY;
		break;

	case SERIAL_GETTING_MESSAGES:
		rmsg = info->curr_msg;
		if (len > IPMI_MAX_MSG_LENGTH) {
			printk(KERN_WARNING PFX "got incoming command "
			       "that was too long: %d\n", info->name,
			       info->line, len);
			break;
		}
		rmsg->rsp_size = len;
		memcpy(rmsg->rsp, msg, rmsg->rsp_size);
		if (rmsg->rsp[2] != 0) {
			/* Error getting message, probably done. */
			rmsg->done(rmsg);
			info->curr_msg = NULL;
		} else {
			serial_inc_stat(info, incoming_messages);
			queue_return_msg(info);
		}
		break;
	}
	info->state = SERIAL_EMPTY; /* Tell start_next_msg to do something. */
	start_next_msg(info, flags);
}
EXPORT_SYMBOL(ipmi_serial_ll_recv);

void ipmi_serial_ll_async(struct ipmi_serial_info *info,
			  const unsigned char *msg, unsigned int len)
{
	struct ipmi_smi_msg *rmsg;
	unsigned long *flags, oflags;

	if ((msg[0] >> 2) != IPMI_NETFN_APP_RESPONSE) {
		printk(KERN_WARNING PFX "Got invalid async NETFN: 0x%x",
		       info->name, info->line, msg[0] >> 2);
		return;
	}

	flags = ipmi_serial_lock_cond(info, &oflags);
	if (!info->intf) {
		ipmi_serial_unlock_cond(info, flags);
		return;
	}

	/* Validate that the message is allowed and do stats. */
	switch (msg[1]) {
	case IPMI_GET_MSG_CMD:
		serial_inc_stat(info, incoming_messages);
		break;

	case IPMI_READ_EVENT_MSG_BUFFER_CMD:
		serial_inc_stat(info, events);
		break;

	default:
		printk(KERN_WARNING PFX "Got invalid async command: 0x%x",
		       info->name, info->line, msg[1]);
		goto out;
	}

	if (len > IPMI_MAX_MSG_LENGTH) {
		/* FIXME - add peg */
		printk(KERN_WARNING PFX "got incoming async msg "
		       "that was too long: %d\n", info->name, info->line, len);
		goto out;
	}
	rmsg = ipmi_alloc_smi_msg();
	if (!rmsg) {
		/* FIXME - add peg */
		printk(KERN_WARNING PFX "Dropped incoming async msg, "
		       "could not allocate message\n", info->name, info->line);
		goto out;
	}

	/* Make it look like we sent the command. */
	rmsg->data[0] = msg[0] & ~(1 << 2); /* Convert response to request */
	rmsg->data[1] = msg[1]; /* command */
	rmsg->data_size = 2;

	rmsg->rsp_size = len;
	memcpy(rmsg->rsp, msg, rmsg->rsp_size);
	queue_async_msg(info, rmsg);

 out:
	start_next_msg(info, flags);
}
EXPORT_SYMBOL(ipmi_serial_ll_async);

void ipmi_serial_ll_protocol_violation(struct ipmi_serial_info *info)
{
	serial_inc_stat(info, protocol_violations);
}
EXPORT_SYMBOL(ipmi_serial_ll_protocol_violation);

void ipmi_serial_ll_checksum_error(struct ipmi_serial_info *info)
{
	serial_inc_stat(info, checksum_errors);
}
EXPORT_SYMBOL(ipmi_serial_ll_checksum_error);

static void ipmi_serial_handle_char(struct uart_port *port,
				    unsigned int status,
				    unsigned int overrun, unsigned int ch,
				    unsigned int flag)
{
	struct ipmi_serial_info *info = port->state->direct->direct_data;
	unsigned int next;

	if (debug & IPMI_SER_DEBUG_CHAR_TIMING) {
		struct timeval t;
		do_gettimeofday(&t);
		printk(KERN_DEBUG PFX "Got char %2.2x(%c) at %ld.%6.6ld\n",
		       info->name, info->line,
		       ch, isprint(ch) ? ch : ' ',
		       t.tv_sec, t.tv_usec);
	}

	serial_inc_stat(info, recv_chars);

	if (status & overrun)
		serial_inc_stat(info, overruns);

	if (debug & IPMI_SER_DEBUG_DATA)
		printk(KERN_DEBUG PFX "Incoming char: %2.2x '%c'\n",
		       info->name, info->line, ch, isprint(ch) ? ch : ' ');

	/*
	 * No lock is needed.  We are the only place that changes
	 * sbuf_next, and we are single-threaded here due to the port
	 * lock.
	 */
	next = SBUF_NEXT(info->sbuf_next);
	if (next == info->sbuf_start) {
		serial_inc_stat(info, overruns);
		return;
	}
	info->sbuf[info->sbuf_next] = ch;
	smp_wmb();
	info->sbuf_next = next;
}

static void ipmi_serial_push(struct uart_port *port)
{
	struct ipmi_serial_info *info = port->state->direct->direct_data;
	unsigned long oflags, *flags;
	struct ipmi_serial_codec_data *cdata;
	unsigned char ch;

	if (debug & IPMI_SER_DEBUG_CHAR_TIMING) {
		struct timeval t;
		do_gettimeofday(&t);
		printk(KERN_DEBUG PFX "Got push at %ld.%6.6ld\n",
		       info->name, info->line, t.tv_sec, t.tv_usec);
	}

	cdata = info->codec_data;
	if (!cdata)
		return;

	flags = ipmi_serial_lock_cond(info, &oflags);
	if (info->delivering_char)
		goto out_unlock;
	info->delivering_char = 1;
	while (info->sbuf_next != info->sbuf_start) {
		ch = info->sbuf[info->sbuf_start];
		info->sbuf_start = SBUF_NEXT(info->sbuf_start);
		ipmi_serial_unlock_cond(info, flags);
		info->codec->handle_char(cdata, ch);
		flags = ipmi_serial_lock_cond(info, &oflags);
	}
	info->delivering_char = 0;
 out_unlock:
	ipmi_serial_unlock_cond(info, flags);
}

static void ipmi_serial_tx_ready(unsigned long data)
{
	struct ipmi_serial_info *info = (struct ipmi_serial_info *) data;
	struct ipmi_serial_codec_data *cdata;

	cdata = info->codec_data;
	if (cdata)
		info->codec->tx_ready(cdata);
}

static void timeout_handling(struct ipmi_serial_info *info,
			     unsigned int            delay_since_last)
{
	unsigned long oflags, *flags;

	if (info->codec->timer_tick)
		info->codec->timer_tick(info->codec_data, delay_since_last);

	flags = ipmi_serial_lock_cond(info, &oflags);

	info->flag_timer -= delay_since_last;
	if (info->flag_timer <= 0) {
		if (!info->stop_operation && info->supports_flags)
			info->req_flags = true;
		info->flag_timer = info->flag_timeout;
	}

	info->msg_timeout -= delay_since_last;
	if (info->msg_timeout > 0)
		/* Haven't timed out yet. */
		goto out_unlock;

	serial_inc_stat(info, timeouts);

	/*
	 * Note that if we are doing flags, events, or things of that
	 * nature, just ignore failures and continue on.  The
	 * operation will be retried later, so no need to fret.
	 * Just retry messages.
	 */
	if (debug & IPMI_SER_DEBUG_STATE)
		printk(KERN_DEBUG PFX "timeout state = %d\n",
		       info->name, info->line, info->state);
	switch (info->state) {
	case SERIAL_EMPTY:
		break;

	case SERIAL_HANDLING_MSG:
		info->retries--;
		if (info->retries > 0)
			send_curr_msg(info); /* Retry the send */
		else {
			queue_return_err_msg(info, IPMI_TIMEOUT_ERR);
			info->state = SERIAL_EMPTY;
		}
		break;

	case SERIAL_GETTING_FLAGS:
	case SERIAL_CLEARING_FLAGS:
		info->curr_msg->done(info->curr_msg);
		info->curr_msg = NULL;
		info->state = SERIAL_EMPTY;
		 /*
		  * Don't bother with flags again if they have been
		  * re-requested, so we can handle some messages.
		  */
		info->req_flags = false;
		break;

	case SERIAL_GETTING_EVENTS:
		info->curr_msg->done(info->curr_msg);
		info->curr_msg = NULL;
		info->state = SERIAL_EMPTY;
		 /*
		  * Don't bother with events again if they have been
		  * re-requested, so we can handle some messages.
		  */
		info->req_events = false;
		break;

	case SERIAL_GETTING_MESSAGES:
		info->curr_msg->done(info->curr_msg);
		info->curr_msg = NULL;
		info->state = SERIAL_EMPTY;
		break;
	}

 out_unlock:
	start_next_msg(info, flags);
}

static void ipmi_serial_timeout(unsigned long data)
{
	struct ipmi_serial_info *info = (struct ipmi_serial_info *) data;
	unsigned long curr_jif = jiffies;

	serial_inc_stat(info, timer_ticks);

	timeout_handling(info, ((curr_jif - info->last_timeout_jiffies)
				* IPMI_SERIAL_USEC_PER_JIFFY));

	info->last_timeout_jiffies = curr_jif;
	mod_timer(&info->timer, curr_jif + IPMI_SERIAL_TIMEOUT_JIFFIES);
}

static int ipmi_serial_start_processing(void *send_info, ipmi_smi_t intf)
{
	struct ipmi_serial_info *info = send_info;
	unsigned long curr_jif = jiffies;

	info->intf = intf;
	smp_wmb(); /* Make sure intf is set before anything else. */

	info->upper_layer_ready = true;
	info->last_timeout_jiffies = curr_jif;
	mod_timer(&info->timer, curr_jif + IPMI_SERIAL_TIMEOUT_JIFFIES);
	return 0;
}

static void ipmi_serial_poll(void *send_info)
{
	struct ipmi_serial_info *info = send_info;
	struct uart_port *port;
	struct circ_buf *circ;

	port = info->port;
	if (!port)
		return;

	/*
	 * No need for locks here, that would result in a deadlock and
	 * the locks are unnecessary.
	 */
	udelay(10);
	timeout_handling(info, 10);
	if (info->run_to_completion) {
		port->ops->poll(port, UART_POLL_FLAGS_TX | UART_POLL_FLAGS_RX);
	} else {
		unsigned long flags;
		spin_lock_irqsave(&port->lock, flags);
		port->ops->poll_startup(port, &info->poll_state);
		port->ops->poll(port, UART_POLL_FLAGS_TX | UART_POLL_FLAGS_RX);
		port->ops->poll_shutdown(port, info->poll_state);
		spin_unlock_irqrestore(&port->lock, flags);
	}
	if (info->sbuf_next != info->sbuf_start)
		ipmi_serial_push(port);
	circ = uart_get_circ_buf(port);
	if (uart_circ_chars_free(circ) > 0)
		ipmi_serial_tx_ready((unsigned long) info);
}

static void ipmi_serial_sender(void                *send_info,
			       struct ipmi_smi_msg *msg,
			       int                 priority)
{
	struct ipmi_serial_info *info = send_info;
	unsigned long           oflags, *flags;

	flags = ipmi_serial_lock_cond(info, &oflags);
	if (!info->port) {
		info->curr_msg = msg;
		queue_return_err_msg(info, IPMI_ERR_UNSPECIFIED);
	} else if (priority > 0)
		list_add_tail(&msg->link, &info->hp_xmit_msgs);
	else
		list_add_tail(&msg->link, &info->xmit_msgs);
	start_next_msg(info, flags);

	if (info->run_to_completion) {
		while (info->curr_msg)
			ipmi_serial_poll(info);
	}
}

static void ipmi_serial_request_events(void *send_info)
{
	struct ipmi_serial_info *info = send_info;
	unsigned long           oflags, *flags;

	flags = ipmi_serial_lock_cond(info, &oflags);
	if (!info->stop_operation && info->port && info->do_event_request
				&& info->has_event_buffer)
		info->req_events = true;
	start_next_msg(info, flags);
}

static void ipmi_serial_set_run_to_completion(void *send_info, bool rtc_on)
{
	struct ipmi_serial_info *info = send_info;
	struct uart_port *port;

	if (info->run_to_completion == rtc_on)
		return;

	port = info->port;
	if (!port)
		return;

	info->run_to_completion = rtc_on;
	if (rtc_on) {
		port->ops->poll_startup(port, &info->poll_state);
		while (info->curr_msg)
			ipmi_serial_poll(info);
	} else
		port->ops->poll_shutdown(port, info->poll_state);
}

static int ipmi_serial_inc_usecount(void *send_info)
{
	struct ipmi_serial_info *info = send_info;

	if (!try_module_get(info->codec->owner))
		return -ENODEV;
	return 0;
}

static void ipmi_serial_dec_usecount(void *send_info)
{
	struct ipmi_serial_info *info = send_info;

	module_put(info->codec->owner);
}

static struct ipmi_smi_handlers handlers = {
	.owner                  = THIS_MODULE,
	.start_processing       = ipmi_serial_start_processing,
	.sender			= ipmi_serial_sender,
	.request_events		= ipmi_serial_request_events,
	.set_run_to_completion  = ipmi_serial_set_run_to_completion,
	.poll			= ipmi_serial_poll,
	.inc_usecount		= ipmi_serial_inc_usecount,
	.dec_usecount		= ipmi_serial_dec_usecount
};

static int smi_type_proc_show(struct seq_file *m, void *v)
{
	return seq_printf(m, "serial\n");
}

static int smi_type_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, smi_type_proc_show, inode->i_private);
}

static const struct file_operations smi_type_proc_ops = {
	.open		= smi_type_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int smi_param_proc_show(struct seq_file *m, void *data)
{
	struct ipmi_serial_info *info = data;
	struct ktermios         *t = &info->termios;
	int                     i;
	char                    parity, bits, stop, opts[4];

	/* Translate from termios and mctrl. */
	for (i = 0; baud_rates[i].rate; i++) {
		if ((t->c_cflag & CBAUD) == baud_rates[i].cflag)
			break;
	}
	if (!(t->c_cflag & PARENB))
		parity = 'n';
	else if (t->c_cflag & PARODD)
		parity = 'o';
	else
		parity = 'e';
	switch (t->c_cflag & CSIZE) {
	case CS5:
		bits = '5';
		break;
	case CS6:
		bits = '6';
		break;
	case CS7:
		bits = '7';
		break;
	case CS8:
		bits = '8';
		break;
	default:
		bits = '?';
	}
	if (t->c_cflag & CSTOPB)
		stop = '2';
	else
		stop = '1';
	opts[0] = '\0';
	if (t->c_cflag & CRTSCTS)
		strcat(opts, "r");
	if (!(info->port->mctrl & TIOCM_RTS))
		strcat(opts, "R");
	if (!(info->port->mctrl & TIOCM_DTR))
		strcat(opts, "D");

	seq_printf(m, "%s%d,%s%c%c%c%s,%s",
		   info->name, info->line,
		   baud_rates[i].name, parity, bits, stop, opts,
		   info->codec->name);
	info->codec->add_options(info->codec_data, m);
	seq_printf(m, "\n");
	return 0;
}

static int smi_param_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, smi_param_proc_show, inode->i_private);
}

static const struct file_operations smi_param_proc_ops = {
	.open		= smi_param_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int smi_stat_proc_show(struct seq_file *m, void *data)
{
	struct ipmi_serial_info *info = data;

	seq_printf(m, "timer_ticks:           %u\n",
		   serial_get_stat(info, timer_ticks));
	seq_printf(m, "attentions:            %u\n",
		   serial_get_stat(info, attentions));
	seq_printf(m, "flag_fetches:          %u\n",
		   serial_get_stat(info, flag_fetches));
	seq_printf(m, "complete_transactions: %u\n",
		   serial_get_stat(info, complete_transactions));
	seq_printf(m, "events:                %u\n",
		   serial_get_stat(info, events));
	seq_printf(m, "watchdog_pretimeouts:  %u\n",
		   serial_get_stat(info, watchdog_pretimeouts));
	seq_printf(m, "xmit_chars:            %u\n",
		   serial_get_stat(info, xmit_chars));
	seq_printf(m, "recv_chars:            %u\n",
		   serial_get_stat(info, recv_chars));
	seq_printf(m, "protocol_violations:   %u\n",
		   serial_get_stat(info, protocol_violations));
	seq_printf(m, "checksum_errors:       %u\n",
		   serial_get_stat(info, checksum_errors));
	seq_printf(m, "timeouts:              %u\n",
		   serial_get_stat(info, timeouts));
	seq_printf(m, "overruns:              %u\n",
		   serial_get_stat(info, overruns));
	return 0;
}

static int smi_stat_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, smi_stat_proc_show, inode->i_private);
}

static const struct file_operations smi_stat_proc_ops = {
	.open		= smi_stat_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void devid_handler(struct ipmi_serial_info *info,
			  const unsigned char     *data,
			  unsigned int            len)
{
	int rv;

	rv = ipmi_demangle_device_id(data, len, &info->device_id);
	if (rv) {
		info->device_id_valid = rv;
		return;
	}

	info->device_id_valid = 1;

	if (info->codec->check_dev_id)
		info->codec->check_dev_id(info->codec_data, &info->device_id);
}

static int try_get_dev_id(struct ipmi_serial_info *info)
{
	unsigned char msg[2];
	int           rv;
	int           timeout;

	msg[0] = IPMI_NETFN_APP_REQUEST << 2;
	msg[1] = IPMI_GET_DEVICE_ID_CMD;

	info->device_id_valid = 0;
	info->internal_msg_handler = devid_handler;
	rv = info->codec->send_msg(info->codec_data, msg, 2, 0);
	if (rv)
		goto out_err;

	timeout = IPMI_SERIAL_RETRY_TIMEOUT;
	while (!info->device_id_valid && (timeout > 0)) {
		msleep(10);
		timeout -= 10000;
	}
	if (!info->device_id_valid)
		rv = -ETIMEDOUT;
	else if (info->device_id_valid < 0)
		rv = info->device_id_valid;

 out_err:
	info->internal_msg_handler = NULL;
	return rv;
}

static void set_global_enable_handler(struct ipmi_serial_info *info,
				      const unsigned char     *data,
				      unsigned int            len)
{
	if (len < 3 ||
			data[0] != (IPMI_NETFN_APP_REQUEST | 1) << 2 ||
			data[1] != IPMI_SET_BMC_GLOBAL_ENABLES_CMD) {
		printk(KERN_WARNING PFX
		       "Invalid return from set global "
		       " enables command, cannot enable the event"
		       " buffer.\n", info->name, info->line);
		info->global_enable_valid = -EINVAL;
		return;
	}

	if (data[2] != 0)
		info->global_enable_valid = 1;
	else
		info->global_enable_valid = -ENOENT;
}

static void get_global_enable_handler(struct ipmi_serial_info *info,
				      const unsigned char     *data,
				      unsigned int            len)
{
	unsigned char msg[3];
	int rv;

	if (len < 4 ||
			data[0] != (IPMI_NETFN_APP_REQUEST | 1) << 2 ||
			data[1] != IPMI_GET_BMC_GLOBAL_ENABLES_CMD   ||
			data[2] != 0) {
		printk(KERN_WARNING PFX
		       "Invalid return from get global"
		       " enables command, cannot enable the event"
		       " buffer.\n", info->name, info->line);
		info->global_enable_valid = -EINVAL;
		return;
	}

	if (data[3] & IPMI_BMC_EVT_MSG_BUFF) {
		/* Nothing to do, it's enabled. */
		info->global_enable_valid = 1;
		return;
	}

	msg[0] = IPMI_NETFN_APP_REQUEST << 2;
	msg[1] = IPMI_SET_BMC_GLOBAL_ENABLES_CMD;
	msg[2] = data[3] | IPMI_BMC_EVT_MSG_BUFF;
	info->internal_msg_handler = set_global_enable_handler;
	rv = info->codec->send_msg(info->codec_data, msg, 3, 0);
	if (rv)
		info->global_enable_valid = rv;
}

static int try_enable_event_buffer(struct ipmi_serial_info *info)
{
	unsigned char msg[3];
	int           timeout;
	int           rv = 0;

	msg[0] = IPMI_NETFN_APP_REQUEST << 2;
	msg[1] = IPMI_GET_BMC_GLOBAL_ENABLES_CMD;
	info->global_enable_valid = 0;
	info->internal_msg_handler = get_global_enable_handler;
	rv = info->codec->send_msg(info->codec_data, msg, 2, 0);
	if (rv)
		goto out_err;

	timeout = IPMI_SERIAL_RETRY_TIMEOUT;
	while (!info->global_enable_valid && (timeout > 0)) {
		msleep(10);
		timeout -= 10000;
	}
	if (!info->global_enable_valid)
		rv = -ETIMEDOUT;
	else if (info->global_enable_valid < 0)
		rv = info->global_enable_valid;

 out_err:
	info->internal_msg_handler = NULL;
	return rv;
}

void ipmi_serial_ll_init_complete(struct ipmi_serial_info *info,
				  unsigned char slave_addr,
				  int err)
{
	if (err)
		info->init_status = err;
	else {
		if (slave_addr)
			info->slave_addr = slave_addr;
		info->init_status = 1;
	}
}
EXPORT_SYMBOL(ipmi_serial_ll_init_complete);

static int setup_termios(struct ipmi_serial_info *info)
{
	char *o = info->options;
	int baud = 0;
	int i;
	unsigned int mctrl = 0;
	unsigned long flags;
	int rv;

	/* Init some defaults. */
	info->termios.c_cflag = CLOCAL | CREAD;
	info->termios.c_iflag = IGNPAR;
	info->termios.c_oflag = 0;
	info->termios.c_lflag = 0;
	info->termios.c_cc[VTIME] = 0; /* inter-character timer used */

	rv = info->codec->setup_termios(&info->termios);
	if (rv)
		return rv;

	if (o && *o) {
		char *end;
		baud = simple_strtoul(o, &end, 10);
		if (end == o) {
			printk(KERN_ERR PFX "no baud rate given\n",
			       info->name, info->line);
			return -EINVAL;
		}
		o = end;
	}
	for (i = 0; baud_rates[i].rate != 0; i++) {
		if (baud_rates[i].rate == baud)
			break;
	}
	if (baud_rates[i].rate == 0) {
		printk(KERN_ERR PFX "invalid baud rate\n",
		       info->name, info->line);
		return -EINVAL;
	}

	info->termios.c_cflag |= baud_rates[i].cflag;

	if (o && *o) {
		switch (*o) {
		case 'o': case 'O':
			info->termios.c_cflag |= PARODD;
			/*fall through*/
		case 'e': case 'E':
			info->termios.c_cflag |= PARENB;
			break;
		case 'n': case 'N':
			break;
		default:
			printk(KERN_ERR PFX "Invalid parity: '%c'\n",
			       info->name, info->line, *o);
			return -EINVAL;
		}
		o++;
	}
	if (o && *o) {
		switch (*o) {
		case '5':
			info->termios.c_cflag |= CS5;
			break;
		case '6':
			info->termios.c_cflag |= CS6;
			break;
		case '7':
			info->termios.c_cflag |= CS7;
			break;
		case '8':
			info->termios.c_cflag |= CS8;
			break;
		default:
			info->termios.c_cflag |= CS8;
			printk(KERN_ERR PFX "Invalid bits: '%c'\n",
			       info->name, info->line, *o);
			return -EINVAL;
		}
		o++;
	} else {
		info->termios.c_cflag |= CS8;
	}

	if (o && *o) {
		switch (*o) {
		case '1':
			break;
		case '2':
			info->termios.c_cflag |= CSTOPB;
			break;
		default:
			printk(KERN_ERR PFX "Invalid stop bits: '%c'\n",
			       info->name, info->line, *o);
			return -EINVAL;
		}
		o++;
	}

	/*
	 * Note that unless overridden always enable DTR and sending
	 * RTS to the device.  The device can ignore it, but it can't
	 * hurt to send it.  It can be overridden with options.
	 */
	mctrl |= TIOCM_RTS | TIOCM_DTR;

	while (o && *o) {
		switch (*o) {
		case 'r':
			info->termios.c_cflag |= CRTSCTS;
			break;
		case 'R':
			mctrl &= ~TIOCM_RTS;
			break;
		case 'D':
			mctrl &= ~TIOCM_DTR;
			break;
		default:
			printk(KERN_ERR PFX "Invalid config option: '%c'\n",
			       info->name, info->line, *o);
			return -EINVAL;
		}
		o++;
	}

	spin_lock_irqsave(&info->port->lock, flags);
	info->port->mctrl |= mctrl;
	spin_unlock_irqrestore(&info->port->lock, flags);
	info->port->ops->set_termios(info->port, &info->termios,
				     &info->old_termios);
	return 0;
}

/*
 * Called when the serial layer says it found a serial driver that
 * matches.  We use this to kick things off.
 */
static int ipmi_serial_found(struct ipmi_serial_info *info)
{
	int                     rv;
	int                     timeout;
	int                     retries;
	unsigned int            capabilities;
	char		        sysfs_name[sizeof(info->name) + 4];

	printk(KERN_INFO PFX "Found a matching serial port\n",
	       info->name, info->line);

	info->stop_operation = false;

	setup_termios(info);

	info->init_status = 0;
	retries = IPMI_SERIAL_INIT_RETRIES;
 retry:
	rv = info->codec->init(info->alloc_codec_data, info,
			       info->codec_options);
	if (rv) {
		printk(KERN_ERR PFX "codec initialization failed, "
		       "interface is not usable: %d\n", info->name,
		       info->line, rv);
		return rv;
	}

	/* Allow the receiver to send to the codec. */
	info->codec_data = info->alloc_codec_data;

	if (info->codec->start) {
		rv = info->codec->start(info->codec_data);
		if (rv) {
			printk(KERN_ERR PFX "codec start failed, "
			       "interface is not usable: %d\n", info->name,
			       info->line, rv);
			goto out_err;
		}
	} else
		ipmi_serial_ll_init_complete(info, 0, 0);

	timeout = IPMI_SERIAL_INIT_TIMEOUT;
	while (!info->init_status && (timeout > 0)) {
		msleep(10);
		timeout -= 10000;
	}
	if (!info->init_status) {
		retries--;
		if (retries > 0) {
			/* Turn the receiver off and let it clear out. */
			info->codec_data = NULL;
			synchronize_sched();
			info->codec->cleanup(info->alloc_codec_data);
			goto retry;
		}

		printk(KERN_ERR PFX "codec initialization timed out, "
		       "interface is not usable\n", info->name, info->line);
		rv = -ETIMEDOUT;
		goto out_err;
	} else if (info->init_status < 0) {
		rv = info->init_status;
		printk(KERN_ERR PFX "Initialization failed: %d\n",
		       info->name, info->line, rv);
		goto out_err;
	}

	retries = IPMI_SERIAL_MAX_ERROR_RETRIES;
 retry_devid:
	rv = try_get_dev_id(info);
	if (rv) {
		retries--;
		if (retries > 0)
			goto retry_devid;
		printk(KERN_ERR PFX "Device ID fetch failed, "
		       "interface is not usable: %d\n", info->name,
		       info->line, rv);
		goto out_err;
	}

	capabilities = info->codec->capabilities(info->codec_data);
	info->do_event_request = (capabilities
				  & IPMI_SERIAL_SUPPORTS_EVENT_BUFFER);
	if (capabilities & IPMI_SERIAL_NEEDS_GET_FLAGS_POLLING)
		info->flag_timeout = POLLING_FLAG_TIMEOUT;
	else
		info->flag_timeout = NO_POLLING_FLAG_TIMEOUT;
	info->flag_timer = info->flag_timeout;
	info->supports_flags = (capabilities
				& IPMI_SERIAL_SUPPORTS_GET_FLAGS);

	if (info->do_event_request) {
		if (try_enable_event_buffer(info) == 0)
			info->has_event_buffer = true;
	}


	snprintf(sysfs_name, sizeof(sysfs_name), "bmc:%s%d", info->name,
		 info->line);

	rv = ipmi_register_smi(&handlers,
			       info,
			       &info->device_id,
			       info->port->dev,
			       sysfs_name,
			       info->slave_addr);
	if (rv) {
		printk(KERN_ERR PFX "Unable to register the "
		       "interface with the IPMI message handler: %d\n",
		       info->name, info->line, rv);
		if (info->intf)
			del_timer_sync(&info->timer);
		info->intf = NULL;
		goto out_err_stop_timer;
	}

	rv = ipmi_smi_add_proc_entry(info->intf, "type",
				     &smi_type_proc_ops,
				     info);
	if (rv) {
		printk(KERN_ERR PFX
		       "Unable to create proc entry: %d\n", info->name,
		       info->line, rv);
		goto out_err_stop_timer;
	}

	rv = ipmi_smi_add_proc_entry(info->intf, "params",
				     &smi_param_proc_ops,
				     info);
	if (rv) {
		printk(KERN_ERR PFX
		       "Unable to create proc entry: %d\n", info->name,
		       info->line, rv);
		goto out_err_stop_timer;
	}

	rv = ipmi_smi_add_proc_entry(info->intf, "serial_stats",
				     &smi_stat_proc_ops,
				     info);
	if (rv) {
		printk(KERN_ERR PFX
		       "Unable to create proc entry: %d\n", info->name,
		       info->line, rv);
		goto out_err_stop_timer;
	}

	return 0;

 out_err_stop_timer:
	if (info->intf) {
		del_timer_sync(&info->timer);
		ipmi_unregister_smi(info->intf);
		info->intf = NULL;
	}
 out_err:
	/* Turn the receiver off and let it clear out. */
	info->codec_data = NULL;
	synchronize_sched();

	info->codec->cleanup(info->alloc_codec_data);
	return rv;
}

static void setup_intf(struct ipmi_serial_info *info,
		       struct ipmi_serial_codec *codec)
{
	int rv;

	info->alloc_codec_data = kmalloc(codec->size(), GFP_KERNEL);
	if (!info->alloc_codec_data) {
		printk(KERN_ERR PFX "Unable to allocate codec data\n",
		       info->name, info->line);
		return;
	}

	info->codec = codec;

	/*
	 * Tell the serial layer that we want to take over a specific
	 * serial interface.
	 */
	info->port = uart_get_direct_port(info->name, info->line);
	if (!info->port) {
		printk(KERN_ERR PFX "Unable to find serial port\n",
		       info->name, info->line);
		return;
	}
	info->port->state->direct = &info->direct;
	info->port->state->xmit.buf = info->uart_buffer;
	uart_circ_clear(&info->port->state->xmit);
	info->port->state->usflags |= UART_STATE_BOOT_ALLOCATED;

	rv = info->port->ops->startup(info->port);
	if (rv) {
		printk(KERN_ERR PFX "Unable setup serial port %d\n",
		       info->name, info->line, rv);
		goto err_out;
	}

	if (ipmi_serial_found(info))
		goto err_shutdown;

	return;

err_shutdown:
	info->port->ops->shutdown(info->port);
err_out:
	info->port->state->xmit.buf = NULL;
	info->port->state->usflags &= ~UART_STATE_BOOT_ALLOCATED;
	uart_put_direct_port(info->port);
	info->port = NULL;
}

static void free_info_memory(struct ipmi_serial_info *info)
{
	kfree(info->keepstr);
	kfree(info);
}

static int name_line_from_str(const char *str, char *name, int len)
{
	if (str[0] >= '0' && str[0] <= '9') {
		/* We convert a number to ttyS# */
		strncpy(name, "ttyS", len);
		return simple_strtoul(str, NULL, 10);
	} else {
		int size;
		const char *s;
		s = str;
		while (*s && *s != ',' && (*s < '0' || *s > '9'))
			s++;
		size = s - str;
		if (size >= len)
			size = len - 1;
		memcpy(name, str, size);
		name[size] = '\0';
		return simple_strtoul(s, NULL, 10);
	}
}

/*
 *	Setup a list of ipmi serial system interfaces.
 */
static void ipmi_serial_setup_one(const char *istr)
{
	struct ipmi_serial_codec *codec;
	struct ipmi_serial_info  *info;
	struct ipmi_serial_info  *cinfo;
	char *str;
	int i;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		printk(KERN_ERR NPFX "unable to allocate serial info: %s\n",
		       istr);
		return;
	}
	info->keepstr = kstrdup(istr, GFP_KERNEL);
	if (!info->keepstr) {
		printk(KERN_ERR NPFX "unable to allocate serial string: %s\n",
		       istr);
		kfree(info);
		return;
	}
	str = info->keepstr;

	spin_lock_init(&info->lock);
	setup_timer(&info->timer, ipmi_serial_timeout, (long) info);
	INIT_LIST_HEAD(&info->xmit_msgs);
	INIT_LIST_HEAD(&info->hp_xmit_msgs);
	INIT_LIST_HEAD(&info->msgs_to_deliver);
	info->slave_addr = 0x20; /* default */

	for (i = 0; i < SERIAL_NUM_STATS; i++)
		atomic_set(&info->stats[i], 0);

	/*
	 * Decode str into name, options, codec, codec options.
	 */
	if (strlen(str) == 0) {
		printk(KERN_ERR NPFX "Empty serial port name specified\n");
		free_info_memory(info);
		return;
	}

	info->line = name_line_from_str(str, info->name, sizeof(info->name));

	/* First ',' points to serial port options */
	info->options = strchr(str, ',');
	if (info->options) {
		*(info->options++) = '\0';

		/* Second ',' points to the codec name. */
		info->codec_name = strchr(info->options, ',');
	}

	if (info->codec_name) {
		*(info->codec_name++) = '\0';

		/* Third ',' points to the codec options. */
		info->codec_options = strchr(info->codec_name, ',');
	}

	if (info->codec_options)
		*(info->codec_options++) = 0;

#ifdef __sparc__
	if (!strcmp(info->name, "ttya")) {
		strcpy(info->name, "ttyS");
		info->line = 0;
	}
	if (!strcmp(info->name, "ttyb")) {
		strcpy(info->name, "ttyS");
		info->line = 1;
	}
#endif

	info->direct.direct_data = info;
	info->direct.handle_char = ipmi_serial_handle_char;
	info->direct.push = ipmi_serial_push;

	mutex_lock(&list_lock);
	/* Check for dups. */
	list_for_each_entry(cinfo, &info_list, link) {
		if ((strcmp(cinfo->name, info->name) == 0) &&
					(cinfo->line == info->line)) {
			printk(KERN_ERR NPFX "Duplicate port given: %s\n",
			       str);
			free_info_memory(info);
			goto out_unlock;
		}
	}
	list_add(&info->link, &info_list);

	/* Try to match up this new interface with any registered codecs. */
	list_for_each_entry(codec, &codec_list, link) {
		if (strcmp(codec->name, info->codec_name) == 0) {
			setup_intf(info, codec);
			break;
		}
	}
 out_unlock:
	mutex_unlock(&list_lock);
}

static void ipmi_serial_remove_one(const char *str)
{
	struct ipmi_serial_info *info;
	char                    *s;
	int line;
	char name[TTY_NAME_LEN];

	line = name_line_from_str(str, name, sizeof(name));

	s = strchr(str, ',');

	mutex_lock(&list_lock);
	list_for_each_entry(info, &info_list, link) {
		if ((strcmp(name, info->name) == 0) && (info->line == line)) {
			ipmi_serial_cleanup_one(info);
			list_del(&info->link);
			free_info_memory(info);
			goto out_unlock;
		}
	}
	printk(KERN_ERR NPFX "Could not find port to remove: %s.\n", str);
 out_unlock:
	mutex_unlock(&list_lock);
}

static int hotmod_handler(const char *istr, struct kernel_param *kp)
{
	char *str = kstrdup(istr, GFP_KERNEL);
	char *s;
	int  rv = -EINVAL;
	int  len, i;
	char *next, *curr;

	if (!str)
		return -ENOMEM;

	/* Kill any trailing spaces, as we can get a "\n" from echo. */
	len = strlen(str);
	i = len - 1;
	while ((i >= 0) && isspace(str[i])) {
		str[i] = '\0';
		i--;
	}

	for (curr = str; curr; curr = next) {
		next = strchr(curr, ':');
		if (next) {
			*next = '\0';
			next++;
		}

		s = strchr(curr, ',');
		if (!s) {
			printk(KERN_WARNING NPFX
			       "No hotmod operation given.\n");
			break;
		}
		*s = '\0';
		s++;

		if (strcmp(curr, "add") == 0) {
			ipmi_serial_setup_one(s);
			rv = len;
		} else if (strcmp(curr, "remove") == 0) {
			ipmi_serial_remove_one(s);
			rv = len;
		} else {
			printk(KERN_WARNING NPFX
			       "Invalid hotmod operation given: '%s'.\n",
			       curr);
			break;
		}
	}
	kfree(str);

	return rv;
}

static int __init init_ipmi_serial(void)
{
	int count = 0;
	char *next, *curr, *str;

	printk(KERN_INFO "IPMI Serial System Interface driver\n");

	if (setup_str[0] == '\0') {
		if (unload_when_empty) {
			printk(KERN_WARNING NPFX "no interfaces specified\n");
			return -ENODEV;
		}
		return 0;
	}

	curr = setup_str;
	while (curr) {
		next = strchr(curr, ':');
		if (!next) {
			str = kstrdup(curr, GFP_KERNEL);
		} else {
			/* Duplicate up to (but not including) the ':'. */
			str = kstrndup(curr, next - curr, GFP_KERNEL);
			next++;
		}
		if (!str) {
			printk(KERN_ERR NPFX
			       "could not allocate string setup: %s\n",
			       curr);
			if (count == 0)
				return -ENOMEM;
			else
				/* We configured one, so don't error. */
				return 0;
		} else {
			count++;
			ipmi_serial_setup_one(str);
			kfree(str);
		}
		curr = next;
	}

	return 0;
}
/*
 * We have to initialize after the serial core has initialized because
 * we needs its sysfs entries initialized.  So delay initialization to
 * the end.
 */
late_initcall(init_ipmi_serial);

static void __exit cleanup_ipmi_serial(void)
{
	struct ipmi_serial_info  *info, *s;

	list_for_each_entry_safe(info, s, &info_list, link) {
		list_del(&info->link);
		free_info_memory(info);
	}
}
module_exit(cleanup_ipmi_serial);

static void ipmi_serial_cleanup_one(struct ipmi_serial_info *to_clean)
{
	int           rv;
	unsigned long flags;
	ipmi_smi_t    intf;

	if (!to_clean)
		return;

	intf = to_clean->intf;

	if (intf) {
		to_clean->stop_operation = true; /* Don't start anything new. */

		del_timer_sync(&to_clean->timer);

		/*
		 * Interrupts and timeouts are stopped, now flush out
		 * all the messages.
		 */
		spin_lock_irqsave(&to_clean->lock, flags);
		while ((to_clean->state != SERIAL_EMPTY)
				|| to_clean->msg_delivery_in_progress) {
			spin_unlock_irqrestore(&to_clean->lock, flags);
			msleep(10);
			timeout_handling(to_clean, 10000);
			spin_lock_irqsave(&to_clean->lock, flags);
		}
		to_clean->intf = NULL; /* No more messages allowed. */
		spin_unlock_irqrestore(&to_clean->lock, flags);

		/*
		 * At this point we won't send any messages up, so we can
		 * unregister the SMI.
		 */
		rv = ipmi_unregister_smi(intf);
		if (rv) {
			printk(KERN_ERR PFX
			       "Unable to unregister device: errno=%d\n",
			       to_clean->name, to_clean->line, rv);
		}
	}

	if (to_clean->codec_data) {
		to_clean->codec_data = NULL;
		synchronize_sched();
		to_clean->codec->cleanup(to_clean->alloc_codec_data);
	}

	if (to_clean->port) {
		to_clean->port->state->direct = NULL;
		to_clean->port->ops->shutdown(to_clean->port);
		if (to_clean->port->state->xmit.buf == to_clean->uart_buffer) {
			to_clean->port->state->xmit.buf = NULL;
			to_clean->port->state->usflags &=
					~UART_STATE_BOOT_ALLOCATED;
		}
		uart_put_direct_port(to_clean->port);
	}
}

int ipmi_serial_codec_register(struct ipmi_serial_codec *codec)
{
	struct ipmi_serial_codec *c;
	struct ipmi_serial_info  *info;

	printk(KERN_INFO NPFX "Registering %s codec\n", codec->name);

	mutex_lock(&list_lock);
	/* Check for dups. */
	list_for_each_entry(c, &codec_list, link) {
		if (strcmp(c->name, codec->name) == 0) {
			printk(KERN_WARNING NPFX
			       "Registering duplicate codec: %s\n",
			       codec->name);
			mutex_unlock(&list_lock);
			return -EBUSY;
		}
	}
	list_add(&codec->link, &codec_list);

	list_for_each_entry(info, &info_list, link) {
		if (strcmp(info->codec_name, codec->name) == 0)
			setup_intf(info, codec);
	}
	mutex_unlock(&list_lock);
	return 0;
}
EXPORT_SYMBOL(ipmi_serial_codec_register);

void ipmi_serial_codec_unregister(struct ipmi_serial_codec *codec)
{
	struct ipmi_serial_info *info;

	/*
	 * Guaranteed to be no users on any devices on this codec, it
	 * can't unregister unless it's module is being unloaded and
	 * the module's refcount is zero.
	 */

	printk(KERN_INFO NPFX "Unregistering %s codec\n", codec->name);

	mutex_lock(&list_lock);
	list_del(&codec->link);
	list_for_each_entry(info, &info_list, link) {
		if (info->codec == codec)
			/* Does not remove the entry from the list. */
			ipmi_serial_cleanup_one(info);
	}
	mutex_unlock(&list_lock);
}
EXPORT_SYMBOL(ipmi_serial_codec_unregister);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Corey Minyard <minyard@mvista.com>");
MODULE_DESCRIPTION("Support for IPMI over serial.");
