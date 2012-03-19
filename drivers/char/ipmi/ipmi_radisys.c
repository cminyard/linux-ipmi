/*
 * ipmi_radisys.c
 *
 * Radisys emulation for the IPMI interface.
 *
 * Author: MontaVista Software, Inc.
 *         Corey Minyard <minyard@mvista.com>
 *         source@mvista.com
 *
 * Copyright 2002 MontaVista Software Inc.
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
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/ipmi.h>
#include <linux/ipmi_radisys.h>
#include <linux/init.h>

#define IPMI_RADISYS_VERSION   "v34"

struct priv_data
{
	/* This is for supporting the old Radisys interface. */
	ipmi_user_t       rs_user;
	spinlock_t        rs_lock;

	/* A list of responses in the queue. */
	struct list_head  rs_waiting_rsps;

	/* A list of things waiting for responses.  We wake them all up
	   when a response comes in. */
	wait_queue_head_t rs_waiting_rsp_rcvrs;

	/* A list of commands that have come in. */
	struct list_head  rs_waiting_cmds;

	/* A list of thing waiting for commands.  We wake them all up
	   when a command comes in. */
	wait_queue_head_t rs_waiting_cmd_rcvrs;

	/* The registered command receiver value.  This only allows someone
	   with the "magic number" to issue commands. */
	unsigned long     rs_cmd_receiver;

	/* Is someone already waiting for a command?  The Radisys driver
	   only allows one waiter, this enforces that. */
	int               rs_cmd_waiting;

	/* A list of IPMI events waiting to be delivered.  (not that
           the Radisys driver calls incoming commands "events", this
           variable is actual IPMI events, not incoming commands). */
	struct list_head  rs_waiting_events;

#define RS_EVENT_QUEUE_LIMIT	16 /* Allow up to 16 events. */
	/* The number of events in the event queue. */
	unsigned int      rs_waiting_event_count;
};


/* We cheat and use a piece of the address as the timeout. */
static long *rs_timeout(struct ipmi_recv_msg *msg)
{
	char *base = (char *) &(msg->addr);
	base += sizeof(struct ipmi_ipmb_addr);
	return (long *) base;
}

static void rs_msg_recv(struct ipmi_recv_msg *msg,
			void                 *data)
{
	struct priv_data *priv = data;
	unsigned long    flags;

	spin_lock_irqsave(&(priv->rs_lock), flags);
	if (msg->recv_type == IPMI_RESPONSE_RECV_TYPE) {
		*rs_timeout(msg) = 5000;
		list_add_tail(&(msg->link), &(priv->rs_waiting_rsps));
		wake_up_all(&(priv->rs_waiting_rsp_rcvrs));
	} else if (msg->recv_type == IPMI_CMD_RECV_TYPE) {
		*rs_timeout(msg) = 5000;
		list_add_tail(&(msg->link), &(priv->rs_waiting_cmds));
		wake_up_all(&(priv->rs_waiting_cmd_rcvrs));
	} else if (msg->recv_type == IPMI_ASYNC_EVENT_RECV_TYPE) {
		if (priv->rs_waiting_event_count > RS_EVENT_QUEUE_LIMIT) {
			ipmi_free_recv_msg(msg);
		} else {
			list_add_tail(&(msg->link),&(priv->rs_waiting_events));
			(priv->rs_waiting_event_count)++;
		}
	} else {
		ipmi_free_recv_msg(msg);
	}
	spin_unlock_irqrestore(&(priv->rs_lock), flags);
}

/* We emulate the event queue in the driver for the Radisys emulation. */
static int rs_handle_event_request(struct priv_data *priv)
{
	struct list_head     *entry;
	unsigned long        flags;
	struct ipmi_recv_msg *msg = NULL;
	int                  rv = 0;

	spin_lock_irqsave(&(priv->rs_lock), flags);
	if (list_empty(&(priv->rs_waiting_events))) {
		/* Nothing in the event queue, just return an error. */
		msg = ipmi_alloc_recv_msg();
		if (msg == NULL) {
			rv = -EAGAIN;
			goto out_err;
		}
		msg->addr.addr_type = IPMI_SYSTEM_INTERFACE_ADDR_TYPE;
		msg->addr.channel = IPMI_BMC_CHANNEL;
		msg->msg.cmd = IPMI_READ_EVENT_MSG_BUFFER_CMD;
		msg->msgid = 0;
		msg->recv_type = IPMI_ASYNC_EVENT_RECV_TYPE;
		msg->msg.netfn = IPMI_NETFN_APP_RESPONSE;
		msg->msg.data = msg->msg_data;
		msg->msg.data[0] = 0x80; /* Data not available. */
		msg->msg.data_len = 1;
	} else {
		/* Pull an item from the event queue . */
		entry = priv->rs_waiting_events.next;
		list_del(entry);
		msg = list_entry(entry, struct ipmi_recv_msg, link);
		(priv->rs_waiting_event_count)--;
	}

	/* Put the response into the list of waiting responses and
           wake all the response waiters up. */
	*rs_timeout(msg) = 5000;
	list_add_tail(&(msg->link), &(priv->rs_waiting_rsps));
	wake_up_all(&(priv->rs_waiting_rsp_rcvrs));

 out_err:
	spin_unlock_irqrestore(&(priv->rs_lock), flags);
	return rv;
}

static struct ipmi_recv_msg *rs_find_in_list(struct list_head *q,
					     unsigned char    slave_addr,
					     unsigned char    lun,
					     unsigned char    netfn,
					     unsigned char    cmd,
					     unsigned char    seq)
{
	struct list_head      *entry;
	struct ipmi_recv_msg  *msg;
	struct ipmi_addr      addr;
	unsigned char         msg_seq;

	if (slave_addr == 1) {
		struct ipmi_system_interface_addr *smi_addr;
		smi_addr = (struct ipmi_system_interface_addr *) &addr;
		smi_addr->addr_type = IPMI_SYSTEM_INTERFACE_ADDR_TYPE;
		smi_addr->lun = lun;
		/* Slave address 1 means no matching sequence in the
                   Radisys driver. */
	} else {
		struct ipmi_ipmb_addr *ipmb_addr;
		ipmb_addr = (struct ipmi_ipmb_addr *) &addr;
		ipmb_addr->addr_type = IPMI_IPMB_ADDR_TYPE;
		ipmb_addr->slave_addr = slave_addr;
		ipmb_addr->lun = lun;
	}

	list_for_each(entry, q) {
		msg = list_entry(entry, struct ipmi_recv_msg, link);
		if (msg->addr.channel == IPMI_BMC_CHANNEL)
			msg_seq = 0;
		else
			msg_seq = msg->msgid;

		/* We ignore the channel for these comparisons, since the
		   Radisys driver seems to ignore it. */
		addr.channel = msg->addr.channel;

		if ((msg_seq == seq)
		    && (msg->msg.cmd == cmd)
		    && (msg->msg.netfn == (netfn >> 2))
		    && ipmi_addr_equal(&addr, &(msg->addr)))
		{
			list_del(entry);
			return msg;
		}
	}

	return NULL;
}

static struct priv_data *ipmi_user;
static unsigned int user_count = 0; /* How many users have this open. */
static spinlock_t dev_lock = __SPIN_LOCK_UNLOCKED(&dev_lock);

static int ipmi_open(struct inode *inode, struct file *file)
{
	file->private_data = ipmi_user;
	spin_lock(&dev_lock);
	if (user_count == 0)
		ipmi_set_gets_events(ipmi_user->rs_user, 1);
	user_count++;
	spin_unlock(&dev_lock);

	return 0;
}

static int ipmi_release(struct inode *inode, struct file *file)
{
	spin_lock(&dev_lock);
	user_count--;
	if (user_count == 0)
		ipmi_set_gets_events(ipmi_user->rs_user, 0);
	spin_unlock(&dev_lock);
	return 0;
}

static unsigned char
ipmb_checksum(unsigned char *data, int size)
{
	unsigned char csum = 0;

	for (; size > 0; size--, data++)
		csum += *data;

	return -csum;
}

static long ipmi_ioctl(struct file   *file,
		       unsigned int  cmd,
		       unsigned long data)
{
	struct priv_data *priv = file->private_data;
	int              rv = -EINVAL;

	switch(cmd) {
	case IOCTL_IPMI_RCV:   /* get ipmi message */
	{
		IPMI_MSGDESC         rsp;
		struct ipmi_recv_msg *msg;
		unsigned long        flags;
		long                 timeout;
		wait_queue_t         wait;

		if (copy_from_user(&rsp, (void *) data, sizeof(rsp))) {
			rv = -EFAULT;
			break;
		}

		rv = 0;

		spin_lock_irqsave(&(priv->rs_lock), flags);

		msg = rs_find_in_list(&(priv->rs_waiting_rsps),
				      rsp.Dest.uchSlave,
				      rsp.Dest.uchLun,
				      rsp.uchNetFn,
				      rsp.uchCmd,
				      rsp.uchSeq);
		init_waitqueue_entry(&wait, current);
		add_wait_queue(&(priv->rs_waiting_rsp_rcvrs),
			       &wait);
		timeout = (5000 * HZ) / 1000;
		while (msg == NULL) {
			set_current_state(TASK_INTERRUPTIBLE);
			if (!signal_pending(current)) {
				spin_unlock_irqrestore
					(&(priv->rs_lock), flags);
				timeout = schedule_timeout(timeout);
				spin_lock_irqsave
					(&(priv->rs_lock), flags);
			} else {
				rv = -ERESTARTSYS;
				break;
			}
			if (timeout <= 0) {
				rsp.uchComplete = IPMI_TIMEOUT_COMPLETION_CODE;
				break;
			} else {
				msg = rs_find_in_list
					(&(priv->rs_waiting_rsps),
					 rsp.Dest.uchSlave,
					 rsp.Dest.uchLun,
					 rsp.uchNetFn,
					 rsp.uchCmd,
					 rsp.uchSeq);
			}
		}
		remove_wait_queue(&(priv->rs_waiting_rsp_rcvrs),
				  &wait);
		spin_unlock_irqrestore(&(priv->rs_lock), flags);

		if (msg != NULL) {
			rsp.uchComplete = msg->msg.data[0];
			/* The Radisys driver expects all the data to
			   be there in the data, even the stuff we
			   already have processed for it.  So make is
			   so. */
			if (msg->addr.channel == IPMI_BMC_CHANNEL) {
				struct ipmi_system_interface_addr *smi_addr;

				smi_addr = ((struct ipmi_system_interface_addr *)
					    &(msg->addr));
				memcpy(&(rsp.auchBuffer[2]),
				       &(msg->msg.data[0]),
				       msg->msg.data_len);
				rsp.ulLength = msg->msg.data_len+2;
				rsp.auchBuffer[0] = ((msg->msg.netfn << 2)
						     | (smi_addr->lun));
				rsp.auchBuffer[1] = msg->msg.cmd;
			} else {
				struct ipmi_ipmb_addr *ipmb_addr;

				ipmb_addr = (struct ipmi_ipmb_addr *) &msg->addr;
				memcpy(&(rsp.auchBuffer[9]),
				       &(msg->msg.data[0]),
				       msg->msg.data_len);
				rsp.ulLength = msg->msg.data_len+10;
				rsp.auchBuffer[0] = IPMI_NETFN_APP_REQUEST << 2;
				rsp.auchBuffer[1] = IPMI_GET_MSG_CMD;
				rsp.auchBuffer[2] = 0;
				rsp.auchBuffer[3] = msg->addr.channel;
				rsp.auchBuffer[4] = ((msg->msg.netfn << 2)
						     | 2);
				rsp.auchBuffer[5]
					= ipmb_checksum(&(rsp.auchBuffer[3]),
							2);
				rsp.auchBuffer[6] = ipmb_addr->slave_addr;
				rsp.auchBuffer[7] = ((msg->msgid << 2)
						     | ipmb_addr->lun);
				rsp.auchBuffer[8] = msg->msg.cmd;
				rsp.auchBuffer[msg->msg.data_len+9]
					= ipmb_checksum(&(rsp.auchBuffer[6]),
							msg->msg.data_len+3);
			}
			ipmi_free_recv_msg(msg);
		}

		if (copy_to_user((void *) data, &rsp, sizeof(rsp))) {
			rv = -EFAULT;
			break;
		}

		break;
	}

	case IOCTL_IPMI_SEND: /* send ipmi message */
	{
		IPMI_MSGDESC     req;
		struct ipmi_addr addr;
		struct kernel_ipmi_msg  msg;
		unsigned char    source_address;
		unsigned char    source_lun;
		unsigned int     start_offset;
		unsigned char    seq;

		if (copy_from_user(&req, (void *) data, sizeof(req))) {
			rv = -EFAULT;
			break;
		}

		if (((req.auchBuffer[0] >> 2) != IPMI_NETFN_APP_REQUEST)
		    || (req.auchBuffer[1] != IPMI_SEND_MSG_CMD))
		{
			/* It's not a send message, so it's a message
			   directly to the BMC. */
			struct ipmi_system_interface_addr *smi_addr
				= (struct ipmi_system_interface_addr *) &addr;
			if (((req.auchBuffer[0] >> 2)
			     == (IPMI_NETFN_APP_REQUEST << 2))
			    && (req.auchBuffer[1]
				== IPMI_READ_EVENT_MSG_BUFFER_CMD))
			{
				/* The driver gets event messages
                                   automatically, so we emulate
                                   this. */
				rv = rs_handle_event_request(priv);
				break;
			}

			smi_addr->addr_type = IPMI_SYSTEM_INTERFACE_ADDR_TYPE;
			smi_addr->channel = IPMI_BMC_CHANNEL;
			smi_addr->lun = req.auchBuffer[0] & 0x3;
			msg.netfn = req.auchBuffer[0] >> 2;
			msg.cmd = req.auchBuffer[1];
			seq = req.uchSeq;
			source_address = 0;
			source_lun = 0;
			start_offset = 2;
		} else {
			struct ipmi_ipmb_addr *ipmb_addr =
				(struct ipmi_ipmb_addr *) &addr;

			ipmb_addr->addr_type = IPMI_IPMB_ADDR_TYPE;
			ipmb_addr->channel = req.auchBuffer[2] & 0xf;
			ipmb_addr->slave_addr = req.auchBuffer[3];
			ipmb_addr->lun = req.auchBuffer[4] & 0x3;
			msg.netfn = req.auchBuffer[4] >> 2;
			msg.cmd = req.auchBuffer[8];
			seq = req.auchBuffer[7] >> 2;
			source_address = req.auchBuffer[6];
			source_lun = req.auchBuffer[7] & 0x3;
			start_offset = 9;
			req.ulLength--; /* Remove the checksum the userland
					   process adds. */
		}

		msg.data = req.auchBuffer + start_offset;
		msg.data_len = req.ulLength - start_offset;

		rv = ipmi_request_with_source(priv->rs_user,
					      &addr,
					      seq,
					      &msg,
					      NULL,
					      0,
					      source_address,
					      source_lun);
		if (rv)
			req.uchComplete = IPMI_UNKNOWN_ERR_COMPLETION_CODE;
		else
			req.uchComplete = 0;
		/* The Radisys driver does no error checking here. */
		rv = copy_to_user((void *) data, &req, sizeof(req));
		rv = 0;
		break;
	}

	case IOCTL_IPMI_EVENT:  /* get an incoming command.  Don't be
                                   fooled by the name, these are
                                   commands, not IPMI events. */
	{
		IPMI_MSGDESC         rsp;
		struct ipmi_recv_msg *msg = NULL;
		struct list_head     *entry;
		unsigned long        flags;
		long                 timeout;
		unsigned long        receiver;
		wait_queue_t         wait;

		if (copy_from_user(&receiver, (void *) data, sizeof(receiver)))
		{
			rv = -EFAULT;
			break;
		}

		if (copy_from_user(&timeout,
				   (void *) (data + sizeof(receiver)),
				   sizeof(timeout)))
		{
			rv = -EFAULT;
			break;
		}

		rv = 0;

		spin_lock_irqsave(&(priv->rs_lock), flags);

		/* If someone else is already waiting, the Radisys driver
		   returns EFAULT, so we do to. */
		if (priv->rs_cmd_waiting) {
			spin_unlock_irqrestore(&(priv->rs_lock), flags);
			rv = -EFAULT;
			break;
		}

		/* If the user thread doesn't match up, abort. */
		if (receiver != priv->rs_cmd_receiver) {
			spin_unlock_irqrestore(&(priv->rs_lock), flags);
			rsp.uchComplete = IPMI_INVALID_CMD_COMPLETION_CODE;
			break;
		}

		init_waitqueue_entry(&wait, current);
		add_wait_queue(&(priv->rs_waiting_cmd_rcvrs),
			       &wait);
		priv->rs_cmd_waiting = 1;
		timeout = (timeout * HZ) / 1000; /* from ms to jiffies */
		while ((timeout > 0) &&
		       list_empty(&(priv->rs_waiting_cmds)))
		{
			set_current_state(TASK_INTERRUPTIBLE);
			if (!signal_pending(current)) {
				spin_unlock_irqrestore
					(&(priv->rs_lock), flags);
				timeout = schedule_timeout(timeout);
				spin_lock_irqsave
					(&(priv->rs_lock), flags);
			} else {
				rv = -ERESTARTSYS;
				break;
			}
		}
		if (!list_empty(&(priv->rs_waiting_cmds))) {
			entry = priv->rs_waiting_cmds.next;
			list_del(entry);
			msg = list_entry(entry, struct ipmi_recv_msg, link);
		}
		priv->rs_cmd_waiting = 0;
		remove_wait_queue(&(priv->rs_waiting_cmd_rcvrs),
				  &wait);
		spin_unlock_irqrestore(&(priv->rs_lock), flags);

		if (msg != NULL) {
			/* The Radisys driver expects all the data to
			   be there in the data, even the stuff we
			   already have processed for it.  So make is
			   so. */
			struct ipmi_ipmb_addr *ipmb_addr;

			ipmb_addr = (struct ipmi_ipmb_addr *) &msg->addr;
			memcpy(&(rsp.auchBuffer[9]),
			       &(msg->msg.data[0]),
			       msg->msg.data_len);
			rsp.ulLength = msg->msg.data_len+9;
			rsp.auchBuffer[0] = IPMI_NETFN_APP_REQUEST << 2;
			rsp.auchBuffer[1] = IPMI_SEND_MSG_CMD;
			rsp.auchBuffer[2] = 0;
			rsp.auchBuffer[3] = msg->addr.channel;
			rsp.auchBuffer[4] = ((msg->msg.netfn << 2)
					     | 2);
			rsp.auchBuffer[5]
				= ipmb_checksum(&(rsp.auchBuffer[3]),
						2);
			rsp.auchBuffer[6] = ipmb_addr->slave_addr;
			rsp.auchBuffer[7] = ((msg->msgid << 2)
					     | ipmb_addr->lun);
			rsp.auchBuffer[8] = msg->msg.cmd;

			rsp.uchNetFn = (msg->msg.netfn << 2);
			rsp.uchCmd = msg->msg.cmd;
			rsp.uchSeq = msg->msgid;
			rsp.ulLength = msg->msg.data_len + 9;
			ipmi_free_recv_msg(msg);
		} else if (!rv) {
			/* On a time out, the Radisys driver returns
			   IPMIRC_ERROR in the completion code, for
			   some wierd reason. */
			rsp.uchComplete = IPMI_UNKNOWN_ERR_COMPLETION_CODE;
		}

		/* The Radisys driver does no error checking here. */
		rv = copy_to_user((void *) data, &rsp, sizeof(rsp));
		rv = 0;
		break;
	}

	case IOCTL_IPMI_REGISTER: /* register as event receiver */
	{
		unsigned long receiver;
		unsigned char rc = LOWLRC_SUCCESS;
		unsigned long flags;

		if (copy_from_user(&receiver, (void *) data, sizeof(receiver)))
		{
			rv = -EFAULT;
			break;
		}

		spin_lock_irqsave(&(priv->rs_lock), flags);
		if (priv->rs_cmd_receiver == 0) {
			rv = ipmi_register_all_cmd_rcvr(priv->rs_user);
			if (rv)	{
				priv->rs_cmd_receiver = receiver;
			} else {
				rc = LOWLRC_ERROR;
			}
		} else if (priv->rs_cmd_receiver != receiver) {
			rc = LOWLRC_ERROR;
		}
		spin_unlock_irqrestore(&(priv->rs_lock), flags);

		/* The Radisys driver does no error checking here. */
		rv = copy_to_user((void *) data, &rc, sizeof(rc));
		rv = 0;
		break;
	}

	case IOCTL_IPMI_UNREGISTER:   /* unregister as event receiver */
	{
		unsigned long receiver;
		unsigned char rc = LOWLRC_SUCCESS;
		unsigned long flags;

		if (copy_from_user(&receiver, (void *) data, sizeof(receiver)))
		{
			rv = -EFAULT;
			break;
		}

		spin_lock_irqsave(&(priv->rs_lock), flags);
		if (priv->rs_cmd_receiver == receiver) {
			ipmi_unregister_all_cmd_rcvr(priv->rs_user);
			priv->rs_cmd_receiver = 0;
		} else {
			rc = LOWLRC_ERROR;
		}
		spin_unlock_irqrestore(&(priv->rs_lock), flags);

		/* The Radisys driver does no error checking here. */
		rv = copy_to_user((void *) data, &rc, sizeof(rc));
		rv = 0;
		break;
	}

	case IOCTL_IPMI_CLEAR: /* clear registered event receiver */
	{
		unsigned char rc = LOWLRC_SUCCESS;
		unsigned long flags;

		spin_lock_irqsave(&(priv->rs_lock), flags);
		ipmi_unregister_all_cmd_rcvr(priv->rs_user);
		priv->rs_cmd_receiver = 0;
		spin_unlock_irqrestore(&(priv->rs_lock), flags);

		/* The Radisys driver does no error checking here. */
		rv = copy_to_user((void *) data, &rc, sizeof(rc));
		rv = 0;
		break;
	}
	}

	return rv;
}

static struct file_operations ipmi_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= ipmi_ioctl,
	.open		= ipmi_open,
	.release	= ipmi_release
};

static struct timer_list ipmi_radisys_timer;

/* Call every 100 ms. */
#define IPMI_TIMEOUT_TIME	100
#define IPMI_TIMEOUT_JIFFIES	((IPMI_TIMEOUT_TIME * HZ) / 1000)

/* Request events from the queue every second.  Hopefully, in the
   future, IPMI will add a way to know immediately if an event is
   in the queue. */
#define IPMI_REQUEST_EV_TIME	(1000 / (IPMI_TIMEOUT_TIME))

static volatile int stop_operation = 0;
static volatile int timer_stopped = 0;

static void ipmi_radisys_timeout(unsigned long data)
{
	struct list_head     *entry, *entry2;
	struct priv_data     *priv = (struct priv_data *) data;
	int                  timeout_period = IPMI_TIMEOUT_TIME;
	struct ipmi_recv_msg *msg;

	if (stop_operation) {
		timer_stopped = 1;
		return;
	}

	/* Now time out any messages in the Radisys message queue. */
	spin_lock(&(priv->rs_lock));
	list_for_each_safe(entry, entry2, &(priv->rs_waiting_rsps)) {
		long *timeout;
		msg = list_entry(entry, struct ipmi_recv_msg, link);
		timeout = rs_timeout(msg);
		*timeout -= timeout_period;
		if ((*timeout) <= 0) {
			list_del(entry);
			ipmi_free_recv_msg(msg);
		}
	}
	list_for_each_safe(entry, entry2, &(priv->rs_waiting_cmds)) {
		long *timeout;
		msg = list_entry(entry, struct ipmi_recv_msg, link);
		timeout = rs_timeout(msg);
		*timeout -= timeout_period;
		if ((*timeout) <= 0) {
			list_del(entry);
			ipmi_free_recv_msg(msg);
		}
	}
	spin_unlock(&priv->rs_lock);

	ipmi_radisys_timer.expires += IPMI_TIMEOUT_JIFFIES;
	add_timer(&ipmi_radisys_timer);
}

#define DEVICE_NAME     "ipmi"

static int ipmi_radisys_major = 0;
module_param(ipmi_radisys_major, int, 0);

static struct ipmi_user_hndl ipmi_hndlrs =
{
	ipmi_recv_hndl : rs_msg_recv
};


static int init_ipmi_radisys(void)
{
	int rv;

	if (ipmi_radisys_major < 0)
		return -EINVAL;

	ipmi_user = kmalloc(sizeof(*ipmi_user), GFP_KERNEL);
	if (!ipmi_user) {
		printk("ipmi_radisys: Unable to allocate memory\n");
		return -ENOMEM;
	}

	/* Create the Radisys interface user. */
	spin_lock_init(&(ipmi_user->rs_lock));
	INIT_LIST_HEAD(&(ipmi_user->rs_waiting_rsps));
	init_waitqueue_head(&(ipmi_user->rs_waiting_rsp_rcvrs));
	INIT_LIST_HEAD(&(ipmi_user->rs_waiting_cmds));
	init_waitqueue_head(&(ipmi_user->rs_waiting_cmd_rcvrs));
	ipmi_user->rs_cmd_waiting = 0;
	INIT_LIST_HEAD(&(ipmi_user->rs_waiting_events));

	rv = ipmi_create_user(0,
			      &ipmi_hndlrs,
			      ipmi_user,
			      &(ipmi_user->rs_user));
	if (rv) {
		printk("ipmi_radisys: Unable to create an IPMI user, probably"
		       " no physical devices present.\n");
		kfree(ipmi_user);
		ipmi_user = NULL;
		return rv;
	}

	rv = register_chrdev(ipmi_radisys_major, DEVICE_NAME, &ipmi_fops);
	if (rv < 0)
	{
		printk("ipmi_radisys: Unable to create the character device\n");
		kfree(ipmi_user);
		ipmi_user = NULL;
		printk(KERN_ERR "ipmi: can't get major %d\n",
		       ipmi_radisys_major);
		return rv;
	}

	if (ipmi_radisys_major == 0)
	{
		ipmi_radisys_major = rv;
	}

	init_timer(&ipmi_radisys_timer);
	ipmi_radisys_timer.data = (long) ipmi_user;
	ipmi_radisys_timer.function = ipmi_radisys_timeout;
	ipmi_radisys_timer.expires = jiffies + IPMI_TIMEOUT_JIFFIES;
	add_timer(&ipmi_radisys_timer);

	printk(KERN_INFO "ipmi_radisys: driver initialized at char major %d\n",
	       ipmi_radisys_major);

	return 0;
}

#ifdef MODULE
static void free_recv_msg_list(struct list_head *q)
{
	struct list_head     *entry, *entry2;
	struct ipmi_recv_msg *msg;

	list_for_each_safe(entry, entry2, q) {
		msg = list_entry(entry, struct ipmi_recv_msg, link);
		list_del(entry);
		ipmi_free_recv_msg(msg);
	}
}

static void cleanup_ipmi_radisys(void)
{
	/* Tell the timer to stop, then wait for it to stop.  This avoids
	   problems with race conditions removing the timer here. */
	stop_operation = 1;
	while (!timer_stopped) {
		schedule_timeout(1);
	}

	ipmi_destroy_user(ipmi_user->rs_user);

	free_recv_msg_list(&(ipmi_user->rs_waiting_rsps));
	free_recv_msg_list(&(ipmi_user->rs_waiting_cmds));
	free_recv_msg_list(&(ipmi_user->rs_waiting_events));

	kfree(ipmi_user);
	ipmi_user = NULL;

	unregister_chrdev(ipmi_radisys_major, DEVICE_NAME);
}
module_exit(cleanup_ipmi_radisys);
#else
static int __init ipmi_radisys_setup (char *str)
{
	int x;

	if (get_option (&str, &x)) {
		/* ipmi=x sets the major number to x. */
		ipmi_radisys_major = x;
	} else if (!strcmp(str, "off")) {
		ipmi_radisys_major = -1;
	}

	return 1;
}
__setup("ipmi_radisys=", ipmi_radisys_setup);
#endif

module_init(init_ipmi_radisys);
MODULE_LICENSE("GPL");
