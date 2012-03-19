/*
 * ipmi_imb.c
 *
 * Intel IMB emulation for the IPMI interface.
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
#include <linux/ipmi_imb.h>
#include <linux/init.h>
#include <linux/mm.h>


#define IPMI_IMB_VERSION   "v34"
#define MAX_BUFFER_SIZE                 64
#define BMC_SA				0x20

struct waiting_rsp
{
	/* The message id being waited for. */
	unsigned long     msgid;

	/* Used to wake the waiting thread. */
	wait_queue_head_t waitq;

	/* The message handler puts the message here. */
	struct ipmi_recv_msg *msg;

	struct list_head link;
};

struct priv_data
{
	/* This is for supporting the old Imb interface. */
	ipmi_user_t       imb_user;
	spinlock_t        imb_lock;

	unsigned long     curr_msgid;

	/* A list of responses waiting in the queue, a list of struct
	   waiting_rsp. */
	struct list_head  imb_waiting_rsps;

	/* A list of commands that have come in. */
	struct list_head  imb_waiting_cmds;

	/* First async command sequence number in list of commands that have come in.*/
	unsigned long    FirstCmdSeqenceNo;
	/* Last async command sequence number in list of commands that have come in.*/
	unsigned long    LastCmdSeqenceNo;

	/* A list of thing waiting for commands.  We wake them all up
	   when a command comes in. */
	wait_queue_head_t imb_waiting_cmd_rcvrs;

	/* The registered command receiver value.  This only allows someone
	   with the "magic number" to issue commands. */
	unsigned long     imb_cmd_receiver;

	/* Is someone already waiting for a command?  The Imb driver
	   only allows one waiter, this enforces that. */
	int               imb_cmd_waiting;

	/* A list of IPMI events waiting to be delivered.  (not that
           the Imb driver calls incoming commands "events", this
           variable is actually IPMI events, not incoming commands). */
	struct list_head  imb_waiting_events;

#define IMB_EVENT_QUEUE_LIMIT	16 /* Allow up to 16 events. */
	/* The number of events in the event queue. */
	unsigned int      imb_waiting_event_count;
};

/* We cheat and use a piece of the address as the timeout. */
static long *imb_timeout(struct ipmi_recv_msg *msg)
{
	char *base = (char *) &(msg->addr);
	if(msg->addr.addr_type == IPMI_LAN_ADDR_TYPE )
		base += sizeof(struct ipmi_lan_addr);
	else
		base += sizeof(struct ipmi_ipmb_addr);
	return (long *) base;
}

static void imb_msg_recv(struct ipmi_recv_msg *msg,
			 void                 *data)
{
	struct priv_data *priv = data;
	unsigned long    flags;
	struct list_head   *entry;
	struct waiting_rsp *rsp;

	spin_lock_irqsave(&(priv->imb_lock), flags);
	if (msg->recv_type == IPMI_RESPONSE_RECV_TYPE ||
	     msg->recv_type == IPMI_RESPONSE_RESPONSE_TYPE) {

		list_for_each(entry, &(priv->imb_waiting_rsps)) {
			rsp = list_entry(entry, struct waiting_rsp, link);
			if (rsp->msgid == msg->msgid) {
				rsp->msg = msg;
				msg = NULL;
				list_del(entry);
				wake_up(&(rsp->waitq));
				break;
			}
		}
		if (msg) {
//			printk(KERN_INFO "imb_msg_recv: DROPPING netfn %x/cmd %x/type %x/channel %x/msgid %ld\n",
//						msg->msg.netfn, msg->msg.cmd, msg->addr.addr_type, msg->addr.channel, msg->msgid);
			/* No waiter found, this was either a "don't
			 * wait for the response" message or the
			 * waiter was interrupted. */
			ipmi_free_recv_msg(msg);
		}
	} else if (msg->recv_type == IPMI_CMD_RECV_TYPE) {
		/* Leave commands in the command wait queue for 5 seconds. */
		/* All commands in the command wait queue are left to timeout. */
		*imb_timeout(msg) = 5000;
		priv->LastCmdSeqenceNo += 1;
		list_add_tail(&(msg->link), &(priv->imb_waiting_cmds));
		wake_up_all(&(priv->imb_waiting_cmd_rcvrs));

	} else if (msg->recv_type == IPMI_ASYNC_EVENT_RECV_TYPE) {
		if (priv->imb_waiting_event_count > IMB_EVENT_QUEUE_LIMIT) {
			ipmi_free_recv_msg(msg);
		} else {
			list_add_tail(&(msg->link),&(priv->imb_waiting_events));
			(priv->imb_waiting_event_count)++;
		}
	} else {
		ipmi_free_recv_msg(msg);
	}
	spin_unlock_irqrestore(&(priv->imb_lock), flags);
}

/* We emulate the event queue in the driver for the imb emulation. */
static int imb_handle_event_request(struct priv_data     *priv,
				    struct ipmi_recv_msg **rsp)
{
	struct list_head     *entry;
	unsigned long        flags;
	struct ipmi_recv_msg *msg = NULL;
	int                  rv = 0;

	spin_lock_irqsave(&(priv->imb_lock), flags);
	if (list_empty(&(priv->imb_waiting_events))) {
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
		entry = priv->imb_waiting_events.next;
		list_del(entry);
		msg = list_entry(entry, struct ipmi_recv_msg, link);
		(priv->imb_waiting_event_count)--;
	}

	*rsp = msg;

 out_err:
	spin_unlock_irqrestore(&(priv->imb_lock), flags);
	return rv;
}

static struct priv_data *ipmi_user;
static unsigned int user_count = 0; /* How many users have this open. */
static spinlock_t dev_lock = __SPIN_LOCK_UNLOCKED(&dev_lock);

static int ipmi_imb_open(struct inode *inode, struct file *file)
{
	int rv;

	if (user_count == 0) {
		rv = ipmi_register_all_cmd_rcvr(ipmi_user->imb_user);
		if (rv)	{
			return rv;
		}
	}

	file->private_data = ipmi_user;
	spin_lock(&dev_lock);
	if (user_count == 0)
		ipmi_set_gets_events(ipmi_user->imb_user, 1);
	user_count++;
	spin_unlock(&dev_lock);

	return 0;
}

static int ipmi_imb_release(struct inode *inode, struct file *file)
{
	spin_lock(&dev_lock);
	user_count--;
	if (user_count == 0) {
		ipmi_set_gets_events(ipmi_user->imb_user, 0);
		ipmi_unregister_all_cmd_rcvr(ipmi_user->imb_user);
	}
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

extern void ipmi_delayed_shutdown(long delay, int power_off);

static long ipmi_imb_ioctl(struct file   *file,
			   unsigned int  cmd,
			   unsigned long data)
{
	struct priv_data *priv = file->private_data;
	int              rv = -EINVAL;
	struct smi       smi;
	unsigned long    flags;

	if (copy_from_user((caddr_t)&smi, (caddr_t)data, sizeof(smi))) {
		return -EFAULT;
	}

	switch(cmd) {
	case IOCTL_IMB_POLL_ASYNC:
		/*
		 * No-op for this, the low-level driver polls.
		 * Successful return is needed, though.
		 */
		rv = 0;
		break;

	case IOCTL_IMB_GET_ASYNC_MSG:
	{
		unsigned char        req[MAX_ASYNC_RESP_SIZE];
		unsigned char        resp[MAX_ASYNC_RESP_SIZE];
		unsigned long	     seq = 0;
		struct ipmi_recv_msg *msg = NULL;
		ImbAsyncRequest      *pAsyncReq = (ImbAsyncRequest *) req;
		ImbAsyncResponse     *pAsyncResp = (ImbAsyncResponse *) resp;
		unsigned long        length = 0;

		if (smi.cbInBuffer < sizeof(ImbAsyncRequest))
			return -EINVAL;
		if (smi.cbOutBuffer < MIN_ASYNC_RESP_SIZE)
			return -EINVAL;

		if (copy_from_user( pAsyncReq, smi.lpvInBuffer,
					sizeof(ImbAsyncRequest)) == -1) {
			return(-EFAULT);
		}

		spin_lock_irqsave(&(priv->imb_lock), flags);
		while (msg == NULL) {
			wait_queue_t wait;
			if( (! list_empty(&(priv->imb_waiting_cmds)))  &&
			    (pAsyncReq->lastSeq < priv->LastCmdSeqenceNo) ) {
				struct list_head *entry, *entry2;
				unsigned long inc = priv->FirstCmdSeqenceNo+1;

				seq = pAsyncReq->lastSeq;
				if (seq < priv->FirstCmdSeqenceNo)
					seq = priv->FirstCmdSeqenceNo;
				seq ++;
				list_for_each_safe(entry, entry2, &(priv->imb_waiting_cmds)) {
					if (seq == inc) {
						msg = list_entry(entry, struct ipmi_recv_msg, link);
						break;
					}
					inc ++ ;
				}
				if (msg != NULL)
					break;
			}
			// AsyncReq->timeOut is not used anymore
			// remove the following line to reenable timeouts
			pAsyncReq->timeOut = 0;
			if (pAsyncReq->timeOut == 0) {
				/* No command waiting, just return an error. */
				rv = IMB_NO_ASYNC_MSG;
				break;
			}

			init_waitqueue_entry(&wait, current);
			add_wait_queue(&(priv->imb_waiting_cmd_rcvrs),
			       &wait);
			set_current_state(TASK_INTERRUPTIBLE);
			if (!signal_pending(current)) {
				spin_unlock_irqrestore
					(&(priv->imb_lock), flags);
				schedule();
				spin_lock_irqsave
					(&(priv->imb_lock), flags);
			} else {
				rv = -ERESTARTSYS;
			}
			remove_wait_queue(&(priv->imb_waiting_cmd_rcvrs),
					  &wait);
			if (rv ==  -ERESTARTSYS)
				break;
			// decrement timeOut Usec by 1/10 second or 100000 usec
			if (pAsyncReq->timeOut < 100000 )
				pAsyncReq->timeOut = 0;
			else
				pAsyncReq->timeOut -= 100000;
		}
		spin_unlock_irqrestore(&(priv->imb_lock), flags);

		if (msg != NULL) {
			pAsyncResp->thisSeq = seq;

//			pAsyncResp->data[0] = IPMI_NETFN_APP_REQUEST << 2;
//			pAsyncResp->data[1] = IPMI_GET_MSG_CMD;
//			pAsyncResp->data[2] = 0;
			if (msg->addr.addr_type == IPMI_IPMB_ADDR_TYPE) {
				struct ipmi_ipmb_addr *ipmb_addr;
				unsigned char *rdata = pAsyncResp->data;
				ipmb_addr = (struct ipmi_ipmb_addr *) &(msg->addr);
				rdata[0] = msg->addr.channel;
				rdata[1] = ((msg->msg.netfn << 2)
						     | 2);
				rdata[2] = ipmb_checksum(&(rdata[1]), 1);
				rdata[3] = ipmb_addr->slave_addr;
				rdata[4] = ((msg->msgid << 2) | ipmb_addr->lun);
				rdata[5] = msg->msg.cmd;

				memcpy(&(rdata[6]),
				       &(msg->msg.data[0]),
				       msg->msg.data_len);

				length = sizeof(pAsyncResp->thisSeq) +
					msg->msg.data_len + 7; //MIN_ASYNC_RESP_SIZE;

				rdata[length-1]	= ipmb_checksum(&(rdata[1]),
								length-2);
			} else  if (msg->addr.addr_type == IPMI_LAN_ADDR_TYPE) {
				struct ipmi_lan_addr *lan_addr;
				unsigned char *rdata = pAsyncResp->data;
				lan_addr = (struct ipmi_lan_addr *) &(msg->addr);

				rdata[0] = (lan_addr->privilege<<4) | lan_addr->channel;
				rdata[1] = lan_addr->session_handle;
				rdata[2] = lan_addr->local_SWID;
				rdata[3] = ((msg->msg.netfn << 2) | 2);
				rdata[4] = ipmb_checksum(&(rdata[2]), 2);
				rdata[5] = lan_addr->remote_SWID;
				rdata[6] = ((msg->msgid << 2) | lan_addr->lun);
				rdata[7] = msg->msg.cmd;

				memcpy(&(rdata[8]),
				       &(msg->msg.data[0]),
				       msg->msg.data_len);

				length = msg->msg.data_len + 8; //MIN_ASYNC_RESP_SIZE;

				rdata[length] = ipmb_checksum(&(rdata[5]),
							      length-5);
				length += sizeof(pAsyncResp->thisSeq) + 1;
			}

			if (copy_to_user(smi.lpvOutBuffer, pAsyncResp, length))
			{
				return -EFAULT;
			}
			rv = STATUS_SUCCESS;
		}
		if (copy_to_user(smi.lpcbBytesReturned,
				 &length,
				 sizeof(length)))
		{
			return -EFAULT;
		}
		break;
	}

	case IOCTL_IMB_SEND_MESSAGE:
	{
		unsigned char        imbReqBuffer[MAX_IMB_RESPONSE_SIZE + 8];
		unsigned char        imbRespBuffer[MAX_IMB_RESPONSE_SIZE + 8];
		ImbRequestBuffer     *pImbReq=(ImbRequestBuffer *)imbReqBuffer;
		ImbResponseBuffer    *pImbResp=(ImbResponseBuffer*)imbRespBuffer;
		struct ipmi_addr     addr;
		struct kernel_ipmi_msg msg;
		unsigned long        msgid = 0;
		     unsigned char    source_address = 0;
		     unsigned char    source_lun = 0;
		struct ipmi_recv_msg *rsp = NULL;
		unsigned long        length;
		wait_queue_t         wait;
		struct waiting_rsp   waiter;


		if ((smi.cbInBuffer < MIN_IMB_REQ_BUF_SIZE)
		    || (smi.cbOutBuffer < MIN_IMB_RESP_BUF_SIZE))
		{
			return -EINVAL;
		}

		if (smi.cbInBuffer > MAX_BUFFER_SIZE) {
			/* Input buffer is too large */
			return -EINVAL;
		}

		if (copy_from_user(pImbReq, smi.lpvInBuffer, smi.cbInBuffer)) {
			return -EFAULT;
		}
		if ((pImbReq->req.dataLength + MIN_IMB_REQ_BUF_SIZE)
		    > smi.cbInBuffer)
		{
			return -EINVAL;
		}
		if (pImbReq->req.dataLength > MAX_BUFFER_SIZE) {
			return -EINVAL;
		}

		if (pImbReq->req.cmd == IPMI_SEND_MSG_CMD) {
			struct ipmi_lan_addr  *lan_addr
				= (struct ipmi_lan_addr *) &addr;
			char *rdata = pImbReq->req.data;

			lan_addr->addr_type = IPMI_LAN_ADDR_TYPE;
			lan_addr->channel = rdata[0];
			lan_addr->session_handle = rdata[1];
			lan_addr->remote_SWID = rdata[2];
			lan_addr->local_SWID = rdata[5];
			lan_addr->lun = rdata[3] & 0x3;
			msgid = rdata[6] >> 2;
			source_lun = rdata[6] & 0x3;

			msg.netfn = rdata[3] >> 2;
			msg.cmd = rdata[7];
			msg.data = &rdata[8];
			msg.data_len = pImbReq->req.dataLength-9;
		} else if (pImbReq->req.rsSa == BMC_SA) {
			struct ipmi_system_interface_addr *smi_addr
				= (struct ipmi_system_interface_addr *) &addr;

			if ((pImbReq->req.netFn
			     == (IPMI_NETFN_APP_REQUEST << 2))
			    && (pImbReq->req.cmd
				== IPMI_READ_EVENT_MSG_BUFFER_CMD))
			{
				/* The driver gets event messages
                                   automatically, so we emulate
                                   this. */
				rv = imb_handle_event_request(priv, &rsp);
				goto copy_resp;
			} else {
				smi_addr->addr_type
				    = IPMI_SYSTEM_INTERFACE_ADDR_TYPE;
				smi_addr->channel = IPMI_BMC_CHANNEL;
				smi_addr->lun = 0;
			}
			msg.netfn = pImbReq->req.netFn;
			msg.cmd = pImbReq->req.cmd;
			msg.data = pImbReq->req.data;
			msg.data_len = pImbReq->req.dataLength;
		} else {
			struct ipmi_ipmb_addr *ipmb_addr =
				(struct ipmi_ipmb_addr *) &addr;

			ipmb_addr->addr_type = IPMI_IPMB_ADDR_TYPE;
			ipmb_addr->slave_addr = pImbReq->req.rsSa;
			ipmb_addr->lun = pImbReq->req.rsLun;
			ipmb_addr->channel = 0;
			msg.netfn = pImbReq->req.netFn;
			msg.cmd = pImbReq->req.cmd;
			msg.data = pImbReq->req.data;
			msg.data_len = pImbReq->req.dataLength;
		}

		if (pImbReq->flags & NO_RESPONSE_EXPECTED) {
			msgid = 0;
		} else {
			if( msgid == 0 ) {
				spin_lock(&priv->imb_lock);
				msgid = priv->curr_msgid;
				(priv->curr_msgid)++;
				if (priv->curr_msgid == 0)
					(priv->curr_msgid)++;
				spin_unlock(&priv->imb_lock);
			}
			waiter.msgid = msgid;
			init_waitqueue_head(&waiter.waitq);
			waiter.msg = NULL;
			spin_lock_irqsave(&priv->imb_lock, flags);
			list_add_tail(&waiter.link, &priv->imb_waiting_rsps);
			spin_unlock_irqrestore(&priv->imb_lock, flags);
		}

		if (pImbReq->req.cmd == IPMI_SEND_MSG_CMD) {
			rv = ipmi_request_with_source(priv->imb_user,
						  &addr,
						  msgid,
						  &msg,
						  NULL,
						  0,
						source_address,
						source_lun);
		} else {
			rv = ipmi_request_settime(priv->imb_user,
						  &addr,
						  msgid,
						  &msg,
						  NULL,
						  0,
						  1,
						  pImbReq->timeOut);
		}
		if (rv) {
			rv = IMB_SEND_REQUEST_FAILED;
			if (!(pImbReq->flags & NO_RESPONSE_EXPECTED)) {
				spin_lock_irqsave(&(priv->imb_lock), flags);
				list_del(&waiter.link);
				spin_unlock_irqrestore(&priv->imb_lock, flags);
			}
			goto no_response;
		}

		if (pImbReq->flags & NO_RESPONSE_EXPECTED)
			goto no_response;

		/* Now wait for the response to come back. */
		spin_lock_irqsave(&(priv->imb_lock), flags);
		if (!waiter.msg) {
			/* No message yet, wait for it. */
			init_waitqueue_entry(&wait, current);
			add_wait_queue(&waiter.waitq, &wait);
			set_current_state(TASK_INTERRUPTIBLE);
			if (!signal_pending(current)) {
				spin_unlock_irqrestore
					(&(priv->imb_lock), flags);
				schedule();
				spin_lock_irqsave
					(&(priv->imb_lock), flags);
			} else {
				rv = -ERESTARTSYS;
			}
			remove_wait_queue(&waiter.waitq, &wait);

			/* At this point, we are either woken by a
			 * signal or we have a message. */
			if (!waiter.msg) {
				/* No message, we need to remove
				   ourself from the queue. */
				list_del(&waiter.link);
			}
		}
		spin_unlock_irqrestore(&(priv->imb_lock), flags);

		rsp = waiter.msg;
	copy_resp:
		if (rsp != NULL) {
			pImbResp->cCode = rsp->msg.data[0];
			if( rsp->msg.data_len > 1 )
				memcpy(pImbResp->data,
				       rsp->msg.data+1,
				       rsp->msg.data_len-1);
			length = (rsp->msg.data_len - 1
				  + MIN_IMB_RESP_BUF_SIZE);

			ipmi_free_recv_msg(rsp);

			if (copy_to_user(smi.lpvOutBuffer, pImbResp, length)) {
				return -EFAULT;
			}

			if (copy_to_user(smi.lpcbBytesReturned,
					 &length,
					 sizeof(length)))
			{
				return -EFAULT;
			}
		}
	no_response:
		break;
	}

	case IOCTL_IMB_SHUTDOWN_CODE:
	{
		ShutdownCmdBuffer shutdownCmd;

		if (copy_from_user(&shutdownCmd,
				   smi.lpvInBuffer,
				   sizeof(ShutdownCmdBuffer)))
		{
			return -EFAULT;
		}

		if (smi.cbInBuffer < sizeof(ShutdownCmdBuffer))
		{
			return -EINVAL;
		}

		rv = 0;
		switch (shutdownCmd.code) {
		case SD_POWER_OFF:
			ipmi_delayed_shutdown(shutdownCmd.delayTime / 10, 1);
			break;

		case SD_RESET:
			ipmi_delayed_shutdown(shutdownCmd.delayTime / 10, 0);
			break;

		case SD_NO_ACTION:
			break;

		default:
			rv = INVALID_ARGUMENTS;
		}
		break;
        }

	case IOCTL_IMB_REGISTER_ASYNC_OBJ:
	{
		unsigned char     imbRespBuffer[MAX_IMB_RESPONSE_SIZE + 8];
		unsigned long     length;
		ImbResponseBuffer *pImbResp=(ImbResponseBuffer*)imbRespBuffer;
		unsigned long dummy;

		pImbResp->cCode = 00;
		length = sizeof(int);
//		++AsyncHandle;
//		memcpy(pImbResp->data,&AsyncHandle,length);
		//
		//Now copy back the  a bogus imb response and length to
		//user space since the imbapi expects it. We can use this
		//later to return a more meaningful value if necesssary.
		//
		dummy = 0xbabe;
		memcpy(pImbResp->data,&dummy,length);
		if (copy_to_user(smi.lpvOutBuffer, pImbResp, length))
			return -EFAULT;

		if (copy_to_user(smi.lpcbBytesReturned, &length,
				 sizeof(length)))
			return -EFAULT;

		rv = STATUS_SUCCESS;
		break;
	}

	case IOCTL_IMB_DEREGISTER_ASYNC_OBJ:
		rv = STATUS_SUCCESS;
		break;

	case IOCTL_IMB_CHECK_EVENT:
	{
		wait_queue_t wait;

		rv = STATUS_SUCCESS;
		spin_lock_irqsave(&(priv->imb_lock), flags);
		init_waitqueue_entry(&wait, current);
		add_wait_queue(&(priv->imb_waiting_cmd_rcvrs),
			       &wait);
		while (list_empty(&(priv->imb_waiting_cmds))) {
			set_current_state(TASK_INTERRUPTIBLE);
			if (!signal_pending(current)) {
				spin_unlock_irqrestore
					(&(priv->imb_lock), flags);
				schedule();
				spin_lock_irqsave
					(&(priv->imb_lock), flags);
			} else {
				rv = -ERESTARTSYS;
				break;
			}
		}
		remove_wait_queue(&(priv->imb_waiting_cmd_rcvrs),
				  &wait);
		spin_unlock_irqrestore(&(priv->imb_lock), flags);
		break;
	}
	default:
		printk(KERN_ERR "ipmi_imb_ioctl: ioctl cmd  0x%x not supported\n", cmd);
		return(-EINVAL);
	}

	return rv;
}

static int ipmi_imb_mmap(struct file *file, struct vm_area_struct *vma)
{
    off_t offset = vma->vm_pgoff << PAGE_SHIFT;

    if (offset < 0)
	    return -EINVAL;

    if (remap_pfn_range(vma, vma->vm_start, offset,
			vma->vm_end - vma->vm_start,
			vma->vm_page_prot))
    {
	    return -EAGAIN;
    }

    /*vma->vm_inode = what_goes_here; */

    return 0;

}


static struct file_operations ipmi_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl = ipmi_imb_ioctl,
	.open		= ipmi_imb_open,
	.release	= ipmi_imb_release,
	.mmap		= ipmi_imb_mmap
};

static struct timer_list ipmi_imb_timer;

/* Call every 100 ms. */
#define IPMI_TIMEOUT_TIME	100
#define IPMI_TIMEOUT_JIFFIES	((IPMI_TIMEOUT_TIME * HZ) / 1000)

static volatile int stop_operation = 0;
static volatile int timer_stopped = 0;

static void ipmi_imb_timeout(unsigned long data)
{
	struct list_head     *entry, *entry2;
	struct priv_data     *priv = (struct priv_data *) data;
	int                  timeout_period = IPMI_TIMEOUT_TIME;
	struct ipmi_recv_msg *msg;

	if (stop_operation) {
		timer_stopped = 1;
		return;
	}

	/* Now time out any messages in the Imb message queue. */
	spin_lock(&(priv->imb_lock));
	list_for_each_safe(entry, entry2, &(priv->imb_waiting_cmds)) {
		long *timeout;
		msg = list_entry(entry, struct ipmi_recv_msg, link);
		timeout = imb_timeout(msg);
		*timeout -= timeout_period;
		if ((*timeout) <= 0) {
			list_del(entry);
			priv->FirstCmdSeqenceNo +=1;
			ipmi_free_recv_msg(msg);
		}
	}
	spin_unlock(&priv->imb_lock);
	// remove the comment in the following line to reenable timeouts
	//wake_up_all(&(priv->imb_waiting_cmd_rcvrs));

	ipmi_imb_timer.expires += IPMI_TIMEOUT_JIFFIES;
	add_timer(&ipmi_imb_timer);
}

#define DEVICE_NAME     "imb"

static int ipmi_imb_major = 0;
module_param(ipmi_imb_major, int, 0);

static struct ipmi_user_hndl ipmi_hndlrs =
{
	ipmi_recv_hndl : imb_msg_recv
};

static int init_ipmi_imb(void)
{
	int rv;

	if (ipmi_imb_major < 0) {
		printk(KERN_ERR "ipmi: bad major %d\n",
		       ipmi_imb_major);
		return -EINVAL;
	}

	ipmi_user = kmalloc(sizeof(*ipmi_user), GFP_KERNEL);
	if (!ipmi_user) {
		printk(KERN_ERR "ipmi: No Memory for major %d\n",
		       ipmi_imb_major);
		return -ENOMEM;
	}

	/* Create the Imb interface user. */
	spin_lock_init(&(ipmi_user->imb_lock));
	INIT_LIST_HEAD(&(ipmi_user->imb_waiting_rsps));
	ipmi_user->FirstCmdSeqenceNo = ASYNC_SEQ_START;
	ipmi_user->LastCmdSeqenceNo = ASYNC_SEQ_START;
	INIT_LIST_HEAD(&(ipmi_user->imb_waiting_cmds));
	init_waitqueue_head(&(ipmi_user->imb_waiting_cmd_rcvrs));
	ipmi_user->imb_cmd_waiting = 0;
	INIT_LIST_HEAD(&(ipmi_user->imb_waiting_events));

	/* Zero is reserved. */
	ipmi_user->curr_msgid = 1;

	rv = ipmi_create_user(0,
			      &ipmi_hndlrs,
			      ipmi_user,
			      &(ipmi_user->imb_user));
	if (rv) {
		kfree(ipmi_user);
		ipmi_user = NULL;
		printk(KERN_ERR "ipmi: can't create user %d\n",
		       rv);
		return rv;
	}

	rv = register_chrdev(ipmi_imb_major, DEVICE_NAME, &ipmi_fops);
	if (rv < 0)
	{
		kfree(ipmi_user);
		ipmi_user = NULL;
		printk(KERN_ERR "ipmi: can't get major %d\n",
		       ipmi_imb_major);
		return rv;
	}

	if (ipmi_imb_major == 0)
	{
		ipmi_imb_major = rv;
	}

	init_timer(&ipmi_imb_timer);
	ipmi_imb_timer.data = (long) ipmi_user;
	ipmi_imb_timer.function = ipmi_imb_timeout;
	ipmi_imb_timer.expires = jiffies + IPMI_TIMEOUT_JIFFIES;
	add_timer(&ipmi_imb_timer);

	printk(KERN_INFO "ipmi_imb %s driver initialized at char major %d\n",
	       IPMI_IMB_VERSION,ipmi_imb_major);

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

static void cleanup_ipmi_imb(void)
{
	/* Tell the timer to stop, then wait for it to stop.  This avoids
	   problems with race conditions removing the timer here. */
	stop_operation = 1;
	while (!timer_stopped) {
		schedule_timeout(1);
	}

	ipmi_destroy_user(ipmi_user->imb_user);

	free_recv_msg_list(&(ipmi_user->imb_waiting_cmds));
	free_recv_msg_list(&(ipmi_user->imb_waiting_events));

	kfree(ipmi_user);
	ipmi_user = NULL;

	unregister_chrdev(ipmi_imb_major, DEVICE_NAME);
}
module_exit(cleanup_ipmi_imb);
#else
static int __init ipmi_imb_setup (char *str)
{
	int x;

	if (get_option (&str, &x)) {
		/* ipmi=x sets the major number to x. */
		ipmi_imb_major = x;
	} else if (!strcmp(str, "off")) {
		ipmi_imb_major = -1;
	}

	return 1;
}
__setup("ipmi_imb=", ipmi_imb_setup);
#endif

module_init(init_ipmi_imb);
MODULE_LICENSE("GPL");
