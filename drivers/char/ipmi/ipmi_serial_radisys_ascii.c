/*
 * ipmi_serial_radisys_ascii.c
 * Serial interface encoder and decoder routines for Radisys ASCII mode.
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *         Corey Minyard <cminyard@mvista.com>
 *
 * Copyright 2006 MontaVista Software Inc.
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
#include <linux/ipmi_smi.h>
#include <linux/ipmi_msgdefs.h>
#include <linux/ipmi_serial_sm.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>

#define PFX "ipmi_serial_radisys_ascii: "

/*
 * Two bytes for every character plus the trailing newline.  There are
 * five overhead bytes that are for the header and checksum beyond the
 * KCS header we get/send to the upper layer..
 */
#define RA_OVERHEAD 5
#define RA_MAX_CHARS_SIZE (((IPMI_MAX_MSG_LENGTH + RA_OVERHEAD) * 2) + 4)

/*
 * Give the remote end this much time (100ms) to send a response
 * before we give up and retry the given number of times.
 */
#define RA_HANDSHAKE_TIME    5000000
#define RA_HANDSHAKE_RETRIES 5

struct ipmi_serial_codec_data {
	struct ipmi_serial_info *info;

	unsigned char xmit_chars[RA_MAX_CHARS_SIZE];
	unsigned int  xmit_chars_len;
	unsigned int  xmit_chars_pos;
	unsigned int  xmit_chars_seq;
	unsigned char xmit_chars_netfn;
	unsigned char xmit_chars_cmd;

	unsigned char recv_chars[RA_MAX_CHARS_SIZE];
	unsigned int  recv_chars_len;
	int           recv_chars_too_many;

	unsigned char seqno;
	unsigned int seqnum_table[0x40];

	unsigned char xmit_msg[IPMI_MAX_MSG_LENGTH + RA_OVERHEAD];
	unsigned int  xmit_msg_len;
	unsigned int  xmit_msg_seq;
	unsigned char xmit_msg_netfn;
	unsigned char xmit_msg_cmd;
	unsigned char recv_msg[IPMI_MAX_MSG_LENGTH + RA_OVERHEAD];
	unsigned int  recv_msg_len;

	unsigned char bmc_i2c_addr;
	unsigned char smi_i2c_addr;

	/*
	 * Used to time receiving the IPMB address.
	 */
	int handshake_time;
	int handshake_retries_left;
	int handshake_done;

	spinlock_t lock;

	void (*recv_msg_handler)(struct ipmi_serial_codec_data *data,
				 const unsigned char *msg,
				 unsigned int len);
};

static int ra_add_options(struct ipmi_serial_codec_data *data,
			  struct seq_file *m)
{
	return 0;
}

static unsigned char
ipmb_checksum(const unsigned char *data, int size)
{
	unsigned char csum = 0;

	for (; size > 0; size--, data++)
		csum += *data;

	return -csum;
}

static unsigned char hex2char[16] = {
	'0', '1', '2', '3', '4', '5', '6', '7',
	'8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
};

static void format_msg(struct ipmi_serial_codec_data *data,
		       const unsigned char *msg, unsigned int msg_len)
{
	int i;
	int len;
	unsigned char *c = data->xmit_chars;

	/*
	 * No need for checks on the output length, the output buffer is
	 * guaranteed to be big enough.
	 */
	len = 0;
	for (i = 0; i < msg_len; i++) {
		c[len++] = hex2char[msg[i] >> 4];
		c[len++] = hex2char[msg[i] & 0xf];
	}
	c[len++] = 0x0d;

	data->xmit_chars_pos = 0;
	data->xmit_chars_len = len;
	data->xmit_chars_seq = data->xmit_msg_seq;
	data->xmit_chars_netfn = data->xmit_msg_netfn;
	data->xmit_chars_cmd = data->xmit_msg_cmd;
}

static int fromhex(unsigned char c)
{
	if (isdigit(c))
		return c - '0';
	else if (isxdigit(c))
		return tolower(c) - 'a' + 10;
	else
		return -EINVAL;
}

/*
 * Called when the 0x0d is seen.
 */
static int unformat_msg(struct ipmi_serial_codec_data *data)
{
	unsigned char *r = data->recv_chars;
	unsigned char *o = data->recv_msg;
	unsigned int len = data->recv_chars_len;
	unsigned int p = 0;
	unsigned int i = 0;
	int          rv;

	while (p < len) {
		if (i > sizeof(data->recv_msg))
			return -EFBIG;
		rv = fromhex(r[p]);
		if (rv < 0)
			return rv;
		o[i] = rv << 4;
		p++;
		if (p >= len)
			return -EINVAL;
		rv = fromhex(r[p]);
		if (rv < 0)
			return rv;
		o[i] |= rv;
		p++;
		i++;
	}
	data->recv_msg_len = i;
	return 0;
}

static void handle_recv_msg(struct ipmi_serial_codec_data *data)
{
	unsigned int seq;
	unsigned char *m = data->recv_msg;
	unsigned int len = data->recv_msg_len;
	int          i;

	if (data->recv_msg_handler) {
		data->recv_msg_handler(data, m, len);
		return;
	}

	if (len < 8) {
		/* Messages must be at least 8 bytes */
		ipmi_serial_ll_protocol_violation(data->info);
		return;
	}

	if (m[3] == data->bmc_i2c_addr) {
		if (ipmb_checksum(m, len) != 0) {
			ipmi_serial_ll_checksum_error(data->info);
			return;
		}
		/* Remove the final checksum. */
		len--;

		/*
		 * It's a message straight from the BMC (a response).
		 * Take the data from the header that we need and
		 * write over the header.
		 */
		m[0] = (m[1] & 0xfc) | (m[4] & 0x03);
		m[1] = m[5];
		seq = m[4] >> 2;
		for (i = 2; i < (len - 4); i++)
			m[i] = m[i+4];
		ipmi_serial_ll_recv(data->info, m, len - 4,
				    data->seqnum_table[seq]);
	} else {
		/* Not from the BMC, must be from the IPMB. */

		if ((len + 3) > IPMI_MAX_MSG_LENGTH) {
			ipmi_serial_ll_protocol_violation(data->info);
			return;
		}

		/* Make room for a get message command header. */
		for (i = len; i > 0; i--)
			/*
			 * Note: This is i + 3, not i + 4, because the
			 * GET_MSG response does not have the
			 * destination address in the message.  So we
			 * write over that field.
			 */
			m[i + 3] = m[i];
		/* Get message command header. */
		m[0] = IPMI_NETFN_APP_RESPONSE << 2;
		m[1] = IPMI_GET_MSG_CMD;
		m[2] = 0; /* completion code */
		m[3] = 0; /* channel */
		ipmi_serial_ll_async(data->info, m, len + 3);
	}
}

static void try_to_send_data(struct ipmi_serial_codec_data *data,
			     unsigned long *flags)
{
	unsigned char *c;
	unsigned int  left;
	unsigned int  sent;

	if (!data->xmit_chars_len)
		return;

 restart:
	c = data->xmit_chars + data->xmit_chars_pos;
	left = data->xmit_chars_len - data->xmit_chars_pos;
	sent = ipmi_serial_ll_xmit(data->info, c, left);
	if (sent == left) {
		/* We are done with this message. */

		if ((data->xmit_chars_netfn == IPMI_NETFN_APP_REQUEST)
		    && (data->xmit_chars_cmd == IPMI_SEND_MSG_CMD)) {
			/*
			 * The transmitted characters will not get a
			 * response from a send message, it goes
			 * straight out onto the IPMB.  So simulate
			 * the response.
			 */
			char msg[3];
			spin_unlock_irqrestore(&data->lock, *flags);
			/* Make into response */
			msg[0] = (data->xmit_chars_netfn | 1) << 2;
			msg[1] = data->xmit_chars_cmd;
			msg[2] = 0;
			ipmi_serial_ll_recv(data->info, msg, 3,
					    data->xmit_chars_seq);
			spin_lock_irqsave(&data->lock, *flags);
		}

		if (data->xmit_msg_len) {
			/* Send the next message we have waiting. */
			format_msg(data, data->xmit_msg, data->xmit_msg_len);
			data->xmit_msg_len = 0;
			goto restart;
		} else {
			/* Nothing to do. */
			data->xmit_chars_len = 0;
		}
	} else {
		data->xmit_chars_pos += sent;
	}
}

static int ra_setup_termios(struct ktermios *t)
{
	/* Nothing to do, the default is fine. */
	return 0;
}

#define RA_OEM1_NETFN		0x30
#define RA_CONTROLLER_OEM_NETFN	0x3e
#define RA_GET_IPMB_ADDR_CMD	0x12
static unsigned char get_ipmbaddr_msg1[] = { 0x01, /* dest addr */
					     RA_OEM1_NETFN << 2,
					     0x3f, /* checksum1 */
					     0x01, /* source addr */
					     0x00, /* seq num/rsLUN */
					     RA_GET_IPMB_ADDR_CMD,
					     0xed  /* checksum2 */
};

static unsigned char get_ipmbaddr_msg2[] = { 0x01, /* dest addr */
					     RA_CONTROLLER_OEM_NETFN << 2,
					     0x07, /* checksum1 */
					     0x01, /* source addr */
					     0x00, /* seq num/rsLUN */
					     RA_GET_IPMB_ADDR_CMD,
					     0xed  /* checksum2 */
};

static void handle_init_getipmbaddr(struct ipmi_serial_codec_data *data,
				    const unsigned char *msg,
				    unsigned int len)
{
	unsigned long flags;

	if (len < 8)
		return;
	if (ipmb_checksum(msg, len) != 0)
		return;
	len--;

	if (((msg[1] >> 2) != (RA_OEM1_NETFN | 1)
	     && (msg[1] >> 2) != (RA_CONTROLLER_OEM_NETFN | 1))
	    || (msg[5] != RA_GET_IPMB_ADDR_CMD))
		return;

	spin_lock_irqsave(&data->lock, flags);
	if (data->handshake_done) {
		spin_unlock_irqrestore(&data->lock, flags);
		return;
	}

	if (msg[6] != 0) {
		if ((msg[1] >> 2) == (RA_OEM1_NETFN | 1)) {
			format_msg(data, get_ipmbaddr_msg2,
				   sizeof(get_ipmbaddr_msg2));
			try_to_send_data(data, &flags);
			spin_unlock_irqrestore(&data->lock, flags);
		} else {
			printk(KERN_ERR "IPMI: RadisysAscii: Got error"
			       " fetching the IPMB address, this may not"
			       " be a Radisys system.\n");
			data->handshake_done = 1;
			spin_unlock_irqrestore(&data->lock, flags);
			ipmi_serial_ll_init_complete(data->info, 0, -EINVAL);
		}
		return;
	}

	data->bmc_i2c_addr = msg[7];
	data->recv_msg_handler = NULL;
	data->handshake_done = 1;
	spin_unlock_irqrestore(&data->lock, flags);
	ipmi_serial_ll_init_complete(data->info, data->bmc_i2c_addr, 0);
}

static int ra_init(struct ipmi_serial_codec_data *data,
		   struct ipmi_serial_info *info,
		   const char *options)
{
	memset(data, 0, sizeof(*data));
	spin_lock_init(&data->lock);
	data->info = info;
	data->bmc_i2c_addr = 1; /* Initial setting should work ok. */
	data->smi_i2c_addr = 1; /* Initial setting should work ok. */

	data->handshake_time = RA_HANDSHAKE_TIME;
	data->handshake_retries_left = RA_HANDSHAKE_RETRIES;

	return 0;
}

static int ra_start(struct ipmi_serial_codec_data *data)
{
	unsigned long flags;

	data->recv_msg_handler = handle_init_getipmbaddr;
	spin_lock_irqsave(&data->lock, flags);
	format_msg(data, get_ipmbaddr_msg1, sizeof(get_ipmbaddr_msg1));
	try_to_send_data(data, &flags);
	spin_unlock_irqrestore(&data->lock, flags);
	return 0;
}

static void ra_cleanup(struct ipmi_serial_codec_data *data)
{
	/* Nothing to do. */
}

static int ra_size(void)
{
	return sizeof(struct ipmi_serial_codec_data);
}

static void ra_handle_char(struct ipmi_serial_codec_data *data,
			   unsigned char ch)
{
	unsigned int len = data->recv_chars_len;
	unsigned char *r;
	int           rv;

	if (ch == 0x0d) {
		/* End of command, handle it. */
		if (data->recv_chars_too_many) {
			/* Input data overrun. */
			ipmi_serial_ll_protocol_violation(data->info);
			data->recv_chars_too_many = 0;
			data->recv_chars_len = 0;
			return;
		}
		rv = unformat_msg(data);
		if (rv) {
			/* Bad input data. */
			ipmi_serial_ll_protocol_violation(data->info);
			return;
		}
		handle_recv_msg(data);
		data->recv_chars_len = 0;
		return;
	}

	if (data->recv_chars_too_many)
		return;

	r = data->recv_chars;

	if (len >= sizeof(data->recv_chars)) {
		data->recv_chars_too_many = 1;
	} else if ((len > 0) && isspace(r[len - 1]) && isspace(ch)) {
		/* Ignore multiple spaces together. */
	} else {
		r[len] = ch;
		data->recv_chars_len++;
	}
}

static int ra_send_msg(struct ipmi_serial_codec_data *data,
		       const unsigned char *msg, unsigned int msg_len,
		       unsigned int seq)
{
	unsigned long flags;
	int           rv = 0;
	int           i, j;
	unsigned char seqno;

	if (msg_len > IPMI_MAX_MSG_LENGTH)
		return -EFBIG;
	if (msg_len < 2)
		return -EINVAL;

	spin_lock_irqsave(&data->lock, flags);
	if (data->xmit_msg_len) {
		/* Something is still waiting to be sent. */
		goto out_unlock;
		rv = -EBUSY;
	}

	data->xmit_msg_netfn = msg[0] >> 2;
	data->xmit_msg_cmd = msg[1];
	data->xmit_msg_seq = seq;
	i = 0;
	if ((data->xmit_msg_netfn == IPMI_NETFN_APP_REQUEST)
	    && (data->xmit_msg_cmd == IPMI_SEND_MSG_CMD)) {
		if ((msg[2] & 0xf) != 0) {
			/* Only channels 0 is supported. */
			rv = -EINVAL;
			goto out_unlock;
		}
		/*
		 * Skip over the message header and send the raw IPMB
		 * message.
		 */
		for (j = 3; j < msg_len; j++)
			data->xmit_msg[i++] = msg[j];
	} else {
		data->xmit_msg[i++] = data->bmc_i2c_addr;
		data->xmit_msg[i++] = msg[0];		/* NetFN/LUN */
		data->xmit_msg[i++] = ipmb_checksum(data->xmit_msg, 2);
		data->xmit_msg[i++] = data->smi_i2c_addr;
		seqno = data->seqno;
		data->seqno = (data->seqno + 1) & 0x3f;
		data->seqnum_table[seqno] = seq;
		data->xmit_msg[i++] = seqno << 2;	/* seqno/rqLUN */
		data->xmit_msg[i++] = msg[1];		/* cmd */
		for (j = 2; j < msg_len; j++)
			data->xmit_msg[i++] = msg[j];
		data->xmit_msg[i] = ipmb_checksum(data->xmit_msg + 3, i - 3);
		i++;
	}


	if (data->xmit_chars_len == 0) {
		/* Transmit queue is empty, just format it now to go. */
		format_msg(data, data->xmit_msg, i);
		try_to_send_data(data, &flags);
	} else {
		/*
		 * Get it ready to be sent later when the transmit
		 * queue empties.
		 */
		data->xmit_msg_len = i;
	}
 out_unlock:
	spin_unlock_irqrestore(&data->lock, flags);
	return rv;
}

static void ra_tx_ready(struct ipmi_serial_codec_data *data)
{
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);
	try_to_send_data(data, &flags);
	spin_unlock_irqrestore(&data->lock, flags);
}

static unsigned int ra_capabilities(struct ipmi_serial_codec_data *data)
{
	return 0;
}


static void ra_timer_tick(struct ipmi_serial_codec_data *data,
			  unsigned int time_since_last)
{
	unsigned long flags;

	/* Check quickly first to avoid grabbing the lock normally. */
	if (likely(data->handshake_done))
		return;

	spin_lock_irqsave(&data->lock, flags);
	/* Recheck to avoid races */
	if (data->handshake_done)
		goto out_unlock;

	data->handshake_time -= time_since_last;
	if (data->handshake_time <= 0) {
		data->handshake_retries_left--;
		if (data->handshake_retries_left <= 0) {
			data->handshake_done = 1;
			spin_unlock_irqrestore(&data->lock, flags);
			ipmi_serial_ll_init_complete(data->info, 0, -ETIMEDOUT);
			goto out;
		}

		/* Resend the IPMB fetch */
		data->handshake_time = RA_HANDSHAKE_TIME;
		format_msg(data, get_ipmbaddr_msg1,
			   sizeof(get_ipmbaddr_msg1));
		try_to_send_data(data, &flags);
	}
 out_unlock:
	spin_unlock_irqrestore(&data->lock, flags);
 out:
	return;
}

static struct ipmi_serial_codec ra_codec = {
	.owner = THIS_MODULE,
	.name  = "RadisysAscii",

	.capabilities	= ra_capabilities,
	.setup_termios	= ra_setup_termios,
	.init		= ra_init,
	.start		= ra_start,
	.cleanup	= ra_cleanup,
	.size		= ra_size,
	.send_msg	= ra_send_msg,
	.handle_char	= ra_handle_char,
	.tx_ready	= ra_tx_ready,
	.add_options	= ra_add_options,
	.timer_tick     = ra_timer_tick
};

static int __init init_ipmi_serial_ra_codec(void)
{
	return ipmi_serial_codec_register(&ra_codec);
}

module_init(init_ipmi_serial_ra_codec);

static void __exit exit_ipmi_serial_ra_codec(void)
{
	ipmi_serial_codec_unregister(&ra_codec);
}
module_exit(exit_ipmi_serial_ra_codec);
MODULE_LICENSE("GPL");
