/*
 * ipmi_serial_direct.c
 * Serial interface encoder and decoder routines for IPMI direct
 * serial interfaces.
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

#define PFX "ipmi_serial_direct: "

struct tm_options {
#define ATTN_OPTION "attn"
	int attn;
	int attn_forced; /* Set by command line. */

#define EATTN_OPTION "eattn"
	int eattn;
	int eattn_forced; /* Set by command line. */
	unsigned char eattn_char;

#define RQADDR_OPTION "rqa"
	unsigned char rqaddr;
#define RSADDR_OPTION "rsa"
	unsigned char rsaddr;
};

#define DEFAULT_RSADDR 0x20
#define DEFAULT_RQADDR 0x61

/*
 * Give the remote end this much time (100ms) to send a handshake
 * character before we give up.
 */
#define DM_HANDSHAKE_TIME 100000

/*
 * We have 4 bytes more for the header and then the ending checksum
 * above the KCS header size we get/send to the upper layer.
 */
#define DM_HEADER_OVERHEAD_BYTES 5

/*
 * We have start, stop, 4 bytes more for the header beyond the header
 * we get, and the checksum.  The start and stop do not need escaping,
 * thus do not need doubling, but the rest do.  So we have:
 *    2 + (5 * 2) = 12
 */
#define DM_OVERHEAD_BYTES (2 + (DM_HEADER_OVERHEAD_BYTES * 2))

struct ipmi_serial_codec_data {
	struct ipmi_serial_info *info;

	unsigned char xmit_chars[IPMI_MAX_MSG_LENGTH * 2 + DM_OVERHEAD_BYTES];
	unsigned int  xmit_chars_len;
	unsigned int  xmit_chars_pos;

	unsigned char seqno;
	unsigned int seqnum_table[0x40];

	unsigned char xmit_msg[IPMI_MAX_MSG_LENGTH + DM_HEADER_OVERHEAD_BYTES];
	unsigned int  xmit_msg_len;

	unsigned char recv_msg[IPMI_MAX_MSG_LENGTH + DM_HEADER_OVERHEAD_BYTES];
	unsigned int  recv_msg_len;
	int           in_recv_msg;
	int           in_escape;
	int           recv_msg_too_many;

	spinlock_t lock;

	/*
	 * Used to time receiving a handshake.  We give the remote end
	 * so much time to send the handshake before we give up and
	 * just send.
	 */
	int handshake_time;

	struct tm_options options;
};

#define SUN_MFG_ID		0x00002a
#define SUN_CP3020_PROD_ID	0x0bcc

/*
 * BMCs that support using the ASCII escape char (0x1b) as attention.
 */
static struct { unsigned int mfg_id, prod_id; } attn_bmcs[] =
{
	{ 0, 0 }
};

/*
 * BMCs that use a special escape sequence (0xaa <char>) as attention.
 */
static struct { unsigned int mfg_id, prod_id, val; } eattn_bmcs[] =
{
	{ SUN_MFG_ID,		SUN_CP3020_PROD_ID,		0x47 },
	{ 0, 0 }
};

static void check_devid_options(struct ipmi_serial_codec_data *data,
				struct ipmi_device_id *id)
{
	int i;

	if (!data->options.attn_forced) {
		for (i = 0; attn_bmcs[i].mfg_id != 0; i++) {
			if ((id->manufacturer_id == attn_bmcs[i].mfg_id)
			    && (id->product_id == attn_bmcs[i].prod_id)) {
				data->options.attn = 1;
				break;
			}
		}
	}
	if (!data->options.eattn_forced) {
		for (i = 0; eattn_bmcs[i].mfg_id != 0; i++) {
			if ((id->manufacturer_id == eattn_bmcs[i].mfg_id)
			    && (id->product_id == eattn_bmcs[i].prod_id)) {
				data->options.eattn = 1;
				data->options.eattn_char = eattn_bmcs[i].val;
				break;
			}
		}
	}
}

static int check_options(struct ipmi_serial_codec_data *data,
			 const char *options)
{
	const char *s;
	char *optval;
	int  optval_len;
	char *next;
	int  len;
	char *end;

	for (s = options; s; s = next) {
		next = strchr(s, '+');
		if (next) {
			len = next - s;
			next++;
		} else
			len = strlen(s);

		optval = strchr(s, '=');
		if (next && optval >= next)
			optval = NULL;
		if (optval) {
			len = optval - s;
			optval++;
			if (next) {
				optval_len = next - optval;
				next++;
			} else
				optval_len = strlen(optval);
		} else
			optval_len = 0;
		end = NULL;
		if (strncmp(ATTN_OPTION, s, len) == 0) {
			data->options.attn = 1;
			data->options.attn_forced = 1;
		} else if (strncmp("no" ATTN_OPTION, s, len) == 0) {
			data->options.attn = 0;
			data->options.attn_forced = 1;
		} else if (strncmp(EATTN_OPTION, s, len) == 0) {
			data->options.eattn = 1;
			data->options.eattn_forced = 1;
			if (optval)
				data->options.eattn_char
					= simple_strtoul(optval, &end, 0);
			else
				data->options.eattn_char = 0x47;
		} else if (strncmp("no" EATTN_OPTION, s, len) == 0) {
			data->options.attn = 0;
			data->options.attn_forced = 1;
		} else if (strncmp(RQADDR_OPTION, s, len) == 0) {
			if (optval)
				data->options.rqaddr
					= simple_strtoul(optval, &end, 0);
			else {
				printk(KERN_WARNING PFX
				       "rqa option given without value\n");
				return -EINVAL;
			}
		} else if (strncmp(RSADDR_OPTION, s, len) == 0) {
			if (optval)
				data->options.rsaddr
					= simple_strtoul(optval, &end, 0);
			else {
				printk(KERN_WARNING PFX
				       "rsa option given without value\n");
				return -EINVAL;
			}
		} else {
			printk(KERN_WARNING PFX "Unknown options: %s\n",
			       options);
			return -EINVAL;
		}

		if (optval && !end) {
			printk(KERN_WARNING PFX "No value needed at: %s\n", s);
			return -EINVAL;
		}
		if (optval && end != (optval + optval_len)) {
			printk(KERN_WARNING PFX "Invalid value for: %s\n", s);
			return -EINVAL;
		}
	}

	return 0;
}

static int sd_add_options(struct ipmi_serial_codec_data *data,
			  struct seq_file *m)
{
	char pfx = ',';

	if (data->options.attn) {
		seq_printf(m, "%cattn", pfx);
		pfx = '+';
	}
	if (data->options.eattn) {
		seq_printf(m, "%ceattn", pfx);
		if (data->options.eattn_char != 0x47)
			seq_printf(m, "=0x%x", data->options.eattn_char);
		pfx = '+';
	}
	if (data->options.rqaddr != DEFAULT_RQADDR) {
		seq_printf(m, "%crqa=0x%x", pfx, data->options.rqaddr);
		pfx = '+';
	}
	if (data->options.rsaddr != DEFAULT_RSADDR)
		seq_printf(m, "%crsa=0x%x", pfx, data->options.rsaddr);

	return 0;
}

static unsigned char ipmb_checksum(unsigned char *data, int size)
{
	unsigned char csum = 0;

	for (; size > 0; size--, data++)
		csum += *data;

	return -csum;
}

#define DM_START_CHAR		0xA0
#define DM_STOP_CHAR		0xA5
#define DM_PACKET_HANDSHAKE	0xA6
#define DM_DATA_ESCAPE_CHAR	0xAA

static void format_msg(struct ipmi_serial_codec_data *data,
		       const unsigned char *msg, unsigned int msg_len)
{
	unsigned int i;
	unsigned int len = 0;
	unsigned char *c = data->xmit_chars;

	c[len++] = 0xA0;
	for (i = 0; i < msg_len; i++) {
		switch (msg[i]) {
		case 0xA0:
			c[len++] = 0xAA;
			c[len++] = 0xB0;
			break;

		case 0xA5:
			c[len++] = 0xAA;
			c[len++] = 0xB5;
			break;

		case 0xA6:
			c[len++] = 0xAA;
			c[len++] = 0xB6;
			break;

		case 0xAA:
			c[len++] = 0xAA;
			c[len++] = 0xBA;
			break;

		case 0x1B:
			c[len++] = 0xAA;
			c[len++] = 0x3B;
			break;

		default:
			c[len++] = msg[i];
		}

	}
	c[len++] = 0xA5;
	data->xmit_chars_len = len;
	data->xmit_chars_pos = 0;
}

static void handle_recv_msg(struct ipmi_serial_codec_data *data)
{
	unsigned int seq;
	unsigned char *m = data->recv_msg;
	unsigned int len = data->recv_msg_len;
	int          i;

	if (len < 8) {
		/* Messages must be at least 8 bytes to be valid. */
		ipmi_serial_ll_protocol_violation(data->info);
		return;
	}

	/* Note that this validates both checksums in one shot. */
	if (ipmb_checksum(m, len) != 0) {
		ipmi_serial_ll_checksum_error(data->info);
		return;
	}
	len--; /* Remove the checksum */

	seq = m[4] >> 2;

	/* Pull the rsLun and the NetFN together. */
	m[0] = (m[1] & 0xfc) | (m[4] & 0x3);
	/* Now the rest of the data */
	for (i = 1; i < (len - 4); i++)
		m[i] = m[i + 4];
	len -= 4;

	ipmi_serial_ll_recv(data->info, m, len, data->seqnum_table[seq]);
}

static void try_to_send_data(struct ipmi_serial_codec_data *data)
{
	unsigned char *c;
	unsigned int  left;
	unsigned int  sent;

	if (!data->xmit_chars_len)
		return;

	if (data->handshake_time > 0)
		return;

	c = data->xmit_chars + data->xmit_chars_pos;
	left = data->xmit_chars_len - data->xmit_chars_pos;
	sent = ipmi_serial_ll_xmit(data->info, c, left);
	if (sent == left) {
		/* We are done with this message. */
		if (data->xmit_msg_len) {
			/* Send the next message we have waiting. */
			format_msg(data, data->xmit_msg, data->xmit_msg_len);
			data->xmit_msg_len = 0;
		} else {
			/* Nothing to do. */
			data->xmit_chars_len = 0;
		}
		/* Wait for the handshake. */
		data->handshake_time = DM_HANDSHAKE_TIME;
	} else {
		data->xmit_chars_pos += sent;
	}
}

static int sd_setup_termios(struct ktermios *t)
{
	/* Nothing to do, the default is fine. */
	return 0;
}

static int sd_init(struct ipmi_serial_codec_data *data,
		   struct ipmi_serial_info *info,
		   const char *options)
{
	memset(data, 0, sizeof(*data));
	spin_lock_init(&data->lock);
	data->info = info;

	/* Pick some good defaults for header addresses. */
	data->options.rsaddr = DEFAULT_RSADDR;
	data->options.rqaddr = DEFAULT_RQADDR;

	return check_options(data, options);
}

static void sd_cleanup(struct ipmi_serial_codec_data *data)
{
	/* Nothing to do. */
}

static int sd_size(void)
{
	return sizeof(struct ipmi_serial_codec_data);
}

static void sd_handle_char(struct ipmi_serial_codec_data *data,
			   unsigned char ch)
{
	unsigned int len = data->recv_msg_len;
	unsigned long flags;

	switch (ch) {
	case DM_START_CHAR:
		if (data->in_recv_msg)
			ipmi_serial_ll_protocol_violation(data->info);
		data->in_recv_msg = 1;
		data->recv_msg_len = 0;
		data->recv_msg_too_many = 0;
		data->in_escape = 0;
		break;

	case DM_STOP_CHAR:
		if (!data->in_recv_msg)
			ipmi_serial_ll_protocol_violation(data->info);
		else if (data->in_escape) {
			data->in_recv_msg = 0;
			ipmi_serial_ll_protocol_violation(data->info);
		} else if (data->recv_msg_too_many) {
			data->in_recv_msg = 0;
			ipmi_serial_ll_protocol_violation(data->info);
		} else {
			data->in_recv_msg = 0;
			handle_recv_msg(data);
		}
		data->in_escape = 0;
		break;

	case DM_PACKET_HANDSHAKE:
		/* Got a handshake, we can send now. */
		spin_lock_irqsave(&data->lock, flags);
		data->handshake_time = 0;
		try_to_send_data(data);
		spin_unlock_irqrestore(&data->lock, flags);
		data->in_escape = 0;
		break;

	case DM_DATA_ESCAPE_CHAR:
		data->in_escape = 1;
		break;

	case 0x1b:
		if (data->options.attn)
			ipmi_serial_ll_attn(data->info);
		break;

	default:
		if (!data->in_recv_msg)
			/* Ignore characters outside of messages. */
			break;

		if (data->in_escape) {
			data->in_escape = 0;
			if (data->options.eattn
					&& (ch == data->options.eattn_char)) {
				ipmi_serial_ll_attn(data->info);
				goto out;
			}

			switch (ch) {
			case 0xB0:
				ch = DM_START_CHAR;
				break;
			case 0xB5:
				ch = DM_STOP_CHAR;
				break;
			case 0xB6:
				ch = DM_PACKET_HANDSHAKE;
				break;
			case 0xBA:
				ch = DM_DATA_ESCAPE_CHAR;
				break;
			case 0x3B:
				ch = 0x1b;
				break;
			default:
				ipmi_serial_ll_protocol_violation(data->info);
				data->recv_msg_too_many = 1;
				return;
			}
		}

		if (!data->recv_msg_too_many) {
			if (len >= sizeof(data->recv_msg)) {
				data->recv_msg_too_many = 1;
				break;
			}

			data->recv_msg[len] = ch;
			data->recv_msg_len++;
		}
		break;
	}
 out:
	return;
}

static int sd_send_msg(struct ipmi_serial_codec_data *data,
		       const unsigned char *msg, unsigned int msg_len,
		       unsigned int seq)
{
	unsigned long flags;
	int           rv = 0;
	unsigned char *m;
	unsigned int  i, j;
	unsigned char seqno;

	if (msg_len > IPMI_MAX_MSG_LENGTH)
		return -EFBIG;
	if (msg_len < 2)
		return -EINVAL;

	spin_lock_irqsave(&data->lock, flags);
	if (data->xmit_msg_len) {
		/* Something is still waiting to be sent. */
		rv = -EBUSY;
		goto out_unlock;
	}

	i = 0;
	m = data->xmit_msg;
	m[i++] = data->options.rsaddr;	/* rsAddr */
	m[i++] = msg[0];		/* NetFN/rsLUN */
	m[i++] = ipmb_checksum(m, 2);	/* checksum1 */
	m[i++] = data->options.rqaddr;	/* rqAddr */

	seqno = data->seqno;
	data->seqno = (data->seqno + 1) & 0x3f;
	data->seqnum_table[seqno] = seq;
	m[i++] = seqno << 2;		/* seqno/rqLUN */
	m[i++] = msg[1];		/* cmd */
	for (j = 2; j < msg_len; j++)
		m[i++] = msg[j];	/* data */
	m[i] = ipmb_checksum(m + 3, i - 3); /* checksum2 */
	i++;

	if (data->xmit_chars_len == 0) {
		/* Transmit queue is empty, just format it now to go. */
		format_msg(data, m, i);
		try_to_send_data(data);
	} else {
		/* Save it for after the transmit queue emptying. */
		data->xmit_msg_len = i;
	}

 out_unlock:
	spin_unlock_irqrestore(&data->lock, flags);
	return rv;
}

static void sd_tx_ready(struct ipmi_serial_codec_data *data)
{
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);
	try_to_send_data(data);
	spin_unlock_irqrestore(&data->lock, flags);
}

static void sd_timer_tick(struct ipmi_serial_codec_data *data,
			  unsigned int time_since_last)
{
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);
	if (data->handshake_time > 0) {
		data->handshake_time -= time_since_last;
		if (data->handshake_time <= 0)
			try_to_send_data(data);
	}
	spin_unlock_irqrestore(&data->lock, flags);
}

static unsigned int sd_capabilities(struct ipmi_serial_codec_data *data)
{
	unsigned int c = (IPMI_SERIAL_SUPPORTS_GET_FLAGS
			  | IPMI_SERIAL_SUPPORTS_EVENT_BUFFER);
	if ((data->options.attn) || (data->options.eattn))
		c |= IPMI_SERIAL_HAS_ATTN;
	else
		c |= IPMI_SERIAL_NEEDS_GET_FLAGS_POLLING;
	return c;
}

static struct ipmi_serial_codec sd_codec = {
	.owner = THIS_MODULE,
	.name  = "Direct",

	.capabilities	= sd_capabilities,
	.setup_termios	= sd_setup_termios,
	.init		= sd_init,
	.cleanup	= sd_cleanup,
	.size		= sd_size,
	.send_msg	= sd_send_msg,
	.handle_char	= sd_handle_char,
	.check_dev_id   = check_devid_options,
	.timer_tick     = sd_timer_tick,
	.tx_ready	= sd_tx_ready,
	.add_options	= sd_add_options
};

static int __init init_ipmi_serial_direct_codec(void)
{
	return ipmi_serial_codec_register(&sd_codec);
}

module_init(init_ipmi_serial_direct_codec);

static void __exit exit_ipmi_serial_direct_codec(void)
{
	ipmi_serial_codec_unregister(&sd_codec);
}
module_exit(exit_ipmi_serial_direct_codec);
MODULE_LICENSE("GPL");
