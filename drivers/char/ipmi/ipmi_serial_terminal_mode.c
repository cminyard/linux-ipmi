/*
 * ipmi_serial_terminal_mode.c
 * Serial interface encoder and decoder routines for terminal mode
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
#include <linux/ctype.h>

#define PFX "ipmi_serial_terminal_mode: "

struct tm_options {
#define PIGEONPOINT_OPTION "pp"
	int pigeonpoint;
	int pigeonpoint_forced; /* Set by command line. */
#define PIGEONPOINT_INVIANA_OPTION "ppInvIANA"
	int pigeonpoint_iana_bad;
	int pigeonpoint_iana_bad_forced; /* Set by command line. */
#define ATTN_OPTION "attn"
	int attn;
	int attn_forced; /* Set by command line. */
	unsigned char attn_char;
};

/*
 * Three bytes for every character plus the bridge/seqno field and add
 * the [, ], and newline and perhaps an extra space
 */
#define TM_MAX_CHARS_SIZE (((IPMI_MAX_MSG_LENGTH + 1) * 3) + 4)

/*
 * Give the remote end this much time (100ms) to send a response
 * before we give up and retry the given number of times.
 */
#define TM_HANDSHAKE_TIME    100000
#define TM_HANDSHAKE_RETRIES 5


struct ipmi_serial_codec_data {
	struct ipmi_serial_info *info;

	unsigned char xmit_chars[TM_MAX_CHARS_SIZE];
	unsigned int  xmit_chars_len;
	unsigned int  xmit_chars_pos;

	unsigned char recv_chars[TM_MAX_CHARS_SIZE];
	unsigned int  recv_chars_len;
	int           recv_chars_too_many;

	unsigned char seqno;
	unsigned int seqnum_table[0x40];

	unsigned char xmit_msg[IPMI_MAX_MSG_LENGTH];
	unsigned int  xmit_msg_len;
	unsigned int  xmit_msg_seq;
	unsigned char recv_msg[IPMI_MAX_MSG_LENGTH];
	unsigned int  recv_msg_len;

	char echo_on;
	char pp_iana_retried;

	spinlock_t lock;

	/*
	 * Handle receipt of a message.  This is a function var so it
	 * can be replaced at init time and when special handling is
	 * required for messages.
	 */
	void (*recv_msg_handler)(struct ipmi_serial_codec_data *data,
				 const unsigned char *msg,
				 unsigned int len,
				 unsigned int seq);

	struct ipmi_device_id id;
	struct tm_options options;

	/*
	 * Used to time initialization messages.
	 */
	int handshake_time;
	int handshake_retries_left;
	int handshake_done;
};

#define MOTOROLA_MFG_ID			0x0000a1
#define MOTOROLA_ATCA_F101_PROD_ID	0x0051
#define MOTOROLA_ATCA_6101_PROD_ID	0x0053
/* Table of BMCs that support the PigeonPoint echo handling. */
static struct { unsigned int mfg_id, prod_id; } pp_bmcs[] =
{
	{ MOTOROLA_MFG_ID,	MOTOROLA_ATCA_F101_PROD_ID },
	{ MOTOROLA_MFG_ID,	MOTOROLA_ATCA_6101_PROD_ID },
	{ 0, 0 }
};

static struct { unsigned int mfg_id, prod_id, val; } attn_bmcs[] =
{
	{ MOTOROLA_MFG_ID,	MOTOROLA_ATCA_F101_PROD_ID,	0x07 },
	{ MOTOROLA_MFG_ID,	MOTOROLA_ATCA_6101_PROD_ID,	0x07 },
	{ 0, 0 }
};

static void check_devid_options(struct ipmi_serial_codec_data *data)
{
	int i;

	if (!data->options.pigeonpoint_forced) {
		for (i = 0; pp_bmcs[i].mfg_id != 0; i++) {
			if ((data->id.manufacturer_id == pp_bmcs[i].mfg_id)
			    && (data->id.product_id == pp_bmcs[i].prod_id)) {
				data->options.pigeonpoint = 1;
				break;
			}
		}
	}
	if (!data->options.attn_forced) {
		for (i = 0; attn_bmcs[i].mfg_id != 0; i++) {
			if ((data->id.manufacturer_id == attn_bmcs[i].mfg_id)
			    && (data->id.product_id == attn_bmcs[i].prod_id)) {
				data->options.attn = 1;
				data->options.attn_char = attn_bmcs[i].val;
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
		if (strncmp(PIGEONPOINT_OPTION, s, len) == 0) {
			data->options.pigeonpoint = 1;
			data->options.pigeonpoint_forced = 1;
		} else if (strncmp("no" PIGEONPOINT_OPTION, s, len) == 0) {
			data->options.pigeonpoint = 0;
			data->options.pigeonpoint_forced = 1;
		} else if (strncmp(PIGEONPOINT_INVIANA_OPTION, s, len) == 0) {
			data->options.pigeonpoint_iana_bad = 1;
			data->options.pigeonpoint_iana_bad_forced = 1;
		} else if (strncmp("no" PIGEONPOINT_INVIANA_OPTION, s, len)
									== 0) {
			data->options.pigeonpoint_iana_bad = 0;
			data->options.pigeonpoint_iana_bad_forced = 1;
		} else if (strncmp(ATTN_OPTION, s, len) == 0) {
			data->options.attn = 1;
			data->options.attn_forced = 1;
			if (optval)
				data->options.attn_char
					= simple_strtoul(optval, &end, 0);
			else
				/* default to bell */
				data->options.attn_char = 0x07;
		} else if (strncmp("no" ATTN_OPTION, s, len) == 0) {
			data->options.attn = 0;
			data->options.attn_forced = 1;
		} else {
			printk(KERN_WARNING PFX "Unknown options: %s\n",
			       options);
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

static int tm_add_options(struct ipmi_serial_codec_data *data,
			  struct seq_file *m)
{
	char pfx = ',';

	if (data->options.pigeonpoint) {
		seq_printf(m, "%cpp", pfx);
		pfx = '+';
	}
	if (data->options.attn) {
		seq_printf(m, "%cattn", pfx);
		if (data->options.attn_char != 0x07)
			seq_printf(m, "=0x%x", data->options.attn_char);
	}

	return 0;
}

static unsigned char hex2char[16] = {
	'0', '1', '2', '3', '4', '5', '6', '7',
	'8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
};

static void format_msg(struct ipmi_serial_codec_data *data,
		       const unsigned char *msg, unsigned int msg_len,
		       unsigned int seq)
{
	int i;
	int len;
	unsigned char *c = data->xmit_chars;
	unsigned char seqno;

	len = 0;
	c[len] = '[';
	len++;

	c[len] = hex2char[msg[0] >> 4];
	len++;
	c[len] = hex2char[msg[0] & 0xf];
	len++;

	/*
	 * Insert the sequence number and bridge bits.  Bridge bits
	 * are always zero.
	 */
	seqno = data->seqno;
	data->seqno = (data->seqno + 1) & 0x3f;
	data->seqnum_table[seqno] = seq;
	seqno <<= 2;
	c[len] = hex2char[seqno >> 4];
	len++;
	c[len] = hex2char[seqno & 0xf];
	len++;

	/* Now the rest of the message. */
	for (i = 1; ; ) {
		c[len] = hex2char[msg[i] >> 4];
		len++;
		c[len] = hex2char[msg[i] & 0xf];
		len++;
		i++;
		if (i == msg_len)
			break;
		c[len] = ' ';
		len++;
	}
	c[len] = ']';
	len++;
	c[len] = 0x0a;
	len++;
	c[len] = 0x0d; /* We definitely have room for this byte. */
	len++;

	data->xmit_chars_pos = 0;
	data->xmit_chars_len = len;
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
 * Called when the ']' is seen, the leading '[' is removed, too.  We
 * get this with a leading space and no more than one space between
 * items.
 */
static int unformat_msg(struct ipmi_serial_codec_data *data)
{
	unsigned char *r = data->recv_chars;
	unsigned char *o = data->recv_msg;
	unsigned int len = data->recv_chars_len;
	unsigned int p = 0;
	unsigned int i = 0;
	int          rv;

	if (isspace(r[p]))
		p++;
	while (p < len) {
		if (i >= sizeof(data->recv_msg))
			return -EFBIG;
		if (p >= len)
			return -EINVAL;
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
		if (isspace(r[p]))
			p++;
	}
	data->recv_msg_len = i;
	return 0;
}

static void normal_recv(struct ipmi_serial_codec_data *data,
			const unsigned char *msg,
			unsigned int len,
			unsigned int seq)
{
	if (((msg[0] >> 2) & 1) == 0)
		/*
		 * If the bottom bit of the message is zero, then it
		 * is a command and most likely an echo.
		 */
		return;

	if (len < 3) {
		/* Responses must be at least 3 bytes after processing. */
		ipmi_serial_ll_protocol_violation(data->info);
		return;
	}

	ipmi_serial_ll_recv(data->info, msg, len, seq);
}


static void handle_recv_msg(struct ipmi_serial_codec_data *data)
{
	unsigned int seq;
	unsigned char *m = data->recv_msg;
	unsigned int len = data->recv_msg_len;
	int          i;

	if (len < 3) {
		/* Messages must be at least 3 bytes */
		ipmi_serial_ll_protocol_violation(data->info);
		return;
	}

	seq = m[1] >> 2;

	/* Now remove the seq# */
	for (i = 1; i < len-1; i++)
		m[i] = m[i+1];
	len--;

	data->recv_msg_handler(data, m, len, data->seqnum_table[seq]);
}

static void try_to_send_data(struct ipmi_serial_codec_data *data)
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
		if (data->xmit_msg_len) {
			/* Send the next message we have waiting. */
			format_msg(data, data->xmit_msg, data->xmit_msg_len,
				   data->xmit_msg_seq);
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

static void finish_init(struct ipmi_serial_codec_data *data, int err)
{
	ipmi_serial_ll_init_complete(data->info, 0, err);
}

/*
 * Handling for pigeonpoint-specific codec, we have special ways to
 * turn on/off echo.
 */
#define PP_OEM_CHARS		0x0a, 0x40, 0x00
#define PP_GET_SERIAL_INTF_CMD	0x01
#define PP_SET_SERIAL_INTF_CMD	0x02
#define PP_GET_SER_INTF_SIZE 6
static unsigned char pp_get_ser_intf_cmd[PP_GET_SER_INTF_SIZE]
	 = {IPMI_NETFN_OEM_REQUEST << 2, PP_GET_SERIAL_INTF_CMD,
	    PP_OEM_CHARS, 0x01};
#define PP_SET_SER_INTF_SIZE 7
static unsigned char pp_set_ser_intf_cmd[PP_SET_SER_INTF_SIZE]
	 = {IPMI_NETFN_OEM_REQUEST << 2, PP_SET_SERIAL_INTF_CMD,
	    PP_OEM_CHARS, 0x01, 0x00};

static void handle_pp_set_ser(struct ipmi_serial_codec_data *data,
			      const unsigned char *msg,
			      unsigned int len,
			      unsigned int seq)
{
	unsigned long flags;

	if (((msg[0] >> 2) & 1) == 0)
		/* Got the echo */
		return;

	if (((msg[0] >> 2) != IPMI_NETFN_OEM_RESPONSE)
	    || (msg[1] != PP_SET_SERIAL_INTF_CMD))
		/* Not what we were expecting */
		return;

	spin_lock_irqsave(&data->lock, flags);
	if (data->handshake_done) {
		spin_unlock_irqrestore(&data->lock, flags);
		return;
	}

	if (msg[2] != 0) {
		data->handshake_done = 1;
		spin_unlock_irqrestore(&data->lock, flags);
		printk(KERN_WARNING PFX
		       "Error setting pigeonpoint serial parms: 0x%x\n",
		       msg[2]);
		finish_init(data, -EINVAL);
	}

	data->handshake_done = 1;
	data->recv_msg_handler = normal_recv;
	spin_unlock_irqrestore(&data->lock, flags);
	finish_init(data, 0);
}

static void pp_start_disable_echo(struct ipmi_serial_codec_data *data);

static void handle_pp_get_ser(struct ipmi_serial_codec_data *data,
			      const unsigned char *msg,
			      unsigned int len,
			      unsigned int seq)
{
	unsigned long flags;

	if (((msg[0] >> 2) & 1) == 0)
		/* Got the echo */
		return;

	if (((msg[0] >> 2) != IPMI_NETFN_OEM_RESPONSE)
	    || (msg[1] != PP_GET_SERIAL_INTF_CMD))
		/* Not what we were expecting */
		return;

	spin_lock_irqsave(&data->lock, flags);
	if (data->handshake_done) {
		spin_unlock_irqrestore(&data->lock, flags);
		return;
	}

	if (msg[2] != 0) {
		if (!data->options.pigeonpoint_iana_bad_forced &&
		    !data->pp_iana_retried) {
			/* Try swapping the IANA and see if that helps. */
			data->pp_iana_retried = 1;
			data->options.pigeonpoint_iana_bad
				= !data->options.pigeonpoint_iana_bad;
			pp_start_disable_echo(data);
			spin_unlock_irqrestore(&data->lock, flags);
			return;
		}
		data->handshake_done = 1;
		spin_unlock_irqrestore(&data->lock, flags);
		printk(KERN_WARNING PFX
		       "Error getting pigeonpoint serial parms: 0x%x\n",
		       msg[2]);
		finish_init(data, -EINVAL);
		return;
	}

	if (len < 6) {
		data->handshake_done = 1;
		spin_unlock_irqrestore(&data->lock, flags);
		printk(KERN_WARNING PFX
		       "Pigeonpoint serial parms too short: %d\n", len);
		finish_init(data, -EINVAL);
		return;
	}

	memcpy(data->xmit_msg, pp_set_ser_intf_cmd,
	       sizeof(pp_set_ser_intf_cmd));
	if (data->options.pigeonpoint_iana_bad) {
		data->xmit_msg[2] = 0x00;
		data->xmit_msg[4] = 0x0a;
	}
	data->xmit_msg[6] &= 0x7f; /* And off echo bit */

	data->recv_msg_handler = handle_pp_set_ser;
	data->handshake_time = TM_HANDSHAKE_TIME;
	data->handshake_retries_left = TM_HANDSHAKE_RETRIES;
	format_msg(data, data->xmit_msg, sizeof(pp_set_ser_intf_cmd), 0);
	data->xmit_msg_len = 0;
	try_to_send_data(data);

	spin_unlock_irqrestore(&data->lock, flags);
}


static void pp_start_disable_echo(struct ipmi_serial_codec_data *data)
{
	memcpy(data->xmit_msg, pp_get_ser_intf_cmd,
	       sizeof(pp_get_ser_intf_cmd));
	if (data->options.pigeonpoint_iana_bad) {
		data->xmit_msg[2] = 0x00;
		data->xmit_msg[4] = 0x0a;
	}

	data->recv_msg_handler = handle_pp_get_ser;
	data->handshake_time = TM_HANDSHAKE_TIME;
	data->handshake_retries_left = TM_HANDSHAKE_RETRIES;
	format_msg(data, data->xmit_msg, sizeof(pp_get_ser_intf_cmd), 0);
	data->xmit_msg_len = 0;
	try_to_send_data(data);
}


static int tm_setup_termios(struct ktermios *t)
{
	/* Nothing to do, the default is fine. */
	return 0;
}

static unsigned char devid_msg[] = { IPMI_NETFN_APP_REQUEST << 2,
				     IPMI_GET_DEVICE_ID_CMD };

static void handle_init_devid(struct ipmi_serial_codec_data *data,
			      const unsigned char *msg,
			      unsigned int len,
			      unsigned int seq)
{
	int rv;
	unsigned long flags;

	if ((len == sizeof(devid_msg))
				&& (memcmp(msg, devid_msg, len) == 0)) {
		data->echo_on = 1;
		return;
	}

	spin_lock_irqsave(&data->lock, flags);
	if (data->handshake_done) {
		spin_unlock_irqrestore(&data->lock, flags);
		return;
	}

	rv = ipmi_demangle_device_id(msg, len, &data->id);
	if (rv) {
		data->handshake_done = 1;
		spin_unlock_irqrestore(&data->lock, flags);
		printk(KERN_WARNING PFX "invalid device id: %d\n", rv);
		finish_init(data, rv);
		return;
	}

	check_devid_options(data);

	if (data->options.pigeonpoint && data->echo_on) {
		pp_start_disable_echo(data);
		spin_unlock_irqrestore(&data->lock, flags);
	} else {
		data->handshake_done = 1;
		data->recv_msg_handler = normal_recv;
		spin_unlock_irqrestore(&data->lock, flags);
		finish_init(data, 0);
	}
}

static int tm_init(struct ipmi_serial_codec_data *data,
		   struct ipmi_serial_info *info,
		   const char *options)
{
	memset(data, 0, sizeof(*data));
	spin_lock_init(&data->lock);
	data->info = info;
	return check_options(data, options);
}

static int tm_start(struct ipmi_serial_codec_data *data)
{
	unsigned long flags;

	/*
	 * Send a "get device id" to test basic function and get
	 * useful info.
	 */
	spin_lock_irqsave(&data->lock, flags);
	data->recv_msg_handler = handle_init_devid;
	data->handshake_time = TM_HANDSHAKE_TIME;
	data->handshake_retries_left = TM_HANDSHAKE_RETRIES;
	format_msg(data, devid_msg, sizeof(devid_msg), 0);
	data->xmit_msg_len = 0;
	try_to_send_data(data);
	spin_unlock_irqrestore(&data->lock, flags);
	return 0;
}

static void tm_cleanup(struct ipmi_serial_codec_data *data)
{
	/* Nothing to do. */
}

static int tm_size(void)
{
	return sizeof(struct ipmi_serial_codec_data);
}

static void tm_handle_char(struct ipmi_serial_codec_data *data,
			   unsigned char ch)
{
	unsigned int len = data->recv_chars_len;
	unsigned char *r;
	int           rv;

	if (data->options.attn && (ch == data->options.attn_char)) {
		ipmi_serial_ll_attn(data->info);
		return;
	}

	if (ch == '[') {
		/*
		 * Start of a command.  Note that if a command is
		 * already in progress (len != 0) we abort it.
		 */
		if (len != 0)
			ipmi_serial_ll_protocol_violation(data->info);

		/* Convert the leading '[' to a space, that's innocuous. */
		data->recv_chars[0] = ' ';
		data->recv_chars_len = 1;
		data->recv_chars_too_many = 0;
		return;
	}

	if (len == 0)
		/* Ignore everything outside [ ]. */
		return;

	if (ch == ']') {
		/* End of command, handle it. */
		if (data->recv_chars_too_many) {
			/* Input data overrun. */
			ipmi_serial_ll_protocol_violation(data->info);
			data->recv_chars_too_many = 0;
			data->recv_chars_len = 0;
			return;
		}
		rv = unformat_msg(data);
		data->recv_chars_len = 0;
		if (rv) {
			/* Bad input data. */
			ipmi_serial_ll_protocol_violation(data->info);
			return;
		}
		handle_recv_msg(data);
		return;
	}

	if (data->recv_chars_too_many)
		return;

	r = data->recv_chars;

	if (len >= sizeof(data->recv_chars)) {
		data->recv_chars_too_many = 1;
	} else if (isspace(r[len-1]) && isspace(ch)) {
		/* Ignore multiple spaces together. */
	} else {
		r[len] = ch;
		data->recv_chars_len++;
	}
}

static int tm_send_msg(struct ipmi_serial_codec_data *data,
		       const unsigned char *msg, unsigned int msg_len,
		       unsigned int seq)
{
	unsigned long flags;
	int           rv = 0;

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

	if (data->xmit_chars_len == 0) {
		/* Transmit queue is empty, just format it now to go. */
		format_msg(data, msg, msg_len, seq);
		try_to_send_data(data);
	} else {
		/*
		 * Get it ready to be sent later when the transmit
		 * queue empties.
		 */
		memcpy(data->xmit_msg, msg, msg_len);
		data->xmit_msg_len = msg_len;
		data->xmit_msg_seq = seq;
	}

 out_unlock:
	spin_unlock_irqrestore(&data->lock, flags);
	return rv;
}

static void tm_tx_ready(struct ipmi_serial_codec_data *data)
{
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);
	try_to_send_data(data);
	spin_unlock_irqrestore(&data->lock, flags);
}

static unsigned int tm_capabilities(struct ipmi_serial_codec_data *data)
{
	unsigned int c = (IPMI_SERIAL_SUPPORTS_GET_FLAGS
			  | IPMI_SERIAL_SUPPORTS_EVENT_BUFFER);
	if (data->options.attn)
		c |= IPMI_SERIAL_HAS_ATTN;
	else
		c |= IPMI_SERIAL_NEEDS_GET_FLAGS_POLLING;
	return c;
}

static void tm_timer_tick(struct ipmi_serial_codec_data *data,
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
	if (time_since_last <= 0) {
		data->handshake_retries_left--;
		if (data->handshake_retries_left <= 0) {
			data->handshake_done = 1;
			spin_unlock_irqrestore(&data->lock, flags);
			finish_init(data, -ETIMEDOUT);
			goto out;
		}

		/* Resend the appropriate message */
		data->handshake_time = TM_HANDSHAKE_TIME;
		if (data->recv_msg_handler == handle_init_devid) {
			format_msg(data, devid_msg, sizeof(devid_msg), 0);
			data->xmit_msg_len = 0;
			try_to_send_data(data);
		} else if (data->recv_msg_handler == handle_pp_get_ser) {
			format_msg(data, data->xmit_msg,
				   sizeof(pp_get_ser_intf_cmd), 0);
			data->xmit_msg_len = 0;
			try_to_send_data(data);
		} else if (data->recv_msg_handler == handle_pp_set_ser) {
			data->xmit_msg_len = sizeof(pp_set_ser_intf_cmd);
			format_msg(data, data->xmit_msg,
				   data->xmit_msg_len, 0);
			data->xmit_msg_len = 0;
			try_to_send_data(data);
		}
	}
 out_unlock:
	spin_unlock_irqrestore(&data->lock, flags);
 out:
	return;
}

static struct ipmi_serial_codec tm_codec = {
	.owner = THIS_MODULE,
	.name  = "TerminalMode",

	.capabilities	= tm_capabilities,
	.setup_termios	= tm_setup_termios,
	.init		= tm_init,
	.start		= tm_start,
	.cleanup	= tm_cleanup,
	.size		= tm_size,
	.send_msg	= tm_send_msg,
	.handle_char	= tm_handle_char,
	.tx_ready	= tm_tx_ready,
	.add_options	= tm_add_options,
	.timer_tick     = tm_timer_tick
};

static int __init init_ipmi_serial_tm_codec(void)
{
	return ipmi_serial_codec_register(&tm_codec);
}

module_init(init_ipmi_serial_tm_codec);

static void __exit exit_ipmi_serial_tm_codec(void)
{
	ipmi_serial_codec_unregister(&tm_codec);
}
module_exit(exit_ipmi_serial_tm_codec);
MODULE_LICENSE("GPL");
