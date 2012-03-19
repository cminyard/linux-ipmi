/*
 * ipmi_serial_sm.h
 *
 * State machine interface for low-level IPMI serial interface driver
 * state machines.  This code is the interface between
 * the ipmi_serial code and the supported codec(s)
 *
 * Author: MontaVista Software, Inc.
 *         dgriego@mvista.com
 *         cminyard@mvista.com
 *         source@mvista.com
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

#ifndef _IPMI_SERIAL_SM_H
#define _IPMI_SERIAL_SM_H

#include <linux/termios.h>
#include <linux/ipmi_smi.h>
#include <linux/seq_file.h>

/*
 * This is the interface between the IPMI serial code and the codecs
 * themselves.
 *
 * The IPMI serial code handles the interface to the IPMI message
 * handler, the queuing of messages to transmit, the configuration of
 * the serial port, and dealing directly with the serial port.  It also
 * requests and handles interface flags.
 *
 * The lower layer is concerned with transmission of one message at a
 * time, the handling of received bytes to assemble into full received
 * messages.
 *
 * All messages are formatted in the same manner as a standard KCS
 * command:
 *   send: [(netfn << 2) | lun] [command] [data1] ...
 *   recv: [(netfn << 2) | lun] [command] [completion code] [data1] ...
 *
 * Note that some commands may have a special meaning:
 *   IPMI_GET_MSG_FLAGS_CMD - If the BMC does not handle this and transfers
 *     this information via a different mechanism, the lower layer may
 *     return -ENOTSUP for a send of this command.
 *   IPMI_READ_EVENT_MSG_BUFFER_CMD - Used to report a received event.
 *     The lower layer may send these asynchronously to the upper layer
 *     if it has a different mechanism to receive these.
 *   IPMI_GET_MSG_CMD - Used to report a received command.
 *     The lower layer may send these asynchronously to the upper layer
 *     if it has a different mechanism to receive these.
 * The lower layer may not asynchronously send anything else with
 * the async function except what is allowed above.
 *
 * See the end of this file for expected call flows.
 */

/*
 * This is defined by the codecs themselves, it is an opaque
 * data type for them to use.
 */
struct ipmi_serial_codec_data;

/*
 * Opaque data used in the serial interface that the state machines
 * must supply to the serial interface in callbacks.
 */
struct ipmi_serial_info;

/*
 * Handlers for the IPMI serial state machine.
 */
struct ipmi_serial_codec {
	struct module *owner;
	char          *name;

#define IPMI_SERIAL_NEEDS_GET_FLAGS_POLLING	0x00000001
#define IPMI_SERIAL_SUPPORTS_GET_FLAGS		0x00000002
#define IPMI_SERIAL_SUPPORTS_EVENT_BUFFER	0x00000004
#define IPMI_SERIAL_HAS_ATTN			0x00000008
	unsigned int (*capabilities)(struct ipmi_serial_codec_data *data);

	/*
	 * Set up the basic termios for the serial port for this
	 * interface type.  The upper layer will call this when
	 * setting up the serial port.  The upper layer will change
	 * some of these values based upon information from the user,
	 * primarily baud, number of bits/char, stop bits, and parity.
	 */
	int (*setup_termios)(struct ktermios *t);

	/*
	 * Initialize the data structure for the codec.  The info must
	 * be supplied to callbacks.  Note that this shouldn't send or
	 * receive anything, it should just initialize.  The user may
	 * pass options in on the serial configuration command line,
	 * these will be passed to this function.
	 */
	int (*init)(struct ipmi_serial_codec_data *data,
		    struct ipmi_serial_info *info,
		    const char *options);

	/*
	 * Start processing.  When this is done,
	 * ipmi_serial_ll_init_complete() must be called by the codec.
	 * This function may send and receive data.  If this is NULL,
	 * the codec is assumed to already be fully operational after
	 * the init call.
	 */
	 int (*start)(struct ipmi_serial_codec_data *data);

	/*
	 * Cleanup anything that needs to be cleaned up before the codec
	 * is freed.
	 */
	void (*cleanup)(struct ipmi_serial_codec_data *data);

	/*
	 * Return the size of the ipmi_serial_codec_data structure in
	 * bytes.  The upper layer will use this to allocate the data
	 * structure for the codec.
	 */
	int (*size)(void);

	/*
	 * Send a message.  Only one message send at a time is
	 * allowed.  When the send is complete the lower layer must
	 * call ipmi_serial_ll_send_complete().  May return an error
	 * code if there is already a message in progress.  The response
	 * that comes back will have the given seq value in the receive
	 * call.
	 */
	int (*send_msg)(struct ipmi_serial_codec_data *data,
			const unsigned char *msg, unsigned int msg_len,
			unsigned int seq);

	/*
	 * Handle a single received character from the serial port.
	 * Note that this is called without the serial code claiming a
	 * lock, but will be single-threaded.
	 */
	void (*handle_char)(struct ipmi_serial_codec_data *data,
			    unsigned char ch);

	/*
	 * Called when the transmitter has space to take more
	 * characters.  If ipmi_serial_xmit_data returns less queued
	 * characters than asked to transmit, the lower layer should
	 * wait for this call and then send more data.
	 */
	void (*tx_ready)(struct ipmi_serial_codec_data *data);


	/*
	 * If not-NULL, this will be called periodically.  The time
	 * since the last call will be passed, time is in microseconds.
	 */
	void (*timer_tick)(struct ipmi_serial_codec_data *data,
			   unsigned int time_since_last);

	/*
	 * Once the device id is fetched by the main serial code, this
	 * will be called if it is not NULL.  This allows the codec to
	 * enable certain hacks for certain machines.
	 */
	void (*check_dev_id)(struct ipmi_serial_codec_data *data,
			     struct ipmi_device_id *dev_id);

	/*
	 * Print the current options in use by the codec.  If there are
	 * options printed, the codec should prepend a ','.
	 */
	int (*add_options)(struct ipmi_serial_codec_data *data,
			   struct seq_file *m);

	/*
	 * Used by the serial interface, the codec shouldn't touch
	 * anything below.
	 */
	struct list_head link;
};

/*
 * Called by the lower layer when initialization is complete.  Pass
 * in zero as the slave_addr if you are unable to compute it.
 */
extern void ipmi_serial_ll_init_complete(struct ipmi_serial_info *info,
					 unsigned char slave_addr,
					 int err);

/*
 * Called by the lower layer when it needs to send some data.  Returns the
 * actual number of bytes queued for transmit.
 */
extern unsigned int ipmi_serial_ll_xmit(struct ipmi_serial_info *info,
					const unsigned char *data,
					unsigned int len);

/*
 * Called by the lower layer when it detects that message flags are
 * available.
 */
extern void ipmi_serial_ll_attn(struct ipmi_serial_info *info);

/*
 * Called by the lower layer when a full message response is received.
 * The seq will be set to the value supplied to the state machine
 * at send time.
 */
extern void ipmi_serial_ll_recv(struct ipmi_serial_info *info,
				const unsigned char *data,
				unsigned int len,
				unsigned int seq);

/*
 * Called when the lower layer receives an async message (assuming it
 * has a different way to do this than normal flag handling) These may
 * be received commands or events based upon the rules specified
 * above.
 */
extern void ipmi_serial_ll_async(struct ipmi_serial_info *info,
				 const unsigned char *data,
				 unsigned int len);

/*
 * Used to report various low-level errors.
 */
extern void ipmi_serial_ll_protocol_violation(struct ipmi_serial_info *info);
extern void ipmi_serial_ll_checksum_error(struct ipmi_serial_info *info);

/*
 * These functions allow codecs to register and unregister with the serial
 * system interface layer.
 */
extern int ipmi_serial_codec_register(struct ipmi_serial_codec *codec);
extern void ipmi_serial_codec_unregister(struct ipmi_serial_codec *codec);

/*
 * Expected flow how how things happen on this interface:
 *
 *     Upper Layer                                  Lower Layer
 *     	   |   	       	       	       			|
 * 	   |----------------init----------------------->|
 * 	   |			       			|
 *     	   |<---------------xmit------------------------|
 * 	   |			       			|
 *     	   |-----------handle_char--------------------->|
 *     	   |-----------handle_char--------------------->|
 *     	   |-----------handle_char--------------------->|
 *     	   |   	       		       			|
 *     	   |<------------init_complete------------------|
 * 	   |			       			|
 *     	   |--------------send_msg--------------------->|
 * 	   |			       			|
 *     	   |<-----------xmit (return less)--------------|
 * 	   |			       	  		|
 *     	   |------------tx_ready----------------------->|
 * 	   |			       			|
 *     	   |<---------------xmit------------------------|
 * 	   |			       	     		|
 *     	   |<-------------send_complete-----------------|
 * 	   |			       	     		|
 *     	   |-----------handle_char--------------------->|
 *     	   |-----------handle_char--------------------->|
 *     	   |-----------handle_char--------------------->|
 * 	   |			       	     		|
 *     	   |<---------------recv------------------------|
 * 	   |			       	     		|
 *     	   |-----------handle_char--------------------->|
 * 	   |			       	     		|
 *     	   |<---------------attn------------------------|
 * 	   |			       	     		|
 *     	   |--------------send_msg(GET_FLAGS)---------->|
 * 	   |			       	     		|
 *     	   |<---------------xmit------------------------|
 * 	   |			       	     		|
 *     	   |<-------------send_complete-----------------|
 * 	   |			       	     		|
 *     	   |-----------handle_char--------------------->|
 *     	   |-----------handle_char--------------------->|
 *     	   |-----------handle_char--------------------->|
 * 	   |			       	     		|
 *     	   |<---------------recv------------------------|
 * 	   |			       			|
 *     	   |--------------cleanup---------------------->|
 * 	   |			       			|
 */
#endif /* _IPMI_SERIAL_SM_H */
