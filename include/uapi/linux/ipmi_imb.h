/*
 * ipmi_imb.h
 *
 * Intels IMB emulation on the MontaVista IPMI interface
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

#ifndef __LINUX_IPMI_IMB_H
#define __LINUX_IPMI_IMB_H

typedef struct overlapped_s {
    unsigned long Internal;
    unsigned long InternalHigh;
    unsigned long Offset;
    unsigned long OffsetHigh;
} overlapped_t;

struct smi {
    unsigned long smi_VersionNo;
    unsigned long smi_Reserved1;
    unsigned long smi_Reserved2;
    void          *ntstatus;	      /* address of NT status block*/
    void          *lpvInBuffer;       /* address of buffer for input data*/
    unsigned long cbInBuffer;         /* size of input buffer*/
    void          *lpvOutBuffer;      /* address of output buffer*/
    unsigned long cbOutBuffer;        /* size of output buffer*/
    unsigned long *lpcbBytesReturned; /* address of actual bytes of output*/
    overlapped_t  *lpoOverlapped;     /* address of overlapped structure*/
};


#define MAX_IMB_PACKET_SIZE	33

typedef struct {
	unsigned char rsSa;
	unsigned char cmd;
	unsigned char netFn;
	unsigned char rsLun;
	unsigned char dataLength;
	unsigned char data[1];
} ImbRequest;

typedef struct {
	unsigned long flags;
#define	      		NO_RESPONSE_EXPECTED	0x01

	unsigned long timeOut;
	ImbRequest    req;
} ImbRequestBuffer;

#define MIN_IMB_REQ_BUF_SIZE	13


typedef struct {
	unsigned char cCode;
	unsigned char data[1];
} ImbResponseBuffer;

#define ASYNC_SEQ_START		0	// starting sequence number

#define MIN_IMB_RESP_BUF_SIZE	1	// a buffer without any request data
#define MAX_IMB_RESP_SIZE		(MIN_IMB_RESP_BUF_SIZE + MAX_IMB_RESPONSE_SIZE)

#define MIN_IMB_RESPONSE_SIZE	7
#define MAX_IMB_RESPONSE_SIZE	MAX_IMB_PACKET_SIZE

typedef struct {
	unsigned long timeOut;
	unsigned long lastSeq;
} ImbAsyncRequest;

typedef struct {
	unsigned long thisSeq;
	unsigned char data[1];
} ImbAsyncResponse;

#define MIN_ASYNC_RESP_SIZE		sizeof(unsigned long)
#define MAX_ASYNC_RESP_SIZE		(MIN_ASYNC_RESP_SIZE + MAX_IMB_PACKET_SIZE)

#define STATUS_SUCCESS                  (0x00000000U)
#define IMB_NO_ASYNC_MSG		((unsigned long)0xE0070012L)
#define IMB_SEND_REQUEST_FAILED		((unsigned long)0xE0070013L)
#define INVALID_ARGUMENTS		((unsigned long)0xE0070002L)


#define FILE_DEVICE_IMB			0x00008010
#define IOCTL_IMB_BASE			0x00000880

#define CTL_CODE(DeviceType, Function, Method, Access)\
		_IO(DeviceType & 0x00FF, Function & 0x00FF)

#define FILE_DEVICE_IMB			0x00008010
#define IOCTL_IMB_BASE			0x00000880
#define METHOD_BUFFERED                 0
#define FILE_ANY_ACCESS                 0


typedef struct {
	int	code;
#define		SD_NO_ACTION				0
#define		SD_RESET				1
#define		SD_POWER_OFF				2

	int	delayTime;  /* in units of 100 millisecond */
} ShutdownCmdBuffer;


/* BMC added parentheses around IOCTL_IMB_BASE + 2 */
#define IOCTL_IMB_SEND_MESSAGE		CTL_CODE(FILE_DEVICE_IMB, (IOCTL_IMB_BASE + 2), METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_IMB_GET_ASYNC_MSG		CTL_CODE(FILE_DEVICE_IMB, (IOCTL_IMB_BASE + 8), METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_IMB_MAP_MEMORY		CTL_CODE(FILE_DEVICE_IMB, (IOCTL_IMB_BASE + 14),METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_IMB_UNMAP_MEMORY		CTL_CODE(FILE_DEVICE_IMB, (IOCTL_IMB_BASE + 16),METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_IMB_SHUTDOWN_CODE		CTL_CODE(FILE_DEVICE_IMB, (IOCTL_IMB_BASE + 18),METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_IMB_REGISTER_ASYNC_OBJ	CTL_CODE(FILE_DEVICE_IMB, (IOCTL_IMB_BASE + 24),METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_IMB_DEREGISTER_ASYNC_OBJ	CTL_CODE(FILE_DEVICE_IMB, (IOCTL_IMB_BASE + 26),METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_IMB_CHECK_EVENT		CTL_CODE(FILE_DEVICE_IMB, (IOCTL_IMB_BASE + 28),METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_IMB_POLL_ASYNC		CTL_CODE(FILE_DEVICE_IMB, (IOCTL_IMB_BASE + 20),METHOD_BUFFERED, FILE_ANY_ACCESS)


#endif /* __LINUX_IPMI_IMB_H */
