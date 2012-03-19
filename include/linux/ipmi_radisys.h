/*
 * ipmi_radisys.h
 *
 * An emulation of the Radisys IPMI interface on top of the MontaVista
 * interface.
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

#ifndef __LINUX_IPMI_RADISYS_H
#define __LINUX_IPMI_RADISYS_H

/******************************************************************************
 * This is the old IPMI interface defined by Radisys.  We are
 * compliant with that.  Don't use it for new designs, though.
 */
#define IOCTL_IPMI_RCV          ( IPMI_IOC_MAGIC<<8 | 1 )
#define IOCTL_IPMI_SEND         ( IPMI_IOC_MAGIC<<8 | 2 )
#define IOCTL_IPMI_EVENT        ( IPMI_IOC_MAGIC<<8 | 3 )
#define IOCTL_IPMI_REGISTER     ( IPMI_IOC_MAGIC<<8 | 4 )
#define IOCTL_IPMI_UNREGISTER   ( IPMI_IOC_MAGIC<<8 | 5 )
#define IOCTL_IPMI_CLEAR        ( IPMI_IOC_MAGIC<<8 | 9 )

/* These don't seem to be implemented in the Radisys driver.
#define IOCTL_IPMI_RESET_BMC    ( IPMI_IOC_MAGIC<<8 | 6 )
#define IOCTL_IPMI_GET_BMC_ADDR ( IPMI_IOC_MAGIC<<8 | 7 )
#define IOCTL_IPMI_SET_BMC_ADDR ( IPMI_IOC_MAGIC<<8 | 8 )
*/

/*
 * Network Function Codes
 */
#define IPMI_NETFN_CHASSIS          0x00    /* Chassis - 0x00 << 2 */
#define IPMI_NETFN_CHASSIS_RESP     0x04    /* Chassis - 0x01 << 2 */

#define IPMI_NETFN_BRIDGE           0x08    /* Bridge - 0x02 << 2 */
#define IPMI_NETFN_BRIDGE_RESP      0x0c    /* Bridge - 0x03 << 2 */

#define IPMI_NETFN_SENSOR_EVT       0x10    /* Sensor/Event - 0x04 << 2 */
#define IPMI_NETFN_SENSOR_EVT_RESP  0x14    /* Sensor/Event - 0x05 << 2 */

#define IPMI_NETFN_APP              0x18    /* Application - 0x06 << 2 */
#define IPMI_NETFN_APP_RESP         0x1c    /* Application - 0x07 << 2 */

#define IPMI_NETFN_FIRMWARE         0x20    /* Firmware - 0x08 << 2 */
#define IPMI_NETFN_FIRMWARE_RESP    0x24    /* Firmware - 0x09 << 2 */

#define IPMI_NETFN_STORAGE          0x28    /* Storage - 0x0a << 2 */
#define IPMI_NETFN_STORAGE_RESP     0x2c    /* Storage - 0x0b << 2 */

#define IPMI_NETFN_OEM_1            0xC0    /* Storage - 0x30 << 2 */
#define IPMI_NETFN_OEM_1_RESP       0xC4    /* Storage - 0x31 << 2 */

/* there are 15 other OEM netfn pairs (OEM - 0x30-0x3f) */

typedef struct _IPMI_LIST_ENTRY {
   struct _IPMI_LIST_ENTRY * volatile Flink;
   struct _IPMI_LIST_ENTRY * volatile Blink;
} IPMI_LIST_ENTRY, *PIPMI_LIST_ENTRY;

typedef struct IPMI_semaphore   IPMI_KSEMAPHORE;
typedef struct IPMI_semaphore * IPMI_PKSEMAPHORE;

/* IPMI Address structure */
typedef struct _IPMI_ADDR {
    unsigned char uchSlave;                /* Slave Address */
    unsigned char uchLun;                  /* Logical Unit Number */
} IPMI_ADDR, *PIPMI_ADDR;

#define IPMI_MAX_MSG_SIZE	36

/* IPMI Message Descriptor structure */
typedef struct _IPMI_MSGDESC {
                                         /************************************/
                                         /* Device Driver Specific Elements  */
                                         /************************************/
    IPMI_LIST_ENTRY  Entry;                  /* Linked list element */
    void             *pIRPacket;             /* Pointer to IRP object */
    IPMI_PKSEMAPHORE pSema;                  /* Semaphore Object */
    long             lTimeout;               /* Timeout value */
                                         /************************************/
                                         /* Shared elements                  */
                                         /************************************/
    unsigned char   auchBuffer[IPMI_MAX_MSG_SIZE]; /* Message buffer */
    unsigned long   ulLength;               /* Length of message in bytes */
    int             fDefer;                 /* TRUE - Defer I/O
					       operation, doesn't seem
					       to be used in the
					       Radisys driver. */
    IPMI_ADDR       Dest;                   /* Destination IPM Address */
    unsigned char   uchNetFn;               /* Network Function */
    unsigned char   uchCmd;                 /* Command */
    unsigned char   uchSeq;                 /* Sequence Number */
    unsigned char   uchComplete;            /* Completion Code */
} IPMI_MSGDESC, *PIPMI_MSGDESC;

/* Byte return codes for some things. */
#define LOWLRC_SUCCESS              0x00 /* routine completed successfully */
#define LOWLRC_ERROR                0xff /* routine did not complete */
#define LOWLRC_INVALID_PARAMETERS   0xfe /* invalid parameters */
#define LOWLRC_INVALID_REQUEST_DATA 0xfd /* invalid request data */

#endif /* __LINUX_IPMI_RADISYS_H */
