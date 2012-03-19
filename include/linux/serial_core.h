/*
 *  linux/drivers/char/serial_core.h
 *
 *  Copyright (C) 2000 Deep Blue Solutions Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef LINUX_SERIAL_CORE_H
#define LINUX_SERIAL_CORE_H


#include <linux/compiler.h>
#include <linux/interrupt.h>
#include <linux/circ_buf.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/tty.h>
#include <linux/mutex.h>
#include <linux/sysrq.h>
#include <linux/tty_flip.h>
#include <uapi/linux/serial_core.h>

#ifdef CONFIG_SERIAL_CORE_CONSOLE
#define uart_console(port) \
	((port)->cons && (port)->cons->index == (port)->line)
#else
#define uart_console(port)      (0)
#endif

struct uart_port;
struct serial_struct;
struct device;

#define UART_POLL_FLAGS_RX	(1 << 0)	/* Poll Receiver */
#define UART_POLL_FLAGS_TX	(1 << 1)	/* Poll Transmitter */
#define UART_POLL_FLAGS_MCTRL	(1 << 2)	/* Poll modem control */

/*
 * This structure describes all the operations that can be done on the
 * physical hardware.  See Documentation/serial/driver for details.
 */
struct uart_ops {
	unsigned int	(*tx_empty)(struct uart_port *);
	void		(*set_mctrl)(struct uart_port *, unsigned int mctrl);
	unsigned int	(*get_mctrl)(struct uart_port *);
	void		(*stop_tx)(struct uart_port *);
	void		(*start_tx)(struct uart_port *);
	void		(*throttle)(struct uart_port *);
	void		(*unthrottle)(struct uart_port *);
	void		(*send_xchar)(struct uart_port *, char ch);
	void		(*stop_rx)(struct uart_port *);
	void		(*enable_ms)(struct uart_port *);
	void		(*break_ctl)(struct uart_port *, int ctl);
	int		(*startup)(struct uart_port *);
	void		(*shutdown)(struct uart_port *);
	void		(*flush_buffer)(struct uart_port *);
	void		(*set_termios)(struct uart_port *, struct ktermios *new,
				       struct ktermios *old);
	void		(*set_ldisc)(struct uart_port *, int new);
	void		(*pm)(struct uart_port *, unsigned int state,
			      unsigned int oldstate);


	/*
	 * Note: Poll routines must be called with the port lock held and
	 * interrupts off.
	 */

	/*
	 * The startup and shutdown routines must be called before
	 * poll is used and after done calling poll.  You cannot allow
	 * the driver code to be run by interrupts (or anything else)
	 * between this.  A state value is returned by the startup
	 * routine in pstate, you must pass that to the shutdown
	 * routine.
	 */
	int		(*poll_startup)(struct uart_port *,
					unsigned long *pstate);
	void		(*poll_shutdown)(struct uart_port *,
					 unsigned long pstate);

	/*
	 * Check the serial chip for I/O.  Flags is used to specify
	 * what to check, see UART_POLL_FLAGS_xxx above.
	 */
	void		(*poll)(struct uart_port *, unsigned int flags);

	/*
	 * Is the port in flow-control (CTS is not asserted).  It is
	 * optional an may be NULL and is only called if UPF_CONS_FLOW
	 * is set in port->flags.
	 */
	int             (*in_flow_control)(struct uart_port *);

	/*
	 * Return reasonable settings for the port; it is primarily
	 * there so firmware can pass console settings for the
	 * console.
	 */
	int		(*port_defaults)(struct uart_port *,
					 int *baud, int *parity, int *bits,
					 int *flow);

	/*
	 * Return a string describing the type of the port
	 */
	const char	*(*type)(struct uart_port *);

	/*
	 * Release IO and memory resources used by the port.
	 * This includes iounmap if necessary.
	 */
	void		(*release_port)(struct uart_port *);

	/*
	 * Request IO and memory resources used by the port.
	 * This includes iomapping the port if necessary.
	 */
	int		(*request_port)(struct uart_port *);
	void		(*config_port)(struct uart_port *, int);
	int		(*verify_port)(struct uart_port *, struct serial_struct *);
	int		(*ioctl)(struct uart_port *, unsigned int, unsigned long);
#ifdef CONFIG_CONSOLE_POLL
	int		(*poll_init)(struct uart_port *);
	void		(*poll_put_char)(struct uart_port *, unsigned char);
	int		(*poll_get_char)(struct uart_port *);
#endif
};

#define NO_POLL_CHAR		0x00ff0000
#define UART_CONFIG_TYPE	(1 << 0)
#define UART_CONFIG_IRQ		(1 << 1)

struct uart_icount {
	__u32	cts;
	__u32	dsr;
	__u32	rng;
	__u32	dcd;
	__u32	rx;
	__u32	tx;
	__u32	frame;
	__u32	overrun;
	__u32	parity;
	__u32	brk;
	__u32	buf_overrun;
};

typedef unsigned int __bitwise__ upf_t;

struct uart_port {
	spinlock_t		lock;			/* port lock */
	unsigned long		iobase;			/* in/out[bwl] */
	unsigned char __iomem	*membase;		/* read/write[bwl] */
	unsigned int		(*serial_in)(struct uart_port *, int);
	void			(*serial_out)(struct uart_port *, int, int);
	void			(*set_termios)(struct uart_port *,
				               struct ktermios *new,
				               struct ktermios *old);
	int			(*handle_irq)(struct uart_port *);
	void			(*pm)(struct uart_port *, unsigned int state,
				      unsigned int old);
	void			(*handle_break)(struct uart_port *);
	unsigned int		irq;			/* irq number */
	unsigned long		irqflags;		/* irq flags  */
	unsigned int		uartclk;		/* base uart clock */
	unsigned int		fifosize;		/* tx fifo size */
	unsigned char		x_char;			/* xon/xoff char */
	unsigned char		regshift;		/* reg offset shift */
	unsigned char		iotype;			/* io access style */
	unsigned char		unused1;

#define UPIO_PORT		(0)
#define UPIO_HUB6		(1)
#define UPIO_MEM		(2)
#define UPIO_MEM32		(3)
#define UPIO_AU			(4)			/* Au1x00 and RT288x type IO */
#define UPIO_TSI		(5)			/* Tsi108/109 type IO */

	unsigned int		read_status_mask;	/* driver specific */
	unsigned int		ignore_status_mask;	/* driver specific */
	struct uart_state	*state;			/* pointer to parent state */
	struct uart_icount	icount;			/* statistics */

	struct console		*cons;			/* struct console, if any */
#if defined(CONFIG_SERIAL_CORE_CONSOLE) || defined(SUPPORT_SYSRQ)
	unsigned long		sysrq;			/* sysrq timeout */
#endif

	upf_t			flags;

#define UPF_FOURPORT		((__force upf_t) (1 << 1))
#define UPF_SAK			((__force upf_t) (1 << 2))
#define UPF_SPD_MASK		((__force upf_t) (0x1030))
#define UPF_SPD_HI		((__force upf_t) (0x0010))
#define UPF_SPD_VHI		((__force upf_t) (0x0020))
#define UPF_SPD_CUST		((__force upf_t) (0x0030))
#define UPF_SPD_SHI		((__force upf_t) (0x1000))
#define UPF_SPD_WARP		((__force upf_t) (0x1010))
#define UPF_SKIP_TEST		((__force upf_t) (1 << 6))
#define UPF_AUTO_IRQ		((__force upf_t) (1 << 7))
#define UPF_HARDPPS_CD		((__force upf_t) (1 << 11))
#define UPF_LOW_LATENCY		((__force upf_t) (1 << 13))
#define UPF_BUGGY_UART		((__force upf_t) (1 << 14))
#define UPF_NO_TXEN_TEST	((__force upf_t) (1 << 15))
#define UPF_MAGIC_MULTIPLIER	((__force upf_t) (1 << 16))
/* Port has hardware-assisted h/w flow control (iow, auto-RTS *not* auto-CTS) */
#define UPF_HARD_FLOW		((__force upf_t) (1 << 21))
/* Port has hardware-assisted s/w flow control */
#define UPF_SOFT_FLOW		((__force upf_t) (1 << 22))
#define UPF_CONS_FLOW		((__force upf_t) (1 << 23))
#define UPF_SHARE_IRQ		((__force upf_t) (1 << 24))
#define UPF_EXAR_EFR		((__force upf_t) (1 << 25))
#define UPF_BUG_THRE		((__force upf_t) (1 << 26))
/* The exact UART type is known and should not be probed.  */
#define UPF_FIXED_TYPE		((__force upf_t) (1 << 27))
#define UPF_BOOT_AUTOCONF	((__force upf_t) (1 << 28))
#define UPF_FIXED_PORT		((__force upf_t) (1 << 29))
#define UPF_DEAD		((__force upf_t) (1 << 30))
#define UPF_IOREMAP		((__force upf_t) (1 << 31))

#define UPF_CHANGE_MASK		((__force upf_t) (0x17fff))
#define UPF_USR_MASK		((__force upf_t) (UPF_SPD_MASK|UPF_LOW_LATENCY))

	unsigned int		mctrl;			/* current modem ctrl settings */
	unsigned int		timeout;		/* character-based timeout */
	unsigned int		type;			/* port type */
	const struct uart_ops	*ops;
	unsigned int		custom_divisor;
	unsigned int		line;			/* port index */
	resource_size_t		mapbase;		/* for ioremap */
	struct device		*dev;			/* parent device */
	unsigned char		hub6;			/* this should be in the 8250 driver */
	unsigned char		suspended;
	unsigned char		irq_wake;
	unsigned char		unused[2];
	void			*private_data;		/* generic platform data pointer */
};

static inline int serial_port_in(struct uart_port *up, int offset)
{
	return up->serial_in(up, offset);
}

static inline void serial_port_out(struct uart_port *up, int offset, int value)
{
	up->serial_out(up, offset, value);
}

/**
 * enum uart_pm_state - power states for UARTs
 * @UART_PM_STATE_ON: UART is powered, up and operational
 * @UART_PM_STATE_OFF: UART is powered off
 * @UART_PM_STATE_UNDEFINED: sentinel
 */
enum uart_pm_state {
	UART_PM_STATE_ON = 0,
	UART_PM_STATE_OFF = 3, /* number taken from ACPI */
	UART_PM_STATE_UNDEFINED,
};

/*
 * This is the state information which is persistent across opens.
 */
struct uart_state {
	struct tty_port		port;

	enum uart_pm_state	pm_state;
	struct circ_buf		xmit;

	unsigned int		usflags;

	struct uart_port	*uart_port;
};

#define UART_XMIT_SIZE	PAGE_SIZE

#define UART_STATE_TTY_REGISTERED 1 /* Devices is register with TTY layer. */
#define UART_STATE_BOOT_ALLOCATED 2 /* Buffer was allocated by serial core */


/* number of characters left in xmit buffer before we ask for more */
#define WAKEUP_CHARS		256

struct module;
struct tty_driver;

struct uart_driver {
	struct module		*owner;
	const char		*driver_name;
	const char		*dev_name;
	int			 major;
	int			 minor;
	int			 nr;
	struct console		*cons;

	/*
	 * If nr_pollable is non-zero, then pollable_ports is an array of uart
	 * ports that can be used for polling.  The serial_core code
	 * will assume two things:
	 *   1) The driver has poll capability in the uart_ops.
	 *   2) The driver wants the serial_core to manage the console
	 *      pointed to by "cons" above.  It uses the poll capability
	 *      to do this.
	 */
	int			 nr_pollable;
	struct uart_port	**pollable_ports;

	/*
	 * these are private; the low level driver should not
	 * touch these; they should be initialised to NULL
	 */
	struct uart_state	*state;
	struct tty_driver	*tty_driver;
};

void uart_write_wakeup(struct uart_port *port);

/*
 * Baud rate helpers.
 */
void uart_update_timeout(struct uart_port *port, unsigned int cflag,
			 unsigned int baud);
unsigned int uart_get_baud_rate(struct uart_port *port, struct ktermios *termios,
				struct ktermios *old, unsigned int min,
				unsigned int max);
unsigned int uart_get_divisor(struct uart_port *port, unsigned int baud);

/* Base timer interval for polling */
static inline int uart_poll_timeout(struct uart_port *port)
{
	int timeout = port->timeout;

	return timeout > 6 ? (timeout / 2 - 2) : 1;
}

/*
 * Console helpers.
 */
struct earlycon_device {
	struct console *con;
	struct uart_port port;
	char options[16];		/* e.g., 115200n8 */
	unsigned int baud;
};
int setup_earlycon(char *buf, const char *match,
		   int (*setup)(struct earlycon_device *, const char *));

extern int of_setup_earlycon(unsigned long addr,
			     int (*setup)(struct earlycon_device *, const char *));

#define EARLYCON_DECLARE(name, func) \
static int __init name ## _setup_earlycon(char *buf) \
{ \
	return setup_earlycon(buf, __stringify(name), func); \
} \
early_param("earlycon", name ## _setup_earlycon);

#define OF_EARLYCON_DECLARE(name, compat, fn)				\
	_OF_DECLARE(earlycon, name, compat, fn, void *)

struct uart_port *uart_get_console(struct uart_port *ports, int nr,
				   struct console *c);
void uart_parse_options(char *options, int *baud, int *parity, int *bits,
			int *flow);
int uart_set_options(struct uart_port *port, struct console *co, int baud,
		     int parity, int bits, int flow);
struct tty_driver *uart_console_device(struct console *co, int *index);
void uart_console_write(struct uart_port *port, const char *s,
			unsigned int count,
			void (*putchar)(struct uart_port *, int));

/*
 * Port/driver registration/removal
 */
void uart_register_polled(struct uart_driver *uart);
void uart_unregister_polled(struct uart_driver *uart);
int uart_register_driver(struct uart_driver *uart);
void uart_unregister_driver(struct uart_driver *uart);
int uart_add_one_port(struct uart_driver *reg, struct uart_port *port);
int uart_remove_one_port(struct uart_driver *reg, struct uart_port *port);
int uart_match_port(struct uart_port *port1, struct uart_port *port2);

/*
 * Power Management
 */
int uart_suspend_port(struct uart_driver *reg, struct uart_port *port);
int uart_resume_port(struct uart_driver *reg, struct uart_port *port);

#define uart_circ_empty(circ)		((circ)->head == (circ)->tail)
#define uart_circ_clear(circ)		((circ)->head = (circ)->tail = 0)
#define uart_get_circ_buf(port)		(&(port)->state->xmit)
#define uart_wrap_circ_buf(val)		((val) & (UART_XMIT_SIZE - 1))

#define uart_circ_chars_pending(circ)	\
	(CIRC_CNT((circ)->head, (circ)->tail, UART_XMIT_SIZE))

#define uart_circ_chars_free(circ)	\
	(CIRC_SPACE((circ)->head, (circ)->tail, UART_XMIT_SIZE))

static inline int uart_tx_stopped(struct uart_port *port)
{
	struct tty_struct *tty = port->state->port.tty;
	if (!tty)
		return 0;
	if(tty->stopped || tty->hw_stopped)
		return 1;
	return 0;
}

/*
 * The following are helper functions for the low level drivers.
 */

extern void uart_handle_dcd_change(struct uart_port *uport,
		unsigned int status);
extern void uart_handle_cts_change(struct uart_port *uport,
		unsigned int status);

extern void uart_insert_char(struct uart_port *port, unsigned int status,
		 unsigned int overrun, unsigned int ch, unsigned int flag);

#ifdef SUPPORT_SYSRQ
static inline int
uart_handle_sysrq_char(struct uart_port *port, unsigned int ch)
{
	if (port->sysrq) {
		if (ch && time_before(jiffies, port->sysrq)) {
			handle_sysrq(ch);
			port->sysrq = 0;
			return 1;
		}
		port->sysrq = 0;
	}
	return 0;
}
#else
#define uart_handle_sysrq_char(port,ch) ({ (void)port; 0; })
#endif

/*
 * We do the SysRQ and SAK checking like this...
 */
static inline int uart_handle_break(struct uart_port *port)
{
	struct uart_state *state = port->state;

	if (port->handle_break)
		port->handle_break(port);

#ifdef SUPPORT_SYSRQ
	if (port->cons && port->cons->index == port->line) {
		if (!port->sysrq) {
			port->sysrq = jiffies + HZ*5;
			return 1;
		}
		port->sysrq = 0;
	}
#endif
	if (port->flags & UPF_SAK)
		do_SAK(state->port.tty);
	return 0;
}

static inline void
uart_push(struct uart_port *port)
{
	if (!port->state->port.tty)
		return;

	tty_flip_buffer_push(&port->state->port);
}

/*
 *	UART_ENABLE_MS - determine if port should enable modem status irqs
 */
#define UART_ENABLE_MS(port,cflag)	((port)->flags & UPF_HARDPPS_CD || \
					 (cflag) & CRTSCTS || \
					 !((cflag) & CLOCAL))

#endif /* LINUX_SERIAL_CORE_H */
