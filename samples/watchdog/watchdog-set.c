// SPDX-License-Identifier: GPL-2.0
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/watchdog.h>

/* In case we are compiling with older kernel include files. */
#ifndef WDIOC_SETACTION
#define	WDIOC_SETACTION		_IOWR(WATCHDOG_IOCTL_BASE, 11, int)
#define	WDIOC_GETACTION		_IOR(WATCHDOG_IOCTL_BASE, 12, int)
#define	WDIOC_SETPREACTION	_IOWR(WATCHDOG_IOCTL_BASE, 13, int)
#define	WDIOC_GETPREACTION	_IOR(WATCHDOG_IOCTL_BASE, 14, int)
#define	WDIOC_SETPREGOV		_IOWR(WATCHDOG_IOCTL_BASE, 15, char)
#define	WDIOC_GETPREGOV		_IOR(WATCHDOG_IOCTL_BASE, 16, char)

/*
 * Buffer for WDIOC_GETPREGOV must be at least this big.  WDIOC_SETPRGOV
 * will take at max this many bytes - 1, excess will be ignored.
 */
#define WATCHDOG_GOV_NAME_MAXLEN	20

/* Actions for WDIOC_xxxACTION ioctls. */
#define WDIOA_RESET		0	/* Reset the system. */
#define WDIOA_POWER_OFF		1	/* Power off the system. */
#define WDIOA_POWER_CYCLE	2	/* Power cycle the system. */

/* Actions for WDIOC_xxxPREACTION ioctls. */
#define WDIOP_NONE		0	/* Do nothing. */
#define WDIOP_NMI		1	/* Issue an NMI. */
#define WDIOP_SMI		2	/* Issue a system management irq. */
#define WDIOP_INTERRUPT		3	/* Issue a normal irq. */
#endif

struct bitflags {
	int flag;
	char *name;
};

static struct bitflags flags[] = {
	{ WDIOF_OVERHEAT,	"overheat" },
	{ WDIOF_FANFAULT,	"fanfault" },
	{ WDIOF_EXTERN1,	"extern1" },
	{ WDIOF_EXTERN2,	"extern2" },
	{ WDIOF_POWERUNDER,	"powerunder" },
	{ WDIOF_CARDRESET,	"cardreset" },
	{ WDIOF_POWEROVER,	"powerover" },
	{ WDIOF_SETTIMEOUT,	"settimeout" },
	{ WDIOF_MAGICCLOSE,	"magicclose" },
	{ WDIOF_PRETIMEOUT,	"pretimeout" },
	{ WDIOF_ALARMONLY,	"alarmonly" },
	{ WDIOF_KEEPALIVEPING,	"keepaliveping" },
	{ }
};

static struct bitflags options[] = {
	{ WDIOS_DISABLECARD,	"disablecard" },
	{ WDIOS_ENABLECARD,	"enablecard" },
	{ WDIOS_TEMPPANIC,	"temppanic" },
	{ }
};

struct actionvals {
	int action;
	char *name;
};

static struct actionvals actions[] = {
	{ WDIOA_RESET,		"reset" },
	{ WDIOA_POWER_OFF,	"poweroff" },
	{ WDIOA_POWER_CYCLE,	"powercycle" },
	{ }
};

static struct actionvals preactions[] = {
	{ WDIOP_NONE,		"none" },
	{ WDIOP_NMI,		"nmi" },
	{ WDIOP_SMI,		"smi" },
	{ WDIOP_INTERRUPT,	"interrupt" },
	{ }
};

static void print_bits(int bitmask, struct bitflags *flags)
{
	unsigned int i;

	for (i = 0; flags[i].name; i++) {
		if (flags[i].flag & bitmask) {
			bitmask &= ~flags[i].flag;
			printf(" %s", flags[i].name);
		}
	}
	i = 0;
	while (bitmask) {
		while (!(bitmask & (1 << i)))
			i++;
		printf(" bit(%d)", i);
		bitmask &= ~(1 << i);
	}
}

static void print_action(int val, struct actionvals *actions)
{
	unsigned int i;

	for (i = 0; actions[i].name; i++) {
		if (val == actions[i].action) {
			printf("%s\n", actions[i].name);
			return;
		}
	}
	printf("%d\n", val);
}

static int action_to_val(char *action, struct actionvals *actions)
{
	unsigned int i;
	int val;
	char *end;

	for (i = 0; actions[i].name; i++) {
		if (strcmp(action, actions[i].name) == 0)
			return actions[i].action;
	}

	val = strtoul(action, &end, 0);
	if (end == action || *end != '\0')
		return -1;

	return val;
}

static int status(int wdfd, int argc, char *argv[])
{
	struct watchdog_info info;
	int val;
	int ret;
	char gov[WATCHDOG_GOV_NAME_MAXLEN];

	printf("info:");
	ret = ioctl(wdfd, WDIOC_GETSUPPORT, &info);
	if (ret == -1) {
		printf(" error:%s\n", strerror(errno));
	} else {
		printf("\n  options:");
		print_bits(info.options, flags);
		printf("\n  fwver: %d (0x%x)", info.firmware_version,
		       info.firmware_version);
		printf("\n  identity: %s\n", info.identity);
	}

	printf("status:");
	ret = ioctl(wdfd, WDIOC_GETSTATUS, &val);
	if (ret == -1) {
		printf(" error:%s\n", strerror(errno));
	} else {
		print_bits(val, flags);
		printf("\n");
	}

	printf("bootstatus:");
	ret = ioctl(wdfd, WDIOC_GETBOOTSTATUS, &val);
	if (ret == -1) {
		printf(" error:%s\n", strerror(errno));
	} else {
		print_bits(val, flags);
		printf("\n");
	}

	ret = ioctl(wdfd, WDIOC_GETTEMP, &val);
	if (ret != -1) /* Not usually implemented. */
		printf("temp: %d\n", val);

	ret = ioctl(wdfd, WDIOC_GETTIMEOUT, &val);
	if (ret == -1)
		printf("timeout: error:%s\n", strerror(errno));
	else
		printf("timeout: %d\n", val);

	ret = ioctl(wdfd, WDIOC_GETPRETIMEOUT, &val);
	if (ret == -1)
		printf("pretimeout: error:%s\n", strerror(errno));
	else
		printf("pretimeout: %d\n", val);

	ret = ioctl(wdfd, WDIOC_GETACTION, &val);
	if (ret == -1) {
		if (errno != ENOTTY) /* If not an older kernel. */
			printf("action: error:%s\n", strerror(errno));
	} else {
		printf("action: ");
		print_action(val, actions);
	}

	ret = ioctl(wdfd, WDIOC_GETPREACTION, &val);
	if (ret == -1) {
		if (errno != ENOTTY) /* If not an older kernel. */
			printf("preaction: error:%s\n", strerror(errno));
	} else {
		printf("preaction: ");
		print_action(val, preactions);
	}

	ret = ioctl(wdfd, WDIOC_GETPREGOV, gov);
	if (ret == -1) {
		if (errno != ENOTTY) /* If not an older kernel. */
			printf("governor: error:%s\n", strerror(errno));
	} else {
		printf("governor: %s\n", gov);
	}

	return 0;
}

static int setoptions(int wdfd, int argc, char *argv[])
{
	int ret, val = 0;
	int i;
	unsigned int j;
	char *end;

	for (i = 0; i < argc; ) {
		for (j = 0; options[j].name; j++) {
			if (strcmp(argv[i], options[j].name) == 0) {
				val |= options[j].flag;
				goto found;
			}
		}
		val |= strtoul(argv[i], &end, 0);
		if (end == argv[i] || *end != '\0') {
			fprintf(stderr, "invalid option: '%s'\n", argv[i]);
			return 1;
		}
found:
		i++;
	}

	ret = ioctl(wdfd, WDIOC_SETOPTIONS, &val);
	if (ret == -1) {
		fprintf(stderr, "Set options error: %s\n", strerror(errno));
		return 1;
	}

	return 0;
}

static int ping(int wdfd, int argc, char *argv[])
{
	int ret;

	ret = ioctl(wdfd, WDIOC_KEEPALIVE);
	if (ret == -1) {
		printf("ping error:%s\n", strerror(errno));
		return 1;
	}

	return 0;
}

static int settimeout(int wdfd, int argc, char *argv[])
{
	int ret, val;
	char *end;

	if (argc < 1) {
		fprintf(stderr, "No value for timeout\n");
		return 1;
	}

	val = strtoul(argv[0], &end, 0);
	if (end == argv[0] || *end != '\0') {
		fprintf(stderr, "Invalid number for timeout: '%s'\n", argv[0]);
		return 1;
	}

	ret = ioctl(wdfd, WDIOC_SETTIMEOUT, &val);
	if (ret == -1) {
		fprintf(stderr, "Set timeout error: %s\n", strerror(errno));
		return 1;
	}

	return 0;
}

static int gettimeout(int wdfd, int argc, char *argv[])
{
	int ret, val;

	ret = ioctl(wdfd, WDIOC_GETTIMEOUT, &val);
	if (ret == -1) {
		fprintf(stderr, "Get timeout error: %s\n", strerror(errno));
		return 1;
	}

	printf("%d\n", val);
	return 0;
}

static int setpretimeout(int wdfd, int argc, char *argv[])
{
	int ret, val;
	char *end;

	if (argc < 1) {
		fprintf(stderr, "No value for pretimeout\n");
		return 1;
	}

	val = strtoul(argv[0], &end, 0);
	if (end == argv[0] || *end != '\0') {
		fprintf(stderr, "Invalid number for pretimeout: '%s'\n",
			argv[0]);
		return 1;
	}

	ret = ioctl(wdfd, WDIOC_SETPRETIMEOUT, &val);
	if (ret == -1) {
		fprintf(stderr, "Set pretimeout error: %s\n", strerror(errno));
		return 1;
	}

	return 0;
}

static int getpretimeout(int wdfd, int argc, char *argv[])
{
	int ret, val;

	ret = ioctl(wdfd, WDIOC_GETPRETIMEOUT, &val);
	if (ret == -1) {
		fprintf(stderr, "Get pretimeout error: %s\n", strerror(errno));
		return 1;
	}

	printf("%d\n", val);
	return 0;
}

static int gettimeleft(int wdfd, int argc, char *argv[])
{
	int ret, val;

	ret = ioctl(wdfd, WDIOC_GETTIMELEFT, &val);
	if (ret == -1) {
		fprintf(stderr, "Get time left error: %s\n", strerror(errno));
		return 1;
	}

	printf("%d\n", val);
	return 0;
}

static int setaction(int wdfd, int argc, char *argv[])
{
	int val, ret;

	if (argc < 1) {
		fprintf(stderr, "No value for action\n");
		return 1;
	}

	val = action_to_val(argv[0], actions);
	if (val == -1) {
		fprintf(stderr, "Invalid action: '%s'\n", argv[0]);
		return 1;
	}

	ret = ioctl(wdfd, WDIOC_SETACTION, &val);
	if (ret == -1) {
		fprintf(stderr, "Set action error: %s\n", strerror(errno));
		return 1;
	}

	return 0;
}

static int getaction(int wdfd, int argc, char *argv[])
{
	int val, ret;

	ret = ioctl(wdfd, WDIOC_GETACTION, &val);
	if (ret == -1) {
		fprintf(stderr, "Get action error: %s\n", strerror(errno));
		return 1;
	}

	print_action(val, actions);
	return 0;
}

static int setpreaction(int wdfd, int argc, char *argv[])
{
	int val, ret;

	if (argc < 1) {
		fprintf(stderr, "No value for preaction\n");
		return 1;
	}

	val = action_to_val(argv[0], preactions);
	if (val == -1) {
		fprintf(stderr, "Invalid preaction: '%s'\n", argv[0]);
		return 1;
	}

	ret = ioctl(wdfd, WDIOC_SETPREACTION, &val);
	if (ret == -1) {
		fprintf(stderr, "Set preaction error: %s\n", strerror(errno));
		return 1;
	}

	return 0;
}

static int getpreaction(int wdfd, int argc, char *argv[])
{
	int val, ret;

	ret = ioctl(wdfd, WDIOC_GETPREACTION, &val);
	if (ret == -1) {
		fprintf(stderr, "Get preaction error: %s\n", strerror(errno));
		return 1;
	}

	print_action(val, preactions);
	return 0;
}

static int setpregov(int wdfd, int argc, char *argv[])
{
	int ret;

	if (argc < 1) {
		fprintf(stderr, "No value for pretimeout governor\n");
		return 1;
	}

	ret = ioctl(wdfd, WDIOC_SETPREGOV, argv[0]);
	if (ret == -1) {
		fprintf(stderr, "Set preaction governor error: %s\n",
			strerror(errno));
		return 1;
	}

	return 0;
}

static int getpregov(int wdfd, int argc, char *argv[])
{
	int ret;
	char gov[WATCHDOG_GOV_NAME_MAXLEN];

	ret = ioctl(wdfd, WDIOC_GETPREGOV, gov);
	if (ret == -1) {
		fprintf(stderr, "Get preaction governor error: %s\n",
			strerror(errno));
		return 1;
	}

	printf("%s\n", gov);

	return 0;
}

static int waitdata(int wdfd, int argc, char *argv[])
{
	char dummy;
	int ret;

	ret = read(wdfd, &dummy, 1);
	if (ret == -1)
		perror("read");
	else
		printf("Received data\n");
	return 0;
}

static struct {
	char *name;
	int (*handler)(int wdfd, int argc, char *argv[]);
	char *help;
} handlers[] = {
	{ "status",		status,
	  "- print out the current watchdog status" },
	{ "setoptions",		setoptions,
	  "[<option> [<option> [...]]] - set options to one or more:\n"
	  "    disablecard, enabledcard, temppanic" },
	{ "ping",		ping,
	  "- reset the watchdog timer's timeout" },
	{ "settimeout",		settimeout,
	  "<timeout> - set the value of the timeout, an integer value" },
	{ "gettimeout",		gettimeout,
	  "- get the value of the timeout" },
	{ "setpretimeout",	setpretimeout,
	  "<timeout> - set the value of the pretimeout, an integer value" },
	{ "getpretimeout",	getpretimeout,
	  "- get the value of the pretimeout" },
	{ "gettimeleft",	gettimeleft,
	  "- get the time left before the timeout" },
	{ "setaction",		setaction,
	  "<action> - set the action on timeout: reset, poweroff, powercycle" },
	{ "getaction",		getaction,
	  "- get the action on timeout" },
	{ "setpreaction",	setpreaction,
	  "<action> - set the action on pretimeout: none, nmi, smi,\n"
	  "    interrupt" },
	{ "getpreaction",	getpreaction,
	  "- get the action on pretimeout" },
	{ "setpregov",		setpregov,
	  "<governor> - Set the pretimeout governor: noop, panic, read_data" },
	{ "getpregov",		getpregov,
	  "- get the pretimeout governor" },
	{ "waitdata",		waitdata,
	  "- Wait for read data from the watchdog device" },
	{ }
};

static void print_help(char *progname)
{
	unsigned int i;

	printf("%s [-d devname] [-h] <command>\nCommands are:\n", progname);
	for (i = 0; handlers[i].name; i++)
		printf("  %s %s\n", handlers[i].name, handlers[i].help);
}

int main(int argc, char *argv[])
{
	const char *devfile = "/dev/watchdog";
	int wdfd;
	int ret = 0;
	unsigned int i;
	int carg;

	for (carg = 1; carg < argc && argv[carg][0] == '-'; carg++) {
		if (strcmp(argv[carg], "-d") == 0) {
			carg++;
			if (carg >= argc) {
				fprintf(stderr, "No value given after -d\n");
				exit(EXIT_FAILURE);
			}
			devfile = argv[carg];
		} else if (strcmp(argv[carg], "-h") == 0) {
			print_help(argv[0]);
			exit(0);
		} else if (strcmp(argv[carg], "--") == 0) {
			carg++;
			break;
		}
	}

	wdfd = open(devfile, O_RDWR);
	if (wdfd == -1) {
		perror(devfile);
		exit(EXIT_FAILURE);
	}

	if (carg >= argc) {
		status(wdfd, 0, NULL);
		goto out;
	}

	for (i = 0; handlers[i].name; i++) {
		if (strcmp(handlers[i].name, argv[carg]) == 0) {
			ret = handlers[i].handler(wdfd, argc - carg - 1,
						  &argv[carg + 1]);
			break;
		}
	}
	if (!handlers[i].name) {
		fprintf(stderr, "Unknown operation: %s\n", argv[carg]);
		ret = 1;
	}
out:
	close(wdfd);
	return ret;
}
