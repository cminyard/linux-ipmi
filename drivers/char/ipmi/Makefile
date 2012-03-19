#
# Makefile for the ipmi drivers.
#

ipmi_si-y := ipmi_si_intf.o ipmi_kcs_sm.o ipmi_smic_sm.o ipmi_bt_sm.o

obj-$(CONFIG_IPMI_HANDLER) += ipmi_msghandler.o
obj-$(CONFIG_IPMI_DEVICE_INTERFACE) += ipmi_devintf.o
obj-$(CONFIG_IPMI_SI) += ipmi_si.o
obj-$(CONFIG_IPMI_SMB) += ipmi_smb.o
obj-$(CONFIG_IPMI_SERIAL) += ipmi_serial.o
obj-$(CONFIG_IPMI_SERIAL_TERMINAL_MODE) += ipmi_serial_terminal_mode.o
obj-$(CONFIG_IPMI_SERIAL_DIRECT) += ipmi_serial_direct.o
obj-$(CONFIG_IPMI_SERIAL_RADISYS_ASCII) += ipmi_serial_radisys_ascii.o
obj-$(CONFIG_IPMI_WATCHDOG) += ipmi_watchdog.o
obj-$(CONFIG_IPMI_POWEROFF) += ipmi_poweroff.o
obj-$(CONFIG_IPMI_EMULATE_RADISYS) += ipmi_radisys.o
obj-$(CONFIG_IPMI_EMULATE_IMB) += ipmi_imb.o
