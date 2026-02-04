# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for r8169_xdp - XDP/AF_XDP enabled driver
# For low-latency EtherCAT communication
#

KVER ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KVER)/build
PWD := $(shell pwd)

# Module name
obj-m := r8169_xdp.o
r8169_xdp-y := r8169_main.o r8169_firmware.o r8169_phy_config.o r8169_leds.o

# Build flags
ccflags-y += -DCONFIG_R8169_LEDS

# Build
all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

# Clean
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

# Install (requires root) - unloads other r8169 variants and loads this one
install: all
	@echo "=== Installing r8169_xdp (XDP/AF_XDP enabled) ==="
	@echo "Unloading existing r8169 variants..."
	-sudo rmmod r8169_latest 2>/dev/null || true
	-sudo rmmod r8169_optimized 2>/dev/null || true
	-sudo rmmod r8169_tx_off 2>/dev/null || true
	-sudo rmmod r8169_ethercat_opt 2>/dev/null || true
	-sudo rmmod r8169 2>/dev/null || true
	@echo "Loading r8169_xdp driver..."
	sudo insmod ./r8169_xdp.ko
	@echo ""
	@echo "=== Installation complete ==="
	@echo "Verify with: lsmod | grep r8169"
	@echo "Check interface: ip link show"

# Uninstall - unloads this driver and restores original
uninstall:
	@echo "=== Uninstalling r8169_xdp ==="
	@echo "Unloading r8169_xdp driver..."
	-sudo rmmod r8169_xdp 2>/dev/null || true
	@echo "Restoring original r8169 driver..."
	sudo modprobe r8169
	@echo ""
	@echo "=== Uninstall complete ==="

# Install to system modules directory (persistent across reboots)
install-system: all
	@echo "Installing to /lib/modules/$(KVER)/..."
	sudo $(MAKE) -C $(KDIR) M=$(PWD) modules_install
	sudo depmod -a
	@echo "Module installed. Add to /etc/modules for auto-load."

# Quick reload (uninstall + install)
reload: uninstall install

# Status check
status:
	@echo "=== Driver Status ==="
	@lsmod | grep r8169 || echo "No r8169 driver loaded"
	@echo ""
	@echo "=== Network Interfaces ==="
	@ip link show | grep -A1 "enp\|eth"

.PHONY: all clean install uninstall install-system reload status
