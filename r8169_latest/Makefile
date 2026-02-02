# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for r8169_latest - Latest mainline kernel driver
# Downloaded from kernel.org (master branch)
#

KVER ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KVER)/build

# Module name
obj-m := r8169_latest.o
r8169_latest-y := r8169_main.o r8169_firmware.o r8169_phy_config.o r8169_leds.o

# Build flags
ccflags-y += -DCONFIG_R8169_LEDS

# Build
all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

# Clean
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

# Install (requires root)
install: all
	@echo "Unloading original/other r8169 drivers..."
	-sudo rmmod r8169_optimized 2>/dev/null || true
	-sudo rmmod r8169_tx_off 2>/dev/null || true
	-sudo rmmod r8169 2>/dev/null || true
	@echo "Loading latest mainline r8169 driver..."
	sudo insmod ./r8169_latest.ko
	@echo "Done. Check with: lsmod | grep r8169"

# Uninstall
uninstall:
	@echo "Unloading latest r8169 driver..."
	-sudo rmmod r8169_latest 2>/dev/null || true
	@echo "Reloading original driver..."
	sudo modprobe r8169
	@echo "Done."

.PHONY: all clean install uninstall
