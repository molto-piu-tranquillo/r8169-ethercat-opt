# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for r8169 TX-IRQ-OFF optimized driver
#

KVER ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KVER)/build

# Module name
obj-m := r8169_tx_off.o
r8169_tx_off-y := r8169_main.o r8169_firmware.o r8169_phy_config.o r8169_leds.o

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
	-sudo rmmod r8169 2>/dev/null || true
	@echo "Loading TX-IRQ-OFF optimized driver..."
	sudo insmod ./r8169_tx_off.ko
	@echo "Done. Check with: lsmod | grep r8169"

# Uninstall
uninstall:
	@echo "Unloading TX-IRQ-OFF optimized driver..."
	-sudo rmmod r8169_tx_off 2>/dev/null || true
	@echo "Reloading original driver..."
	sudo modprobe r8169
	@echo "Done."

.PHONY: all clean install uninstall