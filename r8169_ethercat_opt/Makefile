# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for r8169 EtherCAT optimized driver (out-of-tree build)
#

KVER ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KVER)/build

# Module name
obj-m := r8169_ethercat.o
r8169_ethercat-y := r8169_main.o r8169_firmware.o r8169_phy_config.o r8169_leds.o

# Build
all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

# Clean
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

# Install (requires root)
install: all
	@echo "Unloading original r8169 driver (if loaded)..."
	-sudo rmmod r8169 2>/dev/null || true
	@echo "Loading EtherCAT optimized driver..."
	sudo insmod ./r8169_ethercat.ko
	@echo "Done. Check with: lsmod | grep r8169"
	@echo ""
	@echo "To enable busy-poll optimization (recommended for EtherCAT):"
	@echo "  echo 30 | sudo tee /sys/module/r8169_ethercat/parameters/busy_poll_us"

# Uninstall
uninstall:
	@echo "Unloading optimized driver..."
	-sudo rmmod r8169_ethercat 2>/dev/null || true
	@echo "Reloading original driver..."
	sudo modprobe r8169
	@echo "Done."

.PHONY: all clean install uninstall
