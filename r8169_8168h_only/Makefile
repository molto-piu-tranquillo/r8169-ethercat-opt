# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for rtl8168h-only driver (out-of-tree build)
#

KVER ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KVER)/build

# Module name
obj-m := r8169_8168h_only.o
r8169_8168h_only-y := r8169_main.o r8169_firmware.o r8169_phy_config.o r8169_leds.o

# Build
all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

# Clean
clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean

# Install (requires root)
install: all
	@echo "Unloading original r8169 driver (if loaded)..."
	-sudo rmmod r8169 2>/dev/null || true
	@echo "Loading rtl8168h-only driver..."
	sudo insmod ./r8169_8168h_only.ko
	@echo "Done. Check with: lsmod | grep r8169"

# Uninstall
uninstall:
	@echo "Unloading rtl8168h-only driver..."
	-sudo rmmod r8169_8168h_only 2>/dev/null || true
	@echo "Reloading original driver..."
	sudo modprobe r8169
	@echo "Done."

.PHONY: all clean install uninstall
