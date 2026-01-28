# SPDX-License-Identifier: GPL-2.0-only
# Makefile for r8169 Minimal Latency Optimized v2

KVER ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KVER)/build

obj-m := r8169_zerocopy_v2.o
r8169_zerocopy_v2-y := r8169_main.o r8169_firmware.o r8169_phy_config.o r8169_leds.o

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install: all
	@echo "Unloading any existing r8169 drivers..."
	-sudo rmmod r8169 r8169_optimized r8169_irq_optimized r8169_zerocopy r8169_zerocopy_v2 2>/dev/null || true
	@echo "Loading v2 driver..."
	sudo insmod ./r8169_zerocopy_v2.ko
	@echo "Done. Check: lsmod | grep r8169"

uninstall:
	-sudo rmmod r8169_zerocopy_v2 2>/dev/null || true
	sudo modprobe r8169

.PHONY: all clean install uninstall
