# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for r8169 RX-only poll optimized driver
#
# EtherCAT 최적화:
# - TxOK 인터럽트 비활성화
# - Poll에서 TX 처리 제거 (RX only)
# - TX 정리는 xmit에서 on-demand로 수행
#

KVER ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KVER)/build

# Module name
obj-m := r8169_rx_only.o
r8169_rx_only-y := r8169_main.o r8169_firmware.o r8169_phy_config.o r8169_leds.o

# Build
all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

# Clean
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

# Install (requires root)
install: all
	@echo "=== r8169 RX-only Poll Optimized Driver ==="
	@echo "Unloading original r8169 driver (if loaded)..."
	-sudo rmmod r8169 2>/dev/null || true
	@echo "Loading RX-only optimized driver..."
	sudo insmod ./r8169_rx_only.ko
	@echo "Done. Check with: lsmod | grep r8169"
	@echo ""
	@echo "최적화 내용:"
	@echo "  - TxOK 인터럽트 비활성화 (RxOK만 활성)"
	@echo "  - Poll에서 RX만 처리 (TX 오버헤드 제거)"
	@echo "  - TX 정리는 xmit 시점에 on-demand"

# Uninstall
uninstall:
	@echo "Unloading optimized driver..."
	-sudo rmmod r8169_rx_only 2>/dev/null || true
	@echo "Reloading original driver..."
	sudo modprobe r8169
	@echo "Done."

.PHONY: all clean install uninstall
