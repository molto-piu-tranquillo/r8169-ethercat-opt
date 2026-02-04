# SPDX-License-Identifier: GPL-2.0
#
# AF_XDP EtherCAT Test Makefile
#

CLANG ?= clang
LLC ?= llc
CC ?= gcc

CFLAGS = -O2 -g -Wall
LDFLAGS = -lbpf -lxdp -lelf -lz -lpthread

BPF_CFLAGS = -O2 -g -target bpf -D__TARGET_ARCH_x86

PROGS = af_xdp_test
BPF_OBJS = xdp_ethercat.bpf.o

.PHONY: all clean

all: $(PROGS) $(BPF_OBJS)

af_xdp_test: af_xdp_test.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

xdp_ethercat.bpf.o: xdp_ethercat.bpf.c
	$(CLANG) $(BPF_CFLAGS) -c $< -o $@

clean:
	rm -f $(PROGS) $(BPF_OBJS)

# Load XDP program
load: xdp_ethercat.bpf.o
	@echo "To load XDP program:"
	@echo "  sudo ip link set dev <interface> xdpgeneric obj xdp_ethercat.bpf.o sec xdp"
	@echo ""
	@echo "To unload:"
	@echo "  sudo ip link set dev <interface> xdpgeneric off"

.PHONY: help
help:
	@echo "AF_XDP EtherCAT Test Utility"
	@echo ""
	@echo "Targets:"
	@echo "  all      - Build all programs"
	@echo "  clean    - Remove built files"
	@echo "  load     - Show XDP load commands"
	@echo ""
	@echo "Usage:"
	@echo "  1. Build: make"
	@echo "  2. Load XDP program: sudo ip link set dev <iface> xdpgeneric obj xdp_ethercat.bpf.o sec xdp"
	@echo "  3. Run test: sudo ./af_xdp_test -i <iface>"
