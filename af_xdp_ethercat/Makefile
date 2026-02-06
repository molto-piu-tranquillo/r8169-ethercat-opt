# SPDX-License-Identifier: GPL-2.0
#
# AF_XDP EtherCAT Test Makefile (libbpf-only version)
#

CLANG ?= clang
CC ?= gcc

CFLAGS = -O2 -g -Wall
LDFLAGS = -lbpf -lelf -lz -lpthread

# BPF compilation needs proper includes
LIBBPF_HEADERS = $(shell pkg-config --cflags libbpf 2>/dev/null)
BPF_CFLAGS = -O2 -g -target bpf \
	-D__TARGET_ARCH_x86 \
	-I/usr/include/x86_64-linux-gnu \
	$(LIBBPF_HEADERS)

PROGS = af_xdp_test
BPF_OBJS = xdp_ethercat.bpf.o

.PHONY: all clean test-only

# Default: build test utility only (BPF optional)
all: $(PROGS)
	@echo ""
	@echo "=== Build complete ==="
	@echo "Run: sudo ./af_xdp_test -i <interface>"

# Test utility only (no BPF dependency)
af_xdp_test: af_xdp_test.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS) -include arpa/inet.h

# BPF program (optional - requires kernel headers)
bpf: $(BPF_OBJS)

xdp_ethercat.bpf.o: xdp_ethercat.bpf.c
	$(CLANG) $(BPF_CFLAGS) -c $< -o $@

clean:
	rm -f $(PROGS) $(BPF_OBJS)

.PHONY: help
help:
	@echo "AF_XDP EtherCAT Test Utility"
	@echo ""
	@echo "Targets:"
	@echo "  all      - Build test utility"
	@echo "  bpf      - Build XDP BPF program (optional)"
	@echo "  clean    - Remove built files"
	@echo ""
	@echo "Usage:"
	@echo "  1. Build: make"
	@echo "  2. Run test: sudo ./af_xdp_test -i <iface>"
