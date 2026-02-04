# AF_XDP EtherCAT Test Suite

Low-latency EtherCAT communication testing with AF_XDP zero-copy.

## Prerequisites

```bash
sudo apt install libbpf-dev libxdp-dev clang llvm libelf-dev
```

## Build

```bash
make
```

## Usage

### 1. Load the r8169_xdp driver

```bash
# Unload original driver
sudo rmmod r8169

# Load XDP-enabled driver
sudo insmod ../r8169_xdp/r8169_xdp.ko
```

### 2. Load XDP program (optional)

```bash
sudo ip link set dev <interface> xdpgeneric obj xdp_ethercat.bpf.o sec xdp
```

### 3. Run test

```bash
sudo ./af_xdp_test -i <interface> -t 10
```

### 4. Cleanup

```bash
sudo ip link set dev <interface> xdpgeneric off
```

## Files

- `xdp_ethercat.bpf.c` - XDP BPF program for EtherCAT filtering
- `af_xdp_test.c` - AF_XDP latency test utility
- `Makefile` - Build system

## Expected Results

With r8169_xdp driver:

- RX latency: ~5-10 µs (vs ~30-50 µs with SOEM)
- Zero-copy buffer handling
- PREEMPT_RT safe
