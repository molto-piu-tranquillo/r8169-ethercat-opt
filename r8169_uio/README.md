# r8169_uio — RTL8168H UIO Userspace Driver

DPDK-style userspace NIC driver for RTL8168H (r8169). Bypasses the kernel
network stack entirely — direct BAR mmap register access and hugepage-backed
DMA descriptor rings for zero-syscall TX/RX.

Built for EtherCAT send-recv latency optimization on Intel N100 + Linux RT.

## Requirements

- Linux with `iommu=pt` kernel parameter
- `uio_pci_generic` kernel module
- Root privileges
- RTL8168H NIC (PCI vendor:device = 10ec:8168)

## Build

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

## Usage

```bash
# 1. Bind NIC to UIO (unbinds from kernel driver)
sudo ./scripts/bind_uio.sh bind

# 2. Run standalone EtherCAT test
sudo ./build/ethercat_test

# 3. Restore kernel driver
sudo ./scripts/bind_uio.sh unbind
```

## SOEM Integration

[soem-perf-measure](https://github.com/molto-piu-tranquillo/soem-perf-measure) (feat/uio branch)에
r8169_uio 소스가 직접 포함되어 있어 외부 의존성 없이 빌드 가능:

```bash
git clone -b feat/uio https://github.com/molto-piu-tranquillo/soem-perf-measure.git
cd soem-perf-measure && mkdir build && cd build && cmake .. && make
sudo ./test/linux/simple_test/cycle_test_2 eth0
```

## Files

| File | Description |
|------|-------------|
| `src/r8169_uio.c` | NIC init, TX/RX descriptor ring, BAR mmap |
| `src/r8169_dma.c` | 2MB hugepage DMA 할당 |
| `include/r8169_uio.h` | 레지스터 정의, API |
| `scripts/bind_uio.sh` | PCI bind/unbind + hugepage 관리 |
| `test/ethercat_test.c` | EtherCAT BRD 레이턴시 벤치마크 |

## Architecture

```
┌───────────────┐
│ ethercat_test │  (or SOEM cycle_test_2)
├───────────────┤
│  r8169_uio.c  │  r8169_tx() / r8169_rx() — 528B compiled hot path
├───────────────┤
│ 2MB hugepage  │  TX/RX desc rings + packet buffers (1 TLB entry)
├───────────────┤
│   BAR mmap    │  MMIO register access (TxPoll, ChipCmd, ...)
├───────────────┤
│  RTL8168H HW  │
└───────────────┘
  No kernel, no IRQ, no NAPI, no socket, no syscall
```
