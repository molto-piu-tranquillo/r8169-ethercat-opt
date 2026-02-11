# r8169_uio — RTL8168H UIO Userspace Driver

DPDK-style userspace NIC driver for RTL8168H (r8169). Bypasses the kernel network stack entirely — direct BAR mmap register access and hugepage-backed DMA descriptor rings for zero-syscall TX/RX.

## Requirements

- Linux with `iommu=pt` kernel parameter
- `uio_pci_generic` kernel module
- Root privileges
- PHY must be initialized by kernel driver at least once (boot with stock r8169)

## Build

```bash
cd r8169_uio
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

`bind_uio.sh`는 `driver_override`를 사용하여 지정된 PCI BDF(0000:01:00.0)만 UIO에 바인딩합니다. 같은 칩셋의 다른 NIC에는 영향 없음.

## SOEM Integration

`~/EtherCAT/SOEM_DEBUG` (feat/uio branch)에서 `-DUSE_UIO=ON`으로 빌드하면 SOEM의 NIC 드라이버 계층이 이 라이브러리를 사용합니다.

```bash
# UIO bind 후
sudo cycle_test_2 eth0          # sysfs로 PCI BDF 자동 변환
sudo cycle_test_2 0000:01:00.0  # 직접 지정도 가능
```

## Files

| File | Description |
|------|-------------|
| `src/r8169_uio.c` | NIC init, TX/RX descriptor ring 관리, BAR mmap |
| `src/r8169_dma.c` | 2MB hugepage 할당, `/proc/self/pagemap` 물리주소 변환 |
| `include/r8169_uio.h` | 레지스터 정의, 디스크립터 구조체, API |
| `scripts/bind_uio.sh` | PCI driver bind/unbind + hugepage 할당/해제 |
| `test/ethercat_test.c` | EtherCAT BRD 프레임 송수신 레이턴시 벤치마크 |

## Architecture

```
┌─────────────┐
│ ethercat_test│  (or SOEM cycle_test_2)
├─────────────┤
│ r8169_uio.c │  r8169_tx() / r8169_rx()
├─────────────┤
│ hugepage DMA│  TX/RX descriptor rings + packet buffers
├─────────────┤
│ BAR mmap    │  NIC register direct access (TxPoll, ChipCmd, ...)
├─────────────┤
│ RTL8168H HW │
└─────────────┘
  No kernel, no IRQ, no NAPI, no socket
```
