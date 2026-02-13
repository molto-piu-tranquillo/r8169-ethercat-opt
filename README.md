# R8169 Low-Latency Driver Experiments

RTL8168H 네트워크 드라이버의 저지연 최적화 실험 저장소.
SOEM EtherCAT 마스터의 send-recv 레이턴시 개선을 위한 다양한 접근 방식을 실험합니다.

## 폴더 구조

### Kernel bypass

| 폴더 | 설명 |
|------|------|
| `r8169_uio/` | **UIO userspace driver** — DPDK-style 커널 바이패스, zero-syscall TX/RX. 현재 최적 드라이버 |
| `r8169_xdp/` | AF_XDP 지원 드라이버 — XSK zero-copy, busy-poll NAPI |

### Kernel driver 최적화 (실험 완료)

| 폴더 | 설명 |
|------|------|
| `r8169_optimized/` | NAPI/인터럽트 경로 최적화 실험 |
| `r8169_latest/` | 최신 메인라인 드라이버 (6.12+ → 6.8 포팅, RTL8168H EEE 지원) |
| `r8169_8168h_only/` | RTL8168H 전용 경량화 드라이버 |
| `r8169_zerocopy_v2/` | Zero-copy RX (page-flip 방식) |
| `r8169_zerocopy/` | Zero-copy RX 초기 버전 |
| `r8169_tx_off_optimized/` | TX 인터럽트 비활성화 최적화 |
| `r8169_tx_hardirq/` | TX를 하드 인터럽트에서 직접 처리 |
| `r8169_rx_only_poll/` | RX 전용 폴링 모드 |
| `r8169_irq_optimized/` | 인터럽트 처리 최적화 |
| `r8169_ethercat_opt/` | EtherCAT 특화 최적화 |
| `r8169_custom/` | 기타 커스텀 실험 |

루트의 `r8169_*.c` 파일들은 커널 원본 소스입니다.

## 빌드

### UIO (r8169_uio)

```bash
cd r8169_uio
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

### 커널 드라이버 (r8169_xdp 등)

```bash
cd r8169_xdp  # 또는 다른 r8169_*
make
sudo make install    # 기존 드라이버 제거 후 로드
sudo make uninstall  # 원본 드라이버 복원
```

## 환경

- 커널: Linux 6.8.0-rt8 (PREEMPT_RT)
- 하드웨어: Intel N100, RTL8168H
- SOEM: [soem-perf-measure](https://github.com/molto-piu-tranquillo/soem-perf-measure) (feat/uio, feat/af-xdp 브랜치)

## 라이선스

GPL-2.0 (Linux kernel driver)
