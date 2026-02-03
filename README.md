# R8169 Low-Latency Driver Experiments

RTL8168/8169 네트워크 드라이버의 저지연 최적화 실험 저장소입니다.  
SOEM EtherCAT 마스터의 send-recv 레이턴시를 31μs 이하로 줄이기 위한 다양한 접근 방식을 실험합니다.

## 폴더 구조

| 폴더                      | 설명                                                              |
| ------------------------- | ----------------------------------------------------------------- |
| `r8169_optimized/`        | **메인 최적화 버전** - 현재 활성 개발 중                          |
| `r8169_latest/`           | **최신 메인라인 드라이버** - 6.12+ → 6.8 포팅 (RTL8168H EEE 지원) |
| `r8169_zerocopy_v2/`      | Zero-copy RX 구현 (page-flip 방식)                                |
| `r8169_zerocopy/`         | Zero-copy RX 초기 버전                                            |
| `r8169_tx_off_optimized/` | TX 인터럽트 비활성화 최적화                                       |
| `r8169_tx_hardirq/`       | TX를 하드 인터럽트에서 직접 처리                                  |
| `r8169_rx_only_poll/`     | RX 전용 폴링 모드                                                 |
| `r8169_irq_optimized/`    | 인터럽트 처리 최적화                                              |
| `r8169_ethercat_opt/`     | EtherCAT 특화 최적화                                              |
| `r8169_custom/`           | 기타 커스텀 실험                                                  |

루트의 `r8169_*.c` 파일들은 커널 6.8 원본 소스입니다.

## r8169_latest 정보

최신 메인라인 커널(6.12+)의 r8169 드라이버를 커널 6.8에서 빌드 가능하도록 포팅한 버전입니다.

**포함된 개선사항:**

- 새 칩셋 지원: RTL8125D, RTL8126A, RTL8127A 등
- RTL8168H ASPM 개선
- RTL8125B 온도 센서(hwmon) 지원
- 각종 버그 수정

**RTL8168H EEE 기능 복원:**

- TX LPI 타이머 설정 (`rtl_set_eee_txidle_timer`)
- TX LPI 활성화 (`rtl_enable_tx_lpi`)

## 빌드 및 설치

각 폴더에서 개별적으로 빌드합니다:

```bash
cd r8169_optimized  # 또는 r8169_latest
make              # 빌드
sudo make install # 기존 드라이버 제거 후 최적화 드라이버 로드
sudo make uninstall # 최적화 드라이버 제거 후 원본 드라이버 복원
```

## 환경

- 커널: Linux 6.8.0-rt8 (PREEMPT_RT 패치)
- 하드웨어: RTL8168H
- 목표: SOEM send-recv ≤ 31μs

## 라이선스

GPL-2.0 (Linux kernel driver)
