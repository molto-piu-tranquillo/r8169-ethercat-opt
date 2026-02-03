# r8169_latest - Latest Mainline Kernel Driver (6.8 Ported)

이 디렉토리는 Linux 커널 mainline에서 가져온 최신 r8169 드라이버를 **커널 6.8에서 빌드 가능하도록 포팅**한 버전입니다.

## 소스 출처

- **Repository**: https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git
- **Path**: `drivers/net/ethernet/realtek/`
- **Download Date**: 2026-02-02
- **Target Kernel**: 6.8.0-rt8

## 빌드 및 사용법

```bash
# 빌드
make

# 설치 (기존 드라이버 교체)
sudo make install

# 제거 (원본 드라이버 복원)
sudo make uninstall
```

## 6.8 호환성 수정사항

### API 변경

| 최신 API                        | 6.8 대체                                      |
| ------------------------------- | --------------------------------------------- |
| `linux/unaligned.h`             | `asm/unaligned.h`                             |
| `struct ethtool_keee`           | `struct ethtool_eee`                          |
| `pcim_iomap_region()`           | `pcim_iomap_regions()` + `pcim_iomap_table()` |
| `disable_work_sync/enable_work` | `cancel_work_sync/set_bit`                    |
| `phy_support_eee()`             | 제거 (6.8에 없음)                             |
| `phydev->enable_tx_lpi`         | `rtl_supports_eee()` 사용                     |
| `dev->ethtool->wol_enabled`     | 제거 (6.8에 없음)                             |

### RTL8168H EEE 기능 복원

- `tx_lpi_timer` 멤버 및 `rtl_set_eee_txidle_timer()` 함수 유지
- `rtl_enable_tx_lpi()` 호출 복원 (링크 UP 시 TX LPI 활성화)
- `rtl_hw_start()` / `rtl8169_change_mtu()`에서 타이머 설정

### 복원 불가능 (6.8 커널 구조체 한계)

- `ethtool_eee.tx_lpi_timer` 반환 (필드 없음)
- `dev->ethtool->wol_enabled` 추적 (멤버 없음)

## 최신 드라이버 개선사항 (vs 6.8 원본)

- 새 칩셋 지원: RTL8125D, RTL8126A, RTL8127A, RTL8125K, RTL9151A
- RTL8125B 온도 센서(hwmon) 지원
- RTL8168H ASPM 지원 개선
- 각종 버그 수정

## 파일 목록

| 파일                 | 설명               |
| -------------------- | ------------------ |
| `r8169_main.c`       | 메인 드라이버 코드 |
| `r8169.h`            | 헤더 파일          |
| `r8169_firmware.c/h` | 펌웨어 로딩        |
| `r8169_phy_config.c` | PHY 설정           |
| `r8169_leds.c`       | LED 제어           |
