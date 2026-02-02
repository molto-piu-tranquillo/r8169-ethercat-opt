# r8169_latest - Latest Mainline Kernel Driver

이 디렉토리는 Linux 커널 mainline(master)에서 가져온 최신 r8169 드라이버 소스입니다.

## 소스 출처

- **Repository**: https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git
- **Path**: `drivers/net/ethernet/realtek/`
- **Download Date**: 2026-02-02

## 파일 목록

| 파일                 | 설명               |
| -------------------- | ------------------ |
| `r8169_main.c`       | 메인 드라이버 코드 |
| `r8169.h`            | 헤더 파일          |
| `r8169_firmware.c/h` | 펌웨어 로딩 관련   |
| `r8169_phy_config.c` | PHY 설정           |
| `r8169_leds.c`       | LED 제어           |

## 빌드 및 사용법

```bash
# 빌드
make

# 설치 (기존 드라이버 교체)
sudo make install

# 제거 (원본 드라이버 복원)
sudo make uninstall

# 정리
make clean
```

## 주의사항

⚠️ **API 호환성**: 최신 커널 소스를 6.8 커널에서 빌드하므로 컴파일 에러가 발생할 수 있습니다. 에러 발생 시 해당 부분을 수정해야 합니다.

## 변경사항 (vs 6.8 커널)

주요 변경사항은 [kernel.org git log](https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/log/drivers/net/ethernet/realtek/r8169_main.c)에서 확인할 수 있습니다.

- 새 칩셋 지원: RTL8125D, RTL8126A, RTL8127A, RTL8125K, RTL9151A
- RTL8125B 온도 센서(hwmon) 지원
- RTL8168H ASPM 지원 개선
- 각종 버그 수정
