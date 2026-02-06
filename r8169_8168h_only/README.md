# r8169_8168h_only

RTL8168H/8111H(`RTL_GIGA_MAC_VER_46`) 전용으로 정리한 out-of-tree 드라이버입니다.

## 목표

- 공용 칩 지원 경로를 제거해 코드 복잡도 축소
- `rtl8168h` 실험/튜닝 시 읽기 쉬운 베이스 제공

## 주요 정리 내용

- PCI ID 매칭: `REALTEK 0x8168`만 유지
- 칩 테이블: `RTL_GIGA_MAC_VER_46`만 유지
- 펌웨어 선언: `rtl_nic/rtl8168h-2.fw`만 유지
- MAC 버전 판별: XID `0x541`(GMII)만 허용
- HW 시작 경로: `rtl_hw_start_8168h_1()` 단일 경로로 고정
- PHY 설정 파일: `r8169_phy_config.c`를 8168h 경로만 남긴 최소 구현으로 교체

## 빌드

```bash
cd r8169_8168h_only
make
```

## 설치/복구

```bash
sudo make install
sudo make uninstall
```

## 주의

- 이 폴더는 실험용 파생 드라이버입니다.
- 비-8168h 칩에서는 probe 단계에서 `-ENODEV`로 거부합니다.
