# R8169 Optimized Driver - Zero-Copy RX for Low Latency

이 드라이버는 SOEM (EtherCAT) 같은 실시간 애플리케이션을 위해 최적화된 Realtek RTL8168/8111 이더넷 드라이버입니다.

## 최적화 내용

1. **이중 DMA 동기화 제거**

   - 기존: `dma_sync_single_for_cpu()` + `dma_sync_single_for_device()`
   - 변경: `dma_sync_single_for_cpu()` 만 사용 (캐시 오버헤드 감소)

2. **Prefetch 최적화**

   - `prefetch()` → `net_prefetch()` 변경
   - SKB 할당 전에 데이터 prefetch 시작 (지연시간 숨김)

3. **모듈 설명 업데이트**
   - Low Latency 최적화 버전임을 명시

## 빌드 방법

```bash
# 디렉토리 이동
cd /lib/modules/6.8.0-rt8-C2-LAXTIVE-1.0.0/build/drivers/net/ethernet/realtek/r8169_optimized

# 빌드
make

# 결과 확인
ls -la *.ko
```

## 설치 및 사용

```bash
# 기존 드라이버 언로드 및 새 드라이버 로드
make install

# 드라이버 확인
lsmod | grep r8169
dmesg | tail -20

# 네트워크 인터페이스 확인
ip link show
```

## 원래 드라이버로 복구

```bash
make uninstall
# 또는 수동으로:
sudo rmmod r8169_optimized
sudo modprobe r8169
```

## 성능 테스트

### SOEM 테스트

```bash
sudo ./simple_test eth0
```

### 인터럽트 지연 측정

```bash
sudo cyclictest -t1 -p80 -i1000 -l10000
```

## 예상 개선 효과

| 항목            | 기존        | 최적화      |
| --------------- | ----------- | ----------- |
| DMA sync 호출   | 2회/패킷    | 1회/패킷    |
| Prefetch 타이밍 | SKB 할당 후 | SKB 할당 전 |
| 캐시 오버헤드   | 높음        | 낮음        |

## 주의사항

- 이 드라이버는 테스트 및 개발 목적입니다
- 운영 환경 적용 전 충분한 테스트가 필요합니다
- 문제 발생 시 `make uninstall`로 원래 드라이버 복구
