# DesmoSAT Firmware Spec (v0.9, 2023 Project)

> 모듈형 캔위성 **DesmoSAT** 펌웨어 레포(DesmoSAT-FW-2023)를 위한 통합 문서.  
> 본 문서는 **임무제안서(2023)** 및 HW 설명을 바탕으로 **펌웨어 구현 규격**과 **빌드/테스트 절차**를 명확히 정의한다.

---

## TL;DR
- **버스 구조**: 모듈 간 **CAN 2.0B / 11-bit / 500 kbps**, 리틀엔디안, 버전 필드 포함
- **지상국 링크**: **RF 433 MHz 1 W ↔ MCU UART** (115200-8N1 권장), TLV 패킷 + CRC16-CCITT
- **모듈 역할**
  - **Common**: 비행 컴퓨터(FC), 센서 허브(IMU/GPS/CDS), RF 링크, 데이터 수집·연산·중재
  - **Custom A**: 화산 임무(가스/분진) 페이로드
  - **Custom B**: 정찰 임무(방사능) 페이로드
  - **Landing**: 낙하/착지(TOF 거리, IR/VL 카메라 I/F, 낙하산 라인 제어)
  - **BMS/Power**: 배터리 보호, 전류/전압 모니터, 모듈 파워 게이팅
  - **Ground Control**: RF 수신, 파싱/시각화, 로그 저장
- **상태머신**: `BOOT → INIT → DISCOVERY → NOMINAL → (SAFE|FAULT) → SHUTDOWN`
- **품질 기준**: 하트비트 10 Hz, 버스오프 자동복구, 리셋원인 로깅, 빌드 재현성(CI)

---

## 1) 저장소 구조
~~~
desmosat-fw-2023/
├─ common/                 # 공통 라이브러리(드라이버/프로토콜/로깅/유틸)
│  ├─ include/             # can_proto.h, rf_proto.h, board_ids.h, error_codes.h, log.h
│  ├─ src/                 # can_wrap.c, crc16.c, ringbuf.c, log_rtt.c, tsync.c
│  └─ drivers/             # mcp2515.c, vl53l1x_port.c, bno085_port.c, eeprom_at24.c ...
├─ modules/
│  ├─ common_fc/           # Common Module FW (STM32F4, bxCAN)
│  ├─ custom_a/            # Gas/PM payload (STM32F4)
│  ├─ custom_b/            # Radiation payload (STM32F1)
│  ├─ landing/             # Landing (STM32L4 + MCP2515 via SPI)
│  └─ bms/                 # Battery & BMS (STM32L0, UART diag)
├─ gcs/                    # Ground Control MCU FW (STM32F1), parser/forwarder
├─ tools/                  # 빌드/플래시/테스트 스크립트
├─ docs/                   # 사양서/다이어그램/테스트 보고서
└─ README.md               # 레포 개요(본 문서 링크)
~~~

> **주요 결정**: Landing은 L4 시리즈(CAN 미탑재)이므로 **외장 CAN 컨트롤러(MCP2515+TJA1050, SPI)** 사용을 표준으로 한다. (HW가 UART만 제공되는 경우, Common과 UART 대체 링크 허용)

---

## 2) 툴체인 & 빌드
- **IDE/Compiler**: STM32CubeIDE 1.14.x / arm-none-eabi-gcc 12.x
- **C 표준**: C11, `-O2 -ffunction-sections -fdata-sections -fmessage-length=0 -Wall -Wextra`
- **링커**: `--gc-sections`, 맵파일 출력, 스택/힙 여유 ≥ 20%
- **포맷터/정적분석**: `clang-format`(LLVM 스타일), `cppcheck`, (선택) `clang-tidy`
- **CI**: GitHub Actions로 모듈별 헤드리스 빌드 + 아티팩트(elf/bin/map) 업로드
- **버전**: `common/include/version.h`에서 `FW_VER_MAJOR.MINOR.PATCH` 관리

~~~bash
# 예시: 공통 빌드 스크립트
./tools/build.sh modules/common_fc
./tools/flash_stlink.sh modules/common_fc/build/firmware.bin
~~~

---

## 3) 시간/ID/패킷 규격
### 3.1 시간 동기
- Common이 1 Hz **TimeSync** 브로드캐스트(`MSG_TIME_SYNC`) → 모듈은 로컬 타임슬롯 보정
- 타임스탬프 단위: **ms (uint32_t)**, 롤오버 처리

### 3.2 노드/메시지 ID
- **NodeID (4-bit)**: `0x1(Common), 0x2(CustomA), 0x3(CustomB), 0x4(Landing), 0x5(BMS)`
- **FW 버전(1-byte)**: 모든 텔레메트리 헤더에 포함
- **엔디안**: **리틀엔디안**(STM32 기본)

---

## 4) CAN 2.0B 프로토콜 (11-bit, 500 kbps)
- **물리**: SN65HVD231 호환, 60%~70% 샘플 포인트, 버스 터미네이션 120 Ω 양단
- **오류 처리**: TEC/REC 모니터링, 에러패시브/버스오프 진입 시 자동 복귀(1s 대기)

### 4.1 메시지 맵
| ID (hex) | 주기 | 방향 | 이름 | DLC | 페이로드(리틀엔디안) |
|---:|:---:|:---:|---|:---:|---|
| 0x100 | 10 Hz | Any→All | **HEARTBEAT** | 4 | `u16 uptime_s, u8 state, u8 node_id` |
| 0x110 | 5 Hz  | BMS→Common | **BMS_STATUS** | 8 | `u16 vbatt_mV, i16 ibatt_mA, i16 temp_c10, u16 flags` |
| 0x120 | 5 Hz  | Common→All | **SYS_STATUS** | 8 | `u8 fw_ver, u8 errors, u16 reserved, u32 timestamp_ms` |
| 0x130 | 10 Hz | Common→All | **IMU_SHORT** | 8 | `i16 ax, i16 ay, i16 az, i16 gx` (LSB: mg/dps) |
| 0x140 | 2 Hz  | CustomA→Common | **GAS1** | 8 | `u16 so2_ppm, u16 co_ppm, u16 co2_ppm, u16 tvoc_ppb` |
| 0x141 | 2 Hz  | CustomA→Common | **GAS2_PM** | 8 | `u16 pm1, u16 pm25, u16 pm10, u16 ch4_ppm` |
| 0x150 | 5 Hz  | Common→Landing | **LAND_CMD** | 4 | `u8 cmd, u8 arg0, u8 arg1, u8 seq` |
| 0x151 | 5 Hz  | Landing→Common | **LAND_STAT** | 8 | `u16 range_mm, u16 servo_mdeg, u16 flags, u16 temp_c10` |
| 0x1A0 | OnDemand | Common→Any | **CFG_WRITE** | 8 | `u8 key, u8 len, u32 val, u8 crc, u8 ver` |
| 0x1A1 | OnDemand | Any→Common | **CFG_ACK** | 2 | `u8 key, u8 rc` |

> ID는 **모듈 네임스페이스 + 기능**으로 고정. 상세는 `common/include/can_proto.h` 참조.

### 4.2 C 헤더 예시 (`common/include/can_proto.h`)
~~~c
#pragma once
#include <stdint.h>

#define CAN_BITRATE_KBPS 500
#define CAN_VER          0x01

typedef enum { NODE_COMMON=1, NODE_CUSTOMA, NODE_CUSTOMB, NODE_LANDING, NODE_BMS } node_id_t;
typedef enum { ST_BOOT=0, ST_INIT, ST_DISCOVERY, ST_NOMINAL, ST_SAFE, ST_FAULT, ST_SHUTDOWN } sys_state_t;

typedef struct __attribute__((packed)) {
    uint16_t uptime_s;
    uint8_t  state;
    uint8_t  node_id;
} can_heartbeat_t;

#define CAN_ID_HEARTBEAT   0x100
#define CAN_ID_BMS_STATUS  0x110
#define CAN_ID_SYS_STATUS  0x120
#define CAN_ID_IMU_SHORT   0x130
#define CAN_ID_GAS1        0x140
#define CAN_ID_GAS2_PM     0x141
#define CAN_ID_LAND_CMD    0x150
#define CAN_ID_LAND_STAT   0x151
~~~

---

## 5) 지상국 RF/UART 프로토콜
- **물리**: UART **115200-8N1**, RF 모듈(433 MHz 1 W) 투명전송
- **프레이밍**: **0xA5 0x5A** 헤더 + **TLV**(Type/Len/Value) + **CRC16-CCITT(0x1021, init 0xFFFF)**

### 5.1 패킷
~~~
+--------+--------+--------+--------+-----+--------------------+--------+
| 0xA5   | 0x5A   | VER(1)| TYPE(1)| LEN |  PAYLOAD ...       | CRC16  |
+--------+--------+--------+--------+-----+--------------------+--------+
~~~
- `TYPE`: 0x01=Heartbeat, 0x02=SysStatus, 0x10=Gas1, 0x11=Gas2/PM, 0x20=IMU, 0x30=GPS, 0x40=Landing
- 큰 데이터(예: IR/VL 이미지)는 **조각 전송(CHUNK)**: `TYPE=0x90`, `chunk_id(2), total(2), index(2), payload(up to 180B)`

### 5.2 C 인터페이스 (`common/include/rf_proto.h`)
~~~c
#pragma once
#include <stdint.h>
#define RF_HDR0 0xA5
#define RF_HDR1 0x5A
typedef enum { RF_T_HEART=0x01, RF_T_SYS=0x02, RF_T_GAS1=0x10, RF_T_GAS2=0x11, RF_T_IMU=0x20, RF_T_GPS=0x30, RF_T_LAND=0x40, RF_T_CHUNK=0x90 } rf_type_t;
typedef struct __attribute__((packed)) { uint8_t h0,h1,ver,type,len; } rf_hdr_t;
uint16_t crc16_ccitt(const uint8_t* data, uint16_t len);
~~~

---

## 6) 모듈별 상태머신
### 6.1 Common
- **BOOT**(WDT/Reset log) → **INIT**(센서/버스 초기화) → **DISCOVERY**(노드 Heartbeat 대기) → **NOMINAL**(수집/연산/전송)  
- Fault 조건: 버스오프>3회/분, 전압 저하, 센서 초기화 실패 → **SAFE**(샘플링 저하, RF 최소화)

### 6.2 Custom A (화산 임무)
- **WARMUP**(가스 센서 워밍업) → **SAMPLE**(2 Hz) → **REPORT**(CAN 전송)  
- PM 센서는 1 Hz 집계 후 전송, 스파이크는 Hampel 필터(윈도 7 샘플)

### 6.3 Custom B (방사능)
- **SAMPLE**(계수기 누적) → **REPORT**(CPM/μSv/h 변환값, 캘리브레이션 계수 적용)

### 6.4 Landing
- **DESCENT**(ToF/IMU 추적) → **GUIDE**(라인/서보 보정) → **TOUCHDOWN**(범퍼 스위치 or ToF 임계)  
- **카메라 프레임**은 CAN이 아닌 **로컬 버퍼**에 저장 후 **RF로 조각 전송**(필요 시)

### 6.5 BMS
- **MON**(전압/전류/온도) → 임계치 이하시 **LOAD SHED**(모듈 파워 게이팅) → 회복 시 **RESTORE**

---

## 7) 안전/신뢰성
- **Watchdog**: Window WDG(예: 200 ms), `kick` 미스 로깅
- **BOR**: 2.9 V 레벨, Reset Cause 레지스터 기록
- **Failsafe**: RF 링크 상실 시 로그 로컬 저장, 전력 부족 시 샘플링률 저하
- **에러코드 규격**: `0xAABB` (AA=서브시스템, BB=세부), 공통 헤더 `error_codes.h`

---

## 8) 캘리브레이션/단위
- **IMU**: 오프셋/스케일 공장 값 저장(EEPROM), 온도에 따른 재보정 옵션
- **가스/PM/방사능**: 각 센서 데이터시트의 정식 변환식을 사용하고 **원시값(raw)** 함께 전송
- **단위 표준**: SI(℃, hPa, ppm, ppb, μSv/h, mg/m³, mm, mdeg)

---

## 9) 테스트(Bring-up & HIL)
- **CAN 루프백/필터**: 수신/전송 카운터, 에러 플래그, 버스오프 자동복구 확인
- **RF 링버퍼 스트레스**: 5 kB/s 스트림 60 s 무손실 검증(CRC 에러=0)
- **전력 이벤트**: 저전압 시나리오에서 샘플링률 저하/복귀 시간 측정
- **로그**: SWO/RTT 실시간 로그 + SD 카드(선택)

---

## 부록 A — 메시지 상세
(추후 `docs/can_messages.md`로 분리)

### HEARTBEAT (0x100, 10 Hz)
~~~
u16 uptime_s, u8 state, u8 node_id
~~~

### BMS_STATUS (0x110, 5 Hz)
~~~
u16 vbatt_mV, i16 ibatt_mA, i16 temp_c10, u16 flags
flags: bit0=low_batt, bit1=ov_curr, bit2=over_temp, bit3=load_shed
~~~

### GAS1 / GAS2_PM (0x140/0x141, 2 Hz)
~~~
GAS1: u16 so2_ppm, u16 co_ppm, u16 co2_ppm, u16 tvoc_ppb
GAS2_PM: u16 pm1, u16 pm25, u16 pm10, u16 ch4_ppm
~~~

### LAND_CMD / LAND_STAT (0x150/0x151, 5 Hz)
~~~
CMD: u8 cmd(0=idle,1=arm,2=steer_left,3=steer_right,4=deploy), u8 arg0, u8 arg1, u8 seq
STAT: u16 range_mm, u16 servo_mdeg, u16 flags, u16 temp_c10
~~~

---

## 부록 B — 에러코드 예시
~~~
0x01xx: CAN (0x0101=busoff, 0x0102=rxovf)
0x02xx: RF  (0x0201=crc, 0x0202=timeout)
0x03xx: PWR (0x0301=low_batt, 0x0302=ov_curr, 0x0303=temp_high)
0x04xx: SENS(0x0401=imu_fail, 0x0402=tof_fail, 0x0403=gas_fail)
~~~

---

## 부록 C — 구현 체크리스트
- [ ] 공통 `can_proto.h / rf_proto.h / error_codes.h` 생성  
- [ ] 각 모듈 CAN 500 kbps로 통일, 필터 설정  
- [ ] Landing: MCP2515 드라이버 통합(SPI, INT 핀), CAN 합류  
- [ ] RF TLV + CRC16 공통화  
- [ ] Reset cause/Watchdog 로깅 추가  
- [ ] GCS 시리얼 파서 + 간단 GUI(LVGL/Processing/Python) 연동  
