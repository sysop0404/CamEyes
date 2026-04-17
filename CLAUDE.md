# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Status

**Phase 1 진행 중** — 디렉토리 구조, Python 환경, 펌웨어 스켈레톤, 스트림 수신기, 캘리브레이션 도구 작성 완료. ESP32-S3-CAM 배송 대기 중 (2026-04-22~27 예상).

## Project Concept

CamEyes는 **핸드헬드 스테레오 카메라 리그**로 실내 공간을 이동하며 실시간 3D 맵을 구축하는 PoC 프로젝트. 최소 2대 카메라로 시작하여 검증 후 확장. 사물 3D 스캔 모드도 동일 리그로 지원 (오프라인 포토그래메트리). 로봇 모드는 연기.

## Hardware (확정, 주문 완료)

| 역할 | 부품 | 수량 | 비고 |
|------|------|:----:|------|
| 카메라 노드 | **ESP32-S3-CAM** (N16R8, OV2640, Micro USB 내장) | 2 | diymore 스토어, 알리 주문 완료 |
| IMU | **GY-9250** (MPU9250) | 1 | 보유 중 |
| 배선/나사/브레드보드 | — | — | 보유 중 |
| 마운트 | 3D 프린트 리그 (PETG) | 1 | FreeCAD 매크로 작성 완료, 배송 후 실측→출력 |

**주문 금액**: ~₩28,837 (배송비 포함)

### ESP32-S3-CAM vs ESP32-CAM (변경 이유)

초기 설계는 ESP32-CAM x6 + DevKit x2였으나 ESP32-S3-CAM으로 변경:
- **Native USB 내장** → MB 어댑터, DevKit 불필요
- **OPI PSRAM 8MB** → 스트리밍 FPS +40% (18-22fps vs 12-15fps)
- **GPIO 풍부** → IMU I2C 배선 자유 (ESP32-CAM은 핀 충돌)
- **더 저렴** → 구매 품목 1종류로 단순화

### IMU 참고 (GY-9250)

- I2C 주소 0x68, ESP32-S3의 자유 GPIO로 I2C 배선
- 라이브러리: `bolderflight/MPU9250`
- WHO_AM_I (0x75) 확인: `0x71`=진품, `0x70`=MPU6500, `0x68`=가품 (모두 VIO에 사용 가능)
- **실내에서 자기계(magnetometer) 사용 안 함**

## Rig Geometry (현재: 스테레오 2대)

```
C1(좌) ●────100mm────● C2(우)     스테레오 쌍
          ╲  IMU  ╱
           ╲  ●  ╱
            핸들
```

최소 구성으로 시작. 검증 후 카메라 추가 확장 가능 (config.yaml에 추가만 하면 됨).

향후 5대 확장 시: C1 전방, C2 후방, C3/C4 좌우, C5 하방

## Architecture

```
[이동 유닛]                              [PC]
  ESP32-S3-CAM × 2 ──WiFi MJPEG──┐
  GY-9250 IMU ──────UDP 200Hz────┤→  OpenVINS / ORB-SLAM3
  보조배터리                       │   실시간 3D 맵 + 시각화
                                  ┘
```

**핵심 원칙:**
1. **PC가 SLAM 처리** — ESP32-S3로도 SLAM 불가
2. **IMU 필수** — 10fps 카메라에서 IMU 없으면 추적 손실 15-30%
3. **WiFi 대역 관리** — 2대 VGA@10fps = ~6 Mbps (ESP32-S3 WiFi 여유 있음)
4. **프레임 동기화** — NTP 타임스탬프 기반 (2대는 GPIO 트리거 없이도 가능)
5. **ChArUco 캘리브레이션 필수** — SLAM 전에 반드시 수행

## Software Stack

- **펌웨어**: PlatformIO, ESP32-S3 Arduino core
- **스트리밍**: MJPEG over HTTP (포트 81)
- **IMU 전송**: UDP 바이너리 (포트 9000)
- **SLAM**: OpenVINS (multi-camera+IMU) 또는 ORB-SLAM3 stereo+IMU
- **캘리브레이션**: OpenCV ChArUco (cv2.aruco)
- **시각화**: Open3D, matplotlib
- **언어**: C++ (펌웨어), Python 3.12 (PC)

## Directory Structure

```
CamEyes/
├── CLAUDE.md, README.md, requirements.txt   # 루트
├── docs/                    # 문서
├── models/                  # 3D 모델링
│   └── rig_stereo_v1.py         FreeCAD 매크로
└── src/                     # 프로그램소스
    ├── firmware/cam_node/       ESP32-S3 펌웨어 (PlatformIO)
    ├── server/
    │   ├── config/config.yaml   설정 파일
    │   ├── capture/             스트림 수신기
    │   ├── calibration/         캘리브레이션 도구
    │   └── (slam, viz, ...)     추후 개발
    └── data/                    런타임 데이터
```

## Development Phases

1. **Infrastructure** — 펌웨어 + 수신기 + 안정 스트리밍 10분+ ← **진행 중**
2. **Calibration** — 내부/외부 파라미터 캘리브레이션
3. **SLAM basic** — 이동하며 실시간 3D 포인트 클라우드
4. **Spatial perception** — 밀집 매핑, 객체 검출, 다중 방
5. (연기) 로봇 모드 / HDR
6. (추후) 사물 3D 스캔 모드

## Constraints

- ESP32-S3에서 SLAM 실행 시도하지 말 것
- 실내에서 자기계 융합하지 말 것
- 카메라 확장 시 WiFi 대역 계산 먼저 할 것
- 로봇 모드 범위 침범하지 말 것
- 사용자 보유 부품(배선, 나사, IMU) 재구매 안내하지 말 것
