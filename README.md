# CamEyes

핸드헬드 스테레오 카메라 리그로 실내 공간을 이동하며 실시간 3D 인식하는 PoC 프로젝트.

## 프로젝트 상태

**Phase 1 소프트웨어 완료, 하드웨어 배송 대기** — 2026-04-17 기준

- [x] 사전조사 (하드웨어, 알고리즘, 기존 사례)
- [x] 하드웨어 확정 및 주문 (ESP32-S3-CAM x2)
- [x] 프로젝트 구조 생성
- [x] Python 환경 구성 (OpenCV 4.13, NumPy 2.4)
- [x] ESP32-S3 펌웨어 스켈레톤 (카메라 센서 자동 감지 포함)
- [x] PC측 스트림 수신기 (더미/웹캠 시뮬레이션 모드)
- [x] IMU UDP 수신기 (시뮬레이션 모드)
- [x] 프레임 동기화 모듈 (타임스탬프 기반)
- [x] 스테레오 깊이 추정 모듈 (SGBM)
- [x] 3D 포인트 클라우드 시각화 모듈
- [x] 캘리브레이션 도구 (ChArUco 보드 생성/캡처/캘리브레이션)
- [x] 통합 파이프라인 (캡처→동기화→깊이→시각화)
- [x] 3D 리그 모델 (FreeCAD 매크로)
- [x] ChArUco 캘리브레이션 보드 이미지 생성 (A3 인쇄 대기)
- [x] 전체 모듈 시뮬레이션 테스트 통과
- [ ] **하드웨어 배송 대기 중** (4/22~27 예상)
- [ ] 보드 실측 → 3D 리그 치수 조정 → 프린트
- [ ] 펌웨어 GPIO 핀 확인 → 업로드
- [ ] 실제 WiFi 스트리밍 테스트
- [ ] ChArUco 보드 인쇄 → 캘리브레이션 실행
- [ ] SLAM 라이브러리 통합 (OpenVINS / ORB-SLAM3)

## 하드웨어

| 역할 | 부품 | 수량 | 상태 |
|------|------|:----:|------|
| 카메라 | **ESP32-S3-CAM** (N16R8, OV2640, Micro USB 내장) | 2 | 주문 완료 |
| IMU | GY-9250 (MPU9250) | 1 | 보유 |
| 마운트 | 3D 프린트 리그 (PETG) | 1 | 모델 완료, 실측 후 출력 |
| 기타 | 배선, 나사, 보조배터리 | - | 보유 |

**주문 금액**: ~₩28,837 (ESP32-S3-CAM with OV2640 x2, 배송비 포함)

## 리그 구성

```
C1(좌) ●────100mm────● C2(우)    스테레오 쌍 (깊이 추정)
          ╲  IMU  ╱
           ╲  ●  ╱               GY-9250 (자세 추적 200Hz)
            핸들                  1/4" 삼각대 마운트
```

최소 2대로 시작 → 검증 후 카메라 추가 확장 가능 (config.yaml만 수정)

## 시스템 아키텍처

```
[이동 유닛]                              [PC]
  ESP32-S3-CAM × 2 ──WiFi MJPEG──┐
  GY-9250 IMU ──────UDP 200Hz────┤→  Visual-Inertial SLAM
  보조배터리                       │   실시간 3D 맵 + 시각화
                                  ┘
```

## 디렉토리 구조

```
CamEyes/
├── docs/                            # 문서
├── models/                          # 3D 모델링
│   └── rig_stereo_v1.py                 FreeCAD 리그 매크로
└── src/                             # 프로그램소스
    ├── firmware/cam_node/               ESP32-S3 펌웨어
    │   ├── platformio.ini
    │   └── src/main.cpp
    ├── server/                          PC측 Python
    │   ├── main.py                      통합 파이프라인
    │   ├── config/config.yaml           설정
    │   ├── capture/
    │   │   ├── stream_receiver.py       MJPEG 스트림 수신
    │   │   ├── imu_receiver.py          IMU UDP 수신
    │   │   └── frame_sync.py            프레임 동기화
    │   ├── calibration/
    │   │   └── charuco_calibrator.py    ChArUco 캘리브레이션
    │   ├── slam/
    │   │   └── stereo_depth.py          스테레오 깊이 추정
    │   └── viz/
    │       └── viewer_3d.py             3D 시각화
    └── data/
        └── calibration/
            └── charuco_board_A3.png     캘리브레이션 보드 (A3 인쇄용)
```

## 빠른 시작

```bash
# Python 환경 설정
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

# 통합 파이프라인 실행 (더미 시뮬레이션 + IMU)
python src/server/main.py --simulate dummy --imu

# 키 조작: q=종료, s=상태, d=깊이맵 토글, 3=3D뷰, c=초기화

# 개별 모듈 테스트
python src/server/capture/stream_receiver.py --simulate dummy
python src/server/capture/imu_receiver.py --simulate
python src/server/calibration/charuco_calibrator.py generate-board
```

## 배송 후 워크플로우

```bash
# 1. 펌웨어 업로드 (PlatformIO)
cd src/firmware/cam_node
pio run --target upload

# 2. 실제 카메라 연결
python src/server/main.py --imu

# 3. 캘리브레이션
python src/server/calibration/charuco_calibrator.py capture
python src/server/calibration/charuco_calibrator.py calibrate-intrinsic --camera cam_left
python src/server/calibration/charuco_calibrator.py calibrate-intrinsic --camera cam_right
python src/server/calibration/charuco_calibrator.py calibrate-stereo

# 4. 스테레오 깊이 + 3D 맵핑
python src/server/main.py --imu   # 'd' 키로 깊이맵 활성화
```

## 개발 로드맵

| Phase | 내용 | 상태 |
|:-----:|------|:----:|
| 1 | 인프라 (펌웨어 + 수신기 + 안정 스트리밍) | SW 완료, HW 대기 |
| 2 | 캘리브레이션 (내부/외부 파라미터) | 도구 완료, 실행 대기 |
| 3 | SLAM 기본 (실시간 3D 포인트 클라우드) | 깊이 모듈 완료, SLAM 통합 대기 |
| 4 | 공간 인식 (밀집 매핑, 객체 검출, 다중 방) | 대기 |
| 5 | (연기) 로봇 모드 | - |
| 6 | (추후) 사물 3D 스캔 | - |

## 예상 성능

| 항목 | 기대치 |
|------|--------|
| 맵 정확도 | 벽 위치 ±10-20cm |
| 위치 추적 | ±5-10cm (IMU 보조) |
| 이동 속도 | 최대 0.2-0.3 m/s |
| 스캔 시간 | 방 1개 3-5분 |

## 기술 스택

| 구분 | 기술 |
|------|------|
| 펌웨어 | PlatformIO / Arduino (ESP32-S3) |
| 스트리밍 | MJPEG over HTTP |
| IMU | UDP 200Hz (MPU9250, bolderflight 라이브러리) |
| SLAM | OpenVINS / ORB-SLAM3 (예정) |
| 깊이 추정 | OpenCV StereoSGBM |
| 캘리브레이션 | OpenCV ChArUco |
| 시각화 | Open3D, matplotlib |
| 언어 | C++ (펌웨어), Python 3.12 (PC) |
