# CamEyes TODO

마지막 업데이트: 2026-04-17

## 진행 중

- [~] ESP32-S3-CAM 배송 추적 및 수령 준비 (배송 예상 2026-04-22~27)
- [~] ChArUco A3 보드 인쇄 준비 (`src/data/calibration/charuco_board_A3.png`)

## 다음 (Up Next)

- [ ] 보드 수령 후 실측 → `models/rig_stereo_v1.py` 치수 조정 → PETG 프린트
- [ ] ESP32-S3-CAM GPIO 핀아웃 확인 → `src/firmware/cam_node/src/main.cpp` 카메라 핀 매핑 검증
- [ ] PlatformIO 펌웨어 업로드 (`pio run --target upload`) x 2대
- [ ] 실제 WiFi MJPEG 스트리밍 10분+ 안정성 테스트
- [ ] GY-9250 I2C 배선 → IMU UDP 200Hz 실측 수신 확인
- [ ] ChArUco 보드로 좌/우 내부 파라미터 캘리브레이션
- [ ] 스테레오 외부 파라미터(baseline 100mm) 캘리브레이션

## 백로그

### Phase 2 — 캘리브레이션 마무리
- [ ] 캘리브레이션 결과 재투영 오차 < 0.5px 확보
- [ ] IMU-카메라 외부 파라미터 캘리브레이션 (Kalibr 또는 수동)

### Phase 3 — SLAM 기본
- [ ] OpenVINS 또는 ORB-SLAM3 (stereo+IMU) 통합 결정 및 빌드
- [ ] 실시간 포즈 추정 → Open3D 뷰어 연동
- [ ] 이동하며 실시간 스파스 포인트 클라우드 생성

### Phase 4 — 공간 인식
- [ ] 밀집 매핑 (TSDF / voxel 기반) 추가
- [ ] 객체 검출 (YOLO 등) 파이프라인 연동
- [ ] 다중 방 맵 병합 / 루프 클로저 안정화

### Phase 5~6 — 연기/추후
- [ ] (연기) 로봇 모드 / HDR
- [ ] (추후) 사물 3D 스캔 모드 (오프라인 포토그래메트리)

### 기타 기술 부채
- [ ] `src/firmware/cam_node/src/main.cpp:407` TODO — 트리거 프레임 버퍼/플래그 처리
- [ ] 카메라 확장 시 WiFi 대역 재계산 (config.yaml 기반)
- [ ] `docs/` 폴더에 하드웨어 셋업/배선도 문서 정리

## 완료 (최근 N개)

- [x] Phase 1 SW 전체 완료 및 시뮬레이션 테스트 통과 (2026-04-17)
- [x] CLAUDE.md 상태 현행화 커밋 `c96c877` (2026-04-17)
- [x] ChArUco 캘리브레이션 보드 이미지(A3) 생성
- [x] FreeCAD 스테레오 리그 매크로 작성 (`models/rig_stereo_v1.py`)
- [x] 통합 파이프라인 `src/server/main.py` (캡처→동기화→깊이→시각화)
- [x] 스테레오 깊이(SGBM) 및 3D 시각화(Open3D) 모듈
- [x] 프레임 동기화 모듈 (타임스탬프 기반)
- [x] MJPEG 스트림 수신기 및 IMU UDP 수신기 (시뮬레이션 모드 포함)
- [x] ESP32-S3 펌웨어 스켈레톤 (카메라 센서 자동 감지)
- [x] Python 3.12 환경 구성 (OpenCV 4.13, NumPy 2.4)
- [x] 하드웨어 확정 및 ESP32-S3-CAM x2 주문 (~₩28,837)

## 막힘 / 대기

- [!] **ESP32-S3-CAM 배송 대기 (2026-04-22~27 예상)** — 최우선 차단 요소. Phase 1 실 하드웨어 검증, Phase 2 캘리브레이션 실행, Phase 3 SLAM 통합 모두 이 배송에 의존.
- [!] 리그 3D 프린트 대기 — 보드 실측 필요, 배송에 종속
- [!] SLAM 라이브러리(OpenVINS / ORB-SLAM3) 선택 보류 — 실 스트림 FPS/지연 측정 후 결정
