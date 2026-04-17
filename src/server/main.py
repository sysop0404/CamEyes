"""
CamEyes - Main Pipeline
========================
전체 파이프라인 통합: 캡처 → 동기화 → 깊이 추정 → 시각화

사용법:
    # 더미 시뮬레이션 (하드웨어 없이)
    python src/server/main.py --simulate dummy

    # 웹캠 시뮬레이션
    python src/server/main.py --simulate webcam

    # 실제 카메라 (배송 후)
    python src/server/main.py

    # 실제 카메라 + IMU
    python src/server/main.py --imu

키 조작:
    q: 종료
    s: 상태 출력
    d: 깊이맵 토글
    3: 3D 포인트 클라우드 표시 (정적)
    c: 포인트 클라우드 초기화
"""

import cv2
import numpy as np
import time
import argparse
import sys
from pathlib import Path

# 프로젝트 루트를 Python 경로에 추가
PROJECT_ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from server.capture.stream_receiver import MultiCameraManager
from server.capture.frame_sync import FrameSynchronizer, StereoFrame
from server.capture.imu_receiver import IMUReceiver, IMUSimulator
from server.slam.stereo_depth import StereoDepthEstimator
from server.viz.viewer_3d import PointCloudViewer, DepthMapViewer


class CamEyesPipeline:
    """CamEyes 통합 파이프라인"""

    def __init__(self, config_path: str, simulate: str = "", use_imu: bool = False):
        self.simulate = simulate
        self.use_imu = use_imu
        self.show_depth = False
        self.frame_count = 0
        self.fps = 0.0
        self._fps_timer = time.time()
        self._fps_counter = 0

        # 1. 카메라 매니저
        self.camera_mgr = MultiCameraManager(config_path, simulate=simulate)

        # 2. 프레임 동기화
        self.sync = FrameSynchronizer(
            left_id="cam_left",
            right_id="cam_right",
            max_time_diff_ms=50.0,
        )

        # 3. 스테레오 깊이
        self.depth = StereoDepthEstimator()

        # 4. 3D 뷰어
        self.viewer_3d = PointCloudViewer(max_points=100000)

        # 5. IMU
        self.imu: IMUReceiver | None = None
        self.imu_sim: IMUSimulator | None = None

    def start(self):
        print("=" * 55)
        print("  CamEyes Pipeline Starting")
        print("=" * 55)

        # 카메라 시작
        self.camera_mgr.start()

        # IMU 시작
        if self.use_imu:
            if self.simulate:
                self.imu_sim = IMUSimulator(port=9000)
                self.imu_sim.start()
            self.imu = IMUReceiver(port=9000)
            self.imu.start()

        # 캘리브레이션 로드 시도
        if self.depth.load_calibration():
            print("[Pipeline] Calibration loaded - stereo depth enabled")
        else:
            print("[Pipeline] No calibration - using uncalibrated depth (demo mode)")

        time.sleep(1)
        print()
        print("[Pipeline] Ready! Keys: q=quit, s=status, d=depth, 3=3D view, c=clear")
        print()

    def stop(self):
        self.camera_mgr.stop()
        if self.imu:
            self.imu.stop()
        if self.imu_sim:
            self.imu_sim.stop()
        cv2.destroyAllWindows()
        print("[Pipeline] Stopped")

    def run(self):
        """메인 루프"""
        self.start()

        try:
            while True:
                # 1. 프레임 수신
                frames = self.camera_mgr.get_frames()

                # 2. 동기화 모듈에 프레임 입력
                for cam_id, frame_data in frames.items():
                    if frame_data and frame_data.image is not None:
                        self.sync.push_frame(
                            cam_id, frame_data.image, frame_data.timestamp_us
                        )

                # 3. 동기화된 스테레오 프레임 가져오기
                stereo = self.sync.get_stereo_frame()

                if stereo is not None:
                    self.frame_count += 1
                    self._update_fps()

                    # 4. 깊이 추정 (토글)
                    if self.show_depth:
                        result = self.depth.compute(stereo.left, stereo.right)
                        if result:
                            disp_color = self.depth.visualize_disparity(result.disparity)
                            info = (
                                f"Sync frames: {self.sync.sync_count}\n"
                                f"Time diff: {stereo.time_diff_ms:.1f}ms\n"
                                f"Points: {result.point_cloud.shape[0]}\n"
                                f"FPS: {self.fps:.1f}"
                            )
                            if self.imu:
                                imu_data = self.imu.get_latest()
                                if imu_data:
                                    info += f"\nIMU: {self.imu.sample_rate_actual:.0f}Hz"

                            key = DepthMapViewer.show(
                                result.rectified_left,
                                result.rectified_right,
                                disp_color,
                                result.depth_map,
                                info,
                            )

                            # 포인트 클라우드 누적
                            if result.point_cloud.shape[0] > 0:
                                # 매 프레임 포인트 수 제한 (성능)
                                if result.point_cloud.shape[0] > 5000:
                                    idx = np.random.choice(
                                        result.point_cloud.shape[0], 5000, replace=False
                                    )
                                    self.viewer_3d.add_points(
                                        result.point_cloud[idx], result.colors[idx]
                                    )
                                else:
                                    self.viewer_3d.add_points(
                                        result.point_cloud, result.colors
                                    )
                        else:
                            key = self._show_stereo_only(stereo)
                    else:
                        key = self._show_stereo_only(stereo)

                    # 키 처리
                    if key == ord("q"):
                        break
                    elif key == ord("s"):
                        self._print_status()
                    elif key == ord("d"):
                        self.show_depth = not self.show_depth
                        print(f"[Pipeline] Depth {'ON' if self.show_depth else 'OFF'}")
                    elif key == ord("3"):
                        print("[Pipeline] Showing 3D view...")
                        self.viewer_3d.show_static("CamEyes - Accumulated Point Cloud")
                    elif key == ord("c"):
                        self.viewer_3d.clear()
                        print("[Pipeline] Point cloud cleared")
                else:
                    # 동기화 대기 중 — 개별 카메라 표시
                    self._show_individual(frames)
                    key = cv2.waitKey(30) & 0xFF
                    if key == ord("q"):
                        break

        except KeyboardInterrupt:
            print("\n[Pipeline] Interrupted")
        finally:
            self.stop()

    def _show_stereo_only(self, stereo: StereoFrame) -> int:
        """깊이 없이 스테레오 뷰만 표시"""
        left = stereo.left.copy()
        right = stereo.right.copy()

        # 정보 오버레이
        info = f"Stereo #{stereo.frame_number} | diff: {stereo.time_diff_ms:.1f}ms | FPS: {self.fps:.1f}"
        cv2.putText(left, "LEFT", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(right, "RIGHT", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        combined = np.hstack([left, right])
        cv2.putText(combined, info, (10, combined.shape[0] - 10),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # 에피폴라 라인 (수평선 — 캘리브레이션 검증용)
        for y in range(0, combined.shape[0], 40):
            cv2.line(combined, (0, y), (combined.shape[1], y), (0, 100, 0), 1)

        scale = min(1.0, 1280 / combined.shape[1])
        if scale < 1.0:
            combined = cv2.resize(combined, None, fx=scale, fy=scale)

        cv2.imshow("CamEyes - Stereo View", combined)
        return cv2.waitKey(30) & 0xFF

    def _show_individual(self, frames: dict):
        """동기화 전 개별 카메라 표시"""
        imgs = []
        for cam_id, fd in frames.items():
            if fd and fd.image is not None:
                img = fd.image.copy()
                cv2.putText(img, cam_id, (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                imgs.append(img)

        if imgs:
            combined = np.hstack(imgs)
            scale = min(1.0, 1280 / combined.shape[1])
            if scale < 1.0:
                combined = cv2.resize(combined, None, fx=scale, fy=scale)
            cv2.imshow("CamEyes - Waiting for sync...", combined)

    def _update_fps(self):
        self._fps_counter += 1
        elapsed = time.time() - self._fps_timer
        if elapsed >= 1.0:
            self.fps = self._fps_counter / elapsed
            self._fps_counter = 0
            self._fps_timer = time.time()

    def _print_status(self):
        print("\n" + "=" * 50)
        print("  CamEyes Pipeline Status")
        print("=" * 50)
        print(f"  Mode: {'simulate=' + self.simulate if self.simulate else 'LIVE'}")
        print(f"  Stereo frames: {self.sync.sync_count}")
        print(f"  Pipeline FPS: {self.fps:.1f}")
        print(f"  Depth: {'ON' if self.show_depth else 'OFF'}")
        print(f"  Calibrated: {self.depth.calibrated}")

        sync_stats = self.sync.get_stats()
        print(f"  Sync avg diff: {sync_stats['avg_time_diff_ms']}ms")
        print(f"  Sync dropped: {sync_stats['dropped_frames']}")

        cam_status = self.camera_mgr.get_status()
        for cid, s in cam_status.items():
            print(f"  {cid}: fps={s['fps']}, frames={s['frames']}")

        if self.imu:
            imu_data = self.imu.get_latest()
            print(f"  IMU: {self.imu.sample_rate_actual:.0f}Hz, "
                  f"connected={self.imu.connected}")
            if imu_data:
                print(f"    acc=({imu_data.accel_x:.2f}, {imu_data.accel_y:.2f}, {imu_data.accel_z:.2f})")

        viewer_stats = self.viewer_3d.get_stats()
        print(f"  3D points: {viewer_stats['total_points']}")
        print("=" * 50 + "\n")


def main():
    parser = argparse.ArgumentParser(description="CamEyes Main Pipeline")
    parser.add_argument(
        "--simulate", choices=["webcam", "dummy", ""], default="",
        help="Simulation mode"
    )
    parser.add_argument("--imu", action="store_true", help="Enable IMU receiver")
    parser.add_argument(
        "--config",
        default=str(PROJECT_ROOT / "src" / "server" / "config" / "config.yaml"),
    )
    args = parser.parse_args()

    pipeline = CamEyesPipeline(
        config_path=args.config,
        simulate=args.simulate,
        use_imu=args.imu,
    )
    pipeline.run()


if __name__ == "__main__":
    main()
