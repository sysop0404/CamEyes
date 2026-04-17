"""
CamEyes - Multi-Camera MJPEG Stream Receiver
=============================================
ESP32-S3-CAM으로부터 MJPEG 스트림을 수신하거나,
하드웨어 없이 웹캠/더미 데이터로 시뮬레이션.

사용법:
    # 실제 카메라
    python stream_receiver.py

    # 웹캠 시뮬레이션 (하드웨어 없이 테스트)
    python stream_receiver.py --simulate webcam

    # 더미 이미지 시뮬레이션
    python stream_receiver.py --simulate dummy
"""

import cv2
import numpy as np
import threading
import time
import yaml
import argparse
from pathlib import Path
from dataclasses import dataclass, field
from typing import Optional
from urllib.request import urlopen
from urllib.error import URLError


@dataclass
class FrameData:
    """수신된 프레임 데이터"""
    image: np.ndarray
    timestamp_us: int = 0
    camera_id: str = ""
    frame_number: int = 0


class MJPEGReceiver:
    """단일 카메라 MJPEG 스트림 수신기"""

    def __init__(self, camera_id: str, stream_url: str):
        self.camera_id = camera_id
        self.stream_url = stream_url
        self.frame: Optional[FrameData] = None
        self.frame_count = 0
        self.fps_actual = 0.0
        self.connected = False
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._fps_timer = time.time()
        self._fps_counter = 0

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)

    def get_frame(self) -> Optional[FrameData]:
        with self._lock:
            return self.frame

    def _receive_loop(self):
        """MJPEG 스트림 수신 루프"""
        while self._running:
            try:
                stream = urlopen(self.stream_url, timeout=5)
                self.connected = True
                print(f"[{self.camera_id}] Connected to {self.stream_url}")

                buf = b""
                while self._running:
                    chunk = stream.read(4096)
                    if not chunk:
                        break
                    buf += chunk

                    # JPEG SOI (FFD8) 와 EOI (FFD9) 마커 찾기
                    start = buf.find(b"\xff\xd8")
                    end = buf.find(b"\xff\xd9")

                    if start != -1 and end != -1 and end > start:
                        jpg_data = buf[start : end + 2]
                        buf = buf[end + 2 :]

                        img = cv2.imdecode(
                            np.frombuffer(jpg_data, dtype=np.uint8),
                            cv2.IMREAD_COLOR,
                        )
                        if img is not None:
                            self.frame_count += 1
                            timestamp = int(time.time() * 1_000_000)

                            with self._lock:
                                self.frame = FrameData(
                                    image=img,
                                    timestamp_us=timestamp,
                                    camera_id=self.camera_id,
                                    frame_number=self.frame_count,
                                )

                            self._update_fps()

            except (URLError, OSError, Exception) as e:
                self.connected = False
                if self._running:
                    print(f"[{self.camera_id}] Connection error: {e}, retrying in 2s...")
                    time.sleep(2)

    def _update_fps(self):
        self._fps_counter += 1
        elapsed = time.time() - self._fps_timer
        if elapsed >= 1.0:
            self.fps_actual = self._fps_counter / elapsed
            self._fps_counter = 0
            self._fps_timer = time.time()


class WebcamSimulator:
    """웹캠을 사용한 스테레오 시뮬레이션"""

    def __init__(self, camera_id: str, webcam_index: int = 0, crop_side: str = "left"):
        self.camera_id = camera_id
        self.webcam_index = webcam_index
        self.crop_side = crop_side  # "left" or "right" - 웹캠 영상을 좌우로 분할
        self.frame: Optional[FrameData] = None
        self.frame_count = 0
        self.fps_actual = 0.0
        self.connected = False
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._cap: Optional[cv2.VideoCapture] = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)
        if self._cap:
            self._cap.release()

    def get_frame(self) -> Optional[FrameData]:
        with self._lock:
            return self.frame

    def _capture_loop(self):
        self._cap = cv2.VideoCapture(self.webcam_index)
        if not self._cap.isOpened():
            print(f"[{self.camera_id}] ERROR: Cannot open webcam {self.webcam_index}")
            return

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.connected = True
        print(f"[{self.camera_id}] Webcam {self.webcam_index} opened (simulating {self.crop_side})")

        fps_timer = time.time()
        fps_count = 0

        while self._running:
            ret, full_frame = self._cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            # 좌우 분할로 스테레오 시뮬레이션
            h, w = full_frame.shape[:2]
            if self.crop_side == "left":
                img = full_frame[:, : w // 2]
            else:
                img = full_frame[:, w // 2 :]

            img = cv2.resize(img, (640, 480))
            self.frame_count += 1

            with self._lock:
                self.frame = FrameData(
                    image=img,
                    timestamp_us=int(time.time() * 1_000_000),
                    camera_id=self.camera_id,
                    frame_number=self.frame_count,
                )

            fps_count += 1
            if time.time() - fps_timer >= 1.0:
                self.fps_actual = fps_count / (time.time() - fps_timer)
                fps_count = 0
                fps_timer = time.time()

            time.sleep(0.066)  # ~15fps


class DummySimulator:
    """더미 이미지 생성기 (웹캠 없을 때)"""

    def __init__(self, camera_id: str, color=(100, 150, 200)):
        self.camera_id = camera_id
        self.color = color
        self.frame: Optional[FrameData] = None
        self.frame_count = 0
        self.fps_actual = 10.0
        self.connected = True
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._generate_loop, daemon=True)
        self._thread.start()
        print(f"[{self.camera_id}] Dummy simulator started")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)

    def get_frame(self) -> Optional[FrameData]:
        with self._lock:
            return self.frame

    def _generate_loop(self):
        while self._running:
            img = np.full((480, 640, 3), self.color, dtype=np.uint8)
            self.frame_count += 1

            # 프레임 정보 표시
            cv2.putText(img, f"{self.camera_id}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            cv2.putText(img, f"Frame: {self.frame_count}", (20, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
            cv2.putText(img, f"Time: {time.strftime('%H:%M:%S')}", (20, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)

            # 움직이는 원 (동기화 확인용)
            cx = int(320 + 200 * np.sin(time.time() * 2))
            cy = int(240 + 100 * np.cos(time.time() * 2))
            cv2.circle(img, (cx, cy), 30, (0, 255, 0), -1)

            with self._lock:
                self.frame = FrameData(
                    image=img,
                    timestamp_us=int(time.time() * 1_000_000),
                    camera_id=self.camera_id,
                    frame_number=self.frame_count,
                )

            time.sleep(0.1)  # 10fps


class MultiCameraManager:
    """다중 카메라 관리자"""

    def __init__(self, config_path: str, simulate: str = ""):
        self.config = self._load_config(config_path)
        self.receivers = {}
        self.simulate = simulate

    def _load_config(self, path: str) -> dict:
        with open(path, "r") as f:
            return yaml.safe_load(f)

    def start(self):
        cameras = self.config.get("cameras", [])

        for i, cam_cfg in enumerate(cameras):
            cam_id = cam_cfg["id"]

            if self.simulate == "webcam":
                side = "left" if i == 0 else "right"
                self.receivers[cam_id] = WebcamSimulator(cam_id, 0, side)
            elif self.simulate == "dummy":
                colors = [(100, 80, 60), (60, 80, 100)]
                self.receivers[cam_id] = DummySimulator(cam_id, colors[i % 2])
            else:
                url = cam_cfg["stream_url"].format(ip=cam_cfg["ip"])
                self.receivers[cam_id] = MJPEGReceiver(cam_id, url)

            self.receivers[cam_id].start()

        mode_str = f"simulate={self.simulate}" if self.simulate else "live"
        print(f"\n[Manager] Started {len(self.receivers)} cameras ({mode_str})")

    def stop(self):
        for r in self.receivers.values():
            r.stop()
        print("[Manager] All cameras stopped")

    def get_frames(self) -> dict[str, Optional[FrameData]]:
        return {cid: r.get_frame() for cid, r in self.receivers.items()}

    def get_status(self) -> dict:
        return {
            cid: {
                "connected": r.connected,
                "fps": round(r.fps_actual, 1),
                "frames": r.frame_count,
            }
            for cid, r in self.receivers.items()
        }


def display_loop(manager: MultiCameraManager):
    """OpenCV 윈도우에 다중 카메라 스트림 표시"""
    print("\n[Display] Press 'q' to quit, 's' for status\n")

    while True:
        frames = manager.get_frames()
        display_imgs = []

        for cam_id, frame_data in frames.items():
            if frame_data and frame_data.image is not None:
                img = frame_data.image.copy()
                # 오버레이 정보
                cv2.putText(img, f"{cam_id} | F:{frame_data.frame_number}",
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                receiver = manager.receivers[cam_id]
                cv2.putText(img, f"FPS: {receiver.fps_actual:.1f}",
                            (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                display_imgs.append(img)
            else:
                placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(placeholder, f"{cam_id}: No signal",
                            (150, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                display_imgs.append(placeholder)

        if display_imgs:
            # 가로로 나란히 표시
            combined = np.hstack(display_imgs)
            scale = min(1.0, 1280 / combined.shape[1])
            if scale < 1.0:
                combined = cv2.resize(combined, None, fx=scale, fy=scale)
            cv2.imshow("CamEyes - Stereo View", combined)

        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            status = manager.get_status()
            print("\n--- Camera Status ---")
            for cid, s in status.items():
                print(f"  {cid}: connected={s['connected']}, fps={s['fps']}, frames={s['frames']}")
            print()

    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="CamEyes Multi-Camera Receiver")
    parser.add_argument(
        "--simulate", choices=["webcam", "dummy", ""],
        default="",
        help="Simulation mode: webcam (split webcam), dummy (generated images), or empty for live"
    )
    parser.add_argument(
        "--config",
        default=str(Path(__file__).parent.parent / "config" / "config.yaml"),
        help="Path to config.yaml"
    )
    args = parser.parse_args()

    print("=" * 50)
    print("  CamEyes - Multi-Camera Stream Receiver")
    print("=" * 50)

    manager = MultiCameraManager(args.config, simulate=args.simulate)

    try:
        manager.start()
        time.sleep(1)  # 연결 대기
        display_loop(manager)
    except KeyboardInterrupt:
        print("\n[Main] Interrupted")
    finally:
        manager.stop()


if __name__ == "__main__":
    main()
