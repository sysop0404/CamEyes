"""
CamEyes - ChArUco Stereo Calibration Tool
==========================================
1. ChArUco 보드 이미지 생성/출력
2. 단일 카메라 내부 파라미터 캘리브레이션
3. 스테레오 외부 파라미터 캘리브레이션
4. 캘리브레이션 결과 저장/로드

사용법:
    # ChArUco 보드 이미지 생성 (A3 인쇄용)
    python charuco_calibrator.py generate-board

    # 캘리브레이션 이미지 캡처 (카메라 또는 시뮬레이션)
    python charuco_calibrator.py capture --simulate dummy

    # 내부 파라미터 캘리브레이션
    python charuco_calibrator.py calibrate-intrinsic --camera cam_left

    # 스테레오 캘리브레이션
    python charuco_calibrator.py calibrate-stereo
"""

import cv2
import numpy as np
import yaml
import json
import argparse
from pathlib import Path
from datetime import datetime


class CharucoCalibrator:
    """ChArUco 기반 스테레오 캘리브레이션 도구"""

    def __init__(self, config_path: str):
        with open(config_path, "r") as f:
            self.config = yaml.safe_load(f)

        cal_cfg = self.config["calibration"]["charuco"]
        self.squares_x = cal_cfg["squares_x"]
        self.squares_y = cal_cfg["squares_y"]
        self.square_length = cal_cfg["square_length_mm"] / 1000.0  # meters
        self.marker_length = cal_cfg["marker_length_mm"] / 1000.0
        self.min_captures = self.config["calibration"]["min_captures"]

        # ArUco 딕셔너리
        dict_name = cal_cfg.get("dictionary", "DICT_4X4_50")
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, dict_name)
        )

        # ChArUco 보드 생성
        self.board = cv2.aruco.CharucoBoard(
            (self.squares_x, self.squares_y),
            self.square_length,
            self.marker_length,
            self.aruco_dict,
        )

        # 데이터 경로
        self.data_dir = Path(self.config["calibration"]["data_dir"])
        self.data_dir.mkdir(parents=True, exist_ok=True)

    def generate_board_image(self, output_path: str = "", dpi: int = 300):
        """ChArUco 보드 이미지 생성 (A3 인쇄용)"""
        # A3 at 300 DPI
        img_w = int(297 / 25.4 * dpi)  # 297mm -> pixels
        img_h = int(420 / 25.4 * dpi)  # 420mm -> pixels

        board_img = self.board.generateImage((img_w, img_h), marginSize=int(dpi * 0.4))

        if not output_path:
            output_path = str(self.data_dir / "charuco_board_A3.png")

        cv2.imwrite(output_path, board_img)
        print(f"[Board] ChArUco board saved: {output_path}")
        print(f"  Size: {self.squares_x}x{self.squares_y}")
        print(f"  Square: {self.square_length*1000:.1f}mm")
        print(f"  Marker: {self.marker_length*1000:.1f}mm")
        print(f"  Print at A3, 300 DPI")
        return output_path

    def detect_charuco(self, image: np.ndarray):
        """이미지에서 ChArUco 코너 검출"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image

        # ArUco 마커 검출
        detector_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(self.aruco_dict, detector_params)
        marker_corners, marker_ids, rejected = detector.detectMarkers(gray)

        if marker_ids is None or len(marker_ids) < 4:
            return None, None, image

        # ChArUco 코너 보간
        retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            marker_corners, marker_ids, gray, self.board
        )

        # 시각화
        vis_img = image.copy()
        cv2.aruco.drawDetectedMarkers(vis_img, marker_corners, marker_ids)
        if charuco_corners is not None and len(charuco_corners) > 0:
            cv2.aruco.drawDetectedCornersCharuco(vis_img, charuco_corners, charuco_ids)

        if retval < 6:  # 최소 6개 코너 필요
            return None, None, vis_img

        return charuco_corners, charuco_ids, vis_img

    def calibrate_intrinsic(self, camera_id: str):
        """단일 카메라 내부 파라미터 캘리브레이션"""
        img_dir = self.data_dir / camera_id
        if not img_dir.exists():
            print(f"[Calibrate] No images found at {img_dir}")
            print(f"  먼저 'capture' 명령으로 이미지를 캡처하세요")
            return None

        image_files = sorted(img_dir.glob("*.jpg")) + sorted(img_dir.glob("*.png"))
        if len(image_files) < self.min_captures:
            print(f"[Calibrate] Need at least {self.min_captures} images, found {len(image_files)}")
            return None

        print(f"[Calibrate] Processing {len(image_files)} images for {camera_id}...")

        all_corners = []
        all_ids = []
        image_size = None

        for img_path in image_files:
            img = cv2.imread(str(img_path))
            if img is None:
                continue

            if image_size is None:
                image_size = (img.shape[1], img.shape[0])

            corners, ids, _ = self.detect_charuco(img)
            if corners is not None:
                all_corners.append(corners)
                all_ids.append(ids)

        if len(all_corners) < 5:
            print(f"[Calibrate] Only {len(all_corners)} valid detections. Need more images.")
            return None

        print(f"[Calibrate] Valid detections: {len(all_corners)}/{len(image_files)}")

        # 캘리브레이션 실행
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
            all_corners, all_ids, self.board, image_size, None, None
        )

        print(f"[Calibrate] Reprojection error: {ret:.4f} px")
        if ret > 1.0:
            print(f"  WARNING: Error > 1.0px. 이미지 품질/보드 인쇄 확인 필요")

        # 결과 저장
        result = {
            "camera_id": camera_id,
            "image_size": list(image_size),
            "reprojection_error": float(ret),
            "camera_matrix": camera_matrix.tolist(),
            "dist_coeffs": dist_coeffs.tolist(),
            "calibrated_at": datetime.now().isoformat(),
            "num_images": len(all_corners),
        }

        output_path = self.data_dir / f"{camera_id}_intrinsic.json"
        with open(output_path, "w") as f:
            json.dump(result, f, indent=2)

        print(f"[Calibrate] Saved: {output_path}")
        print(f"  Focal length: fx={camera_matrix[0,0]:.1f}, fy={camera_matrix[1,1]:.1f}")
        print(f"  Principal point: cx={camera_matrix[0,2]:.1f}, cy={camera_matrix[1,2]:.1f}")

        return result

    def calibrate_stereo(self):
        """스테레오 외부 파라미터 캘리브레이션"""
        pair = self.config["stereo"]["pair"]
        cam_l_id, cam_r_id = pair[0], pair[1]

        # 내부 파라미터 로드
        intrinsic_l = self._load_intrinsic(cam_l_id)
        intrinsic_r = self._load_intrinsic(cam_r_id)
        if not intrinsic_l or not intrinsic_r:
            print("[Stereo] 먼저 양쪽 카메라의 내부 파라미터 캘리브레이션을 완료하세요")
            return None

        K_l = np.array(intrinsic_l["camera_matrix"])
        D_l = np.array(intrinsic_l["dist_coeffs"])
        K_r = np.array(intrinsic_r["camera_matrix"])
        D_r = np.array(intrinsic_r["dist_coeffs"])
        image_size = tuple(intrinsic_l["image_size"])

        # 스테레오 이미지 쌍 로드
        img_dir_l = self.data_dir / cam_l_id
        img_dir_r = self.data_dir / cam_r_id
        files_l = sorted(img_dir_l.glob("*.jpg")) + sorted(img_dir_l.glob("*.png"))
        files_r = sorted(img_dir_r.glob("*.jpg")) + sorted(img_dir_r.glob("*.png"))

        if len(files_l) != len(files_r):
            print(f"[Stereo] Image count mismatch: L={len(files_l)}, R={len(files_r)}")
            return None

        obj_points = []
        img_points_l = []
        img_points_r = []

        for fl, fr in zip(files_l, files_r):
            img_l = cv2.imread(str(fl))
            img_r = cv2.imread(str(fr))
            if img_l is None or img_r is None:
                continue

            corners_l, ids_l, _ = self.detect_charuco(img_l)
            corners_r, ids_r, _ = self.detect_charuco(img_r)

            if corners_l is None or corners_r is None:
                continue

            # 공통 ID만 사용
            common_ids = np.intersect1d(ids_l.flatten(), ids_r.flatten())
            if len(common_ids) < 6:
                continue

            mask_l = np.isin(ids_l.flatten(), common_ids)
            mask_r = np.isin(ids_r.flatten(), common_ids)

            pts_l = corners_l[mask_l]
            pts_r = corners_r[mask_r]

            # 3D 오브젝트 포인트
            obj_pts = self.board.getChessboardCorners()[common_ids]

            obj_points.append(obj_pts.astype(np.float32))
            img_points_l.append(pts_l.astype(np.float32))
            img_points_r.append(pts_r.astype(np.float32))

        if len(obj_points) < 5:
            print(f"[Stereo] Only {len(obj_points)} valid pairs. Need more.")
            return None

        print(f"[Stereo] Calibrating with {len(obj_points)} image pairs...")

        ret, K_l, D_l, K_r, D_r, R, T, E, F = cv2.stereoCalibrate(
            obj_points, img_points_l, img_points_r,
            K_l, D_l, K_r, D_r, image_size,
            flags=cv2.CALIB_FIX_INTRINSIC,
        )

        baseline_mm = np.linalg.norm(T) * 1000
        print(f"[Stereo] Reprojection error: {ret:.4f} px")
        print(f"[Stereo] Baseline: {baseline_mm:.1f} mm")

        result = {
            "pair": pair,
            "reprojection_error": float(ret),
            "R": R.tolist(),
            "T": T.tolist(),
            "E": E.tolist(),
            "F": F.tolist(),
            "baseline_mm": float(baseline_mm),
            "calibrated_at": datetime.now().isoformat(),
            "num_pairs": len(obj_points),
        }

        output_path = self.data_dir / "stereo_calibration.json"
        with open(output_path, "w") as f:
            json.dump(result, f, indent=2)

        print(f"[Stereo] Saved: {output_path}")
        return result

    def _load_intrinsic(self, camera_id: str):
        path = self.data_dir / f"{camera_id}_intrinsic.json"
        if not path.exists():
            print(f"[Load] Not found: {path}")
            return None
        with open(path, "r") as f:
            return json.load(f)

    def capture_session(self, simulate: str = ""):
        """캘리브레이션 이미지 캡처 세션"""
        import sys
        sys.path.insert(0, str(Path(__file__).parent.parent))
        from capture.stream_receiver import MultiCameraManager

        config_path = str(Path(__file__).parent.parent / "config" / "config.yaml")
        manager = MultiCameraManager(config_path, simulate=simulate)
        manager.start()

        cameras = self.config.get("cameras", [])
        for cam in cameras:
            (self.data_dir / cam["id"]).mkdir(parents=True, exist_ok=True)

        print(f"\n[Capture] Calibration capture session")
        print(f"  ChArUco 보드를 카메라 앞에 들고 다양한 각도/거리에서 촬영")
        print(f"  SPACE: 캡처, Q: 종료 (최소 {self.min_captures}장)")
        print()

        capture_count = 0
        import time
        time.sleep(2)

        while True:
            frames = manager.get_frames()
            display_imgs = []

            for cam_id, frame_data in frames.items():
                if frame_data and frame_data.image is not None:
                    corners, ids, vis = self.detect_charuco(frame_data.image)
                    # 검출 상태 표시
                    status = "DETECTED" if corners is not None else "NO BOARD"
                    color = (0, 255, 0) if corners is not None else (0, 0, 255)
                    cv2.putText(vis, f"{cam_id} | {status}", (10, 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    cv2.putText(vis, f"Captured: {capture_count}", (10, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    display_imgs.append(vis)
                else:
                    placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(placeholder, f"{cam_id}: waiting...",
                                (100, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (128, 128, 128), 2)
                    display_imgs.append(placeholder)

            if display_imgs:
                combined = np.hstack(display_imgs)
                scale = min(1.0, 1280 / combined.shape[1])
                if scale < 1.0:
                    combined = cv2.resize(combined, None, fx=scale, fy=scale)
                cv2.imshow("CamEyes - Calibration Capture", combined)

            key = cv2.waitKey(30) & 0xFF
            if key == ord(" "):
                # 모든 카메라에서 동시 저장
                saved = False
                for cam_id, frame_data in frames.items():
                    if frame_data and frame_data.image is not None:
                        fname = f"calib_{capture_count:04d}.jpg"
                        fpath = self.data_dir / cam_id / fname
                        cv2.imwrite(str(fpath), frame_data.image)
                        saved = True
                if saved:
                    capture_count += 1
                    print(f"  Captured #{capture_count}")
            elif key == ord("q"):
                break

        cv2.destroyAllWindows()
        manager.stop()
        print(f"\n[Capture] Total: {capture_count} captures saved to {self.data_dir}")


def main():
    parser = argparse.ArgumentParser(description="CamEyes ChArUco Calibration")
    parser.add_argument(
        "command",
        choices=["generate-board", "capture", "calibrate-intrinsic", "calibrate-stereo"],
        help="Command to execute"
    )
    parser.add_argument("--camera", default="cam_left", help="Camera ID for intrinsic calibration")
    parser.add_argument("--simulate", choices=["webcam", "dummy", ""], default="")
    parser.add_argument(
        "--config",
        default=str(Path(__file__).parent.parent / "config" / "config.yaml"),
    )
    args = parser.parse_args()

    calibrator = CharucoCalibrator(args.config)

    if args.command == "generate-board":
        calibrator.generate_board_image()

    elif args.command == "capture":
        calibrator.capture_session(simulate=args.simulate)

    elif args.command == "calibrate-intrinsic":
        calibrator.calibrate_intrinsic(args.camera)

    elif args.command == "calibrate-stereo":
        calibrator.calibrate_stereo()


if __name__ == "__main__":
    main()
