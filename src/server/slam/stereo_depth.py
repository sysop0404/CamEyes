"""
CamEyes - Stereo Depth Estimator
=================================
캘리브레이션된 스테레오 쌍에서 깊이맵과 3D 포인트 클라우드 생성.

파이프라인:
  1. 캘리브레이션 파라미터 로드
  2. 스테레오 렉티피케이션 (왜곡 보정 + 정렬)
  3. 스테레오 매칭 (SGBM) → 시차맵(disparity)
  4. 시차맵 → 깊이맵 → 3D 포인트 클라우드
"""

import cv2
import json
import numpy as np
from pathlib import Path
from dataclasses import dataclass
from typing import Optional


@dataclass
class DepthResult:
    """깊이 추정 결과"""
    disparity: np.ndarray       # 시차맵 (float32)
    depth_map: np.ndarray       # 깊이맵 (float32, meters)
    point_cloud: np.ndarray     # 3D 포인트 (Nx3, meters)
    colors: np.ndarray          # 포인트 색상 (Nx3, BGR 0-255)
    rectified_left: np.ndarray  # 렉티파이된 좌측 이미지
    rectified_right: np.ndarray # 렉티파이된 우측 이미지


class StereoDepthEstimator:
    """스테레오 깊이 추정기"""

    def __init__(self, calibration_dir: str = "src/data/calibration"):
        self.calib_dir = Path(calibration_dir)
        self.calibrated = False

        # 캘리브레이션 파라미터
        self.K_l: Optional[np.ndarray] = None  # 좌측 카메라 매트릭스
        self.D_l: Optional[np.ndarray] = None  # 좌측 왜곡 계수
        self.K_r: Optional[np.ndarray] = None
        self.D_r: Optional[np.ndarray] = None
        self.R: Optional[np.ndarray] = None    # 회전 행렬 (좌→우)
        self.T: Optional[np.ndarray] = None    # 이동 벡터 (좌→우)
        self.image_size: Optional[tuple] = None

        # 렉티피케이션 맵
        self.map_l1 = None
        self.map_l2 = None
        self.map_r1 = None
        self.map_r2 = None
        self.Q = None  # 시차→3D 변환 행렬

        # SGBM 스테레오 매처
        self.stereo = self._create_stereo_matcher()

    def _create_stereo_matcher(self):
        """StereoSGBM 매처 생성 (파라미터는 튜닝 필요)"""
        min_disp = 0
        num_disp = 64          # 16의 배수, 깊이 범위에 비례
        block_size = 7         # 홀수, 3-11

        stereo = cv2.StereoSGBM.create(
            minDisparity=min_disp,
            numDisparities=num_disp,
            blockSize=block_size,
            P1=8 * 3 * block_size ** 2,
            P2=32 * 3 * block_size ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        return stereo

    def load_calibration(self) -> bool:
        """캘리브레이션 파라미터 로드"""
        intrinsic_l_path = self.calib_dir / "cam_left_intrinsic.json"
        intrinsic_r_path = self.calib_dir / "cam_right_intrinsic.json"
        stereo_path = self.calib_dir / "stereo_calibration.json"

        for p in [intrinsic_l_path, intrinsic_r_path, stereo_path]:
            if not p.exists():
                print(f"[Depth] Missing: {p}")
                print(f"[Depth] 먼저 캘리브레이션을 수행하세요")
                return False

        with open(intrinsic_l_path) as f:
            data_l = json.load(f)
        with open(intrinsic_r_path) as f:
            data_r = json.load(f)
        with open(stereo_path) as f:
            data_s = json.load(f)

        self.K_l = np.array(data_l["camera_matrix"])
        self.D_l = np.array(data_l["dist_coeffs"])
        self.K_r = np.array(data_r["camera_matrix"])
        self.D_r = np.array(data_r["dist_coeffs"])
        self.R = np.array(data_s["R"])
        self.T = np.array(data_s["T"])
        self.image_size = tuple(data_l["image_size"])

        self._compute_rectification()
        self.calibrated = True

        baseline_mm = data_s.get("baseline_mm", np.linalg.norm(self.T) * 1000)
        print(f"[Depth] Calibration loaded (baseline: {baseline_mm:.1f}mm)")
        return True

    def _compute_rectification(self):
        """스테레오 렉티피케이션 맵 계산"""
        R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
            self.K_l, self.D_l, self.K_r, self.D_r,
            self.image_size, self.R, self.T,
            alpha=0,  # 0=유효 영역만, 1=전체 영역
        )

        self.map_l1, self.map_l2 = cv2.initUndistortRectifyMap(
            self.K_l, self.D_l, R1, P1, self.image_size, cv2.CV_32FC1
        )
        self.map_r1, self.map_r2 = cv2.initUndistortRectifyMap(
            self.K_r, self.D_r, R2, P2, self.image_size, cv2.CV_32FC1
        )
        self.Q = Q
        print("[Depth] Rectification maps computed")

    def compute(self, img_left: np.ndarray, img_right: np.ndarray) -> Optional[DepthResult]:
        """스테레오 이미지 쌍에서 깊이 추정"""
        if not self.calibrated:
            return self.compute_uncalibrated(img_left, img_right)

        # 1. 렉티피케이션
        rect_l = cv2.remap(img_left, self.map_l1, self.map_l2, cv2.INTER_LINEAR)
        rect_r = cv2.remap(img_right, self.map_r1, self.map_r2, cv2.INTER_LINEAR)

        # 2. 그레이스케일 변환
        gray_l = cv2.cvtColor(rect_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(rect_r, cv2.COLOR_BGR2GRAY)

        # 3. 스테레오 매칭
        disparity = self.stereo.compute(gray_l, gray_r).astype(np.float32) / 16.0

        # 4. 시차 → 3D
        points_3d = cv2.reprojectImageTo3D(disparity, self.Q)
        depth_map = points_3d[:, :, 2]

        # 5. 유효 포인트만 추출
        mask = (disparity > 0) & (depth_map > 0.1) & (depth_map < 10.0)
        valid_points = points_3d[mask].reshape(-1, 3)
        valid_colors = rect_l[mask].reshape(-1, 3)

        return DepthResult(
            disparity=disparity,
            depth_map=depth_map,
            point_cloud=valid_points,
            colors=valid_colors,
            rectified_left=rect_l,
            rectified_right=rect_r,
        )

    def compute_uncalibrated(self, img_left: np.ndarray, img_right: np.ndarray) -> DepthResult:
        """캘리브레이션 없이 간이 깊이 추정 (데모/테스트용)"""
        gray_l = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)

        disparity = self.stereo.compute(gray_l, gray_r).astype(np.float32) / 16.0

        # 간이 깊이 (캘리브레이션 없이 상대적 깊이만)
        depth_map = np.zeros_like(disparity)
        valid = disparity > 0
        depth_map[valid] = 1.0 / (disparity[valid] + 1e-6)  # 상대적 깊이

        return DepthResult(
            disparity=disparity,
            depth_map=depth_map,
            point_cloud=np.zeros((0, 3)),
            colors=np.zeros((0, 3), dtype=np.uint8),
            rectified_left=img_left,
            rectified_right=img_right,
        )

    def visualize_disparity(self, disparity: np.ndarray) -> np.ndarray:
        """시차맵을 컬러맵으로 시각화"""
        disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)
        # 시차 0인 영역은 검정으로
        disp_color[disparity <= 0] = [0, 0, 0]
        return disp_color
