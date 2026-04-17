"""
CamEyes - 3D Point Cloud Viewer
================================
스테레오 깊이에서 생성된 포인트 클라우드를 실시간 표시.
matplotlib 기반 (Open3D 설치 전 사용 가능).

기능:
  - 실시간 포인트 클라우드 갱신
  - 카메라 궤적 표시
  - 포인트 클라우드 누적 (맵 구축)
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import threading
import time
from typing import Optional
from collections import deque


class PointCloudViewer:
    """matplotlib 기반 3D 포인트 클라우드 뷰어"""

    def __init__(self, max_points: int = 50000, window_title: str = "CamEyes 3D"):
        self.max_points = max_points
        self.window_title = window_title

        # 누적 포인트 클라우드
        self.points = np.zeros((0, 3))
        self.colors = np.zeros((0, 3))

        # 카메라 궤적
        self.trajectory: list[np.ndarray] = []

        # 스레드 동기화
        self._lock = threading.Lock()
        self._new_data = False

    def add_points(self, points: np.ndarray, colors: Optional[np.ndarray] = None):
        """포인트 클라우드 추가 (누적)"""
        if points.shape[0] == 0:
            return

        with self._lock:
            self.points = np.vstack([self.points, points])
            if colors is not None:
                # BGR → RGB, 0-255 → 0-1
                rgb = colors[:, ::-1].astype(np.float32) / 255.0
                self.colors = np.vstack([self.colors, rgb])
            else:
                default_color = np.full((points.shape[0], 3), 0.5)
                self.colors = np.vstack([self.colors, default_color])

            # 최대 포인트 수 제한
            if self.points.shape[0] > self.max_points:
                # 랜덤 다운샘플링
                idx = np.random.choice(self.points.shape[0], self.max_points, replace=False)
                self.points = self.points[idx]
                self.colors = self.colors[idx]

            self._new_data = True

    def add_camera_pose(self, position: np.ndarray):
        """카메라 위치 추가 (궤적)"""
        with self._lock:
            self.trajectory.append(position.copy())

    def clear(self):
        """포인트 클라우드 초기화"""
        with self._lock:
            self.points = np.zeros((0, 3))
            self.colors = np.zeros((0, 3))
            self.trajectory.clear()

    def show_static(self, title: str = ""):
        """현재 포인트 클라우드를 정적으로 표시 (블로킹)"""
        fig = plt.figure(figsize=(12, 8))
        fig.suptitle(title or self.window_title)
        ax = fig.add_subplot(111, projection="3d")

        with self._lock:
            pts = self.points.copy()
            cols = self.colors.copy()
            traj = [t.copy() for t in self.trajectory]

        if pts.shape[0] > 0:
            # 다운샘플링 (표시 성능)
            if pts.shape[0] > 10000:
                idx = np.random.choice(pts.shape[0], 10000, replace=False)
                pts = pts[idx]
                cols = cols[idx]

            ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2],
                       c=cols, s=0.5, alpha=0.6)

        if traj:
            traj_arr = np.array(traj)
            ax.plot(traj_arr[:, 0], traj_arr[:, 1], traj_arr[:, 2],
                    "r-", linewidth=2, label="Camera path")
            ax.scatter(*traj_arr[-1], c="red", s=100, marker="^",
                       label="Current position")

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.legend()

        # 축 비율 맞추기
        if pts.shape[0] > 0:
            max_range = np.max(np.ptp(pts, axis=0)) / 2
            mid = np.mean(pts, axis=0)
            ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
            ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
            ax.set_zlim(mid[2] - max_range, mid[2] + max_range)

        plt.tight_layout()
        plt.show()

    def get_stats(self) -> dict:
        with self._lock:
            return {
                "total_points": self.points.shape[0],
                "trajectory_length": len(self.trajectory),
                "max_points": self.max_points,
            }


class DepthMapViewer:
    """2D 깊이맵 + 스테레오 뷰 표시 (OpenCV 윈도우)"""

    @staticmethod
    def show(
        left: np.ndarray,
        right: np.ndarray,
        disparity_color: np.ndarray,
        depth_map: Optional[np.ndarray] = None,
        info_text: str = "",
    ) -> int:
        """스테레오 + 깊이맵을 하나의 윈도우에 표시

        Returns: cv2.waitKey 결과
        """
        import cv2

        h, w = left.shape[:2]

        # 상단: 좌측 | 우측
        top_row = np.hstack([left, right])

        # 하단: 시차맵(컬러) | 깊이맵 정보
        if depth_map is not None:
            depth_vis = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_MAGMA)
            depth_vis[depth_map <= 0] = [0, 0, 0]
            bottom_row = np.hstack([disparity_color, depth_vis])
        else:
            info_panel = np.zeros((h, w, 3), dtype=np.uint8)
            cv2.putText(info_panel, "CamEyes Stereo", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            if info_text:
                for i, line in enumerate(info_text.split("\n")):
                    cv2.putText(info_panel, line, (20, 80 + i * 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            bottom_row = np.hstack([disparity_color, info_panel])

        combined = np.vstack([top_row, bottom_row])

        # 스케일링
        scale = min(1.0, 1280 / combined.shape[1])
        if scale < 1.0:
            combined = cv2.resize(combined, None, fx=scale, fy=scale)

        cv2.imshow("CamEyes - Stereo Depth", combined)
        return cv2.waitKey(1) & 0xFF
