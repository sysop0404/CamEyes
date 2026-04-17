"""
CamEyes - Frame Synchronizer
=============================
2대 카메라의 프레임을 타임스탬프 기반으로 정렬하여
동기화된 스테레오 프레임 쌍을 생성.

동기화 전략:
  - 각 카메라에서 최신 프레임을 가져옴
  - 타임스탬프 차이가 허용 범위(max_time_diff_ms) 이내면 쌍으로 매칭
  - 범위를 초과하면 더 오래된 프레임을 버리고 다음 프레임 대기
"""

import time
import threading
import numpy as np
from dataclasses import dataclass
from typing import Optional
from collections import deque


@dataclass
class StereoFrame:
    """동기화된 스테레오 프레임 쌍"""
    left: np.ndarray                # 좌측 이미지
    right: np.ndarray               # 우측 이미지
    timestamp_us: int               # 평균 타임스탬프
    time_diff_ms: float             # 좌우 타임스탬프 차이 (ms)
    frame_number: int               # 동기화된 프레임 번호
    left_id: str = "cam_left"
    right_id: str = "cam_right"


class FrameSynchronizer:
    """타임스탬프 기반 스테레오 프레임 동기화"""

    def __init__(
        self,
        left_id: str = "cam_left",
        right_id: str = "cam_right",
        max_time_diff_ms: float = 50.0,
        buffer_size: int = 30,
    ):
        self.left_id = left_id
        self.right_id = right_id
        self.max_time_diff_ms = max_time_diff_ms

        # 프레임 버퍼 (각 카메라별)
        self._left_buffer: deque = deque(maxlen=buffer_size)
        self._right_buffer: deque = deque(maxlen=buffer_size)
        self._lock = threading.Lock()

        # 통계
        self.sync_count = 0
        self.drop_count = 0
        self.avg_time_diff_ms = 0.0
        self._time_diffs: deque = deque(maxlen=100)

        # 최신 동기화 프레임
        self.latest_stereo: Optional[StereoFrame] = None

    def push_frame(self, camera_id: str, image: np.ndarray, timestamp_us: int):
        """카메라 프레임 입력"""
        with self._lock:
            entry = (timestamp_us, image)
            if camera_id == self.left_id:
                self._left_buffer.append(entry)
            elif camera_id == self.right_id:
                self._right_buffer.append(entry)

            self._try_sync()

    def get_stereo_frame(self) -> Optional[StereoFrame]:
        """최신 동기화 스테레오 프레임 반환"""
        with self._lock:
            return self.latest_stereo

    def _try_sync(self):
        """버퍼에서 매칭 가능한 프레임 쌍 찾기"""
        while self._left_buffer and self._right_buffer:
            ts_l, img_l = self._left_buffer[0]
            ts_r, img_r = self._right_buffer[0]

            diff_ms = abs(ts_l - ts_r) / 1000.0

            if diff_ms <= self.max_time_diff_ms:
                # 매칭 성공
                self._left_buffer.popleft()
                self._right_buffer.popleft()

                self.sync_count += 1
                self._time_diffs.append(diff_ms)
                self.avg_time_diff_ms = sum(self._time_diffs) / len(self._time_diffs)

                self.latest_stereo = StereoFrame(
                    left=img_l,
                    right=img_r,
                    timestamp_us=(ts_l + ts_r) // 2,
                    time_diff_ms=diff_ms,
                    frame_number=self.sync_count,
                    left_id=self.left_id,
                    right_id=self.right_id,
                )
                return
            else:
                # 더 오래된 프레임 버리기
                if ts_l < ts_r:
                    self._left_buffer.popleft()
                else:
                    self._right_buffer.popleft()
                self.drop_count += 1

    def get_stats(self) -> dict:
        return {
            "synced_frames": self.sync_count,
            "dropped_frames": self.drop_count,
            "avg_time_diff_ms": round(self.avg_time_diff_ms, 2),
            "max_allowed_diff_ms": self.max_time_diff_ms,
            "left_buffer": len(self._left_buffer),
            "right_buffer": len(self._right_buffer),
        }
