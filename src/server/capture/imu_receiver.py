"""
CamEyes - IMU UDP Receiver
==========================
ESP32 마스터 노드에서 UDP로 전송하는 MPU9250 IMU 데이터 수신.
하드웨어 없이도 시뮬레이션 데이터로 테스트 가능.

UDP 패킷 포맷 (ESP32에서 전송):
  - 4 bytes: timestamp_ms (uint32, little-endian)
  - 6 x 4 bytes: ax, ay, az, gx, gy, gz (float32, little-endian)
  - 총 28 bytes per packet
"""

import struct
import socket
import threading
import time
import math
import numpy as np
from dataclasses import dataclass
from typing import Optional, Callable
from collections import deque


# UDP 패킷 포맷: uint32 timestamp + 6 x float32
IMU_PACKET_FORMAT = "<I6f"  # little-endian: uint32 + 6 floats
IMU_PACKET_SIZE = struct.calcsize(IMU_PACKET_FORMAT)  # 28 bytes


@dataclass
class IMUData:
    """단일 IMU 샘플"""
    timestamp_ms: int       # ESP32 millis()
    timestamp_us: int       # PC 수신 시 마이크로초
    accel_x: float          # m/s^2
    accel_y: float
    accel_z: float
    gyro_x: float           # rad/s
    gyro_y: float
    gyro_z: float

    def to_array(self) -> np.ndarray:
        """[ax, ay, az, gx, gy, gz] numpy 배열"""
        return np.array([
            self.accel_x, self.accel_y, self.accel_z,
            self.gyro_x, self.gyro_y, self.gyro_z
        ])


class IMUReceiver:
    """UDP IMU 데이터 수신기"""

    def __init__(self, port: int = 9000, buffer_size: int = 1000):
        self.port = port
        self.buffer_size = buffer_size
        self.buffer: deque[IMUData] = deque(maxlen=buffer_size)
        self.latest: Optional[IMUData] = None
        self.sample_count = 0
        self.sample_rate_actual = 0.0
        self.connected = False
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._rate_timer = time.time()
        self._rate_counter = 0
        self._callback: Optional[Callable[[IMUData], None]] = None

    def on_data(self, callback: Callable[[IMUData], None]):
        """새 IMU 데이터 수신 시 콜백 등록"""
        self._callback = callback

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()
        print(f"[IMU] Listening on UDP port {self.port}")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)

    def get_latest(self) -> Optional[IMUData]:
        with self._lock:
            return self.latest

    def get_recent(self, n: int = 10) -> list[IMUData]:
        """최근 n개 샘플 반환"""
        with self._lock:
            return list(self.buffer)[-n:]

    def _receive_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", self.port))
        sock.settimeout(1.0)

        while self._running:
            try:
                data, addr = sock.recvfrom(IMU_PACKET_SIZE + 16)
                if not self.connected:
                    self.connected = True
                    print(f"[IMU] Receiving from {addr}")

                if len(data) >= IMU_PACKET_SIZE:
                    ts_ms, ax, ay, az, gx, gy, gz = struct.unpack(
                        IMU_PACKET_FORMAT, data[:IMU_PACKET_SIZE]
                    )
                    ts_us = int(time.time() * 1_000_000)

                    sample = IMUData(
                        timestamp_ms=ts_ms,
                        timestamp_us=ts_us,
                        accel_x=ax, accel_y=ay, accel_z=az,
                        gyro_x=gx, gyro_y=gy, gyro_z=gz,
                    )

                    with self._lock:
                        self.latest = sample
                        self.buffer.append(sample)
                    self.sample_count += 1
                    self._update_rate()

                    if self._callback:
                        self._callback(sample)

            except socket.timeout:
                if self.connected:
                    self.connected = False
                    print("[IMU] Connection timeout")
            except Exception as e:
                if self._running:
                    print(f"[IMU] Error: {e}")

        sock.close()

    def _update_rate(self):
        self._rate_counter += 1
        elapsed = time.time() - self._rate_timer
        if elapsed >= 1.0:
            self.sample_rate_actual = self._rate_counter / elapsed
            self._rate_counter = 0
            self._rate_timer = time.time()


class IMUSimulator:
    """IMU 시뮬레이션 (하드웨어 없이 테스트)"""

    def __init__(self, port: int = 9000, sample_rate: int = 200):
        self.port = port
        self.sample_rate = sample_rate
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._send_loop, daemon=True)
        self._thread.start()
        print(f"[IMU-Sim] Sending to UDP port {self.port} at {self.sample_rate}Hz")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)

    def _send_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        interval = 1.0 / self.sample_rate
        t0 = time.time()

        while self._running:
            t = time.time() - t0
            ts_ms = int(t * 1000) & 0xFFFFFFFF

            # 시뮬레이션: 중력 + 약간의 움직임
            ax = 0.0 + 0.5 * math.sin(t * 0.5)
            ay = 0.0 + 0.3 * math.cos(t * 0.7)
            az = 9.81 + 0.2 * math.sin(t * 0.3)  # 중력 + 노이즈
            gx = 0.01 * math.sin(t * 1.0)  # 약간의 회전
            gy = 0.02 * math.cos(t * 0.8)
            gz = 0.005 * math.sin(t * 1.5)

            packet = struct.pack(IMU_PACKET_FORMAT, ts_ms, ax, ay, az, gx, gy, gz)
            sock.sendto(packet, ("127.0.0.1", self.port))

            time.sleep(interval)

        sock.close()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="CamEyes IMU Receiver")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--simulate", action="store_true", help="Run simulator + receiver")
    args = parser.parse_args()

    if args.simulate:
        sim = IMUSimulator(port=args.port)
        sim.start()

    receiver = IMUReceiver(port=args.port)

    def on_sample(data: IMUData):
        pass  # 콜백 예시

    receiver.on_data(on_sample)
    receiver.start()

    try:
        while True:
            time.sleep(1)
            latest = receiver.get_latest()
            if latest:
                print(
                    f"[IMU] {receiver.sample_rate_actual:.0f}Hz | "
                    f"acc=({latest.accel_x:.2f}, {latest.accel_y:.2f}, {latest.accel_z:.2f}) | "
                    f"gyro=({latest.gyro_x:.4f}, {latest.gyro_y:.4f}, {latest.gyro_z:.4f})"
                )
            else:
                print("[IMU] Waiting for data...")
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()
        if args.simulate:
            sim.stop()
