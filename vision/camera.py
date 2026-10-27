"""
vision/camera.py
----------------
Non-blocking threaded camera reader and mock frame generator for Raspberry Pi 4.

Key Design Choice for Pi 4:
Running cv2.VideoCapture.read() synchronously in the inference loop causes camera
internal buffer lag (latency buildup). This module runs capture in an independent
background thread, always providing the single freshest frame and dropping stale frames.
"""

import time
import threading
from typing import Optional, Tuple, Any
from .config import VisionConfig


class CameraReader:
    """
    Threaded camera interface supporting OpenCV VideoCapture, Picamera2, and Mock sources.
    """

    def __init__(self, config: VisionConfig) -> None:
        self.config = config
        self.width = config.camera_width
        self.height = config.camera_height
        self.fps = config.camera_fps
        self.camera_index = config.camera_index

        self._cap: Any = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()
        self._latest_frame: Any = None
        self._latest_timestamp: float = 0.0
        self._frame_count: int = 0
        self._is_mock = (config.camera_backend == "mock")

    def start(self) -> bool:
        """Initialize camera and start background capture thread."""
        if self._running:
            return True

        if not self._is_mock:
            success = self._init_opencv_camera()
            if not success:
                print(f"[CameraReader] Warning: Could not open camera {self.camera_index}. Falling back to MockCamera.")
                self._is_mock = True

        self._running = True
        self._thread = threading.Thread(target=self._capture_worker, daemon=True)
        self._thread.start()

        # Wait briefly for first frame
        start_wait = time.monotonic()
        while time.monotonic() - start_wait < 1.0:
            with self._lock:
                if self._latest_frame is not None:
                    break
            time.sleep(0.02)

        return True

    def _init_opencv_camera(self) -> bool:
        """Attempt to open hardware camera with OpenCV."""
        try:
            import cv2
        except ImportError:
            print("[CameraReader] OpenCV (cv2) is not installed. Using MockCamera.")
            return False

        try:
            # On Linux / Raspberry Pi, V4L2 is the fastest capture backend
            self._cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
            if not self._cap.isOpened():
                # Fallback to default backend
                self._cap = cv2.VideoCapture(self.camera_index)

            if not self._cap.isOpened():
                return False

            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self._cap.set(cv2.CAP_PROP_FPS, self.fps)
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize internal buffer latency
            return True
        except Exception as e:
            print(f"[CameraReader] Failed to initialize OpenCV camera: {e}")
            return False

    def _capture_worker(self) -> None:
        """Background thread keeping the newest frame in memory."""
        interval = 1.0 / max(1, self.fps)

        while self._running:
            loop_start = time.monotonic()

            if self._is_mock:
                frame = self._generate_mock_frame()
                ret = True
            else:
                try:
                    ret, frame = self._cap.read()
                except Exception:
                    ret, frame = False, None

            if ret and frame is not None:
                with self._lock:
                    self._latest_frame = frame
                    self._latest_timestamp = time.monotonic()
                    self._frame_count += 1

            # Sleep to match target camera FPS
            elapsed = time.monotonic() - loop_start
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _generate_mock_frame(self) -> Any:
        """
        Generate a synthetic RGB frame simulating a person moving laterally.
        Works with numpy if available, or returns a lightweight synthetic structure.
        """
        t = time.monotonic()
        # Person center oscillates horizontally
        norm_x = 0.5 + 0.3 * (0.5 * (1.0 + (t % 6.0) / 3.0 if (t % 6.0) < 3.0 else 1.0 - ((t % 6.0) - 3.0) / 3.0) - 0.5)

        try:
            import numpy as np
            frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            # Draw synthetic background
            frame[:] = (30, 30, 30)

            # Draw synthetic human body (rectangle)
            px_w = int(self.width * 0.20)
            px_h = int(self.height * 0.65)
            cx = int(self.width * norm_x)
            cy = int(self.height * 0.55)
            x1 = max(0, cx - px_w // 2)
            y1 = max(0, cy - px_h // 2)
            x2 = min(self.width - 1, cx + px_w // 2)
            y2 = min(self.height - 1, cy + px_h // 2)

            frame[y1:y2, x1:x2] = (80, 180, 80)  # Greenish figure
            return frame
        except ImportError:
            # Fallback when numpy is not available (e.g. testing in bare environments)
            return {
                "mock": True,
                "width": self.width,
                "height": self.height,
                "mock_person_bbox": (
                    norm_x * self.width - self.width * 0.1,
                    self.height * 0.2,
                    norm_x * self.width + self.width * 0.1,
                    self.height * 0.85,
                ),
            }

    def read(self) -> Tuple[bool, Any, float]:
        """
        Non-blocking read of the latest camera frame.
        
        Returns:
            (ret, frame, timestamp):
                ret (bool): True if frame is valid.
                frame: The image array or mock frame.
                timestamp (float): Monotonic time frame was captured.
        """
        with self._lock:
            if self._latest_frame is None:
                return False, None, 0.0
            return True, self._latest_frame, self._latest_timestamp

    def is_running(self) -> bool:
        return self._running

    def get_frame_count(self) -> int:
        return self._frame_count

    def stop(self) -> None:
        """Stop capture thread and release hardware resources."""
        self._running = False
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

