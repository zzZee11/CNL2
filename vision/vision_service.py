"""
vision/vision_service.py
------------------------
Decoupled vision service pipeline.

Orchestrates:
1. Camera capture (threaded CameraReader)
2. Detection (HumanDetector)
3. Tracking (SinglePersonTracker)
4. Estimation (TargetEstimator)
5. Target Dispatch (polling, callback, or queue interfaces)

Integration Boundary:
Exposes clean, thread-safe interfaces without importing or coupling to navigation code:
- `get_latest_observation() -> TargetObservation`
- `register_callback(callback_fn)`
- `get_target_queue() -> queue.Queue`
"""

import queue
import threading
import time
from typing import Callable, List, Optional, Dict, Any

from .config import VisionConfig
from .camera import CameraReader
from .detector import HumanDetector, create_detector
from .tracker import SinglePersonTracker
from .target_estimator import TargetEstimator
from .target_data import TargetObservation


class VisionService:
    """
    Independent vision pipeline providing continuous target tracking updates.
    """

    def __init__(self, config: Optional[VisionConfig] = None) -> None:
        self.config = config or VisionConfig()
        self.config.validate()

        self.camera = CameraReader(self.config)
        self.detector = create_detector(self.config)
        self.tracker = SinglePersonTracker(self.config)
        self.estimator = TargetEstimator(self.config)

        # Thread-safe observation storage
        self._obs_lock = threading.Lock()
        self._latest_observation = TargetObservation.invalid()

        # Target dispatch consumers
        self._callbacks: List[Callable[[TargetObservation], None]] = []
        self._queues: List[queue.Queue] = []

        # Service thread
        self._running = False
        self._worker_thread: Optional[threading.Thread] = None

        # Performance metrics
        self._fps: float = 0.0
        self._inference_ms: float = 0.0
        self._loop_ms: float = 0.0
        self._total_frames: int = 0

    def start(self) -> bool:
        """Start camera and vision processing pipeline."""
        if self._running:
            return True

        self.camera.start()
        self._running = True
        self._worker_thread = threading.Thread(target=self._pipeline_loop, daemon=True)
        self._worker_thread.start()
        return True

    def stop(self) -> None:
        """Stop processing pipeline and release camera."""
        self._running = False
        if self._worker_thread is not None and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=1.5)
        self.camera.stop()

    def is_running(self) -> bool:
        return self._running

    def get_latest_observation(self) -> TargetObservation:
        """Thread-safe retrieval of the freshest target observation."""
        with self._obs_lock:
            return self._latest_observation

    def register_callback(self, callback_fn: Callable[[TargetObservation], None]) -> None:
        """Register a callback invoked whenever a new target observation is calculated."""
        self._callbacks.append(callback_fn)

    def get_target_queue(self, maxsize: int = 10) -> queue.Queue:
        """Create and return a queue that receives new target observations."""
        q: queue.Queue = queue.Queue(maxsize=maxsize)
        self._queues.append(q)
        return q

    def get_telemetry(self) -> Dict[str, Any]:
        """Return runtime performance and tracking health statistics."""
        with self._obs_lock:
            obs_valid = self._latest_observation.target_valid
            obs_conf = self._latest_observation.confidence
        return {
            "fps": round(self._fps, 1),
            "inference_ms": round(self._inference_ms, 1),
            "loop_ms": round(self._loop_ms, 1),
            "total_frames": self._total_frames,
            "target_valid": obs_valid,
            "confidence": round(obs_conf, 3),
            "backend": self.detector.get_backend_name(),
        }

    def _pipeline_loop(self) -> None:
        """Continuous perception loop."""
        fps_timer = time.monotonic()
        frame_counter = 0

        while self._running:
            loop_start = time.monotonic()

            ret, frame, frame_ts = self.camera.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            # Determine frame dimensions
            if isinstance(frame, dict):
                fw, fh = frame.get("width", 640), frame.get("height", 480)
            elif hasattr(frame, "shape"):
                fh, fw = frame.shape[:2]
            else:
                fw, fh = self.config.camera_width, self.config.camera_height

            # 1. Detection
            inf_start = time.monotonic()
            detections = self.detector.detect(frame)
            self._inference_ms = (time.monotonic() - inf_start) * 1000.0

            # 2. Tracking & Temporal Smoothing
            obs = self.tracker.update(detections, frame_width=fw, frame_height=fh, timestamp=frame_ts)

            # 3. Spatial Estimation (Bearing & Calibrated Distance)
            obs = self.estimator.estimate(obs, frame_width=fw, frame_height=fh)

            # Update shared observation
            with self._obs_lock:
                self._latest_observation = obs

            # 4. Dispatch to callbacks & queues
            for cb in self._callbacks:
                try:
                    cb(obs)
                except Exception:
                    pass

            for q in self._queues:
                if not q.full():
                    try:
                        q.put_nowait(obs)
                    except Exception:
                        pass

            # Performance telemetry calculation
            frame_counter += 1
            self._total_frames += 1
            now = time.monotonic()
            elapsed_sec = now - fps_timer
            if elapsed_sec >= 1.0:
                self._fps = frame_counter / elapsed_sec
                frame_counter = 0
                fps_timer = now

            self._loop_ms = (time.monotonic() - loop_start) * 1000.0

            # Yield briefly to CPU
            time.sleep(0.002)

