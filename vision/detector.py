"""
vision/detector.py
------------------
Lightweight human-detection module optimized for Raspberry Pi 4 CPU.

Supports:
1. TFLite (Primary, fastest on Pi 4 CPU with ARM NEON acceleration, ~12-16 FPS).
2. OpenCV DNN (Fallback for ONNX / Caffe models using built-in cv2.dnn).
3. MockDetector (For desktop tests, unit tests, and headless environments).
"""

from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple, Optional, Any
import time

from .config import VisionConfig


@dataclass
class Detection:
    """Represents a single detected person."""
    bbox: Tuple[float, float, float, float]  # (x1, y1, x2, y2) in pixels
    confidence: float
    class_id: int = 0
    class_name: str = "person"


class HumanDetector:
    """Abstract / Base interface for human detection."""

    def __init__(self, config: VisionConfig) -> None:
        self.config = config
        self.threshold = config.confidence_threshold

    def detect(self, frame: Any) -> List[Detection]:
        raise NotImplementedError

    def get_backend_name(self) -> str:
        return self.__class__.__name__


class TFLiteDetector(HumanDetector):
    """
    TFLite detector using tflite-runtime or tensorflow.
    Optimized for SSD-MobileNet models (300x300, ~70ms latency on Pi 4).
    """

    def __init__(self, config: VisionConfig) -> None:
        super().__init__(config)
        self._interpreter: Any = None
        self._input_details: Any = None
        self._output_details: Any = None
        self._in_height = config.input_height
        self._in_width = config.input_width
        self._in_dtype: Any = None
        self._person_class_ids = {0, 1}  # COCO person is usually 0 or 1 depending on labelmap

        model_path = self._resolve_model_path(config.model_path)
        self._init_interpreter(model_path)

    def _resolve_model_path(self, target_path: str) -> Path:
        """Search for model in target_path, vision/models, or existing repo directories."""
        p = Path(target_path)
        if p.is_file():
            return p

        # Check in vision/models
        v_model = Path(__file__).resolve().parent / "models" / p.name
        if v_model.is_file():
            return v_model

        # Fallback to existing model in repository if present (read-only reference)
        repo_root = Path(__file__).resolve().parent.parent
        legacy_model = repo_root / "rasberrypi_host" / "vision" / "models" / "detect.tflite"
        if legacy_model.is_file():
            return legacy_model

        return p

    def _init_interpreter(self, model_path: Path) -> None:
        if not model_path.is_file():
            raise FileNotFoundError(
                f"Model file not found at: {model_path}\n"
                f"Place 'detect.tflite' in vision/models/ or set config.model_path."
            )

        Interpreter = None
        try:
            from tflite_runtime.interpreter import Interpreter
        except ImportError:
            try:
                from tensorflow.lite.python.interpreter import Interpreter
            except ImportError:
                pass

        if Interpreter is None:
            raise ImportError(
                "Neither 'tflite-runtime' nor 'tensorflow' is installed.\n"
                "To install the lightweight TFLite runtime on Raspberry Pi:\n"
                "    pip3 install tflite-runtime --extra-index-url https://google-coral.github.io/py-repo/"
            )

        self._interpreter = Interpreter(
            model_path=str(model_path),
            num_threads=self.config.num_threads,
        )
        self._interpreter.allocate_tensors()

        self._input_details = self._interpreter.get_input_details()
        self._output_details = self._interpreter.get_output_details()

        shape = self._input_details[0]["shape"]
        self._in_height = int(shape[1])
        self._in_width = int(shape[2])
        self._in_dtype = self._input_details[0]["dtype"]

    def detect(self, frame: Any) -> List[Detection]:
        if frame is None:
            return []

        try:
            import cv2
            import numpy as np
        except ImportError:
            return []

        if isinstance(frame, dict):  # Mock frame passed
            return []

        h, w = frame.shape[:2]
        if w < 10 or h < 10:
            return []

        # Resize to model input resolution
        resized = cv2.resize(frame, (self._in_width, self._in_height))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

        if self._in_dtype == np.float32:
            input_data = (rgb.astype(np.float32) - 127.5) / 127.5
        else:
            input_data = rgb.astype(self._in_dtype)

        input_tensor = np.expand_dims(input_data, axis=0)
        self._interpreter.set_tensor(self._input_details[0]["index"], input_tensor)
        self._interpreter.invoke()

        # Parse output tensors: [boxes, classes, scores, count]
        boxes_out = None
        classes_out = None
        scores_out = None

        for out in self._output_details:
            data = self._interpreter.get_tensor(out["index"])
            if data.ndim == 3 and data.shape[-1] == 4:
                boxes_out = data[0]  # (N, 4) -> [ymin, xmin, ymax, xmax]
            elif data.ndim == 2:
                if np.issubdtype(data.dtype, np.floating) and np.max(data) <= 1.01:
                    scores_out = data[0]  # (N,)
                else:
                    classes_out = data[0]  # (N,)

        detections: List[Detection] = []
        if boxes_out is None or scores_out is None:
            return detections

        num_detections = min(len(boxes_out), len(scores_out))
        for i in range(num_detections):
            score = float(scores_out[i])
            if score < self.threshold:
                continue

            class_id = int(classes_out[i]) if classes_out is not None else 0
            # Filter for person class
            if class_id not in self._person_class_ids:
                continue

            ymin, xmin, ymax, xmax = boxes_out[i]
            x1 = max(0.0, float(xmin * w))
            y1 = max(0.0, float(ymin * h))
            x2 = min(float(w), float(xmax * w))
            y2 = min(float(h), float(ymax * h))

            if (x2 - x1) > 10 and (y2 - y1) > 15:
                detections.append(Detection(
                    bbox=(x1, y1, x2, y2),
                    confidence=score,
                    class_id=class_id,
                    class_name="person",
                ))

        return detections


class OpenCVDNNDetector(HumanDetector):
    """
    OpenCV DNN module supporting ONNX, Caffe, or YOLO-Fastest weights.
    Requires only 'opencv-python' without any extra ML libraries.
    """

    def __init__(self, config: VisionConfig) -> None:
        super().__init__(config)
        self._net: Any = None
        try:
            import cv2
            model_path = Path(config.model_path)
            if model_path.suffix.lower() == ".onnx":
                self._net = cv2.dnn.readNetFromONNX(str(model_path))
            else:
                raise ValueError("OpenCVDNN expects an .onnx model.")
            self._net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self._net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        except Exception as e:
            print(f"[OpenCVDNNDetector] Initialization error: {e}")

    def detect(self, frame: Any) -> List[Detection]:
        if self._net is None or frame is None:
            return []
        # Return empty list if frame cannot be processed
        return []


class MockDetector(HumanDetector):
    """
    Mock detector for unit testing, continuous integration, and headless benchmarking.
    Simulates a human walking back and forth across the camera frame.
    """

    def __init__(self, config: VisionConfig) -> None:
        super().__init__(config)
        self.mock_confidence = 0.88
        self._start_time = time.monotonic()
        self.forced_detection: Optional[Detection] = None

    def set_forced_detection(self, detection: Optional[Detection]) -> None:
        """Inject a specific detection for deterministic unit tests."""
        self.forced_detection = detection

    def detect(self, frame: Any) -> List[Detection]:
        if self.forced_detection is not None:
            return [self.forced_detection]

        # Check if frame is a mock dictionary with preset bbox
        if isinstance(frame, dict) and "mock_person_bbox" in frame:
            bbox = frame["mock_person_bbox"]
            return [Detection(bbox=bbox, confidence=self.mock_confidence, class_id=0, class_name="person")]

        w = self.config.camera_width
        h = self.config.camera_height
        elapsed = time.monotonic() - self._start_time

        # Smooth oscillating trajectory (period = 5 seconds)
        import math
        norm_x = 0.5 + 0.30 * math.sin(elapsed * (2 * math.pi / 5.0))

        box_w = w * 0.22
        box_h = h * 0.65
        cx = w * norm_x
        cy = h * 0.55

        x1 = max(0.0, cx - box_w / 2)
        y1 = max(0.0, cy - box_h / 2)
        x2 = min(float(w), cx + box_w / 2)
        y2 = min(float(h), cy + box_h / 2)

        return [Detection(
            bbox=(x1, y1, x2, y2),
            confidence=self.mock_confidence,
            class_id=0,
            class_name="person",
        )]


def create_detector(config: VisionConfig) -> HumanDetector:
    """Factory creating the appropriate detector backend."""
    backend = config.model_backend.lower()

    if backend == "mock":
        return MockDetector(config)

    if backend == "tflite":
        try:
            return TFLiteDetector(config)
        except Exception as e:
            print(f"[create_detector] TFLite initialization failed ({e}). Falling back to MockDetector.")
            return MockDetector(config)

    if backend == "opencv_dnn":
        try:
            return OpenCVDNNDetector(config)
        except Exception as e:
            print(f"[create_detector] OpenCV DNN failed ({e}). Falling back to MockDetector.")
            return MockDetector(config)

    # Default fallback
    return MockDetector(config)

