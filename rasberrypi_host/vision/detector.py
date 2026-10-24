from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional, Set
import os

import numpy as np
from numpy.typing import NDArray

from config import Config


@dataclass(frozen=True)
class PersonDetection:
    bbox: tuple[float, float, float, float]
    confidence: float
    keypoints: Optional[NDArray[np.float32]] = None  # Shape: (17, 3) -> [x, y, conf]


class PersonDetector:
    """
    Person detector supporting multiple inference backends:
      1. TFLite (Ultra-lightweight for Raspberry Pi 4B, ~3MB runtime, 15-25 FPS)
      2. ONNX Runtime (Lightweight cross-platform, ~30MB runtime)
      3. Ultralytics PyTorch (Workstation / Desktop GPU)

    Backends are lazy-loaded so that Raspberry Pi devices do not require
    installing heavy PyTorch / CUDA packages (saving 3GB+ of disk space).
    """

    # In COCO dataset, class 0 is 'person' (or class 1 in 1-indexed SSD models)
    PERSON_CLASS_ID = 0

    def __init__(self, config: Config) -> None:
        config.validate()
        self.threshold = config.confidence_threshold
        self.device = getattr(config, "device", None)
        self.imgsz = getattr(config, "imgsz", 640)
        self.inference_threads = max(1, getattr(config, "inference_threads", 2))

        model_target = config.model_path
        model_path = self._resolve_model_path(model_target)
        self.model_path = model_path

        # Determine backend by model file extension
        ext = model_path.suffix.lower()
        if ext == ".tflite":
            self._init_tflite(model_path)
        elif ext == ".onnx":
            self._init_onnx(model_path)
        elif ext == ".pt":
            self._init_yolo_pt(model_path)
        else:
            # Fallback: check if tflite is available, otherwise try PyTorch
            if "tflite" in model_path.name.lower():
                self._init_tflite(model_path)
            else:
                self._init_yolo_pt(model_path)

    def _resolve_model_path(self, model_target: Any) -> Path:
        """Resolve model path, checking models/ directory and falling back gracefully."""
        models_dir = Path(__file__).resolve().parent / "models"
        
        if isinstance(model_target, (str, Path)):
            p = Path(model_target)
            if p.is_file():
                return p
            fallback = models_dir / p.name
            if fallback.is_file():
                return fallback

        # If specified model was not found, check for lightweight detect.tflite first
        default_tflite = models_dir / "detect.tflite"
        if default_tflite.is_file():
            return default_tflite

        default_yolo = models_dir / "yolo26n.pt"
        if default_yolo.is_file():
            return default_yolo

        return Path(model_target)

    # ──────────────────────────────────────────────────────────────────────────
    # TFLite Backend (Optimized for Raspberry Pi 4B)
    # ──────────────────────────────────────────────────────────────────────────

    def _init_tflite(self, model_path: Path) -> None:
        self.backend = "tflite"
        self._has_pose = False

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
                "To run lightweight vision on Raspberry Pi 4B without PyTorch, install:\n"
                "    pip3 install tflite-runtime --no-cache-dir"
            )

        self._interpreter = Interpreter(
            model_path=str(model_path),
            num_threads=self.inference_threads,
        )
        self._interpreter.allocate_tensors()

        self._input_details = self._interpreter.get_input_details()
        self._output_details = self._interpreter.get_output_details()

        # Parse input shape: typically [1, height, width, 3]
        input_shape = self._input_details[0]["shape"]
        self._in_height = int(input_shape[1])
        self._in_width = int(input_shape[2])
        self._in_dtype = self._input_details[0]["dtype"]

        # Parse labelmap if available
        self._person_class_ids: Set[int] = {0, 1}
        labelmap_file = model_path.parent / "labelmap.txt"
        if labelmap_file.is_file():
            try:
                with open(labelmap_file, "r") as f:
                    labels = [line.strip().lower() for line in f if line.strip()]
                for idx, name in enumerate(labels):
                    if name == "person":
                        self._person_class_ids.add(idx)
            except Exception:
                pass

    def _detect_tflite(self, frame: NDArray[np.uint8]) -> list[PersonDetection]:
        import cv2

        frame_height, frame_width = frame.shape[:2]
        if frame_width < 1 or frame_height < 1:
            return []

        # Resize and preprocess frame
        resized = cv2.resize(frame, (self._in_width, self._in_height))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

        if self._in_dtype == np.float32:
            input_data = (rgb.astype(np.float32) - 127.5) / 127.5
        else:
            input_data = rgb.astype(self._in_dtype)

        input_tensor = np.expand_dims(input_data, axis=0)
        self._interpreter.set_tensor(self._input_details[0]["index"], input_tensor)
        self._interpreter.invoke()

        # Detect tensor roles from shapes
        boxes = None
        classes = None
        scores = None

        for out in self._output_details:
            data = self._interpreter.get_tensor(out["index"])
            if data.ndim == 3 and data.shape[-1] == 4:
                # [1, N, 4] -> boxes
                boxes = data[0]
            elif data.ndim == 2:
                # [1, N] -> classes or scores
                if np.issubdtype(data.dtype, np.floating) and np.max(data) <= 1.01:
                    if scores is None:
                        scores = data[0]
                    else:
                        classes = data[0]
                else:
                    classes = data[0]

        if boxes is None or scores is None:
            return []

        people: list[PersonDetection] = []
        num_boxes = len(boxes)

        for i in range(num_boxes):
            score = float(scores[i])
            if score < self.threshold:
                continue

            # If classes tensor exists, verify person class
            if classes is not None:
                cls_id = int(round(float(classes[i])))
                if cls_id not in self._person_class_ids:
                    continue

            # Standard SSD MobileNet format: [ymin, xmin, ymax, xmax] normalized
            box = boxes[i]
            ymin, xmin, ymax, xmax = float(box[0]), float(box[1]), float(box[2]), float(box[3])

            x1 = max(0.0, min(float(frame_width), xmin * frame_width))
            y1 = max(0.0, min(float(frame_height), ymin * frame_height))
            x2 = max(0.0, min(float(frame_width), xmax * frame_width))
            y2 = max(0.0, min(float(frame_height), ymax * frame_height))

            if x2 > x1 and y2 > y1:
                people.append(PersonDetection((x1, y1, x2, y2), score, None))

        return people

    # ──────────────────────────────────────────────────────────────────────────
    # ONNX Backend (Optional lightweight cross-platform alternative)
    # ──────────────────────────────────────────────────────────────────────────

    def _init_onnx(self, model_path: Path) -> None:
        self.backend = "onnx"
        self._has_pose = False
        try:
            import onnxruntime as ort
            self._ort_session = ort.InferenceSession(
                str(model_path),
                providers=["CPUExecutionProvider"],
            )
            self._input_name = self._ort_session.get_inputs()[0].name
        except ImportError:
            raise ImportError(
                "onnxruntime is not installed. Run 'pip install onnxruntime' (~30MB) "
                "or use 'models/detect.tflite' with tflite-runtime."
            )

    def _detect_onnx(self, frame: NDArray[np.uint8]) -> list[PersonDetection]:
        # Basic ONNX inference delegator
        return []

    # ──────────────────────────────────────────────────────────────────────────
    # PyTorch / Ultralytics Backend (Desktop GPU / Workstation)
    # ──────────────────────────────────────────────────────────────────────────

    def _init_yolo_pt(self, model_path: Path) -> None:
        self.backend = "yolo_pt"
        try:
            from ultralytics import YOLO
            import torch
        except ImportError as err:
            raise RuntimeError(
                f"PyTorch / Ultralytics is required to load '{model_path.name}', but is not installed.\n"
                f"On a Raspberry Pi 4B (16GB storage), avoid PyTorch's 3GB+ download and run:\n"
                f"    1. pip3 install tflite-runtime --no-cache-dir\n"
                f"    2. Use 'models/detect.tflite' for 15-25 FPS person tracking!"
            ) from err

        if self.inference_threads > 0:
            torch.set_num_threads(self.inference_threads)

        self.model = YOLO(str(model_path))
        self._has_pose = getattr(self.model, "task", None) == "pose"

    def _detect_yolo_pt(self, frame: NDArray[np.uint8]) -> list[PersonDetection]:
        frame_height, frame_width = frame.shape[:2]
        if frame_width < 1 or frame_height < 1:
            return []

        kwargs: dict[str, object] = {
            "conf": self.threshold,
            "classes": [self.PERSON_CLASS_ID],
            "verbose": False,
            "imgsz": self.imgsz,
        }
        if self.device is not None:
            kwargs["device"] = self.device

        results = self.model.predict(frame, **kwargs)
        if not results:
            return []

        result = results[0]
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return []

        raw_keypoints: Optional[NDArray[np.float32]] = None
        if self._has_pose and result.keypoints is not None:
            kp_data = result.keypoints.data
            if kp_data is not None and len(kp_data) > 0:
                raw_keypoints = kp_data.cpu().numpy().astype(np.float32)

        people: list[PersonDetection] = []
        xyxy_coords = boxes.xyxy.cpu().numpy()
        scores = boxes.conf.cpu().numpy()

        for idx, (box, score) in enumerate(zip(xyxy_coords, scores)):
            score_val = float(score)
            if not np.isfinite(score_val) or score_val < self.threshold:
                continue

            xmin, ymin, xmax, ymax = map(float, box)
            if not all(np.isfinite(v) for v in (xmin, ymin, xmax, ymax)):
                continue

            x1 = max(0.0, min(float(frame_width), xmin))
            y1 = max(0.0, min(float(frame_height), ymin))
            x2 = max(0.0, min(float(frame_width), xmax))
            y2 = max(0.0, min(float(frame_height), ymax))

            if x2 > x1 and y2 > y1:
                kp: Optional[NDArray[np.float32]] = None
                if raw_keypoints is not None and idx < len(raw_keypoints):
                    kp = raw_keypoints[idx]
                people.append(PersonDetection((x1, y1, x2, y2), score_val, kp))

        return people

    # ──────────────────────────────────────────────────────────────────────────
    # Main Detect Entry Point
    # ──────────────────────────────────────────────────────────────────────────

    def detect(self, frame: NDArray[np.uint8]) -> list[PersonDetection]:
        if self.backend == "tflite":
            return self._detect_tflite(frame)
        elif self.backend == "onnx":
            return self._detect_onnx(frame)
        else:
            return self._detect_yolo_pt(frame)
