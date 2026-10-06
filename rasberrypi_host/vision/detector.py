from dataclasses import dataclass

import cv2
import numpy as np
from numpy.typing import NDArray

from config import Config


@dataclass(frozen=True)
class PersonDetection:
    bbox: tuple[float, float, float, float]
    confidence: float


class PersonDetector:
    """Decoder for coco_ssd_mobilenet_v1_1.0_quant_2018_06_29."""

    # This model's postprocess class IDs are zero-based: person == 0.
    PERSON_CLASS_ID = 0

    def __init__(self, config: Config) -> None:
        if not config.model_path.is_file():
            raise FileNotFoundError(
                f"Model not found: {config.model_path}. See README.md."
            )

        try:
            from ai_edge_litert.interpreter import Interpreter
        except ImportError as exc:
            raise RuntimeError(
                "Install ai-edge-litert from requirements.txt."
            ) from exc

        self.threshold = config.confidence_threshold
        self.interpreter = Interpreter(
            model_path=str(config.model_path),
            num_threads=config.inference_threads,
        )
        self.interpreter.allocate_tensors()

        inputs = self.interpreter.get_input_details()
        outputs = self.interpreter.get_output_details()
        if len(inputs) != 1 or len(outputs) != 4:
            raise ValueError("Expected one input and four SSD postprocess outputs.")

        self.input_detail = inputs[0]
        shape = self.input_detail["shape"]
        if (
            len(shape) != 4
            or int(shape[0]) != 1
            or int(shape[3]) != 3
            or self.input_detail["dtype"] != np.uint8
        ):
            raise ValueError("Expected an NHWC uint8 quantized SSD input.")

        self.input_height = int(shape[1])
        self.input_width = int(shape[2])

        # The pinned model exposes TFLite_Detection_PostProcess:0..3:
        # boxes, classes, scores, count, respectively.
        by_suffix: dict[int, dict] = {}
        for detail in outputs:
            name = str(detail["name"])
            prefix = "TFLite_Detection_PostProcess"
            if prefix not in name:
                raise ValueError(f"Unexpected SSD output tensor: {name}")
            suffix = name.rsplit(":", 1)
            output_number = int(suffix[1]) if len(suffix) == 2 else 0
            by_suffix[output_number] = detail

        if set(by_suffix) != {0, 1, 2, 3}:
            raise ValueError(
                "Unexpected SSD tensor numbering; inspect this model's outputs."
            )
        self.outputs = by_suffix

    def detect(self, frame: NDArray[np.uint8]) -> list[PersonDetection]:
        frame_height, frame_width = frame.shape[:2]
        if frame_width < 1 or frame_height < 1:
            return []

        resized = cv2.resize(
            frame, (self.input_width, self.input_height),
            interpolation=cv2.INTER_LINEAR,
        )
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        tensor = np.expand_dims(rgb, axis=0)

        self.interpreter.set_tensor(self.input_detail["index"], tensor)
        self.interpreter.invoke()

        boxes = self.interpreter.get_tensor(self.outputs[0]["index"])[0]
        classes = self.interpreter.get_tensor(self.outputs[1]["index"])[0]
        scores = self.interpreter.get_tensor(self.outputs[2]["index"])[0]
        count = int(
            np.asarray(
                self.interpreter.get_tensor(self.outputs[3]["index"])
            ).flat[0]
        )

        people: list[PersonDetection] = []
        for i in range(min(count, len(boxes), len(classes), len(scores))):
            score = float(scores[i])
            if (
                not np.isfinite(score)
                or score < self.threshold
                or int(round(float(classes[i]))) != self.PERSON_CLASS_ID
            ):
                continue

            ymin, xmin, ymax, xmax = map(float, boxes[i])
            if not all(np.isfinite(v) for v in (ymin, xmin, ymax, xmax)):
                continue

            x1 = max(0.0, min(float(frame_width), xmin * frame_width))
            y1 = max(0.0, min(float(frame_height), ymin * frame_height))
            x2 = max(0.0, min(float(frame_width), xmax * frame_width))
            y2 = max(0.0, min(float(frame_height), ymax * frame_height))

            if x2 > x1 and y2 > y1:
                people.append(PersonDetection((x1, y1, x2, y2), score))

        return people
