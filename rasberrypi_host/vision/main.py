import argparse
import json
import logging
from pathlib import Path
import sys
import time
from dataclasses import replace
from typing import Any

import cv2
import numpy as np
from numpy.typing import NDArray

from config import Config
from vision import VisionSystem

LOG = logging.getLogger("computer_vision")


def draw_debug(frame: NDArray[np.uint8], result: dict[str, Any], fps: float) -> NDArray[np.uint8]:
    image = frame.copy()
    height, width = image.shape[:2]
    cv2.line(
        image, (width // 2, 0), (width // 2, height),
        (100, 100, 100), 1,
    )

    for person in result["people"]:
        x1, y1, x2, y2 = (int(v) for v in person["bbox"])
        is_target = result["target_detected"] and (
            person["id"] == result["target_id"]
        )
        color = (0, 255, 255) if is_target else (0, 200, 0)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            image, f"ID {person['id']}  {person['confidence']:.2f}",
            (x1, max(18, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2,
        )

    if result["target_detected"]:
        cx, cy = (int(v) for v in result["target_center"])
        cv2.circle(image, (cx, cy), 6, (0, 0, 255), -1)
        status = (
            f"Target {result['target_id']} "
            f"horizontal error {result['horizontal_error']:+.2f}"
        )
    else:
        status = (
            f"Target {result['target_id']} temporarily lost"
            if result["target_id"] is not None
            else "No target"
        )
    motion_label = (
    f"Motion: {result['movement_direction']} | "
    f"Depth trend: {result['distance_trend']}"
    )
    
    cv2.putText(
    image, motion_label, (10, 75),
    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2,
    )
    
    cv2.putText(
        image, status, (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2,
    )
    cv2.putText(
        image, f"FPS {fps:.1f}", (10, 50),
        cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2,
    )
    return image


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Pi person-following vision with YOLO26")
    parser.add_argument(
        "--model",
        default=str(Path(__file__).resolve().parent / "models" / "yolo26n.pt"),
        help="Path or name of the YOLO26 model",
    )
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--confidence", type=float, default=0.35)
    parser.add_argument("--threads", type=int, default=2)
    parser.add_argument(
        "--device",
        default=None,
        help="Device to run inference on (e.g. 'cpu', 'cuda', '0')",
    )
    parser.add_argument("--imgsz", type=int, default=640, help="Inference image size")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        stream=sys.stderr,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    config = replace(
        Config(),
        model_path=Path(args.model),
        camera_index=args.camera,
        camera_width=args.width,
        camera_height=args.height,
        confidence_threshold=args.confidence,
        inference_threads=args.threads,
        device=args.device,
        imgsz=args.imgsz,
    )

    capture: cv2.VideoCapture | None = None
    try:
        vision = VisionSystem(config)

        capture = cv2.VideoCapture(config.camera_index)
        if not capture.isOpened():
            raise RuntimeError(
                f"Could not open camera index {config.camera_index}."
            )

        capture.set(cv2.CAP_PROP_FRAME_WIDTH, config.camera_width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config.camera_height)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        LOG.info("Camera opened at approximately %sx%s", actual_width, actual_height)

        last_loop_end: float | None = None
        smoothed_fps = 0.0
        failed_reads = 0

        while True:
            success, frame = capture.read()
            if not success or frame is None:
                failed_reads += 1
                LOG.warning("Camera read failed (%s/20)", failed_reads)
                if failed_reads >= 20:
                    raise RuntimeError("Camera stopped delivering frames.")
                time.sleep(0.1)
                continue

            failed_reads = 0
            now = time.monotonic()
            result = vision.process_frame(frame, now)
            loop_end = time.monotonic()

            if last_loop_end is not None:
                elapsed = max(loop_end - last_loop_end, 1e-6)
                instantaneous_fps = 1.0 / elapsed
                smoothed_fps = (
                    instantaneous_fps if smoothed_fps == 0.0
                    else 0.8 * smoothed_fps + 0.2 * instantaneous_fps
                )
            last_loop_end = loop_end

            result["fps"] = round(smoothed_fps, 2)
            print(json.dumps(result, separators=(",", ":")), flush=True)

            if config.debug_view:
                cv2.imshow(
                    "Person-following vision",
                    draw_debug(frame, result, smoothed_fps),
                )
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

        return 0

    except KeyboardInterrupt:
        LOG.info("Stopped by user")
        return 0
    except (OSError, RuntimeError, ValueError) as exc:
        LOG.exception("Vision pipeline stopped: %s", exc)
        return 1
    finally:
        if capture is not None:
            capture.release()
        if config.debug_view:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())