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
from tracker import Track

LOG = logging.getLogger("computer_vision")


# ── COCO-17 skeleton connectivity for drawing ────────────────────────
# Each pair is (keypoint_a, keypoint_b, color_bgr).
_SKELETON_LINKS: list[tuple[int, int, tuple[int, int, int]]] = [
    # Head / torso
    (5, 6, (200, 200, 200)),    # L shoulder – R shoulder
    (5, 11, (200, 200, 200)),   # L shoulder – L hip
    (6, 12, (200, 200, 200)),   # R shoulder – R hip
    (11, 12, (200, 200, 200)),  # L hip – R hip
    # Left arm  (blue-ish)
    (5, 7, (255, 120, 60)),     # L shoulder – L elbow
    (7, 9, (255, 120, 60)),     # L elbow – L wrist
    # Right arm (red-ish)
    (6, 8, (60, 120, 255)),     # R shoulder – R elbow
    (8, 10, (60, 120, 255)),    # R elbow – R wrist
    # Left leg  (blue-ish)
    (11, 13, (255, 180, 60)),   # L hip – L knee
    (13, 15, (255, 180, 60)),   # L knee – L ankle
    # Right leg (red-ish)
    (12, 14, (60, 180, 255)),   # R hip – R knee
    (14, 16, (60, 180, 255)),   # R knee – R ankle
]

# Key joint indices to draw circles on.
_KEY_JOINTS = [5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]


def _draw_skeleton(
    image: NDArray[np.uint8],
    keypoints: NDArray[np.float32],
    conf_thresh: float = 0.35,
) -> None:
    """Draw pose skeleton lines and joint circles on *image*."""
    for idx_a, idx_b, color in _SKELETON_LINKS:
        if (keypoints[idx_a, 2] >= conf_thresh
                and keypoints[idx_b, 2] >= conf_thresh):
            pt_a = (int(keypoints[idx_a, 0]), int(keypoints[idx_a, 1]))
            pt_b = (int(keypoints[idx_b, 0]), int(keypoints[idx_b, 1]))
            cv2.line(image, pt_a, pt_b, color, 2, cv2.LINE_AA)

    for idx in _KEY_JOINTS:
        if keypoints[idx, 2] >= conf_thresh:
            pt = (int(keypoints[idx, 0]), int(keypoints[idx, 1]))
            cv2.circle(image, pt, 3, (0, 255, 255), -1, cv2.LINE_AA)


def draw_debug(
    frame: NDArray[np.uint8],
    result: dict[str, Any],
    fps: float,
    tracks: list[Track] | None = None,
    kp_conf_thresh: float = 0.35,
) -> NDArray[np.uint8]:
    image = frame.copy()
    height, width = image.shape[:2]
    cv2.line(
        image, (width // 2, 0), (width // 2, height),
        (100, 100, 100), 1,
    )

    # ── Draw pose skeletons for all visible tracks ────────────────────
    if tracks:
        for track in tracks:
            if track.keypoints is not None:
                _draw_skeleton(image, track.keypoints, kp_conf_thresh)

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

    # ── Motion & depth labels ─────────────────────────────────────────
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

    # ── Gait state label ──────────────────────────────────────────────
    gait_state = result.get("gait_state", "unknown")
    gait_conf = result.get("gait_confidence", 0.0)
    visibility = result.get("visibility", "none")

    if gait_state == "walking":
        gait_color = (0, 220, 0)
    elif gait_state == "standing":
        gait_color = (0, 220, 255)
    elif gait_state == "slow_or_pausing":
        gait_color = (0, 180, 255)
    else:
        gait_color = (180, 180, 180)

    gait_label = (
        f"Gait: {gait_state.upper()} ({gait_conf:.2f}) "
        f"[{visibility}]"
    )
    cv2.putText(
        image, gait_label, (10, 100),
        cv2.FONT_HERSHEY_SIMPLEX, 0.52, gait_color, 2,
    )

    # ── Depth cascade source label ────────────────────────────────────
    depth_src = result.get("depth_signal_source", "none")
    depth_rate = result.get("depth_expansion_rate")
    ttc = result.get("time_to_collision_s")
    depth_label = f"Depth src: {depth_src}"
    if depth_rate is not None:
        depth_label += f"  rate: {depth_rate:+.4f}"
    if ttc is not None:
        depth_label += f"  TTC: {ttc:.1f}s"
    cv2.putText(
        image, depth_label, (10, 125),
        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (220, 220, 220), 1,
    )

    # ── Car stop status banner ────────────────────────────────────────
    car_stop = result.get("car_stop_required", False)
    emergency = result.get("emergency_brake", False)
    stop_reason = result.get("stop_reason", "none")
    interaction = result.get("interaction_state", "unknown")

    if car_stop:
        if emergency:
            # Flashing red/yellow banner.
            flash = int(time.monotonic() * 6) % 2 == 0
            banner_color = (0, 0, 255) if flash else (0, 160, 255)
            banner_text = (
                f"!!! BRAKE: {stop_reason.upper().replace('_', ' ')} !!!"
            )
        else:
            banner_color = (0, 180, 255)
            banner_text = f"STOP: {stop_reason.replace('_', ' ')}"

        # Draw filled banner at top of frame.
        cv2.rectangle(image, (0, height - 40), (width, height), banner_color, -1)
        cv2.putText(
            image, banner_text,
            (10, height - 12),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2,
        )
    else:
        # Green "safe" indicator.
        safe_label = f"FOLLOWING | {interaction.replace('_', ' ')}"
        cv2.putText(
            image, safe_label,
            (10, height - 12),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0), 2,
        )

    return image


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Pi person-following vision with YOLO26")
    parser.add_argument(
        "--model",
        default=str(Path(__file__).resolve().parent / "models" / "yolo26n-pose.pt"),
        help="Path or name of the YOLO26 model (pose or detection)",
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
                # Collect visible tracks for skeleton drawing.
                visible_tracks = [
                    t for t in vision.tracker.tracks.values()
                    if t.visible
                ]
                cv2.imshow(
                    "Person-following vision",
                    draw_debug(
                        frame, result, smoothed_fps,
                        tracks=visible_tracks,
                        kp_conf_thresh=config.keypoint_confidence_threshold,
                    ),
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