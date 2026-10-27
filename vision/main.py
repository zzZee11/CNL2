"""
vision/main.py
--------------
Standalone CLI entry point for the Raspberry Pi 4 human detection & tracking system.

Usage:
    # Run live with hardware camera and TFLite model:
    python3 vision/main.py

    # Run in mock simulation mode (no camera or model required):
    python3 vision/main.py --mock

    # Run headless over SSH without GUI display:
    python3 vision/main.py --headless

    # Custom model and confidence threshold:
    python3 vision/main.py --model path/to/model.tflite --confidence 0.45
"""

import argparse
import sys
import os
import time

# Support running directly or as a module
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from vision.config import VisionConfig
from vision.vision_service import VisionService


def draw_preview_overlay(frame, obs, telemetry):
    """Render bounding box, center crosshairs, error bar, and telemetry on the frame."""
    try:
        import cv2
    except ImportError:
        return frame

    h, w = frame.shape[:2]
    cx_frame = w // 2
    cy_frame = h // 2

    # Draw frame center crosshair (white)
    cv2.line(frame, (cx_frame - 15, cy_frame), (cx_frame + 15, cy_frame), (255, 255, 255), 1)
    cv2.line(frame, (cx_frame, cy_frame - 15), (cx_frame, cy_frame + 15), (255, 255, 255), 1)

    if obs.target_valid and obs.bbox is not None:
        x1, y1, x2, y2 = [int(v) for v in obs.bbox]
        tcx, tcy = int(obs.center_x), int(obs.center_y)

        # Draw target bounding box (Green)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Target center crosshair (Red)
        cv2.drawMarker(frame, (tcx, tcy), (0, 0, 255), cv2.MARKER_CROSS, 16, 2)

        # Line connecting camera center to target center (Yellow)
        cv2.line(frame, (cx_frame, cy_frame), (tcx, tcy), (0, 255, 255), 1)

        # Target info label
        label = f"Person: {obs.confidence:.0%} | Bearing: {obs.bearing_deg:+.1f} deg"
        if obs.distance is not None:
            label += f" | Dist: {obs.distance:.2f}m"

        cv2.putText(
            frame, label, (x1, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
        )

        # Horizontal error bar indicator at bottom of screen
        bar_y = h - 25
        bar_len = int(obs.horizontal_error * (w // 3))
        cv2.line(frame, (cx_frame - (w // 3), bar_y), (cx_frame + (w // 3), bar_y), (100, 100, 100), 2)
        cv2.line(frame, (cx_frame, bar_y - 6), (cx_frame, bar_y + 6), (255, 255, 255), 2)
        cv2.circle(frame, (cx_frame + bar_len, bar_y), 5, (0, 255, 255), -1)
    else:
        # Lost / Searching banner (Red text)
        cv2.putText(
            frame, "SEARCHING FOR PERSON...", (20, h - 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2
        )

    # Telemetry HUD in top-left
    hud = f"FPS: {telemetry['fps']} | Inf: {telemetry['inference_ms']}ms | Backend: {telemetry['backend']}"
    cv2.putText(frame, hud, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

    return frame


def main():
    parser = argparse.ArgumentParser(description="Raspberry Pi 4 Lightweight Human Detection & Tracking")
    parser.add_argument("--mock", action="store_true", help="Run with simulated mock camera and detector")
    parser.add_argument("--headless", action="store_true", help="Disable GUI preview window (e.g. for SSH)")
    parser.add_argument("--camera", type=int, default=0, help="Camera index (default: 0)")
    parser.add_argument("--model", type=str, default="vision/models/detect.tflite", help="Path to model file")
    parser.add_argument("--confidence", type=float, default=0.40, help="Detection confidence threshold")
    parser.add_argument("--width", type=int, default=640, help="Camera width")
    parser.add_argument("--height", type=int, default=480, help="Camera height")
    parser.add_argument("--threads", type=int, default=4, help="Inference CPU threads")
    parser.add_argument("--distance-proxy", action="store_true", help="Enable height-based distance proxy estimation")
    args = parser.parse_args()

    # Determine whether display is available
    has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    headless = args.headless or not has_display

    config = VisionConfig(
        camera_index=args.camera,
        camera_width=args.width,
        camera_height=args.height,
        model_backend="mock" if args.mock else "tflite",
        camera_backend="mock" if args.mock else "opencv",
        model_path=args.model,
        confidence_threshold=args.confidence,
        num_threads=args.threads,
        distance_calibration_enabled=args.distance_proxy,
        enable_preview=not headless,
        headless=headless,
    )

    print("=" * 65)
    print("  HUMAN DETECTION & TRACKING SYSTEM (Raspberry Pi 4)")
    print("=" * 65)
    print(f"  Backend:         {config.model_backend.upper()}")
    print(f"  Resolution:      {config.camera_width}x{config.camera_height}")
    print(f"  Confidence:      {config.confidence_threshold:.0%}")
    print(f"  Lost Timeout:    {config.track_lost_timeout_s}s")
    print(f"  Headless:        {config.headless}")
    print(f"  Distance Proxy:  {config.distance_calibration_enabled}")
    print("=" * 65)
    print("Press Ctrl+C (or 'q' in preview window) to stop.\n")

    service = VisionService(config)
    if not service.start():
        print("Error: Could not start vision service.")
        sys.exit(1)

    cv2 = None
    if not config.headless:
        try:
            import cv2
        except ImportError:
            pass

    last_log_time = 0.0

    try:
        while True:
            obs = service.get_latest_observation()
            telem = service.get_telemetry()

            # Terminal log update every 0.5s
            now = time.monotonic()
            if now - last_log_time >= 0.5:
                print(f"[Telemetry] FPS: {telem['fps']:4.1f} | Inf: {telem['inference_ms']:5.1f}ms | {obs.summary()}")
                last_log_time = now

            # GUI Preview Window
            if cv2 is not None and not config.headless:
                ret, frame, _ = service.camera.read()
                if ret and frame is not None and hasattr(frame, "shape"):
                    annotated = draw_preview_overlay(frame.copy(), obs, telem)
                    cv2.imshow("Human Detection & Tracking", annotated)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):  # 'q' or ESC
                        print("\nUser requested stop via preview window.")
                        break

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nStopping vision service...")

    finally:
        service.stop()
        if cv2 is not None:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        print("Vision service stopped cleanly.")


if __name__ == "__main__":
    main()

