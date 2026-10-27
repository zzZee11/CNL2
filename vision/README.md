# Lightweight Human-Detection & Tracking System for Raspberry Pi 4

A real-time, CPU-optimized human detection and tracking package designed specifically for the Raspberry Pi 4 (ARM Cortex-A72, 64-bit Linux).

This package operates as an **independent, self-contained perception system** inside `vision/`. It continuously estimates the angular bearing, image center offset, and status of a single person in the camera frame and exposes a thread-safe interface for future path-planning integration.

---

## 1. Architecture Overview

```
                      CAMERA STREAM (/dev/video0 or Mock)
                                     │
┌────────────────────────────────────▼───────────────────────────────────────┐
│ 1. NON-BLOCKING CAPTURE (camera.py: CameraReader)                         │
│    • Independent background thread isolates camera capture from inference. │
│    • Drops stale frames so detector always gets the freshest frame (0 lag).│
└────────────────────────────────────┬───────────────────────────────────────┘
                                     │
┌────────────────────────────────────▼───────────────────────────────────────┐
│ 2. LIGHTWEIGHT HUMAN DETECTION (detector.py: HumanDetector)               │
│    • Primary: TFLite SSD-MobileNet (300x300, ~70ms on Pi 4 CPU with NEON). │
│    • Secondary: OpenCV DNN (ONNX / Caffe) or MockDetector (testing).       │
│    • Filters strictly for class 'person'.                                  │
└────────────────────────────────────┬───────────────────────────────────────┘
                                     │ Detections [bbox, confidence]
┌────────────────────────────────────▼───────────────────────────────────────┐
│ 3. SINGLE-TARGET TRACKER (tracker.py: SinglePersonTracker)                 │
│    • Selects closest candidate to last known center (single-person focus). │
│    • Exponential Moving Average (EMA, α=0.65) filters bounding-box jitter. │
│    • Coasting: Bridges 1-2 missed frames during temporary occlusions.      │
│    • Timeout: Invalidates target after 1.5s of no detection (no staleness).│
└────────────────────────────────────┬───────────────────────────────────────┘
                                     │
┌────────────────────────────────────▼───────────────────────────────────────┐
│ 4. SPATIAL TARGET ESTIMATION (target_estimator.py: TargetEstimator)        │
│    • Horizontal bearing: θ = arctan((center_x - W/2) / f_x) in rad/deg.    │
│    • Calibrated distance proxy (strictly disabled unless calibrated).      │
└────────────────────────────────────┬───────────────────────────────────────┘
                                     │
┌────────────────────────────────────▼───────────────────────────────────────┐
│ 5. DISPATCH & INTEGRATION INTERFACE (target_data.py & vision_service.py)   │
│    • Thread-Safe Observation Container: TargetObservation                  │
│    • Consumers: get_latest_observation(), callbacks, or queue.Queue       │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Model & Runtime Evaluation on Raspberry Pi 4 CPU

| Model / Runtime | Input Size | Pi 4 Inference Latency | Achievable FPS | Memory Footprint | Dependencies & Notes |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **SSD-MobileNet V2 (TFLite)** | **300×300** | **~65 – 80 ms** | **12 – 15 FPS** | **< 60 MB RAM** | **RECOMMENDED**: Ultra-lightweight `tflite-runtime` (~4 MB wheel). ARM NEON acceleration. |
| **YOLOv8n (ONNX / NCNN)** | 320×320 | ~140 – 180 ms | 5 – 7 FPS | ~120 MB RAM | Requires ONNX Runtime or compiling NCNN on ARM. |
| **YOLOv8n / YOLO11n (PyTorch)** | 640×640 | ~400 – 600 ms | 1.5 – 2.5 FPS | > 800 MB RAM | Unsuitable: PyTorch package is >1.2 GB, causes thermal throttling on Pi 4. |

**Decision**: The system defaults to **TFLite with SSD-MobileNet (300×300)**. It delivers smooth real-time tracking at 13+ FPS on the Raspberry Pi 4 CPU without requiring a GPU or dedicated Coral TPU.

---

## 3. Directory Structure

All new code is strictly confined to `vision/` without modifying any repository files:

```text
vision/
├── __init__.py             # Package exports
├── config.py               # VisionConfig (resolutions, thresholds, timeouts)
├── target_data.py          # TargetObservation dataclass (thread-safe state)
├── camera.py               # Threaded non-blocking capture & mock generator
├── detector.py             # HumanDetector (TFLite, OpenCV DNN, Mock)
├── tracker.py              # SinglePersonTracker (EMA smoothing, coasting, timeout)
├── target_estimator.py     # TargetEstimator (bearing angle & distance proxy)
├── vision_service.py       # Decoupled pipeline runner & dispatch interfaces
├── main.py                 # Standalone CLI & interactive preview HUD
├── requirements.txt        # Isolated lightweight dependencies
├── README.md               # Documentation & integration blueprint
└── tests/
    ├── __init__.py
    ├── test_target_data.py
    ├── test_camera.py
    ├── test_detector.py
    ├── test_tracker.py
    ├── test_target_estimator.py
    └── test_vision_service.py
```

---

## 4. Installation & Setup (64-bit Raspberry Pi OS)

To avoid modifying global system packages, create a dedicated Python virtual environment:

### Step 1: Install Hardware-Accelerated Base Libraries
On Raspberry Pi OS:
```bash
sudo apt update
sudo apt install -y python3-opencv python3-numpy python3-pip python3-venv
```

### Step 2: Create a Dedicated Virtual Environment
```bash
python3 -m venv vision_env --system-site-packages
source vision_env/bin/activate
```

### Step 3: Install Lightweight TFLite Runtime (LiteRT)
```bash
pip install ai-edge-litert
```
*(Note: `ai-edge-litert` is Google's official updated package for TFLite runtime on Python 3.11/3.12 64-bit ARM).*

### Step 4: Model File Placement
The detector automatically searches for `detect.tflite` in `vision/models/` or the existing repo models. To copy the model into `vision/models/`:
```bash
mkdir -p vision/models
cp rasberrypi_host/vision/models/detect.tflite vision/models/
```

---

## 5. How to Run

### Mode A: Live Tracking with Camera and Display HUD
```bash
python3 vision/main.py
```
* Shows a real-time OpenCV preview with green target bounding box, red center crosshair, bearing angle, and horizontal error bar.
* Press `q` or `ESC` in the preview window to exit.

### Mode B: Headless Execution Over SSH (No GUI)
```bash
python3 vision/main.py --headless
```
* Runs cleanly without attempting to open an X11/Wayland window.
* Logs live telemetry and target status to the terminal at 2 Hz:
  ```text
  [Telemetry] FPS: 13.8 | Inf: 68.2ms | [Target: VALID | Conf: 86% | Center: (342, 235) | H-Err: +0.07 | Bearing: +2.4° | Dist: uncalibrated]
  ```

### Mode C: Desktop / CI Simulation (No Camera or Model Needed)
```bash
python3 vision/main.py --mock --headless
```
* Simulates synthetic video frames and a moving person to verify the entire tracking and telemetry pipeline on any PC.

---

## 6. Target Observation Interface

The system exposes the single tracked person through the `TargetObservation` dataclass:

```python
from vision.target_data import TargetObservation

# Attributes:
obs.target_valid        # (bool) True if person is actively tracked within timeout
obs.confidence          # (float) Detection confidence in [0.0, 1.0]
obs.bbox                # (tuple) (x1, y1, x2, y2) in pixel coordinates
obs.center_x            # (float) Bounding box center X in pixels
obs.center_y            # (float) Bounding box center Y in pixels
obs.horizontal_error    # (float) Normalized error: -1.0 (far left) to +1.0 (far right)
obs.horizontal_error_px # (float) Raw pixel error: (center_x - frame_width / 2)
obs.bearing             # (float) Estimated horizontal angle in radians
obs.bearing_deg         # (float) Estimated horizontal angle in degrees
obs.distance            # (Optional[float]) Calibrated distance in meters (None if uncalibrated)
obs.timestamp           # (float) Monotonic timestamp of observation
```

### Integration Modes for External Code

#### 1. Polling Interface (Simplest)
```python
from vision import VisionConfig, VisionService

service = VisionService(VisionConfig())
service.start()

# In your control loop:
target = service.get_latest_observation()
if target.target_valid:
    print(f"Follow bearing: {target.bearing_deg:+.1f}°")
else:
    print("Searching for person...")
```

#### 2. Event-Driven Callback Interface
```python
def on_new_target(target: TargetObservation):
    if target.target_valid:
        steer_robot(target.horizontal_error)

service.register_callback(on_new_target)
```

#### 3. Thread-Safe Queue Interface
```python
target_queue = service.get_target_queue(maxsize=1)
target = target_queue.get()
```

---

## 7. Future Integration Blueprint (With Existing Path Planner)

When authorized to integrate with the existing path planner ([`autonomous_navigation/`](file:///home/zee/projects/CNL2/autonomous_navigation/)):

```
┌─────────────────────────┐          ┌───────────────────────────────────────┐
│      VisionService      │          │         AutonomousNavigator           │
│                         │          │                                       │
│ target.horizontal_error │─────────▶│ Dynamic Target Heading Adjustment     │
│ target.bearing_deg      │          │ (Steers robot toward tracked human)   │
│ target.target_valid     │          │                                       │
└─────────────────────────┘          │ VL53L0X ToF Obstacle Evasion (APF/A*) │
                                     │ (Prevents colliding while following)  │
                                     └──────────────────┬────────────────────┘
                                                        │
                                                        ▼
                                                 ESP32-S3 Motors
```

* The path planner simply polls `vision_service.get_latest_observation()`.
* If `target.target_valid` is `True`, the planner adjusts target heading by `target.bearing`.
* If an obstacle is detected in front by the VL53L0X ToF lasers, the existing APF/A* obstacle avoidance overrides the heading to safely steer around obstacles while maintaining visual contact.

---

## 8. Test Suite Verification

Run the complete test suite from the repository root:
```bash
python3 -m unittest discover -s vision/tests -v
```
All 17 tests verify:
1. Camera initialization, frame capture, and graceful hardware-unavailable fallback.
2. Person detection and confidence score filtering.
3. Bounding-box and center coordinate calculations.
4. Horizontal error normalized ($[-1.0, +1.0]$) and raw pixel calculations.
5. Target observation updates and JSON serialization.
6. Temporal EMA jitter filtering.
7. Missed detection coasting (temporary occlusion handling).
8. Target invalidation after timeout ($1.5\text{s}$).
9. Bearing angle trigonometric calculations and distance calibration toggle.
10. Multi-candidate selection and pipeline performance telemetry.

