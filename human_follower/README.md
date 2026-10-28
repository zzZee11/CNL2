# Stage 1 Human Follower: Stationary Turret Tracking

A self-contained, standalone tracking package inside [`human_follower/`](file:///home/zee/projects/CNL2/human_follower/).

### Objective of Stage 1
The robot remains stationary ($v_{\text{forward}} = 0.0\text{ m/s}$) and **pivots on the spot** like a camera turret to keep a moving human centered in the camera frame. This allows verifying vision-to-motor steering and deadband stability safely before enabling forward movement.

---

## 1. How It Works

```
                     CAMERA FEED (USB / Pi Camera)
                                  │
┌─────────────────────────────────▼──────────────────────────────────┐
│  VISION SYSTEM (vision/vision_service.py)                          │
│  • Single-person detection & EMA temporal smoothing (α=0.65)       │
│  • Outputs normalized horizontal error: -1.0 (left) to +1.0 (right)│
└─────────────────────────────────┬──────────────────────────────────┘
                                  │ TargetObservation
┌─────────────────────────────────▼──────────────────────────────────┐
│  TURRET CONTROLLER (human_follower/turret_follower.py)             │
│                                                                    │
│  1. Check Deadband:                                                │
│     If |horizontal_error| <= 0.08 (±8% of center):                 │
│       -> State: LOCKED_CENTER (Motors completely stopped)          │
│                                                                    │
│  2. Proportional Pivot Velocity:                                   │
│     If error > +0.08 (Person is to the RIGHT):                     │
│       -> v_left = +speed, v_right = -speed (PIVOT_RIGHT)           │
│     If error < -0.08 (Person is to the LEFT):                      │
│       -> v_left = -speed, v_right = +speed (PIVOT_LEFT)            │
│                                                                    │
│  3. Stiction Overcome:                                             │
│     • Minimum turn speed (0.18 m/s) ensures gearboxes turn cleanly.│
│     • 100 ms startup kick (0.55 m/s) when starting from standstill.│
│                                                                    │
│  4. Safety Supervisor:                                             │
│     • Target lost > 1.0s -> Safe immediate halt (0.0, 0.0).        │
└─────────────────────────────────┬──────────────────────────────────┘
                                  │ 20 Hz Heartbeat: "<left>,<right>\n"
                                  ▼
                         ESP32-S3 (/dev/serial0)
```

---

## 2. How to Run

### Mode A: Live Hardware Run with GUI Preview (Monitor / VNC)
```bash
python3 human_follower/main.py
```
* Shows the live camera preview with target bounding box, deadband zone, and rotation banners (`PIVOT RIGHT >>>`, `<<< PIVOT LEFT`, `TARGET CENTERED`).
* Press **`q`** or **`ESC`** to exit and stop motors.

### Mode B: Headless Execution Over SSH (No Display)
```bash
python3 human_follower/main.py --headless
```
* Streams live tracking telemetry to your terminal at 2 Hz:
  ```text
  [ PIVOT_RIGHT ] H-Err: +0.22 | Bearing:  +7.2° | Wheels: L=+0.20, R=-0.20 m/s | FPS: 14.2
  [LOCKED_CENTER] H-Err: +0.03 | Bearing:  +1.0° | Wheels: L=+0.00, R=+0.00 m/s | FPS: 14.5
  [ PIVOT_LEFT  ] H-Err: -0.19 | Bearing:  -6.1° | Wheels: L=-0.19, R=+0.19 m/s | FPS: 14.3
  [  SEARCHING  ] H-Err: +0.00 | Bearing:  +0.0° | Wheels: L=+0.00, R=+0.00 m/s | FPS: 14.1
  ```

### Mode C: Mock / Desktop Simulation (No Camera or ESP32 Needed)
```bash
python3 human_follower/main.py --mock --headless
```
* Simulates synthetic video frames and a walking person to verify control math and states on any PC.

---

## 3. Tuning Parameters

You can adjust follower sensitivity directly from the command line:

```bash
python3 human_follower/main.py --kp 0.50 --deadband 0.08 --min-speed 0.18 --max-speed 0.35
```

| Parameter | Default | Purpose |
| :--- | :--- | :--- |
| `--kp` | `0.45` | Steering proportional gain. Higher = faster, more aggressive turning. |
| `--deadband` | `0.08` | Center deadband (±8% of frame center). Prevents motor wiggling when person is centered. |
| `--min-speed` | `0.18` | Minimum wheel speed (m/s) to overcome DC motor gearbox static friction. |
| `--max-speed` | `0.40` | Speed clamp (m/s) to ensure smooth, controlled pivoting. |
| `--confidence`| `0.40` | Minimum detection confidence to accept a target. |

---

## 4. Verification & Unit Tests

Run the test suite:
```bash
python3 -m unittest discover -s human_follower/tests -v
```

All 6 tests verify:
1. Deadband logic (`LOCKED_CENTER` $\to$ speeds `0.0, 0.0`).
2. Right pivot kinematics ($v_L > 0, v_R < 0$).
3. Left pivot kinematics ($v_L < 0, v_R > 0$).
4. Speed clamping between `min_turn_speed` and `max_turn_speed`.
5. Failsafe motor halt when target is lost or stale ($> 1.0\text{s}$).
6. Clean startup and shutdown service lifecycle.

