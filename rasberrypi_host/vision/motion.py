"""Motion analysis, gait stance detection, and vision-only car stop arbiter.

Provides three cooperating subsystems:
1. **GaitAnalyzer** – classifies the target as walking / standing / unknown
   using scale-invariant COCO-17 keypoint dynamics (stride variance, knee
   flexion, upper-body sway).
2. **DepthCascade** – estimates depth-change rate from the best available
   skeletal or bounding-box signal, robust to partial occlusion.
3. **VisionCarStopArbiter** – fuses gait state and depth trend to command
   an immediate vehicle stop when the human halts ahead while the camera
   (and therefore the car) is still closing in.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from config import Config
from tracker import Track, center

# ── COCO-17 keypoint indices ─────────────────────────────────────────
KP_L_SHOULDER = 5
KP_R_SHOULDER = 6
KP_L_HIP = 11
KP_R_HIP = 12
KP_L_KNEE = 13
KP_R_KNEE = 14
KP_L_ANKLE = 15
KP_R_ANKLE = 16
KP_L_ELBOW = 7
KP_R_ELBOW = 8


# ─────────────────────────────────────────────────────────────────────
# Motion config (unchanged from original API)
# ─────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class MotionConfig:
    window_seconds: float = 1.5
    max_observation_gap_seconds: float = 2.0
    min_span_seconds: float = 0.2

    # Normalized image-width/image-height movement per second.
    position_threshold_per_second: float = 0.08

    # Fractional bounding-box-height change per second.
    size_threshold_per_second: float = 0.12


@dataclass(frozen=True)
class Sample:
    timestamp: float
    center_x: float  # normalized to frame width
    center_y: float  # normalized to frame height
    box_height: float  # normalized to frame height
    box_clipped: bool


# ─────────────────────────────────────────────────────────────────────
# Gait Analyzer
# ─────────────────────────────────────────────────────────────────────

class GaitAnalyzer:
    """Classifies a tracked person as walking or standing using skeletal
    keypoint dynamics that are invariant to camera translation/zoom.

    Primary signal (full / lower-body visible):
        Normalized inter-ankle stride distance variance over a rolling
        window.  Walking produces periodic stride oscillation; standing
        freezes it.

    Fallback signal (upper-body only):
        Shoulder width oscillation and vertical torso heave that
        accompany the natural gait cycle.
    """

    def __init__(self, config: Config) -> None:
        self.config = config
        self.target_id: int | None = None
        # (timestamp, normalized_stride, knee_angle_sum)
        self._stride_history: deque[tuple[float, float, float]] = deque()
        # (timestamp, shoulder_ratio, torso_y)
        self._upper_history: deque[tuple[float, float, float]] = deque()
        self._gait_confidence: float = 0.0

    def reset(self) -> None:
        self.target_id = None
        self._stride_history.clear()
        self._upper_history.clear()
        self._gait_confidence = 0.0

    @property
    def gait_confidence(self) -> float:
        return self._gait_confidence

    # ── helpers ───────────────────────────────────────────────────────

    @staticmethod
    def _joint_visible(
        kp: NDArray[np.float32], idx: int, threshold: float
    ) -> bool:
        return bool(kp[idx, 2] >= threshold)

    @staticmethod
    def _angle_between(
        a: NDArray, b: NDArray, c: NDArray
    ) -> float:
        """Angle at vertex *b* formed by segments b→a and b→c (radians)."""
        ba = a[:2] - b[:2]
        bc = c[:2] - b[:2]
        cos_val = float(
            np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-8)
        )
        return math.acos(max(-1.0, min(1.0, cos_val)))

    # ── update ────────────────────────────────────────────────────────

    def update(
        self,
        target: Track | None,
        now: float,
    ) -> dict:
        """Return gait classification dict for the current target.

        Returns keys:
            gait_state  : "walking" | "standing" | "slow_or_pausing"
                          | "warming_up" | "unknown"
            gait_confidence : 0.0 – 1.0
            visibility      : "full_body" | "upper_body" | "none"
        """
        if target is None or target.keypoints is None:
            self.reset()
            return {
                "gait_state": "unknown",
                "gait_confidence": 0.0,
                "visibility": "none",
            }

        if self.target_id != target.id:
            self.reset()
            self.target_id = target.id

        kp = target.keypoints
        conf_thresh = self.config.keypoint_confidence_threshold

        # Try lower-body (ankles + hips) first.
        lower_visible = all(
            self._joint_visible(kp, idx, conf_thresh)
            for idx in (KP_L_HIP, KP_R_HIP, KP_L_ANKLE, KP_R_ANKLE)
        )

        if lower_visible:
            return self._update_lower_body(kp, now, conf_thresh)

        # Fallback to upper-body (shoulders + hips).
        upper_visible = all(
            self._joint_visible(kp, idx, conf_thresh)
            for idx in (KP_L_SHOULDER, KP_R_SHOULDER, KP_L_HIP, KP_R_HIP)
        )
        if upper_visible:
            return self._update_upper_body(kp, now)

        return {
            "gait_state": "unknown",
            "gait_confidence": 0.0,
            "visibility": "none",
        }

    # ── lower body ────────────────────────────────────────────────────

    def _update_lower_body(
        self,
        kp: NDArray[np.float32],
        now: float,
        conf_thresh: float,
    ) -> dict:
        # Reference leg length (average of left and right hip→ankle).
        l_leg = float(
            np.linalg.norm(kp[KP_L_HIP, :2] - kp[KP_L_ANKLE, :2])
        )
        r_leg = float(
            np.linalg.norm(kp[KP_R_HIP, :2] - kp[KP_R_ANKLE, :2])
        )
        leg_ref = (l_leg + r_leg) / 2.0 + 1e-5

        # Scale-invariant stride distance.
        ankle_dist = float(
            np.linalg.norm(kp[KP_L_ANKLE, :2] - kp[KP_R_ANKLE, :2])
        )
        normalized_stride = ankle_dist / leg_ref

        # Knee flexion angles (if knees visible).
        knee_angle_sum = 0.0
        knee_count = 0
        if self._joint_visible(kp, KP_L_KNEE, conf_thresh):
            knee_angle_sum += self._angle_between(
                kp[KP_L_HIP], kp[KP_L_KNEE], kp[KP_L_ANKLE]
            )
            knee_count += 1
        if self._joint_visible(kp, KP_R_KNEE, conf_thresh):
            knee_angle_sum += self._angle_between(
                kp[KP_R_HIP], kp[KP_R_KNEE], kp[KP_R_ANKLE]
            )
            knee_count += 1
        avg_knee = knee_angle_sum / knee_count if knee_count else math.pi

        self._stride_history.append((now, normalized_stride, avg_knee))

        # Trim window.
        cutoff = now - self.config.gait_window_seconds
        while self._stride_history and self._stride_history[0][0] < cutoff:
            self._stride_history.popleft()

        if len(self._stride_history) < 5:
            return {
                "gait_state": "warming_up",
                "gait_confidence": 0.0,
                "visibility": "full_body",
            }

        strides = [s for _, s, _ in self._stride_history]
        stride_var = float(np.var(strides))

        knee_angles = [k for _, _, k in self._stride_history]
        knee_var = float(np.var(knee_angles))

        # Combined confidence weighting.
        stop_thresh = self.config.gait_stop_variance_threshold
        walk_thresh = self.config.gait_walk_variance_threshold

        if stride_var < stop_thresh and knee_var < 0.02:
            state = "standing"
            self._gait_confidence = min(
                1.0, 1.0 - stride_var / stop_thresh
            )
        elif stride_var > walk_thresh or knee_var > 0.05:
            state = "walking"
            self._gait_confidence = min(
                1.0, stride_var / walk_thresh
            )
        else:
            state = "slow_or_pausing"
            self._gait_confidence = 0.5

        return {
            "gait_state": state,
            "gait_confidence": round(self._gait_confidence, 3),
            "visibility": "full_body",
        }

    # ── upper body fallback ───────────────────────────────────────────

    def _update_upper_body(
        self,
        kp: NDArray[np.float32],
        now: float,
    ) -> dict:
        """Use shoulder-width oscillation and torso midpoint vertical
        bounce as a proxy for gait when the lower body is occluded."""
        shoulder_width = float(
            np.linalg.norm(kp[KP_L_SHOULDER, :2] - kp[KP_R_SHOULDER, :2])
        )
        hip_width = float(
            np.linalg.norm(kp[KP_L_HIP, :2] - kp[KP_R_HIP, :2])
        )
        body_ref = (shoulder_width + hip_width) / 2.0 + 1e-5
        shoulder_ratio = shoulder_width / body_ref

        torso_mid_y = float(
            (kp[KP_L_SHOULDER, 1] + kp[KP_R_SHOULDER, 1]
             + kp[KP_L_HIP, 1] + kp[KP_R_HIP, 1]) / 4.0
        )

        self._upper_history.append((now, shoulder_ratio, torso_mid_y))
        cutoff = now - self.config.gait_window_seconds
        while self._upper_history and self._upper_history[0][0] < cutoff:
            self._upper_history.popleft()

        if len(self._upper_history) < 5:
            return {
                "gait_state": "warming_up",
                "gait_confidence": 0.0,
                "visibility": "upper_body",
            }

        ratios = [r for _, r, _ in self._upper_history]
        ys = [y for _, _, y in self._upper_history]
        ratio_var = float(np.var(ratios))
        y_var = float(np.var(ys))

        # Shoulder yaw oscillation + vertical bobbing during walking.
        combined_var = ratio_var + y_var * 1e-4

        stop_thresh = self.config.gait_stop_variance_threshold * 0.3
        walk_thresh = self.config.gait_walk_variance_threshold * 0.3

        if combined_var < stop_thresh:
            state = "standing"
            self._gait_confidence = min(
                1.0, max(0.4, 1.0 - combined_var / stop_thresh)
            )
        elif combined_var > walk_thresh:
            state = "walking"
            self._gait_confidence = min(
                1.0, combined_var / walk_thresh
            )
        else:
            state = "slow_or_pausing"
            self._gait_confidence = 0.4

        return {
            "gait_state": state,
            "gait_confidence": round(self._gait_confidence, 3),
            "visibility": "upper_body",
        }


# ─────────────────────────────────────────────────────────────────────
# Depth Cascade
# ─────────────────────────────────────────────────────────────────────

class DepthCascade:
    """Estimates depth-change rate from the best available skeletal or
    bounding-box signal, in priority order.

    Priority:
        1. Full bbox height (unclipped)
        2. Thigh segment length (hip → knee)
        3. Upper-arm segment length (shoulder → elbow)
        4. Bbox width (unclipped laterally)
        5. Bottom-y displacement rate

    All rates are fractional expansion per second: positive means the
    target is getting closer (proxy scale is growing).
    """

    def __init__(self, config: Config) -> None:
        self.config = config
        self.target_id: int | None = None
        # (timestamp, proxy_value, source_name)
        self._history: deque[tuple[float, float, str]] = deque()
        self._window = max(config.gait_window_seconds, 0.5)

    def reset(self) -> None:
        self.target_id = None
        self._history.clear()

    def update(
        self,
        target: Track | None,
        frame_width: int,
        frame_height: int,
        now: float,
    ) -> dict:
        """Return depth-change rate and source information.

        Keys:
            depth_expansion_rate : float | None  (positive = closing)
            depth_signal_source  : str
            time_to_collision_s  : float | None
        """
        if target is None:
            self._history.clear()
            return {
                "depth_expansion_rate": None,
                "depth_signal_source": "none",
                "time_to_collision_s": None,
            }

        if self.target_id != target.id:
            self._history.clear()
            self.target_id = target.id

        x1, y1, x2, y2 = target.bbox
        kp = target.keypoints
        conf_thresh = self.config.keypoint_confidence_threshold

        clipped_vert = y1 <= 1 or y2 >= frame_height - 1
        clipped_horiz = x1 <= 1 or x2 >= frame_width - 1

        proxy: float | None = None
        source: str = "none"

        # Priority 1: Full bbox height (not clipped top/bottom).
        if not clipped_vert:
            proxy = y2 - y1
            source = "full_height"

        # Priority 2: Thigh segment length.
        if proxy is None and kp is not None:
            l_ok = (
                kp[KP_L_HIP, 2] >= conf_thresh
                and kp[KP_L_KNEE, 2] >= conf_thresh
            )
            r_ok = (
                kp[KP_R_HIP, 2] >= conf_thresh
                and kp[KP_R_KNEE, 2] >= conf_thresh
            )
            if l_ok and r_ok:
                proxy = float(
                    (np.linalg.norm(kp[KP_L_HIP, :2] - kp[KP_L_KNEE, :2])
                     + np.linalg.norm(kp[KP_R_HIP, :2] - kp[KP_R_KNEE, :2]))
                    / 2.0
                )
                source = "thigh_length"
            elif l_ok:
                proxy = float(
                    np.linalg.norm(kp[KP_L_HIP, :2] - kp[KP_L_KNEE, :2])
                )
                source = "thigh_length"
            elif r_ok:
                proxy = float(
                    np.linalg.norm(kp[KP_R_HIP, :2] - kp[KP_R_KNEE, :2])
                )
                source = "thigh_length"

        # Priority 3: Upper-arm segment length.
        if proxy is None and kp is not None:
            l_ok = (
                kp[KP_L_SHOULDER, 2] >= conf_thresh
                and kp[KP_L_ELBOW, 2] >= conf_thresh
            )
            r_ok = (
                kp[KP_R_SHOULDER, 2] >= conf_thresh
                and kp[KP_R_ELBOW, 2] >= conf_thresh
            )
            if l_ok and r_ok:
                proxy = float(
                    (np.linalg.norm(
                        kp[KP_L_SHOULDER, :2] - kp[KP_L_ELBOW, :2])
                     + np.linalg.norm(
                         kp[KP_R_SHOULDER, :2] - kp[KP_R_ELBOW, :2]))
                    / 2.0
                )
                source = "upper_arm"
            elif l_ok:
                proxy = float(
                    np.linalg.norm(
                        kp[KP_L_SHOULDER, :2] - kp[KP_L_ELBOW, :2])
                )
                source = "upper_arm"
            elif r_ok:
                proxy = float(
                    np.linalg.norm(
                        kp[KP_R_SHOULDER, :2] - kp[KP_R_ELBOW, :2])
                )
                source = "upper_arm"

        # Priority 4: Bbox width (not clipped horizontally).
        if proxy is None and not clipped_horiz:
            proxy = x2 - x1
            source = "bbox_width"

        # Priority 5: Bottom-y position.
        if proxy is None:
            proxy = y2
            source = "bottom_y"

        if proxy is None:
            return {
                "depth_expansion_rate": None,
                "depth_signal_source": "none",
                "time_to_collision_s": None,
            }

        self._history.append((now, proxy, source))
        cutoff = now - self._window
        while self._history and self._history[0][0] < cutoff:
            self._history.popleft()

        if len(self._history) < 2:
            return {
                "depth_expansion_rate": None,
                "depth_signal_source": source,
                "time_to_collision_s": None,
            }

        first_t, first_val, _ = self._history[0]
        last_t, last_val, _ = self._history[-1]
        elapsed = last_t - first_t
        if elapsed < 0.1 or first_val <= 0:
            return {
                "depth_expansion_rate": None,
                "depth_signal_source": source,
                "time_to_collision_s": None,
            }

        # Fractional expansion rate: positive → scale growing → closing.
        rate = ((last_val / first_val) - 1.0) / elapsed

        ttc: float | None = None
        if rate > 0.001:
            ttc = 1.0 / rate

        return {
            "depth_expansion_rate": round(rate, 5),
            "depth_signal_source": source,
            "time_to_collision_s": round(ttc, 2) if ttc is not None else None,
        }


# ─────────────────────────────────────────────────────────────────────
# Vision-Only Car Stop Arbiter
# ─────────────────────────────────────────────────────────────────────

class VisionCarStopArbiter:
    """Fuses gait state and depth trend to decide whether the car must
    stop — purely from camera data, no IMU or distance sensors.

    Decision matrix:
        human STANDING  +  depth expanding  →  car closing on stopped human
                                                → BRAKE
        human STANDING  +  depth steady     →  both stationary → hold
        any gait        +  TTC critical     →  collision hazard → BRAKE
        human WALKING   +  depth steady     →  normal follow
    """

    def __init__(self, config: Config) -> None:
        self.config = config
        self._stop_commanded: bool = False
        self._stop_commanded_at: float | None = None

    def reset(self) -> None:
        self._stop_commanded = False
        self._stop_commanded_at = None

    def evaluate(
        self,
        gait_state: str,
        depth_expansion_rate: float | None,
        ttc: float | None,
        now: float,
    ) -> dict:
        """Return car stop decision.

        Keys:
            car_stop_required : bool
            emergency_brake   : bool
            stop_reason       : str
            interaction_state : str
        """
        car_stop = False
        emergency = False
        reason = "none"
        interaction = "unknown"

        human_stopped = gait_state in ("standing",)
        human_active = gait_state in ("walking",)
        rate = depth_expansion_rate or 0.0
        camera_closing = rate > self.config.looming_stop_threshold
        camera_steady = abs(rate) <= self.config.looming_stop_threshold

        # ── Critical TTC override (regardless of gait) ───────────────
        if ttc is not None and ttc < self.config.critical_ttc_seconds:
            car_stop = True
            emergency = True
            reason = "collision_hazard"
            interaction = "collision_imminent"

        # ── Human stopped + camera closing in ─────────────────────────
        elif human_stopped and camera_closing:
            car_stop = True
            emergency = True
            reason = "human_halted_ahead"
            interaction = "human_stopped_car_moving"

        # ── Human stopped + everything steady ─────────────────────────
        elif human_stopped and camera_steady:
            car_stop = True
            emergency = False
            reason = "both_stationary"
            interaction = "both_stopped"

        # ── Human walking + camera closing (approaching robot) ────────
        elif human_active and camera_closing:
            interaction = "human_approaching"
            # Let the downstream controller decide (yield / reverse).

        # ── Normal following ──────────────────────────────────────────
        elif human_active:
            interaction = "both_moving"

        # ── Debounce: hold stop for stop_hold_seconds ─────────────────
        if car_stop:
            self._stop_commanded = True
            self._stop_commanded_at = now
        elif self._stop_commanded and self._stop_commanded_at is not None:
            if now - self._stop_commanded_at < self.config.stop_hold_seconds:
                car_stop = True
                reason = reason if reason != "none" else "debounce_hold"
            else:
                self._stop_commanded = False
                self._stop_commanded_at = None

        return {
            "car_stop_required": car_stop,
            "emergency_brake": emergency,
            "stop_reason": reason,
            "interaction_state": interaction,
        }


# ─────────────────────────────────────────────────────────────────────
# Unified Target Motion Analyzer (public API, wires subsystems)
# ─────────────────────────────────────────────────────────────────────

class TargetMotionAnalyzer:
    """Drop-in replacement for the original TargetMotionAnalyzer.

    Backwards-compatible: still emits the original motion_status /
    movement_direction / velocity / distance_trend keys.  Adds gait,
    depth-cascade, and car-stop keys on top.
    """

    def __init__(
        self,
        config: Config | None = None,
        motion_config: MotionConfig | None = None,
    ) -> None:
        self._cfg = config or Config()
        self._mcfg = motion_config or MotionConfig()

        self.target_id: int | None = None
        self.samples: deque[Sample] = deque()

        # Subsystems
        self.gait = GaitAnalyzer(self._cfg)
        self.depth_cascade = DepthCascade(self._cfg)
        self.car_stop = VisionCarStopArbiter(self._cfg)

    def reset(self) -> None:
        self.target_id = None
        self.samples.clear()
        self.gait.reset()
        self.depth_cascade.reset()
        self.car_stop.reset()

    @staticmethod
    def _direction(vx: float, vy: float, threshold: float) -> str:
        horizontal = (
            "right" if vx > threshold
            else "left" if vx < -threshold
            else ""
        )
        vertical = (
            "down" if vy > threshold
            else "up" if vy < -threshold
            else ""
        )

        if horizontal and vertical:
            return f"{vertical}-{horizontal}"
        return horizontal or vertical or "stationary"

    def update(
        self,
        target: Track | None,
        frame_width: int,
        frame_height: int,
        now: float,
    ) -> dict:
        # ── Gait & depth cascade (always run) ────────────────────────
        gait_result = self.gait.update(target, now)
        depth_result = self.depth_cascade.update(
            target, frame_width, frame_height, now
        )

        # ── Car stop arbiter ──────────────────────────────────────────
        stop_result = self.car_stop.evaluate(
            gait_state=gait_result["gait_state"],
            depth_expansion_rate=depth_result["depth_expansion_rate"],
            ttc=depth_result["time_to_collision_s"],
            now=now,
        )

        if target is None:
            # No measured person box: do not report predicted motion.
            self.samples.clear()
            result = {
                "motion_status": "not_observed",
                "movement_direction": "unknown",
                "horizontal_velocity": None,
                "vertical_velocity": None,
                "distance_trend": "unknown",
                "bbox_height_change_per_second": None,
            }
            result.update(gait_result)
            result.update(depth_result)
            result.update(stop_result)
            return result

        if self.target_id != target.id:
            self.samples.clear()
            self.target_id = target.id

        if self.samples:
            gap = now - self.samples[-1].timestamp
            if gap <= 0 or gap > self._mcfg.max_observation_gap_seconds:
                self.samples.clear()

        x1, y1, x2, y2 = target.bbox
        cx, cy = center(target.bbox)
        clipped = (
            x1 <= 1 or y1 <= 1
            or x2 >= frame_width - 1
            or y2 >= frame_height - 1
        )

        self.samples.append(
            Sample(
                timestamp=now,
                center_x=cx / frame_width,
                center_y=cy / frame_height,
                box_height=(y2 - y1) / frame_height,
                box_clipped=clipped,
            )
        )

        # Keep one sample just before the window boundary if available:
        # this still permits a useful estimate at a low Pi inference FPS.
        while (
            len(self.samples) > 2
            and self.samples[1].timestamp < now - self._mcfg.window_seconds
        ):
            self.samples.popleft()

        first = self.samples[0]
        last = self.samples[-1]
        elapsed = last.timestamp - first.timestamp

        if elapsed < self._mcfg.min_span_seconds:
            result = {
                "motion_status": "warming_up",
                "movement_direction": "unknown",
                "horizontal_velocity": None,
                "vertical_velocity": None,
                "distance_trend": "unknown",
                "bbox_height_change_per_second": None,
            }
            result.update(gait_result)
            result.update(depth_result)
            result.update(stop_result)
            return result

        # Velocities use normalized image dimensions per second.
        vx = (last.center_x - first.center_x) / elapsed
        vy = (last.center_y - first.center_y) / elapsed
        direction = self._direction(
            vx, vy, self._mcfg.position_threshold_per_second
        )

        # A clipped box is not a useful estimate of full person height.
        valid_height_samples = all(
            not sample.box_clipped for sample in self.samples
        )
        height_rate: float | None = None
        distance_trend = "unknown"

        if valid_height_samples and first.box_height > 0:
            height_rate = (
                (last.box_height / first.box_height) - 1.0
            ) / elapsed

            if height_rate > self._mcfg.size_threshold_per_second:
                distance_trend = "appearing_closer"
            elif height_rate < -self._mcfg.size_threshold_per_second:
                distance_trend = "appearing_farther"
            else:
                distance_trend = "roughly_unchanged"

        # ── Determine human_motion_state (enriched) ───────────────────
        gait_state = gait_result["gait_state"]
        if gait_state == "standing":
            human_motion_state = "stopped"
        elif gait_state == "walking":
            human_motion_state = "moving"
        elif gait_state == "slow_or_pausing":
            human_motion_state = "slowing"
        else:
            human_motion_state = "unknown"

        result = {
            "motion_status": "measured",
            "movement_direction": direction,
            "horizontal_velocity": round(vx, 4),
            "vertical_velocity": round(vy, 4),
            "distance_trend": distance_trend,
            "bbox_height_change_per_second": (
                round(height_rate, 4) if height_rate is not None else None
            ),
            "human_motion_state": human_motion_state,
        }
        result.update(gait_result)
        result.update(depth_result)
        result.update(stop_result)
        return result
