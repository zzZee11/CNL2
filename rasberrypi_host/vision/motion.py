from collections import deque
from dataclasses import dataclass

from tracker import Track, center


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


class TargetMotionAnalyzer:
    def __init__(self, config: MotionConfig | None = None) -> None:
        self.config = config or MotionConfig()
        self.target_id: int | None = None
        self.samples: deque[Sample] = deque()

    def reset(self) -> None:
        self.target_id = None
        self.samples.clear()

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
        if target is None:
            # No measured person box: do not report predicted motion.
            self.samples.clear()
            return {
                "motion_status": "not_observed",
                "movement_direction": "unknown",
                "horizontal_velocity": None,
                "vertical_velocity": None,
                "distance_trend": "unknown",
                "bbox_height_change_per_second": None,
            }

        if self.target_id != target.id:
            self.samples.clear()
            self.target_id = target.id

        if self.samples:
            gap = now - self.samples[-1].timestamp
            if gap <= 0 or gap > self.config.max_observation_gap_seconds:
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
            and self.samples[1].timestamp < now - self.config.window_seconds
        ):
            self.samples.popleft()

        first = self.samples[0]
        last = self.samples[-1]
        elapsed = last.timestamp - first.timestamp

        if elapsed < self.config.min_span_seconds:
            return {
                "motion_status": "warming_up",
                "movement_direction": "unknown",
                "horizontal_velocity": None,
                "vertical_velocity": None,
                "distance_trend": "unknown",
                "bbox_height_change_per_second": None,
            }

        # Velocities use normalized image dimensions per second.
        vx = (last.center_x - first.center_x) / elapsed
        vy = (last.center_y - first.center_y) / elapsed
        direction = self._direction(
            vx, vy, self.config.position_threshold_per_second
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

            if height_rate > self.config.size_threshold_per_second:
                distance_trend = "appearing_closer"
            elif height_rate < -self.config.size_threshold_per_second:
                distance_trend = "appearing_farther"
            else:
                distance_trend = "roughly_unchanged"

        return {
            "motion_status": "measured",
            "movement_direction": direction,
            "horizontal_velocity": round(vx, 4),
            "vertical_velocity": round(vy, 4),
            "distance_trend": distance_trend,
            "bbox_height_change_per_second": (
                round(height_rate, 4) if height_rate is not None else None
            ),
        }
