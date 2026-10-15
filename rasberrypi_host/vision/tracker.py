from collections import deque
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from detector import PersonDetection

Box = tuple[float, float, float, float]

def center(box: Box) -> tuple[float, float]:
    x1, y1, x2, y2 = box
    return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)


def iou(a: Box, b: Box) -> float:
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    union = area_a + area_b - intersection
    return intersection / union if union > 0 else 0.0


# Maximum keypoint history entries kept per track (used by gait analysis).
_MAX_KEYPOINT_HISTORY = 30


@dataclass
class Track:
    id: int
    bbox: Box
    confidence: float
    last_seen: float
    last_update: float
    observed_center: tuple[float, float]
    velocity: tuple[float, float] = (0.0, 0.0)  # pixels per second
    hits: int = 1
    visible: bool = True

    # ── Pose keypoints ────────────────────────────────────────────────
    # Latest smoothed keypoints array: shape (17, 3) = [x, y, conf].
    # None when the model is detection-only or detection had no keypoints.
    keypoints: NDArray[np.float32] | None = None

    # Rolling history of (timestamp, raw_keypoints) for gait analysis.
    keypoints_history: deque = field(
        default_factory=lambda: deque(maxlen=_MAX_KEYPOINT_HISTORY)
    )

    # Exponential smoothing factor for keypoint position filtering.
    _kp_alpha: float = 0.55

    def predict_to(self, now: float) -> None:
        dt = max(0.0, min(now - self.last_update, 0.5))
        dx, dy = self.velocity[0] * dt, self.velocity[1] * dt
        x1, y1, x2, y2 = self.bbox
        self.bbox = (x1 + dx, y1 + dy, x2 + dx, y2 + dy)
        self.last_update = now
        self.visible = False

    def correct(
        self,
        detection: PersonDetection,
        now: float,
        kp_alpha: float = 0.55,
    ) -> None:
        new_center = center(detection.bbox)
        dt = max(now - self.last_seen, 1e-3)

        # Smooth motion to reduce box jitter.
        measured_vx = (new_center[0] - self.observed_center[0]) / dt
        measured_vy = (new_center[1] - self.observed_center[1]) / dt
        alpha = 0.45
        self.velocity = (
            (1 - alpha) * self.velocity[0] + alpha * measured_vx,
            (1 - alpha) * self.velocity[1] + alpha * measured_vy,
        )
        self.observed_center = new_center
        self.bbox = detection.bbox
        self.confidence = detection.confidence
        self.last_seen = now
        self.last_update = now
        self.hits += 1
        self.visible = True

        # ── Update keypoints ──────────────────────────────────────────
        if detection.keypoints is not None:
            raw_kp = detection.keypoints  # (17, 3)
            if self.keypoints is not None:
                # Exponentially smooth x, y positions for joints that are
                # confident in both the old and new observation.  Confidence
                # values are always taken from the latest detection.
                smoothed = self.keypoints.copy()
                for j in range(raw_kp.shape[0]):
                    if raw_kp[j, 2] > 0.01 and smoothed[j, 2] > 0.01:
                        smoothed[j, 0] = (
                            (1 - kp_alpha) * smoothed[j, 0]
                            + kp_alpha * raw_kp[j, 0]
                        )
                        smoothed[j, 1] = (
                            (1 - kp_alpha) * smoothed[j, 1]
                            + kp_alpha * raw_kp[j, 1]
                        )
                    else:
                        smoothed[j, 0] = raw_kp[j, 0]
                        smoothed[j, 1] = raw_kp[j, 1]
                    smoothed[j, 2] = raw_kp[j, 2]
                self.keypoints = smoothed
            else:
                self.keypoints = raw_kp.copy()

            self.keypoints_history.append((now, raw_kp.copy()))


class PersonTracker:
    """Small greedy SORT-style tracker; no neural appearance embedding."""

    def __init__(
        self,
        lost_seconds: float,
        kp_smoothing_alpha: float = 0.55,
    ) -> None:
        self.lost_seconds = lost_seconds
        self.kp_smoothing_alpha = kp_smoothing_alpha
        self.tracks: dict[int, Track] = {}
        self.next_id = 1

    @staticmethod
    def _match_score(track: Track, detection: PersonDetection) -> float | None:
        a, b = track.bbox, detection.bbox
        aw, ah = a[2] - a[0], a[3] - a[1]
        bw, bh = b[2] - b[0], b[3] - b[1]
        if min(aw, ah, bw, bh) <= 0:
            return None

        size_ratio = (bw * bh) / (aw * ah)
        if not 0.35 <= size_ratio <= 2.8:
            return None

        acx, acy = center(a)
        bcx, bcy = center(b)
        diagonal = max((aw * aw + ah * ah) ** 0.5, 1.0)
        distance = ((acx - bcx) ** 2 + (acy - bcy) ** 2) ** 0.5
        overlap = iou(a, b)

        # Permit short motion without overlap, but not arbitrary re-ID.
        if overlap < 0.10 and distance > 0.45 * diagonal:
            return None
        return overlap + 0.25 * max(0.0, 1.0 - distance / diagonal)

    def update(
        self, detections: list[PersonDetection], now: float
    ) -> list[Track]:
        for track_id in list(self.tracks):
            if now - self.tracks[track_id].last_seen > self.lost_seconds:
                del self.tracks[track_id]

        for track in self.tracks.values():
            track.predict_to(now)

        candidates: list[tuple[float, int, int]] = []
        for track_id, track in self.tracks.items():
            for detection_index, detection in enumerate(detections):
                score = self._match_score(track, detection)
                if score is not None:
                    candidates.append((score, track_id, detection_index))

        used_tracks: set[int] = set()
        used_detections: set[int] = set()
        for _, track_id, detection_index in sorted(candidates, reverse=True):
            if track_id in used_tracks or detection_index in used_detections:
                continue
            self.tracks[track_id].correct(
                detections[detection_index],
                now,
                kp_alpha=self.kp_smoothing_alpha,
            )
            used_tracks.add(track_id)
            used_detections.add(detection_index)

        for index, detection in enumerate(detections):
            if index in used_detections:
                continue
            track_id = self.next_id
            self.next_id += 1
            new_track = Track(
                id=track_id,
                bbox=detection.bbox,
                confidence=detection.confidence,
                last_seen=now,
                last_update=now,
                observed_center=center(detection.bbox),
            )
            if detection.keypoints is not None:
                new_track.keypoints = detection.keypoints.copy()
                new_track.keypoints_history.append(
                    (now, detection.keypoints.copy())
                )
            self.tracks[track_id] = new_track

        # Only current observations count as detected people. Lost tracks
        # remain internal so a returning observation can recover its ID.
        return sorted(
            (track for track in self.tracks.values() if track.visible),
            key=lambda track: track.id,
        )
