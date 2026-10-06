from tracker import Track, center


class TargetSelector:
    def __init__(self, hold_seconds: float, min_hits: int) -> None:
        self.hold_seconds = hold_seconds
        self.min_hits = min_hits
        self.target_id: int | None = None
        self.last_seen: float | None = None
        self.reference_height: float | None = None

    @staticmethod
    def acquisition_score(track: Track, width: int, height: int) -> float:
        cx, _ = center(track.bbox)
        bbox_height = track.bbox[3] - track.bbox[1]
        centeredness = 1.0 - min(abs(cx - width / 2) / (width / 2), 1.0)
        apparent_size = min(bbox_height / height, 1.0)
        return 0.55 * centeredness + 0.45 * apparent_size

    def select(
        self, tracks: list[Track], width: int, height: int, now: float
    ) -> Track | None:
        visible = {track.id: track for track in tracks}

        if self.target_id is not None:
            existing = visible.get(self.target_id)
            if existing is not None:
                self.last_seen = now
                return existing

            # Output "not detected" during the gap, but do not take another
            # person as target yet.
            if (
                self.last_seen is not None
                and now - self.last_seen <= self.hold_seconds
            ):
                return None

            self.target_id = None
            self.reference_height = None

        eligible = [t for t in tracks if t.hits >= self.min_hits]
        if not eligible:
            return None

        selected = max(
            eligible,
            key=lambda t: self.acquisition_score(t, width, height),
        )
        self.target_id = selected.id
        self.last_seen = now
        self.reference_height = selected.bbox[3] - selected.bbox[1]
        return selected
