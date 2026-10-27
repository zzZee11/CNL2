"""
vision_bridge.py - connects the teammate's vision pipeline (rasberrypi_host/vision)
to robot_main.py WITHOUT editing any of his files.

His VisionSystem.process_frame(frame, now) already returns a dict per frame with
target_detected, horizontal_error (-1..+1, +1 = right edge of the image),
distance_m (only when calibrated), relative_distance_ratio, car_stop_required, ...

This file does three things:
  1. VisionBridge     runs camera + YOLO in a background thread, keeps the latest result
  2. make_target_fn   turns that result into the person's WORLD position (x, y)
  3. make_stop_fn     exposes his car_stop_required / emergency_brake flag

Suggested layout on the Pi:
    rasberrypi_host/
        robot_main.py   vision_bridge.py   a_search_main.py
        vision/         <- the teammate's folder, unchanged
"""
import math
import threading
import time

MAX_AGE = 0.5      # s - a vision result older than this is treated as "no person seen"


class VisionBridge:
    """system  : object with process_frame(frame, now) -> dict   (his VisionSystem)
       capture : object with read() -> (ok, frame)               (cv2.VideoCapture)"""

    def __init__(self, system, capture, max_age=MAX_AGE):
        self.system = system
        self.capture = capture
        self.max_age = max_age
        self.error = None
        self._lock = threading.Lock()
        self._latest = None
        self._stamp = 0.0
        self._failed_reads = 0
        self._running = False

    # ---- one iteration (the thread just calls this in a loop; tests call it directly) ----
    def step(self):
        ok, frame = self.capture.read()
        if not ok:
            self._failed_reads += 1
            if self._failed_reads >= 20:
                self.error = "camera stopped delivering frames"
                self._running = False
            time.sleep(0.1)
            return
        self._failed_reads = 0
        result = self.system.process_frame(frame, time.monotonic())
        with self._lock:
            self._latest, self._stamp = result, time.monotonic()

    def start(self):
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()
        return self

    def _loop(self):
        while self._running:
            try:
                self.step()
            except Exception as exc:               # never let a vision crash kill the robot loop
                self.error = repr(exc)
                self._running = False

    def stop(self):
        self._running = False
        release = getattr(self.capture, "release", None)
        if release:
            release()

    def latest(self):
        """Newest vision result, or None if it is older than max_age (or vision died)."""
        with self._lock:
            if self._latest is not None and time.monotonic() - self._stamp < self.max_age:
                return self._latest
        return None

    @classmethod
    def from_config(cls, config):
        """Build the real thing: his VisionSystem + a cv2 camera (imports torch/opencv here only)."""
        import cv2
        from vision import VisionSystem            # his vision/vision.py (folder must be on sys.path)
        system = VisionSystem(config)
        cap = cv2.VideoCapture(config.camera_index)
        if not cap.isOpened():
            raise RuntimeError(f"Could not open camera index {config.camera_index}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.camera_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.camera_height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cls(system, cap)


def make_target_fn(bridge, link, hfov_deg=62.0, start_dist=1.5):
    """Returns get_target() -> (x, y) of the person in world coordinates, or None.

    Bearing : from horizontal_error and the camera's horizontal field of view.
    Distance: distance_m if his code is calibrated (focal_length_px + assumed_person_height_m
              set in Config). Otherwise start_dist * relative_distance_ratio, which is only
              right if the person was start_dist metres away when tracking began.
    """
    tan_half = math.tan(math.radians(hfov_deg) / 2.0)

    def get_target():
        r, d = bridge.latest(), link.get_data()
        if r is None or d is None or not r.get("target_detected"):
            return None
        dist = r.get("distance_m")
        if dist is None:
            ratio = r.get("relative_distance_ratio")
            if ratio is None:
                return None
            dist = start_dist * ratio
        bearing = math.atan(r["horizontal_error"] * tan_half)     # + = person is to the right
        a = d["theta"] - bearing                                  # robot angles are counter-clockwise
        return (d["x"] + dist * math.cos(a), d["y"] + dist * math.sin(a))

    return get_target


def make_stop_fn(bridge):
    """True when his gait/looming logic says the robot should stop (person standing, too close...)."""
    def should_stop():
        r = bridge.latest()
        return bool(r and (r.get("car_stop_required") or r.get("emergency_brake")))
    return should_stop
