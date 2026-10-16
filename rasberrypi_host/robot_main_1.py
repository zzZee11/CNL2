"""
robot_main.py - Raspberry Pi side: one serial link shared by every mode.

Layout (all can live in this file at first, split into modules later):
    SerialLink       owns the UART. Only this class touches the port.
    GridNavigator    teammate's A* + APF, refactored into step()
    HumanFollower    APF toward a point in front of the person
    RobotController  the state machine. Calls ONE mode per tick,
                     then sends exactly one command to the ESP32.

Needs a_search_main.py (teammate's file) in the same folder.
"""
import math
import queue
import struct
import threading
import time

from a_search_main import (
    create_empty_grid, astar_grid, is_path_blocked, mark_obstacle,
    world_to_grid, calculate_apf, calculate_kinematics, SENSOR_ANGLES,
)

# ---------------------------------------------------------------
# Packet format - AGREE THIS WITH THE ESP32 PERSON, then freeze it.
#   [START][CMD][LEN][PAYLOAD...][XOR][END]
#   XOR = xor of CMD, LEN and every payload byte. Little-endian.
# ---------------------------------------------------------------
START, END = 0xAA, 0x55
CMD_VELOCITY = 0x01      # Pi -> ESP32 : int16 left, int16 right (mm/s)
CMD_SENSORS = 0x02       # ESP32 -> Pi : float x, y, theta, tof_l, tof_c, tof_r
SENSOR_FMT = "<6f"
STALE_AFTER = 0.5        # s without a valid sensor packet => ERROR
TICK = 0.1               # 10 Hz control loop (also the heartbeat: we send every tick)
MAX_PAYLOAD = 32         # longest payload we ever expect; anything bigger is a false header
MAX_WHEEL_SPEED = 1.0    # m/s - set this to what the real motors can actually do


def limit_speeds(vl, vr, vmax=MAX_WHEEL_SPEED):
    """Scale both wheels down together so the turn ratio is kept but neither exceeds vmax."""
    m = max(abs(vl), abs(vr))
    if m > vmax:
        vl, vr = vl * vmax / m, vr * vmax / m
    return vl, vr


class SerialLink:
    """Background thread reads packets; main loop reads the latest one and sends."""

    def __init__(self, port=None, baud=115200):
        self._lock = threading.Lock()
        self._latest = None
        self._stamp = 0.0
        self._buf = bytearray()
        self._ser = None
        if port:                                   # port=None -> mock mode, no hardware
            import serial                          # pip install pyserial
            self._ser = serial.Serial(port, baud, timeout=0.05)
            threading.Thread(target=self._rx_loop, daemon=True).start()

    # ---- receive side ----
    def _rx_loop(self):
        while True:
            self._buf += self._ser.read(64)
            self._parse()

    @staticmethod
    def _xor(data):
        x = 0
        for b in data:
            x ^= b
        return x

    def _parse(self):
        while True:
            i = self._buf.find(bytes([START]))
            if i < 0:
                self._buf.clear()
                return
            del self._buf[:i]
            if len(self._buf) < 3:
                return
            n = self._buf[2]
            if n > MAX_PAYLOAD:                    # not a real header: skip this byte, resync
                del self._buf[:1]
                continue
            total = 3 + n + 2
            if len(self._buf) < total:
                return
            frame = bytes(self._buf[:total])
            if frame[-1] == END and self._xor(frame[1:3 + n]) == frame[3 + n]:
                self._handle(frame[1], frame[3:3 + n])
                del self._buf[:total]
            else:
                del self._buf[:1]                  # bad frame: resync

    def _handle(self, cmd, payload):
        if cmd == CMD_SENSORS and len(payload) == struct.calcsize(SENSOR_FMT):
            x, y, th, l, c, r = struct.unpack(SENSOR_FMT, payload)
            self._store({"x": x, "y": y, "theta": th, "tof": (l, c, r)})

    def _store(self, data):
        with self._lock:
            self._latest, self._stamp = data, time.time()

    def get_data(self):
        """Latest sensor dict, or None if nothing fresh has arrived."""
        with self._lock:
            if self._latest and time.time() - self._stamp < STALE_AFTER:
                return self._latest
        return None

    # ---- transmit side ----
    def send_velocity(self, v_left, v_right):
        payload = struct.pack("<hh", int(v_left * 1000), int(v_right * 1000))
        body = bytes([CMD_VELOCITY, len(payload)]) + payload
        frame = bytes([START]) + body + bytes([self._xor(body), END])
        if self._ser:
            self._ser.write(frame)
        else:
            print(f"[mock tx] L={v_left:.2f} R={v_right:.2f}")


# ---------------------------------------------------------------
# Mode 1: warehouse navigation (teammate's logic, one iteration per call)
# ---------------------------------------------------------------
class GridNavigator:
    def _plan(self, pos):
        # If the robot sits inside an inflated zone, every neighbouring cell is "blocked"
        # and A* gives up (a false "trapped"). Plan on a copy with a small hole around
        # the robot; the real grid, and the ToF-based avoidance, are untouched.
        g = self.grid.copy()
        gx, gy = world_to_grid(*pos)
        r = 2
        for i in range(-r, r + 1):
            for j in range(-r, r + 1):
                nx, ny = gx + i, gy + j
                if 0 <= nx < g.shape[0] and 0 <= ny < g.shape[1] and math.hypot(i, j) <= r:
                    g[nx][ny] = 0
        return astar_grid(pos, self.goal, g)

    def start(self, start_pos, goal_pos):
        self.grid = create_empty_grid()
        self.goal = goal_pos
        self.waypoints = self._plan(start_pos)
        if not self.waypoints:
            return False
        self.waypoints.pop(0)
        return True

    def step(self, d):
        pos, theta, tof = (d["x"], d["y"]), d["theta"], d["tof"]

        map_updated = False
        for i in range(3):
            if tof[i] < 2.0:
                a = theta + SENSOR_ANGLES[i]
                ox, oy = pos[0] + tof[i] * math.cos(a), pos[1] + tof[i] * math.sin(a)
                gx, gy = world_to_grid(ox, oy)
                if self.grid[gx][gy] == 0:
                    mark_obstacle(self.grid, ox, oy)
                    map_updated = True

        if map_updated and is_path_blocked(self.waypoints, self.grid):
            self.waypoints = self._plan(pos)
            if not self.waypoints:
                return "BLOCKED", 0.0, 0.0
            self.waypoints.pop(0)

        while self.waypoints and math.dist(pos, self.waypoints[0]) < 0.15:
            self.waypoints.pop(0)
        if not self.waypoints:
            return "DONE", 0.0, 0.0

        f = calculate_apf(pos, self.waypoints[0], tof, theta)
        vl, vr = calculate_kinematics(f, theta)
        return "DRIVING", vl, vr


# ---------------------------------------------------------------
# Mode 2: human following (APF only, no A*)
# ---------------------------------------------------------------
FOLLOW_DIST = 1.0   # stay this far from the person (> SAFE_DIST so ToF doesn't fight it)


LOST_GRACE_TICKS = 15   # 1.5 s at 10 Hz: a dropped detection should not end follow mode


class HumanFollower:
    def __init__(self, get_target, get_stop=lambda: False):
        # get_target() -> (x, y) of the person in WORLD coordinates, or None if not seen.
        # get_stop()   -> True when the vision code says "stop" (person standing, too close...)
        self.get_target = get_target
        self.get_stop = get_stop
        self.lost_ticks = 0

    def step(self, d):
        t = self.get_target()
        if t is None:
            self.lost_ticks += 1
            if self.lost_ticks > LOST_GRACE_TICKS:
                return "LOST", 0.0, 0.0
            return "FOLLOWING", 0.0, 0.0           # wait in place for the person to reappear
        self.lost_ticks = 0
        if self.get_stop():
            return "FOLLOWING", 0.0, 0.0
        pos, theta, tof = (d["x"], d["y"]), d["theta"], d["tof"]
        dx, dy = t[0] - pos[0], t[1] - pos[1]
        dist = math.hypot(dx, dy)
        if dist <= FOLLOW_DIST:
            return "FOLLOWING", 0.0, 0.0           # close enough, hold position
        k = (dist - FOLLOW_DIST) / dist
        goal = (pos[0] + dx * k, pos[1] + dy * k)  # stop short of the person
        f = calculate_apf(pos, goal, tof, theta)
        vl, vr = calculate_kinematics(f, theta)
        return "FOLLOWING", vl, vr


# ---------------------------------------------------------------
# The state machine
# ---------------------------------------------------------------
class RobotController:
    def __init__(self, link, get_human_target, get_stop=lambda: False):
        self.link = link
        self.nav = GridNavigator()
        self.follower = HumanFollower(get_human_target, get_stop)
        self.cmds = queue.Queue()      # web server puts ("NAV", (x, y)), ("FOLLOW",), ("STOP",)
        self.state = "IDLE"

    def tick(self):
        d = self.link.get_data()

        try:
            cmd = self.cmds.get_nowait()
        except queue.Empty:
            cmd = None

        if cmd:
            if cmd[0] == "STOP":
                self.state = "IDLE"
            elif cmd[0] == "NAV" and d:
                ok = self.nav.start((d["x"], d["y"]), cmd[1])
                self.state = "GRID_NAV" if ok else "IDLE"
            elif cmd[0] == "FOLLOW" and d:
                self.follower.lost_ticks = 0
                self.state = "HUMAN_FOLLOW"

        # Safety: no fresh data from the ESP32 -> stop, whatever mode we were in
        if d is None:
            self.state = "ERROR"
        elif self.state == "ERROR":
            self.state = "IDLE"

        vl = vr = 0.0
        if self.state == "GRID_NAV":
            status, vl, vr = self.nav.step(d)
            if status != "DRIVING":
                self.state = "IDLE"
        elif self.state == "HUMAN_FOLLOW":
            status, vl, vr = self.follower.step(d)
            if status == "LOST":
                self.state = "IDLE"

        vl, vr = limit_speeds(vl, vr)
        self.link.send_velocity(vl, vr)        # every tick, doubles as the heartbeat


# ---- settings you must check on the real robot ----
SERIAL_PORT = "/dev/serial0"      # or /dev/ttyUSB0, /dev/ttyACM0 when using USB
CAMERA_HFOV_DEG = 62.0            # horizontal field of view of YOUR camera (see its datasheet)
FOCAL_LENGTH_PX = None            # calibrate: box_height_px * distance_m / PERSON_HEIGHT_M, once
PERSON_HEIGHT_M = 1.7             # used together with FOCAL_LENGTH_PX
START_DIST_M = 1.5                # only used when NOT calibrated: person's distance when tracking starts


def build_vision(link):
    """Returns (get_target, get_stop). Falls back to 'no person ever seen' if vision can't load."""
    import sys
    from dataclasses import replace
    from pathlib import Path
    vision_dir = Path(__file__).resolve().parent / "vision"
    sys.path.insert(0, str(vision_dir))
    try:
        from config import Config
        from vision_bridge import VisionBridge, make_target_fn, make_stop_fn
        cfg = replace(
            Config(),
            model_path=vision_dir / "models" / "yolo26n.pt",   # yolo26n-pose.pt is not in the repo
            debug_view=False,                                  # no window on the headless Pi
            inference_threads=2,                               # his main.py default; a Pi has few cores
            focal_length_px=FOCAL_LENGTH_PX,
            assumed_person_height_m=PERSON_HEIGHT_M if FOCAL_LENGTH_PX else None,
        )
        bridge = VisionBridge.from_config(cfg).start()
    except Exception as exc:                                   # missing torch, no camera, bad model path...
        print(f"[vision disabled] {exc!r}")
        return (lambda: None), (lambda: False)
    return make_target_fn(bridge, link, CAMERA_HFOV_DEG, START_DIST_M), make_stop_fn(bridge)


def main():
    link = SerialLink(SERIAL_PORT)             # SerialLink(None) = mock
    get_target, get_stop = build_vision(link)
    robot = RobotController(link, get_target, get_stop)
    while True:
        t0 = time.time()
        robot.tick()
        time.sleep(max(0.0, TICK - (time.time() - t0)))


if __name__ == "__main__":
    main()
