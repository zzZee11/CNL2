from dataclasses import dataclass
import time
from typing import Optional, Tuple


@dataclass
class TelemetryData:
    rpm_left: float = 0.0
    rpm_right: float = 0.0
    ticks_left: int = 0
    ticks_right: int = 0
    state: str = "DISCONNECTED"
    flags: str = "INIT"
    dist_left: int = -1    # Millimeters (-1 indicates sensor not connected or out of range)
    dist_center: int = -1  # Millimeters
    dist_right: int = -1   # Millimeters
    timestamp: float = 0.0
    raw_packet: str = ""

    @property
    def is_watchdog_tripped(self) -> bool:
        return "WDG" in self.flags

    @property
    def is_healthy(self) -> bool:
        return "OK" in self.flags

    @property
    def has_distance_sensors(self) -> bool:
        return self.dist_left >= 0 or self.dist_center >= 0 or self.dist_right >= 0

    @property
    def tof_meters(self) -> Tuple[float, float, float]:
        """Convert distance sensor mm readings to meters for APF obstacle avoidance."""
        l_m = (self.dist_left / 1000.0) if self.dist_left > 0 else 2.0
        c_m = (self.dist_center / 1000.0) if self.dist_center > 0 else 2.0
        r_m = (self.dist_right / 1000.0) if self.dist_right > 0 else 2.0
        return (l_m, c_m, r_m)


def parse_telemetry_line(line: str) -> Optional[TelemetryData]:
    """
    Parse a raw serial line into TelemetryData.
    Expected schema:
      Standard (6 tokens): TELE:<rpmL>,<rpmR>,<ticksL>,<ticksR>,<state>,<flags>
      With ToF (9 tokens): TELE:<rpmL>,<rpmR>,<ticksL>,<ticksR>,<state>,<flags>,<distL>,<distC>,<distR>
    Example: TELE:45.2,44.8,1240,1238,FWD,OK,1200,450,1180
    """
    line = line.strip()
    if not line.startswith("TELE:"):
        return None

    payload = line[5:]
    tokens = [t.strip() for t in payload.split(",")]
    if len(tokens) < 6:
        return None

    try:
        rpm_l = float(tokens[0])
        rpm_r = float(tokens[1])
        ticks_l = int(tokens[2])
        ticks_r = int(tokens[3])
        state = tokens[4]
        flags = tokens[5]

        dist_l = -1
        dist_c = -1
        dist_r = -1
        if len(tokens) >= 9:
            try:
                dist_l = int(float(tokens[6]))
                dist_c = int(float(tokens[7]))
                dist_r = int(float(tokens[8]))
            except ValueError:
                dist_l = dist_c = dist_r = -1

        return TelemetryData(
            rpm_left=rpm_l,
            rpm_right=rpm_r,
            ticks_left=ticks_l,
            ticks_right=ticks_r,
            state=state,
            flags=flags,
            dist_left=dist_l,
            dist_center=dist_c,
            dist_right=dist_r,
            timestamp=time.time(),
            raw_packet=line,
        )
    except (ValueError, IndexError):
        return None

