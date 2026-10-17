from dataclasses import dataclass
import time
from typing import Optional


@dataclass
class TelemetryData:
    rpm_left: float = 0.0
    rpm_right: float = 0.0
    ticks_left: int = 0
    ticks_right: int = 0
    state: str = "DISCONNECTED"
    flags: str = "INIT"
    timestamp: float = 0.0
    raw_packet: str = ""

    @property
    def is_watchdog_tripped(self) -> bool:
        return "WDG" in self.flags

    @property
    def is_healthy(self) -> bool:
        return "OK" in self.flags


def parse_telemetry_line(line: str) -> Optional[TelemetryData]:
    """
    Parse a raw serial line into TelemetryData.
    Expected schema: TELE:<rpmL>,<rpmR>,<ticksL>,<ticksR>,<state>,<flags>
    Example: TELE:45.2,44.8,1240,1238,FWD,OK
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

        return TelemetryData(
            rpm_left=rpm_l,
            rpm_right=rpm_r,
            ticks_left=ticks_l,
            ticks_right=ticks_r,
            state=state,
            flags=flags,
            timestamp=time.time(),
            raw_packet=line,
        )
    except (ValueError, IndexError):
        return None

