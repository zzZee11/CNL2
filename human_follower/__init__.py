"""
human_follower
--------------
Standalone Stage 1 Human Follower: Stationary Turret Tracking.
Maintains visual lock on a single human by pivoting on the spot (zero forward movement).
"""

from .config import FollowerConfig
from .turret_follower import TurretFollower

__all__ = ["FollowerConfig", "TurretFollower"]

