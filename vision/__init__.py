"""
Lightweight Human-Detection and Tracking System for Raspberry Pi 4.
Provides decoupled, real-time target observation for single-person following.
"""

from .config import VisionConfig
from .target_data import TargetObservation
from .detector import HumanDetector, Detection, create_detector
from .tracker import SinglePersonTracker
from .target_estimator import TargetEstimator
from .vision_service import VisionService

__all__ = [
    "VisionConfig",
    "TargetObservation",
    "HumanDetector",
    "Detection",
    "create_detector",
    "SinglePersonTracker",
    "TargetEstimator",
    "VisionService",
]

