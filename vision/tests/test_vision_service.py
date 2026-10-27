"""
Unit and integration tests for vision.vision_service (VisionService).
"""

import time
import unittest
from vision.config import VisionConfig
from vision.target_data import TargetObservation
from vision.vision_service import VisionService


class TestVisionService(unittest.TestCase):
    def test_pipeline_polling_and_telemetry(self):
        config = VisionConfig(
            camera_backend="mock",
            model_backend="mock",
            camera_width=320,
            camera_height=240,
            camera_fps=25,
            enable_preview=False,
            headless=True,
        )
        service = VisionService(config)
        self.assertTrue(service.start())
        self.assertTrue(service.is_running())

        # Allow pipeline to process several frames
        time.sleep(0.3)

        obs = service.get_latest_observation()
        self.assertIsInstance(obs, TargetObservation)
        self.assertTrue(obs.target_valid)
        self.assertGreater(obs.confidence, 0.5)
        self.assertIsNotNone(obs.bbox)

        telemetry = service.get_telemetry()
        self.assertIn("fps", telemetry)
        self.assertIn("inference_ms", telemetry)
        self.assertIn("total_frames", telemetry)
        self.assertGreater(telemetry["total_frames"], 0)

        service.stop()
        self.assertFalse(service.is_running())

    def test_callback_and_queue_dispatch(self):
        config = VisionConfig(
            camera_backend="mock",
            model_backend="mock",
            camera_width=320,
            camera_height=240,
            camera_fps=20,
            headless=True,
        )
        service = VisionService(config)

        callback_received = []

        def on_target_update(target_obs: TargetObservation):
            callback_received.append(target_obs)

        service.register_callback(on_target_update)
        target_q = service.get_target_queue(maxsize=5)

        self.assertTrue(service.start())
        time.sleep(0.25)

        # Verify callback was called
        self.assertGreater(len(callback_received), 0)
        self.assertTrue(callback_received[-1].target_valid)

        # Verify queue received observations
        self.assertFalse(target_q.empty())
        item = target_q.get_nowait()
        self.assertIsInstance(item, TargetObservation)

        service.stop()


if __name__ == "__main__":
    unittest.main()

