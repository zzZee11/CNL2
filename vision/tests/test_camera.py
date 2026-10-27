"""
Unit tests for vision.camera (CameraReader & Mock Camera).
"""

import time
import unittest
from vision.config import VisionConfig
from vision.camera import CameraReader


class TestCamera(unittest.TestCase):
    def test_mock_camera_stream(self):
        config = VisionConfig(
            camera_backend="mock",
            camera_width=320,
            camera_height=240,
            camera_fps=20,
        )
        cam = CameraReader(config)
        started = cam.start()
        self.assertTrue(started)
        self.assertTrue(cam.is_running())

        # Give it a moment to capture a frame
        time.sleep(0.15)
        ret, frame, ts = cam.read()
        self.assertTrue(ret)
        self.assertIsNotNone(frame)
        self.assertGreater(ts, 0.0)

        # Check frame count increases
        initial_count = cam.get_frame_count()
        time.sleep(0.1)
        self.assertGreaterEqual(cam.get_frame_count(), initial_count)

        cam.stop()
        self.assertFalse(cam.is_running())

    def test_hardware_unavailable_fallback(self):
        # Attempt to open a non-existent camera index (e.g. 99)
        config = VisionConfig(
            camera_index=99,
            camera_backend="opencv",
            camera_width=320,
            camera_height=240,
        )
        cam = CameraReader(config)
        # Should gracefully fall back to mock rather than crashing
        started = cam.start()
        self.assertTrue(started)
        time.sleep(0.1)
        ret, frame, _ = cam.read()
        self.assertTrue(ret)
        self.assertIsNotNone(frame)
        cam.stop()


if __name__ == "__main__":
    unittest.main()

