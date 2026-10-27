"""
Unit tests for vision.detector (HumanDetector & Detection).
"""

import unittest
from vision.config import VisionConfig
from vision.detector import Detection, MockDetector, create_detector


class TestDetector(unittest.TestCase):
    def test_mock_detector_generates_person(self):
        config = VisionConfig(
            model_backend="mock",
            camera_width=640,
            camera_height=480,
            confidence_threshold=0.50,
        )
        detector = create_detector(config)
        self.assertIsInstance(detector, MockDetector)

        detections = detector.detect(frame=None)
        self.assertEqual(len(detections), 1)
        det = detections[0]

        self.assertEqual(det.class_name, "person")
        self.assertGreaterEqual(det.confidence, 0.50)
        self.assertEqual(len(det.bbox), 4)

        x1, y1, x2, y2 = det.bbox
        self.assertLess(x1, x2)
        self.assertLess(y1, y2)
        self.assertGreaterEqual(x1, 0.0)
        self.assertLessEqual(x2, 640.0)

    def test_forced_detection_injection(self):
        config = VisionConfig(model_backend="mock")
        detector = MockDetector(config)

        custom_det = Detection(
            bbox=(50.0, 60.0, 150.0, 260.0),
            confidence=0.95,
            class_id=0,
            class_name="person",
        )
        detector.set_forced_detection(custom_det)

        results = detector.detect(frame=None)
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].bbox, (50.0, 60.0, 150.0, 260.0))
        self.assertEqual(results[0].confidence, 0.95)


if __name__ == "__main__":
    unittest.main()

