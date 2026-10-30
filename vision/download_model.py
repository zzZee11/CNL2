#!/usr/bin/env python3
"""
vision/download_model.py
------------------------
Utility to verify or download the pretrained SSD-MobileNet TFLite human detection
model (detect.tflite) and COCO labelmap (labelmap.txt) into vision/models/.
"""

import os
import sys
import urllib.request
import zipfile
import shutil

MODEL_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "models"))
MODEL_PATH = os.path.join(MODEL_DIR, "detect.tflite")
LABEL_PATH = os.path.join(MODEL_DIR, "labelmap.txt")

# Official Google Coral SSD MobileNet v2 COCO model URL
CORAL_MODEL_URL = "https://raw.githubusercontent.com/google-coral/test_data/master/ssd_mobilenet_v2_coco_quant_postprocess.tflite"
CORAL_LABEL_URL = "https://raw.githubusercontent.com/google-coral/test_data/master/coco_labels.txt"


def ensure_models() -> bool:
    os.makedirs(MODEL_DIR, exist_ok=True)

    # 1. Check if model already exists and is healthy (> 1MB)
    if os.path.isfile(MODEL_PATH) and os.path.getsize(MODEL_PATH) > 1_000_000:
        print(f"[Model] detect.tflite already present: {MODEL_PATH} ({os.path.getsize(MODEL_PATH) / 1024 / 1024:.2f} MB)")
    else:
        print(f"[Model] Downloading detect.tflite to {MODEL_PATH}...")
        try:
            urllib.request.urlretrieve(CORAL_MODEL_URL, MODEL_PATH)
            print(f"[Model] Successfully downloaded detect.tflite ({os.path.getsize(MODEL_PATH) / 1024 / 1024:.2f} MB)")
        except Exception as e:
            print(f"[Model] Download error: {e}")
            return False

    # 2. Check labelmap
    if not os.path.isfile(LABEL_PATH) or os.path.getsize(LABEL_PATH) < 100:
        print(f"[Model] Downloading labelmap.txt...")
        try:
            urllib.request.urlretrieve(CORAL_LABEL_URL, LABEL_PATH)
            print("[Model] labelmap.txt saved.")
        except Exception as e:
            print(f"[Model] Warning: Could not download labelmap: {e}")

    return True


if __name__ == "__main__":
    success = ensure_models()
    sys.exit(0 if success else 1)

