#!/usr/bin/env python3
"""
Download YOLOv11 model weights for the VLA vision node.

Usage:
  python3 scripts/download_yolo_model.py
"""

import os
import urllib.request
import sys

MODEL_URL = "https://github.com/ultralytics/assets/releases/download/v8.1.0/yolo11n.pt"
MODEL_PATH = "yolo11n.pt"

def download_model():
    """Download YOLOv11 nano model if it doesn't exist."""
    if os.path.exists(MODEL_PATH):
        print(f"✓ Model already exists: {MODEL_PATH}")
        return True

    print(f"Downloading YOLOv11 nano model to {MODEL_PATH}...")
    try:
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
        print("✓ Download complete!")
        return True
    except Exception as e:
        print(f"✗ Download failed: {e}")
        print("Please download manually from:")
        print(f"  {MODEL_URL}")
        return False

if __name__ == "__main__":
    success = download_model()
    sys.exit(0 if success else 1)