import os, sys
import pytest


# Ensure project's `src` directory is on sys.path so `calibration` package is importable
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, ".."))
src_path = os.path.join(project_root, "src")
if src_path not in sys.path:
    sys.path.insert(0, src_path)

from detection import Aruco_Detection


def test_load_argument() -> None:
    detect = Aruco_Detection()
    assert(detect.load_arguments("camera.json"))
    print(detect.camera_matrix, detect.camera_distortion)


if __name__ == "__main__":
    test_load_argument()