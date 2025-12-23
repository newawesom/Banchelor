import os, sys

# Ensure project's `src` directory is on sys.path so `calibration` package is importable
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, ".."))
src_path = os.path.join(project_root, "src")
if src_path not in sys.path:
    sys.path.insert(0, src_path)

from calibration import Camera_Calibration

PATH = os.path.join(current_dir, "run", "20251217_113613")
def test() -> None:
    calibration = Camera_Calibration()
    #calibration.setup_env()
    #path = calibration.fetch_board_images(50)

    calibration.sample_and_calibrate()

if __name__ == "__main__":
    test()