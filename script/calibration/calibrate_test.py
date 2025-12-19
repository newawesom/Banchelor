import camera_calibration as calib
import os, sys


current_dir = os.path.dirname(os.path.abspath(__file__))
PATH = os.path.join(current_dir, "run", "20251217_113613")
def test() -> None:
    calibration = calib.Camera_Calibration()
    #calibration.setup_env()
    #path = calibration.fetch_board_images(50)

    calibration.sample_and_calibrate()

if __name__ == "__main__":
    test()