import camera_calibration as calib


def test() -> None:
    calibration = calib.Camera_Calibration()
    calibration.setup_env()

if __name__ == "__main__":
    test()