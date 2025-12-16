import camera_calibration as calib
import os


current_dir = os.path.dirname(os.path.abspath(__file__))
PATH = os.path.join(current_dir, "run", "20251216_112402")
def test() -> None:
    calibration = calib.Camera_Calibration()
    #calibration.setup_env()
    #calibration.fetch_board_images(10)
    
    calibration.find_chessboard_corners(PATH)

if __name__ == "__main__":
    test()