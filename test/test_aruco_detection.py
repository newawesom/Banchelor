import os, sys, cv2
from pathlib import Path


# Ensure project's `src` directory is on sys.path so `calibration` package is importable
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, ".."))
src_path = os.path.join(project_root, "src")
if src_path not in sys.path:
    sys.path.insert(0, src_path)

from detection import Aruco_Detection


PATH = Path.cwd()
PATH = Path(PATH, "run")
def test_load_argument() -> None:
    detect = Aruco_Detection(cv2.aruco.DICT_6X6_250)
    assert(detect.load_arguments("camera.json"))
    print(detect.camera_matrix, detect.camera_distortion)

def test_aruco_detect() -> None:
    detect = Aruco_Detection(cv2.aruco.DICT_6X6_250)
    image_path1 = Path(PATH, "singlemarkersoriginal.jpg")
    image_path2 = Path(PATH, "fake_image.png")
    img1 = cv2.imread(str(image_path1))
    img2 = cv2.imread(str(image_path2))
    assert(detect.detect_marker(img1))
    assert(not detect.detect_marker(img2))

def test_draw_marker() -> None:
    detect = Aruco_Detection(cv2.aruco.DICT_6X6_250)
    image_path_1 = Path(PATH, "singlemarkersoriginal.jpg")
    img_1 = cv2.imread(str(image_path_1))
    assert(detect.detect_marker(img_1))
    img1 = detect.draw_marker()
    # cv2.waitKey(0)
    image_path_2 = Path(PATH, "fake_image.png")
    img_2 = cv2.imread(str(image_path_2))
    assert(not detect.detect_marker(img_2))
    img2 = detect.draw_marker()
    cv2.imshow("ArUco1", img1)
    cv2.imshow("ArUco2", img2)
    cv2.waitKey(0)


if __name__ == "__main__":
    test_draw_marker()