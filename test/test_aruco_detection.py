import os, sys, cv2
from pathlib import Path
import VisionCaptureApi
import time


# Ensure project's `src` directory is on sys.path so `calibration` package is importable
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, ".."))
src_path = os.path.join(project_root, "src")
if src_path not in sys.path:
    sys.path.insert(0, src_path)

from detection import Aruco_Detection


PATH = Path.cwd()
CONFIG_PATH = Path(PATH, "config")
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

def test_estimate_pose() -> None:
    detect = Aruco_Detection(cv2.aruco.DICT_6X6_250)
    image_path = Path(PATH, "singlemarkersoriginal.jpg")
    config_path = Path(CONFIG_PATH, "camera.json")
    img = cv2.imread(str(image_path))
    assert(detect.detect_marker(img))
    assert(detect.load_arguments(str(config_path)))
    detect.estimate_pose()

def test_draw_marker_axis() -> None:
    detect = Aruco_Detection(cv2.aruco.DICT_6X6_250)
    image_path = Path(PATH, "singlemarkersoriginal.jpg")
    config_path = Path(CONFIG_PATH, "camera.json")
    img = cv2.imread(str(image_path))
    assert(detect.detect_marker(img))
    assert(detect.load_arguments(str(config_path)))
    detect.estimate_pose()
    image = detect.draw_marker_axis()
    cv2.imshow("ArUco", image)
    cv2.waitKey(0)

def test_calculate_reprojection_error()->None:
    detect = Aruco_Detection(cv2.aruco.DICT_6X6_250)
    image_path = Path(PATH, "singlemarkersoriginal.jpg")
    config_path = Path(CONFIG_PATH, "camera.json")
    img = cv2.imread(str(image_path))
    assert(detect.detect_marker(img))
    assert(detect.load_arguments(str(config_path)))
    print(detect.estimate_pose())
    print(detect.calculate_reprojection_error())

def test_in_online_env()->None:
    detect = Aruco_Detection(cv2.aruco.DICT_7X7_1000, 1.0)
    detect.load_arguments(str(Path(CONFIG_PATH, "camera.json")))
    vis = VisionCaptureApi.VisionCaptureApi()
    vis.jsonLoad(-1, str(Path(CONFIG_PATH, "Config.json")))
    is_suss = vis.sendReqToUE4()
    if not is_suss:
        print('[ERROR]Can not send request to UE4, please execute RflySim3D first.')
        sys.exit(1)
    vis.startImgCap(True)
    time.sleep(1)

    last_time = time.time()
    time_interval = 1.0 / 30.0 # 触发的最小时间间隔，1s/30fps
    while(vis.hasData[0]):
        # [last_time]=======[now] less-> wait until
        # [last_time] + time_interval|
        # [last_time]==================[now] more-> trigger immediately and set [last_time] to now
        last_time = last_time + time_interval
        sleep_time = last_time - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            last_time = time.time()
        # 取图
        image_down = vis.Img[0]
        detect.detect_marker(image_down)
        drown_marker_down = detect.draw_marker()
        cv2.imshow("Drown_Marker_Down", drown_marker_down)
        cv2.waitKey(1)
        image_front = vis.Img[1]
        detect.detect_marker(image_front)
        drown_marker_front = detect.draw_marker()
        cv2.imshow("Drown_Maker_Front", drown_marker_front)
        cv2.waitKey(1)


if __name__ == "__main__":
    test_in_online_env()