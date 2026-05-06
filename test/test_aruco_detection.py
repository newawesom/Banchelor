import os, sys, cv2
import numpy as np
from pathlib import Path
import VisionCaptureApi
import UE4CtrlAPI
import time


# Ensure project's `src` directory is on sys.path so `calibration` package is importable
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, ".."))
src_path = os.path.join(project_root, "src")
if src_path not in sys.path:
    sys.path.insert(0, src_path)

from detection import Aruco_Detection
import utils


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

def test_pack()->None:
    detect = Aruco_Detection(cv2.aruco.DICT_7X7_1000, 1.0)
    detect.marker_ids = np.array([[0]])
    euler = [11.0, 12.0, 13.0]
    detect.rot_mats.append(utils.euler_to_rotmat(euler, degree=True))
    detect.t_vecs.append(np.array([1, 1, 1]))
    detect.reproject_errors.append(0.1)
    print(detect.pack())

def test_in_online_env()->None:
    detect_down = Aruco_Detection(cv2.aruco.DICT_4X4_50, 1.0)
    detect_front = Aruco_Detection(cv2.aruco.DICT_4X4_50, 1.0)
    detect_down.load_arguments(str(Path(CONFIG_PATH, "camera.json")))
    detect_front.load_arguments(str(Path(CONFIG_PATH, "camera.json")))
    vis = VisionCaptureApi.VisionCaptureApi()
    ue = UE4CtrlAPI.UE4CtrlAPI()
    ue.sendUE4Cmd('r.setres 1280x720w',0) # 设置UE4窗口分辨率，注意本窗口仅限于显示，取图分辨率在json中配置，本窗口设置越小，资源需求越少。
    ue.sendUE4Cmd('t.MaxFPS 30',0) # 设置UE4最大刷新频率，同时也是取图频率
    vis.jsonLoad(jsonPath=str(Path(CONFIG_PATH, "Config.json")))
    is_suss = vis.sendReqToUE4()
    if not is_suss:
        print('[ERROR]Can not send request to UE4, please execute RflySim3D first.')
        sys.exit(1)
    vis.startImgCap()
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
        image_front = vis.Img[1]
        detect_down.detect_marker(image_down)
        detect_down.estimate_pose()
        drown_marker_down = detect_down.draw_marker_axis()
        print("Down:\n", detect_down.marker_corners)
        cv2.imshow("Drown_Marker_Down", drown_marker_down)
        cv2.waitKey(1)
        detect_front.detect_marker(image_front)
        detect_front.estimate_pose()
        drown_marker_front = detect_front.draw_marker_axis()
        print("Front:\n", detect_front.marker_corners)
        cv2.imshow("Drown_Maker_Front", drown_marker_front)
        cv2.waitKey(1)


if __name__ == "__main__":
    test_in_online_env()