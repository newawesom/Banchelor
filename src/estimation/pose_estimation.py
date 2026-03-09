import UE4CtrlAPI as UE4CtrlAPI
import ReqCopterSim
import VisionCaptureApi

import numpy as np
import cv2
import time, sys, json
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor


from detection import Aruco_Detection
from transformation import *
from utils import *

PATH = Path.cwd()
CONFIG_PATH = Path(PATH, "config")


class Pose_Estimation():
    def __init__(self) -> None:
        self.aruco_detection_down = Aruco_Detection(cv2.aruco.DICT_7X7_1000, maker_length= 1.0)
        self.aruco_detection_down.load_arguments(str(Path(CONFIG_PATH, "camera.json")))
        self.aruco_detection_front = Aruco_Detection(cv2.aruco.DICT_7X7_1000, maker_length= 1.0)
        self.aruco_detection_front.load_arguments(str(Path(CONFIG_PATH, "camera.json")))
        self.coordinate_transformation = Coordinate_Transformation()
        self.coordinate_transformation.parse_config(str(Path(CONFIG_PATH, "Config.json")), str(Path(CONFIG_PATH, "install_markers.json")))
        self.camera_down_config = {}
        self.camera_front_config = {}

        self.vis = VisionCaptureApi.VisionCaptureApi()
        self.vis.jsonLoad(-1, str(Path(CONFIG_PATH, "Config.json")))
        is_suss = self.vis.sendReqToUE4()
        if not is_suss:
            print('[ERROR]Can not send request to UE4, please execute RflySim3D first.')
            sys.exit(1)
        self.vis.startImgCap(True)
        time.sleep(1)

    def visualize(self) -> None:
        '''
        可视化ArUco码的检测

        注：此函数只用作调试
        
        :param self: 说明
        '''
        # 设置定时触发器
        last_time = time.time()
        time_interval = 1.0 / 30.0 # 触发的最小时间间隔，1s/30fps

        # 设置子线程池函数
        def process_frame(detector, image):
            detector.detect_marker(image)
            (marker_ids, r_vecs, t_vecs) = detector.estimate_pose()
            return detector.draw_marker_axis(), marker_ids, r_vecs, t_vecs

        # 当相机有信号时
        with ThreadPoolExecutor(max_workers=2) as executor:
            while(self.vis.hasData[0] and self.vis.hasData[1]):
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
                image_down = self.vis.Img[0]
                image_front = self.vis.Img[1]

                down_future = executor.submit(process_frame, self.aruco_detection_down, image_down)
                front_future = executor.submit(process_frame, self.aruco_detection_front, image_front)

                image_down, maker_ids_down, r_vecs_down, t_vecs_down = down_future.result()
                image_front, maker_ids_front, r_vecs_front, t_vecs_front = front_future.result()
                # 显示
                cv2.imshow("Camera-Down", image_down)
                print(maker_ids_down, r_vecs_down, t_vecs_down)
                cv2.waitKey(1)
                cv2.imshow("Camera-Front", image_front)
                print(maker_ids_front, r_vecs_front, t_vecs_front)
                cv2.waitKey(1)


    def load_camera_config(self) -> None:
        '''
        加载相机相对于无人机质心的位置和姿态
        处理位置和姿态角为旋转矩阵和转移向量
        
        :param self: 说明
        '''
        try:
            with open(str(Path(CONFIG_PATH, "Config.json"))) as f:
                data_raw = json.load(f)
                cameras_config = data_raw["VisionSensors"]
                for camera in cameras_config:
                    if camera["SeqID"] == 0:
                        self.camera_down_config["Pose"] = camera["SensorPosXYZ"]
                        self.camera_down_config["t_vec"] = camera["SensorPosXYZ"]
                        self.camera_down_config["Attitude"] = camera["SensorAngEular"]
                    elif camera["SeqID"] == 1:
                        self.camera_front_config["Pose"] = camera["SensorPosXYZ"]
                        self.camera_front_config["t_vec"] = camera["SensorPosXYZ"]
                        self.camera_front_config["Attitude"] = camera["SensorAngEular"]
        except:
            print("Can not load json file!")
        
    def fusion_loop(self)->None:
        # 设置定时触发器
        last_time = time.time()
        time_interval = 1.0 / 30.0 # 触发的最小时间间隔，1s/30fps

        with ThreadPoolExecutor(max_workers=2) as executor:
            while(self.vis.hasData[0] and self.vis.hasData[1]):
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
                image_down = self.vis.Img[0]
                image_front = self.vis.Img[1]
                # 处理
                down_future = executor.submit(self.process_frame, self.aruco_detection_down, 0, image_down)
                front_future = executor.submit(self.process_frame, self.aruco_detection_front, 1, image_front)
                # 获取结果
                markers_down = down_future.result()
                markers_front = front_future.result()
                # 【调试】打印结果
                print(f"Markers from down camera:{markers_down}")
                print(f"Markers from front camera:{markers_front}")

    def process_frame(self, detector: Aruco_Detection, sensor_id: int, frame: np.ndarray)->list:
        '''
        [取图]->[处理图像]->[识别]->[获取字段表]->[变换]->[输出变换后的字段表]

        输入：detector Aruco_Detection()类 frame 一帧图像
        输出：字段列表
        '''
        # [取图]传入参数frame
        # [处理图像]
        # TODO:预处理，以加速线程
        # [识别]
        detector.detect_marker(frame)
        # PnP解算
        detector.estimate_pose()
        # 计算重映射误差
        detector.calculate_reprojection_error()
        # [获取字段表]
        markers = detector.pack()
        # [刚体变换]
        result = []
        for marker in markers:
            marker_res = self.coordinate_transformation.transform(marker, sensor_id)
            result.append(marker_res)
        # [输出变换后字段表列表]
        return result
    