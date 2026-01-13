import UE4CtrlAPI as UE4CtrlAPI
import ReqCopterSim
import VisionCaptureApi

import numpy as np
import cv2
import time, sys, json
from pathlib import Path


from detection import Aruco_Detection

PATH = Path.cwd()
CONFIG_PATH = Path(PATH, "config")


class Pose_Estimation():
    def __init__(self) -> None:
        self.aruco_detection_down = Aruco_Detection(cv2.aruco.DICT_7X7_1000, maker_length= 1.0)
        self.aruco_detection_down.load_arguments(str(Path(CONFIG_PATH, "camera.json")))
        self.aruco_detection_front = Aruco_Detection(cv2.aruco.DICT_7X7_1000, maker_length= 1.0)
        self.aruco_detection_front.load_arguments(str(Path(CONFIG_PATH, "camera.json")))
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
        # 当相机有信号时
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
            # TODO:将以下改为并行计算
            # 取图
            image_down = self.vis.Img[0]
            image_front = self.vis.Img[1]
            # 识别
            self.aruco_detection_down.detect_marker(image_down)
            self.aruco_detection_front.detect_marker(image_front)
            # 计算位姿
            self.aruco_detection_down.estimate_pose()
            self.aruco_detection_front.estimate_pose()
            # 输出标记
            
            # 绘制标记
            image_down = self.aruco_detection_down.draw_marker_axis()
            image_front = self.aruco_detection_front.draw_marker_axis()
            # 显示
            cv2.imshow("Camera-Down", image_down)
            cv2.waitKey(1)
            cv2.imshow("Camera-Front", image_front)
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
                        self.camera_front_config["t_vec"] = camera["SensorPoseXYZ"]
                        self.camera_front_config["Attitude"] = camera["SensorAngEular"]
        except:
            print("Can not load json file!")
        
    def euler_to_rotmat(self, euler: list[float]) -> None:
        pass

    def transform(self):
        # 根据从Config.json中读取传感器安装数据，接收marker相对于相机的r_vec和t_vec，输出marker相对于无人机质心的r_vec和t_vec
        pass
