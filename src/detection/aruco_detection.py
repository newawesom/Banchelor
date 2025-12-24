'''
@file aruco_detection.py
@author NIUHAO
@brief 接收一帧图像，从中找出可能的ArUco码，并绘制出轮廓图和三维坐标轴，输出ArUco相对于相机的位置向量和姿态矩阵
@input Image_file/MatLike/UMat
@output r_vec[] & t_vec[] -> ndarray
'''
import cv2
import numpy as np
import json
from pathlib import Path


class Aruco_Detection():
    def __init__(self, dictionary: int) -> None:
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary)
        self.detector_parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.detector_parameters)
        self.input_image = cv2.typing.MatLike
        self.marker_corners = [cv2.typing.MatLike]
        self.marker_ids = cv2.typing.MatLike
        self.camera_matrix = np.zeros((3, 3), dtype=float)
        self.camera_distortion = np.zeros((1, 5), dtype=float)
        
    def detect_marker(self, input_image: cv2.Mat | cv2.UMat | np.ndarray) -> bool:
        '''
        检测图中是否存在ArUco码
        
        :param input_image: 需要检测的图片
        :type input_image: cv2.UMat | cv2.Mat | np.ndarray
        :return: 是否找到ArUco码
        :rtype: bool
        '''
        self.input_image = input_image
        self.marker_corners, self.marker_ids, _ = self.detector.detectMarkers(self.input_image, None, None, None)
        return self.marker_corners != () and self.marker_ids is not None
    
    def draw_marker(self) -> None:
        '''
        绘出图中存在的ArUco码
        
        :param self: 说明
        '''
        image = cv2.aruco.drawDetectedMarkers(self.input_image, self.marker_corners, self.marker_ids)
        cv2.imshow("ArUco", image)
        cv2.waitKey(1)

    def load_arguments(self, fname: str) -> bool:
        '''
        加载相机参数
        
        :param fname: 存储相机参数的json文件
        :type fname: str
        :return: 是否正确加载参数
        :rtype: bool
        '''
        with open(fname, 'r') as f:
            json_data = json.load(f)
            data = {}
            for key, val in json_data.items():
                arr = np.array(val)
                data[key] = arr
        self.camera_matrix = data.get("camera_matrix")
        self.camera_distortion = data.get("distortion")
        if self.camera_matrix is not None and self.camera_distortion is not None:
            print(f"Load arguments from file {fname}.")
            return True
        else:
            print(f"[ERROR] Can not load arguments!")
            return False