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
    def __init__(self) -> None:
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_1000)
        self.detector_parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.detector_parameters)
        self.input_image = cv2.UMat()
        self.marker_corner = []
        self.marker_ids = []
        self.camera_matrix = np.zeros((3, 3), dtype=float)
        self.camera_distortion = np.zeros((1, 5), dtype=float)
        
    def detect_marker(self, input_image: cv2.UMat | cv2.Mat | np.ndarray) -> None:
        self.input_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)

    def load_arguments(self, fname: str) -> bool:
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