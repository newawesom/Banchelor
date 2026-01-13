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
    def __init__(self, dictionary: int, maker_length: float = 0.06) -> None:
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary)
        self.detector_parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.detector_parameters)
        self.input_image = cv2.typing.MatLike
        self.marker_corners = [cv2.typing.MatLike]
        self.marker_ids = cv2.typing.MatLike
        self.camera_matrix = np.zeros((3, 3), dtype=np.float32)
        self.camera_distortion = np.zeros((1, 5), dtype=np.float32)
        self.marker_length = maker_length
        self.r_vecs = []
        self.t_vecs = []
        
    def detect_marker(self, input_image: cv2.Mat | cv2.UMat | np.ndarray) -> bool:
        '''
        加载图片，并检测图中是否存在ArUco码
        
        :param input_image: 需要检测的图片
        :type input_image: cv2.UMat | cv2.Mat | np.ndarray
        :return: 是否找到ArUco码
        :rtype: bool
        '''
        self.input_image = input_image
        self.marker_corners, self.marker_ids, _ = self.detector.detectMarkers(self.input_image, None, None, None)
        return self.marker_corners != () and self.marker_ids is not None
    
    def draw_marker(self) -> cv2.typing.MatLike:
        '''
        绘出图中存在的ArUco码
        
        :param self: 说明
        :return: 如果找到ArUco码则返回绘出边缘和编号，否则返回原图像
        :rtype: cv2.MatLike
        '''
        if self.marker_corners != () and self.marker_ids is not None:
            image = cv2.aruco.drawDetectedMarkers(self.input_image, self.marker_corners, self.marker_ids)
        else:
            image = self.input_image
        return image
        # cv2.waitKey(0)

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
        
    def estimate_pose(self) -> tuple:
        '''
        调用solvePnP方法估计ArUco码的位置和姿态
        
        :return: 返回r_vecs 和 t_vecs 的元组
        :rtype: tuple[Any, ...]
        '''
        # 定义世界坐标，ArUco码的四个角，从左上角开始，顺时针方向定义
        object_points = [[-self.marker_length / 2.0, self.marker_length / 2.0, 0],
                         [self.marker_length / 2.0, self.marker_length / 2.0, 0],
                         [self.marker_length / 2.0, -self.marker_length / 2.0, 0],
                         [-self.marker_length / 2.0, -self.marker_length / 2.0, 0]]
        object_points = np.array(object_points)
        # 定义t_vecs\r_vecs
        # r_vecs = []
        # t_vecs = []
        r_vecs_LM = []
        t_vecs_LM = []
        num_markers = len(self.marker_corners)
        if(self.marker_corners != () and self.marker_ids is not None):
            for index in range(num_markers):
                # 调用 solvePnP 方法计算r_vec和t_vec
                _, r_vec, t_vec = cv2.solvePnP(object_points, self.marker_corners[index], self.camera_matrix, self.camera_distortion, None, None, False, cv2.SOLVEPNP_IPPE_SQUARE)
                # 按照marker_ids列表顺序列出的ID号依次调用solvePnP方法
                #r_vecs.append(r_vec)
                #t_vecs.append(t_vec)
                # 调用solvePnPRefineLM 方法优化
                r_vec_LM, t_vec_LM = cv2.solvePnPRefineLM(object_points, self.marker_corners[index], self.camera_matrix, self.camera_distortion, r_vec, t_vec)
                r_vecs_LM.append(r_vec_LM)
                t_vecs_LM.append(t_vec_LM)
        
        self.r_vecs = r_vecs_LM
        self.t_vecs = t_vecs_LM
        return self.marker_ids, r_vecs_LM, t_vecs_LM
    
    def draw_marker_axis(self) -> cv2.typing.MatLike:
        '''
        画出三条坐标轴
        
        :return: 返回绘出的图像
        :rtype: MatLike
        '''
        image = self.draw_marker()
        if self.marker_corners != () and self.marker_ids is not None:
            for index in range(len(self.marker_corners)):
                image = cv2.drawFrameAxes(image, self.camera_matrix, self.camera_distortion, self.r_vecs[index], self.t_vecs[index], self.marker_length * 1.5, 2)
        return image