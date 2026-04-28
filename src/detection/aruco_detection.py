'''
@file aruco_detection.py
@author NIUHAO
@brief 接收一帧图像，从中找出可能的ArUco码，并绘制出轮廓图和三维坐标轴，输出相机相对于ArUco码的位置向量和姿态矩阵
@input Image_file/MatLike/UMat
@output r_vec[] & t_vec[] -> ndarray
'''
import cv2
import numpy as np
import json
from pathlib import Path
import utils


class Aruco_Detection():
    def __init__(self, dictionary: int, maker_length: float = 0.06) -> None:
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary)
        self.detector_parameters = cv2.aruco.DetectorParameters()
        self.detector_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector_parameters.cornerRefinementWinSize = 5
        self.detector_parameters.cornerRefinementMaxIterations = 30
        self.detector_parameters.cornerRefinementMinAccuracy = 0.04
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.detector_parameters)
        self.input_image = cv2.typing.MatLike
        self.input_image_raw = cv2.typing.MatLike
        self.object_points = np.ndarray
        self.marker_corners = [cv2.typing.MatLike]
        self.marker_ids = np.ndarray
        self.camera_matrix = np.zeros((3, 3), dtype=np.float32)
        self.camera_distortion = np.zeros((1, 5), dtype=np.float32)
        self.marker_length = maker_length
        self.r_vecs = []
        self.rot_mats = []
        self.t_vecs = []
        self.reproject_errors = []

    def _preprocess_image(self, input_image: cv2.Mat |cv2.UMat | np.ndarray):
        self.input_image_raw = input_image
        self.input_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)
        threshold = 65
        maxval = 255
        #_, dst = cv2.threshold(self.input_image, threshold, maxval, cv2.THRESH_BINARY)
        #_, dst = cv2.threshold(self.input_image, threshold, maxval, cv2.THRESH_OTSU)
        #self.input_image = dst
        return None
        #return dst
        
    def detect_marker(self, input_image: cv2.Mat | cv2.UMat | np.ndarray) -> bool:
        '''
        加载图片，并检测图中是否存在ArUco码
        
        :param input_image: 需要检测的图片
        :type input_image: cv2.UMat | cv2.Mat | np.ndarray
        :return: 是否找到ArUco码
        :rtype: bool
        '''
        self._preprocess_image(input_image)
        self.marker_corners, self.marker_ids, _ = self.detector.detectMarkers(self.input_image, None, None, None)
        return self.marker_corners != () and self.marker_ids is not None
    
    def draw_marker(self):
        '''
        绘出图中存在的ArUco码
        
        :param self: 说明
        :return: 如果找到ArUco码则返回绘出边缘和编号，否则返回原图像
        :rtype: cv2.MatLike
        '''
        if self.marker_corners != () and self.marker_ids is not None:
            image = cv2.aruco.drawDetectedMarkers(self.input_image_raw, self.marker_corners, self.marker_ids)
        else:
            image = self.input_image_raw
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
        object_points = np.array(object_points, dtype=np.float32)
        self.object_points = object_points
        # 定义t_vecs\r_vecs
        # r_vecs = []
        # t_vecs = []
        r_vecs_Refine = []
        t_vecs_Refine = []
        rot_mats = []
        if(self.marker_corners != () and self.marker_ids is not None):
            for index, _ in enumerate(self.marker_corners):
                # 调用 solvePnP 方法计算r_vec和t_vec
                img_pts = np.asarray(self.marker_corners[index], dtype=np.float32).reshape(-1, 2)
                _, r_vec, t_vec = cv2.solvePnP(object_points, img_pts, self.camera_matrix, self.camera_distortion, None, None, False, cv2.SOLVEPNP_IPPE_SQUARE)
                # 按照marker_ids列表顺序列出的ID号依次调用solvePnP方法
                #r_vecs.append(r_vec)
                #t_vecs.append(t_vec)
                # 调用solvePnPRefineLM 方法优化
                r_vec_Refine, t_vec_Refine = cv2.solvePnPRefineLM(object_points, img_pts, self.camera_matrix, self.camera_distortion, r_vec, t_vec)
                # 调用solvePnPRansac()方法优化
                r_vecs_Refine.append(r_vec_Refine)
                rot_mat, _ = cv2.Rodrigues(r_vec_Refine)
                rot_mats.append(rot_mat)
                t_vecs_Refine.append(t_vec_Refine)
        
        self.r_vecs = r_vecs_Refine
        self.t_vecs = t_vecs_Refine
        self.rot_mats = rot_mats
        return self.marker_ids, rot_mats, t_vecs_Refine
    
    def calculate_reprojection_error(self) -> list[float]:
        '''
        对每一个可能的Marker计算重投影误差
        '''
        self.reproject_errors = []
        if(self.marker_ids is not None):
            ids_flat = self.marker_ids.flatten()
            for i, mid in enumerate(ids_flat):
                projected_points, _ = cv2.projectPoints(self.object_points, self.r_vecs[i], self.t_vecs[i], self.camera_matrix, self.camera_distortion)
                projected_points = projected_points.reshape(-1, 2)
                errors = self.marker_corners[i] - projected_points
                per_point_error = np.linalg.norm(errors, axis=1)
                rmse = np.sqrt(np.mean(per_point_error ** 2))
                self.reproject_errors.append(rmse)
        return self.reproject_errors

    
    def draw_marker_axis(self):
        '''
        画出三条坐标轴
        
        :return: 返回绘出的图像
        :rtype: MatLike
        '''
        image = self.draw_marker()
        if self.marker_corners != () and self.marker_ids is not None:
            for index in range(len(self.marker_corners)):
                image = cv2.drawFrameAxes(image, self.camera_matrix, self.camera_distortion, self.r_vecs[index], self.t_vecs[index], self.marker_length * 3, 1)
        return image
    
    def pack(self) -> list:
        '''
        将后续模块可能用到的marker的相关信息打包成便于查阅的字典
        
        {
            "id",
            "rot_mat",
            "t_vec",
            "error"
        }
        '''
        markers = []
        if (self.marker_ids is not None):
            ids_flat = self.marker_ids.flatten()
            for i, mid in enumerate(ids_flat):
                marker_dict = {}
                marker_dict["id"] = mid
                marker_dict["rot_mat"] = self.rot_mats[i]
                marker_dict["t_vec"] = self.t_vecs[i]
                marker_dict["range"] = np.linalg.norm(x=self.t_vecs[i].flatten(), ord=2)
                euler = utils.rotmat_to_euler(self.rot_mats[i], degree=False)
                marker_dict["theta"] = np.pi - np.arccos(np.cos(euler[1]) * np.cos(euler[0]))
                marker_dict["error"] = self.reproject_errors[i]
                markers.append(marker_dict)
        return markers
