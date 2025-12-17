import  random
import os
import time
import math
import sys
import datetime
import numpy as np
import glob

import cv2
import VisionCaptureApi
import UE4CtrlAPI


class Camera_Calibration:
    def __init__(self) -> None:
        '''
        初始化参数列表
        
        :param self: 自体指针
        '''
        self.ue = UE4CtrlAPI.UE4CtrlAPI() # UE4控制接口
        self.vis = VisionCaptureApi.VisionCaptureApi() # 视觉取图接口
        self.vehicle_pos = [4.5, 11.2, -1.21 - 3] # 无人机的初始位置
        self.vehicle_att = [0, 0, 0] # 无人机的初始姿态(以欧拉角描述)
        self.object_points = [] # 标定板角点3D世界坐标
        self.image_points = []  # 标定板角点2D像素坐标
        self.image_size = (0, 0)
        
    
    def setup_env(self) -> None:
        '''
        设置标定板采样的环境
        
        :param self: 说明
        '''
        # 在UE中切换环境到Factory_drone
        self.ue.sendUE4Cmd('RflyChangeMapbyName Factory_drone')
        time.sleep(5)

        # 设置UE的窗口分辨率
        self.ue.sendUE4Cmd('r.setres 1920x1080w', 0)
        self.ue.sendUE4Cmd('t.MaxFPS 30', 0)
        time.sleep(2)

        # 创建无人机模型搭载RGB相机，以标定相机
        self.ue.sendUE4Pos(1, 0, 0, self.vehicle_pos, self.vehicle_att)
        time.sleep(2)

        # 加载Config.json中的传感器配置文件
        self.vis.jsonLoad()

        # 向RflySim3D发送取图请求
        if not self.vis.sendReqToUE4():
            sys.exit(0)
        
        # 开启取图，模式为共享内存
        self.vis.startImgCap(True)
        time.sleep(1)

    def fetch_board_images(self, num: int) -> str:
        '''
        获取用于标定的标定板图片采样
        
        :param self: 自体指针
        :param num: 所需要采样标定板图片的张数
        :type num: int

        :returns out: 返回采样图片存储的路径
        '''
        board_init_pos = [self.vehicle_pos[0] + 0.6, self.vehicle_pos[1], self.vehicle_pos[2] - 0.1]
        board_init_att = [self.vehicle_att[0] + math.pi / 2, self.vehicle_att[1], self.vehicle_att[2] + math.pi / 2]

        #创建标定板
        self.ue.sendUE4Pos(100, 40, 0, board_init_pos, board_init_att)
        time.sleep(1)

        # 以当前时间和日期创建文件夹，准备写入图片
        path_prefix = sys.path[0] # 当前工作路径
        path_dir = os.path.join(path_prefix, "run",datetime.datetime.now().strftime("%Y%m%d_%H%M%S")) # 在./run文件夹下以“年月日_时分秒”的格式创建保存目录
        os.makedirs(path_dir)
        
        # 进入取图主循环
        start_time = time.time()
        last_time = time.time()
        time_interval = 0.1
        cnt = 0
        idx = 0
        while cnt < num:
            last_time = last_time + time_interval
            sleep_time = last_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                last_time = time.time()

            idx += 1
            if idx % 5 == 0:
                # 赋予标定板一定的随机位置和姿态
                board_pos = [board_init_pos[0] + random.randint(0, 100) / 400.0,
                             board_init_pos[1] + random.randint(-50, 50) / 300.0,
                             board_init_pos[2] + random.randint(-50, 50) / 100.0 * 0.3]
                board_att = [board_init_att[0] + random.randint(-50, 50) / 50.0 * 30 / 180.0 * math.pi,
                             board_init_att[1] + random.randint(-50, 50) / 50.0 * 30 / 180.0 * math.pi,
                             board_init_att[2] + random.randint(-50, 50) / 50.0 * 30 / 180.0 * math.pi]
                self.ue.sendUE4Pos(100, 40, 0, board_pos, board_att, -1)
                time.sleep(0.2)

                # 取图，将其转化为灰度图像并安装编号命名各个图片
                if self.vis.hasData[0]:
                    img = self.vis.Img[0]
                    pic = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
                    # cv2.imshow("Board", pic)
                    cv2.imwrite(os.path.join(path_dir, f"{cnt + 1}.jpg"), pic)
                cnt += 1
        print(f"All {cnt} images have been written to path:{path_dir}")
        return path_dir

    def find_chessboard_corners(self, image_dir: str) -> None:
        '''
        标定相机
        
        :param self: 自体指针
        :param image_dir: 采样图片存储的路径
        :type image_dir: str
        '''
        # 获取标定板角点的世界坐标位置
        object_point = np.zeros((8 * 12, 3), np.float32) # 8 * 12 代表棋盘格的内角点个数；3代表有XYZ三个维度
        object_point[:, :2] = np.mgrid[0:12, 0:8].T.reshape(-1, 2) # 将世界坐标系建立在标定板上，所有点的Z坐标均为0，只需要确定X和Y的值
        object_point = 6 * object_point # 棋盘格单格边长为6cm
        obj_points = [] # 存储3D点
        img_points = [] # 存储2D点

        # 获取图片位置
        images_path = os.path.join(image_dir, "*.jpg")
        images = glob.glob(images_path)

        # 设置寻找亚像素角点的参数，采用的停止准则是最大循环次数30和最大误差容限0.001
        criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)

        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            size = gray.shape[::-1]
            ret, corners = cv2.findChessboardCorners(gray, (12, 8), None, (cv2.CALIB_CB_ADAPTIVE_THRESH 
                                                     + cv2.CALIB_CB_NORMALIZE_IMAGE 
                                                     + cv2.CALIB_CB_FAST_CHECK))
            if ret:
                obj_points.append(object_point)
                corners_precise = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
                if [corners_precise]:
                    img_points.append(corners_precise)
                else:
                    img_points.append(corners)
            
            # 画出角点
            cv2.drawChessboardCorners(img, (12, 8), corners, ret)
            cv2.imshow("Chessboard Corners", img)
            cv2.waitKey(100)
        print(f"Corners are found in {len(img_points)} board images.")
        cv2.destroyAllWindows()
        self.object_points = obj_points
        self.image_points = img_points
        self.image_size = size
        #TODO: 将数据存储在可存储文件当中

    def calibrate_camera(self) -> tuple[float, cv2.UMat, cv2.UMat]:
        '''
        由@fn find_chessboard_corners()找到的角点的世界坐标和像素坐标进行标定，使用前必须先调用@fn find_chessboard_corners()找到角点

        :return: 返回标定精度、内参数矩阵、畸变参数的元组
        :rtype: tuple[float, UMat, UMat]
        '''
        #criteria = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 30, 0.001)
        #TODO：从文件中读取数据
        if self.object_points == [] or self.image_points == []:
            print("[ERROR]Empty object points & image points, please call find_chessboard_corner first!")
            sys.exit(0)
        ret, camera_matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(self.object_points, self.image_points, self.image_size, None, None, None, None)
        print("ret:", ret)
        print("mtx:\n", camera_matrix)
        print("dist:\n", distortion)
        #print("rvecs:\n", r_vecs)
        #print("tvecs:\n", t_vecs)

        return (ret, camera_matrix, distortion)
        # TODO：将结果数据存储在文件中以便读取
        