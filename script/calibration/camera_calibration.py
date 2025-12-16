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

        
