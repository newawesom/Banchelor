import time
import math
import sys

import PX4MavCtrlV4 as PX4MavCtrl
import UE4CtrlAPI

'''
@file PX4MavCtrl_template.py
@brief 基于PX4MavCtrl库通过Mavlink协议对无人机进行Offboard控制的模板文件
@dependency RflySim
@author NIUHAO
@email niuhao02@outlook.com
'''

# 创建Mavlink控制句柄
mav = PX4MavCtrl.PX4MavCtrler(1)

# 创建UE4控制句柄
ue = UE4CtrlAPI.UE4CtrlAPI()

################UE4设置################

######################################

################开启Mavlink监听################
mav.InitMavLoop()
time.sleep(1)
###########开启Offboard模式##########
mav.initOffboard()
###########用户代码############

##############################
mav.endOffboard()
time.sleep(1)
####################################
mav.endMavLoop()
##############################################