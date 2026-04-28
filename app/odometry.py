from types import GeneratorType
from typing import Generator

import numpy as np
import os, sys
import cv2
from pathlib import Path

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, PoseStamped

import VisionCaptureApi
import UE4CtrlAPI
import PX4MavCtrlV4ROS as PX4MavCtrl
import ReqCopterSim
import RflyRosStart

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, ".."))
src_path = os.path.join(project_root, "src")
if src_path not in sys.path:
    sys.path.insert(0, src_path)

import utils
from estimation import Pose_Estimation, pose_estimation

PATH = Path.cwd()
CONFIG_PATH = Path(PATH, "config")

    
def get_odometry(pose_estimator:Generator)->PoseStamped:
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    seg_table = next(pose_estimator)
    t_vec = np.asarray(seg_table["t_vec"], dtype=float).reshape(3)
    quat = np.asarray(seg_table["quat"], dtype=float).reshape(4)
    pose.pose.position.x = float(t_vec[0])
    pose.pose.position.y = float(t_vec[1])
    pose.pose.position.z = float(t_vec[2])
    pose.pose.orientation.w = float(quat[0])
    pose.pose.orientation.x = float(quat[1])
    pose.pose.orientation.y = float(quat[2])
    pose.pose.orientation.z = float(quat[3])
    return pose

def publish_odometry(pub:rospy.Publisher, pose:PoseStamped)->None:
    #通过mavros发布里程计信息到"mavros/vision_pose/pose"或者"/mavros/odometry/out"
    pose.header.frame_id = "map"
    pub.publish(pose)

def main():
    #=========================SETUP=========================#
    # 启动ROS发布模式
    VisionCaptureApi.isEnableRosTrans = True
    req = ReqCopterSim.ReqCopterSim() #获取局域网内所有CopterSim程序电脑的IP
    StartCopterID = 1 # 飞机的ID号
    TargetIP = req.getSimIpID(StartCopterID)

    # 开启mavros
    if not (RflyRosStart.isLinux and RflyRosStart.isRosOk):
        print("This program can only run on linux with ROS")
        sys.exit(0)
    ros = RflyRosStart.RflyRosStart(StartCopterID, TargetIP)
    # 开启视觉取图
    vis = VisionCaptureApi.VisionCaptureApi(TargetIP)
    vis.jsonLoad(jsonPath=str(Path(CONFIG_PATH, "Config.json")))
    vis.startImgCap()
    vis.sendImuReqCopterSim(StartCopterID, TargetIP)
    print("[1]")
    pose_estimation = Pose_Estimation(vis=vis)
    print("[2]")
    #=========================ROS INIT=========================#
    print("[3]")
    rospy.init_node("odom_pub")
    print("[4]")
    pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=10)
    rate = rospy.Rate(30)

    #=========================LOOP==========================#
    print("[5]")
    pose_generator = pose_estimation.yield_fusion_pose()
    print("[6]")
    while not rospy.is_shutdown():
        pose = get_odometry(pose_generator)
        publish_odometry(pub, pose=pose)
        rate.sleep()


if __name__ == "__main__":
    main()
