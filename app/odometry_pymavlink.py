from typing import Any, Generator

import numpy as np
import os, sys, time
from pathlib import Path

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil

import VisionCaptureApi
import UE4CtrlAPI
import PX4MavCtrlV4
import ReqCopterSim

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, ".."))
src_path = os.path.join(project_root, "src")
if src_path not in sys.path:
    sys.path.insert(0, src_path)

import utils
from estimation import Pose_Estimation, pose_estimation

PATH = Path.cwd()
CONFIG_PATH = Path(PATH, "config")

def get_odometry(pose_estimator: Generator) -> dict[str, Any]:
    seg_table = next(pose_estimator)
    t_vec = np.asarray(seg_table["t_vec"], dtype=float).reshape(3)
    quat = np.asarray(seg_table["quat"], dtype=float).reshape(4)
    return {
        "timestamp": int(time.time() * 1e6),
        "position": t_vec,
        "quat": quat,
    }


def send_odometry(master: mavutil.mavudp | Any, odometry_msg: dict[str, Any]) -> None:
    position = odometry_msg["position"]
    quat = odometry_msg["quat"]
    estimator_type_vision = getattr(
        mavutil.mavlink,
        "MAV_ESTIMATOR_TYPE_VISION",
        getattr(mavutil.mavlink, "MAVESTIMATOR_TYPE_VISION", 2),
    )
    pose_covariance = [float("nan")] + [0.0] * 20
    velocity_covariance = [float("nan")] + [0.0] * 20

    if hasattr(master.mav, "odometry_send"):
        master.mav.odometry_send(
            odometry_msg["timestamp"],
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            float(position[0]),
            float(position[1]),
            float(position[2]),
            [float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])],
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            pose_covariance,
            velocity_covariance,
            0,
            estimator_type_vision,
            0,
        )
        return

    if hasattr(master.mav, "vision_position_estimate_send"):
        roll, pitch, yaw = utils.quat_to_euler(quat)
        master.mav.vision_position_estimate_send(
            odometry_msg["timestamp"],
            float(position[0]),
            float(position[1]),
            float(position[2]),
            float(roll),
            float(pitch),
            float(yaw),
        )
        return

    raise AttributeError(
        f"{type(master.mav).__name__} does not support ODOMETRY or VISION_POSITION_ESTIMATE messages."
    )

def main():
    #=====SETUP=====#
    req = ReqCopterSim.ReqCopterSim()
    StartCopterID = 1
    TargetIP = req.getSimIpID(StartCopterID)
    # mav = PX4MavCtrlV4.PX4MavCtrler(1)
    

    vis = VisionCaptureApi.VisionCaptureApi(TargetIP)
    vis.jsonLoad(jsonPath=str(Path(CONFIG_PATH, "Config.json")))
    vis.sendReqToUE4()
    vis.startImgCap()
    vis.sendImuReqCopterSim(StartCopterID, TargetIP)
    time.sleep(1)
    pose_estimation = Pose_Estimation(vis=vis)
    print(vis.hasData)
    generator = pose_estimation.yield_fusion_pose()
    #======MAVLINK INIT=====#
    master = mavutil.mavlink_connection('udpout:127.0.0.1:20100')
    #master.wait_heartbeat()
    last_time = time.time()
    time_interval = 1.0 / 30.0
    #=====LOOP======#
    while True:
        last_time = last_time + time_interval
        sleep_time = last_time - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            last_time = time.time()

        send_odometry(master, get_odometry(generator))


if __name__ == "__main__":
    main()
