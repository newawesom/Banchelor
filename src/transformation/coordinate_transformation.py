import numpy as np
import json
from pathlib import Path


PATH = Path.cwd()
CONFIG_PATH = Path(PATH, "config")

class Coordinate_Transformation():
    def __init__(self) -> None:
        self.vision_sensors = {}
        self.markers = {}

    def parse_config(self, sensors_path:str, markers_path:str) -> None:
        self.vision_sensors = {}
        self.markers = {}
        with open(sensors_path, 'r') as f:
            sensors_data = json.load(f)
            data_raw = sensors_data["VisionSensors"]
            # 处理传感器的位置变换
            for sensor in data_raw:
                euler = sensor["SensorAngEular"]
                rot_mat = euler_to_rotmat(euler, degree=True)
                t_vec = sensor["SensorPosXYZ"]
                sensor_id = sensor["SeqID"]
                segment = {"id": sensor_id,
                           "rot_mat": rot_mat,
                           "t_vec": t_vec}
                self.vision_sensors[sensor_id] = segment

        with open(markers_path, 'r') as f:
            markers_data = json.load(f)
            markers_raw = markers_data["Markers"]
            # 处理标记的位置变换
            for marker in markers_raw:
                marker_id = marker["id"]
                euler = marker["attitude"]
                rot_mat = euler_to_rotmat(euler, degree=True)
                t_vec = marker["position"]
                segment = {"id": marker_id,
                           "rot_mat": rot_mat,
                           "t_vec": t_vec}
                self.markers[marker_id] = segment
    

    def transform(self, segment_table:dict, sensor_id:int) -> dict:
        '''
        获取输入字段表，返回刚体坐标变换后的字段表
        '''
        if sensor_id not in self.vision_sensors:
            raise ValueError(f"sensor_id = {sensor_id} not found.")
        if segment_table["id"] not in self.markers:
            raise ValueError(f"marker_id = {segment_table['id']} not found.")
        body2camera_T_inv = np.ndarray  # 相机到机体变换矩阵 1
        marker2odom_T = np.ndarray      # 标记到里程计变换矩阵 3
        world2marker_T = np.ndarray     # 世界到标记变换矩阵 4
        odom2camera_T = np.ndarray      # 里程计到相机变换矩阵 2
        world2body_T = np.ndarray       # 【目标】世界到机体变换矩阵

        # 处理相机到机体变换矩阵 1
        _, body2camera_T_inv = rotmat_to_T(self.vision_sensors[sensor_id]["rot_mat"], self.vision_sensors[sensor_id]["t_vec"])
        # 处理标记到里程计变换矩阵 3
        marker_id = segment_table["id"]
        rot_mat = segment_table["rot_mat"]
        t_vec = segment_table["t_vec"]
        marker2odom_T, _ = rotmat_to_T(rot_mat, t_vec)
        # 处理世界到标记变换矩阵 4
        world2marker_T, _ = rotmat_to_T(self.markers[marker_id]["rot_mat"], self.markers[marker_id]["t_vec"])
        # 处理里程计到相机变换矩阵 2
        euler = [0.0, -90.0, -90.0]
        rot_mat = euler_to_rotmat(euler, degree=True)
        t_vec = [0, 0, 0]
        odom2camera_T, _ = rotmat_to_T(rot_mat, t_vec)

        # 变换运算
        world2body_T = ((world2marker_T @ marker2odom_T) @ odom2camera_T) @ body2camera_T_inv
        
        # 重新封装
        rot_mat, t_vec = T_to_rotmat(world2body_T)
        segment_table_new = segment_table.copy()
        segment_table_new["rot_mat"] = rot_mat
        segment_table_new["t_vec"] = t_vec

        return segment_table_new
                
        

def euler_to_rotmat(euler: list[float], degree=False):
        if degree:
            roll = np.deg2rad(euler[0])
            pitch = np.deg2rad(euler[1])
            yaw = np.deg2rad(euler[2])
        else:
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]
        cr = np.cos(roll)
        sr = np.sin(roll)
        cp = np.cos(pitch)
        sp = np.sin(pitch)
        cy = np.cos(yaw)
        sy = np.sin(yaw)

        rot_mat = np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                            [-sp,   cp*sr,            cp*cr]
                            ])
        return rot_mat
    
def rotmat_to_euler(rot_mat: np.ndarray, degree=False):
    if abs(rot_mat[2, 0]) < 1:
        pitch = -np.arcsin(rot_mat[2, 0])
        roll  = np.arctan2(rot_mat[2,1], rot_mat[2,2])
        yaw   = np.arctan2(rot_mat[1,0], rot_mat[0,0])
    else:
        pitch = np.pi/2 if rot_mat[2,0] <= -1 else -np.pi/2
        roll  = 0
        yaw   = np.arctan2(-rot_mat[0,1], rot_mat[1,1])
    if degree:
        roll  = np.rad2deg(roll)
        pitch = np.rad2deg(pitch)
        yaw   = np.rad2deg(yaw)
    return roll, pitch, yaw

def rotmat_to_T(rot_mat, t_vec):
    T = np.eye(4)
    T[:3, :3] = rot_mat
    T[:3, 3] = t_vec
    T_inv = np.eye(4)
    T_inv[:3, :3] = rot_mat.T
    T_inv[:3, 3] = -rot_mat.T @ t_vec
    return T, T_inv

def T_to_rotmat(T):
    rot_mat = T[:3, :3]
    t_vec = T[:3, 3]
    return rot_mat, t_vec