import numpy as np
from scipy.spatial.transform import Rotation as R

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

def quat_to_rotmat(quat: np.ndarray) -> np.ndarray:
    q = [quat[1], quat[2], quat[3], quat[0]]
    rot = R.from_quat(q)
    R_mat = rot.as_matrix()
    return R_mat

def rotmat_to_quat(rotmat: np.ndarray) -> np.ndarray:
    rot = R.from_matrix(rotmat)
    q = rot.as_quat()
    return np.array([q[3], q[0], q[1], q[2]])

def quat_to_euler(quat: np.ndarray, degree=False) -> np.ndarray:
    q = [quat[1], quat[2], quat[3], quat[0]]
    rot = R.from_quat(q)
    euler = rot.as_euler('zyx')
    if degree:
        roll  = np.rad2deg(euler[2])
        pitch = np.rad2deg(euler[1])
        yaw   = np.rad2deg(euler[0])
    else:
        roll = euler[2]
        pitch = euler[1]
        yaw = euler[0]
    return np.array([roll, pitch, yaw])