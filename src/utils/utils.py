import numpy as np

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