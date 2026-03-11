import numpy as np
import utils

class Pose_Fusion():
    def __init__(self) -> None:
        self.WEIGHT_MEAN_FUSION = 0

    def pose_fusion(self, pose: list[dict], method:int=0)->dict:
        match method:
            case 0:
                return self._weighted_mean_fusion(pose)
            case _:
                return self._weighted_mean_fusion(pose)
    
    def _weighted_mean_fusion(self, pose: list[dict])->dict:
        if pose:
            weighted_mean_rotmat = np.ndarray
            sum_roll_div_error_square = 0
            sum_pitch_div_error_square = 0
            sum_yaw_div_error_square = 0
            sum_one_div_error_square = 0
            weighted_mean_tvec = np.ndarray
            sum_tvec_div_error_square = np.zeros((3, 1))
            for m in pose:
                error2 = m["error"] * m["error"]
                roll, pitch, yaw = utils.rotmat_to_euler(m["rot_mat"])
                sum_roll_div_error_square += roll / error2
                sum_pitch_div_error_square += pitch /error2
                sum_yaw_div_error_square += yaw /error2
                sum_tvec_div_error_square += np.asarray(m["t_vec"]).reshape(3,1) / error2
                sum_one_div_error_square += 1.0 / error2
            weighted_mean_roll = sum_roll_div_error_square / sum_one_div_error_square
            weighted_mean_pitch = sum_roll_div_error_square / sum_one_div_error_square
            weighted_mean_yaw = sum_yaw_div_error_square / sum_one_div_error_square
            weighted_mean_rotmat = utils.euler_to_rotmat([weighted_mean_roll,
                                                          weighted_mean_pitch,
                                                          weighted_mean_yaw])
            weighted_mean_tvec = sum_tvec_div_error_square / sum_one_div_error_square
            return {"rot_mat": weighted_mean_rotmat, "t_vec": weighted_mean_tvec}
        else:
            return {}
            
