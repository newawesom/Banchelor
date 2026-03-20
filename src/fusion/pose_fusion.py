import numpy as np
import utils

class Pose_Fusion():
    def __init__(self) -> None:
        self.WEIGHT_MEAN_FUSION = 0
        self.IMPROVED_WEIGHT_MEAN_FUSION = 1
        self.LEAST_SQAURE_MEHTOD = 2

    def pose_fusion(self, pose: list[dict], method:int=0)->dict:
        match method:
            case 0:
                return self.__weighted_mean_fusion(pose)
            case 1:
                return self.__improved_weighted_mean_fusion(pose)
            case _:
                return self.__weighted_mean_fusion(pose)
            
    def __calculate_weight(self, seg:dict) -> float:
        seg_error = seg["error"]
        seg_range = seg["range"]
        seg_theta = seg["theta"]
        weight = 1.0 / ((seg_range * seg_error / np.cos(seg_theta)) * (seg_range * seg_error / np.cos(seg_theta)))
        return weight
    
    def __weighted_mean_fusion(self, pose: list[dict])->dict:
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
        
    def __improved_weighted_mean_fusion(self, pose: list[dict])->dict:
        if pose:
            sum_weighted_euler = np.zeros((3, 1))
            sum_weighted_tvec = np.zeros((3, 1))
            sum_weights = 0.0
            for m in pose:
                weight = self.__calculate_weight(m)
                eluer = np.asarray(utils.rotmat_to_euler(m["rot_mat"])).reshape(3, 1)
                sum_weighted_euler = sum_weighted_euler + eluer * weight
                t_vec = np.asarray(m["t_vec"]).reshape(3, 1)
                sum_weighted_tvec = sum_weighted_tvec + t_vec * weight
                sum_wieghts += weight
            weighted_mean_euler = sum_weighted_euler / sum_weights
            weighted_mean_tvec = sum_weighted_tvec / sum_weights
            weighted_mean_rotmat = utils.euler_to_rotmat([weighted_mean_euler[0], weighted_mean_euler[1], weighted_mean_euler[2]])
            return {"rot_mat": weighted_mean_rotmat, "t_vec": weighted_mean_tvec}
        else:
            return {}
            
