import numpy as np

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
            sum_rotmat_div_error_square = np.zeros((3, 3))
            sum_one_div_error_square = 0
            weighted_mean_tvec = np.ndarray
            sum_tvec_div_error_square = np.zeros((3, 1))
            for m in pose:
                error2 = m["error"] * m["error"]
                sum_rotmat_div_error_square += m["rot_mat"] / error2
                sum_tvec_div_error_square += m["t_vec"] / error2
                sum_one_div_error_square += 1.0 / error2
            weighted_mean_rotmat = sum_rotmat_div_error_square / sum_one_div_error_square
            weighted_mean_tvec = sum_tvec_div_error_square / sum_one_div_error_square
            return {"rot_mat": weighted_mean_rotmat, "t_vec": weighted_mean_tvec}
        else:
            return {}
            
