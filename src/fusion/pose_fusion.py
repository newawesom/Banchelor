import numpy as np
import utils
from sklearn.covariance import MinCovDet

class Pose_Fusion():
    def __init__(self) -> None:
        self.WEIGHT_MEAN_FUSION = 0
        self.IMPROVED_WEIGHT_MEAN_FUSION = 1
        self.LEAST_SQUARE_METHOD = 2
        self.SEPARATE_WEIGHT_MEAN_FUSION = 3
        self.ELIMINATE_OUTLIERS_SEPARATE_WEIGHT_MEAN_FUSION = 4

    def pose_fusion(self, pose: list[dict], method:int=0)->dict:
        match method:
            case self.WEIGHT_MEAN_FUSION:
                return self.__weighted_mean_fusion(pose)
            case self.IMPROVED_WEIGHT_MEAN_FUSION:
                return self.__improved_weighted_mean_fusion(pose)
            case self.SEPARATE_WEIGHT_MEAN_FUSION:
                return self.__separate_weighted_mean_fusion(pose)
            case 4:
                return self.__eliminate_outliers_separate_weight_mean_fusion(pose)
            case _:
                return self.__weighted_mean_fusion(pose)
            
    def __pre_process_data(self, pose: list[dict], threshold: float, robust=False) -> list[dict]:
        if(len(pose) <= 3):
        # Too few sample
            return pose
        else:
            data_mat = np.array((len(pose), 7))
            pose_new = []
            # Filling data_mat with [t_vec, quat] from pose
            for index, pose_m in enumerate(pose):
                t_vec = pose_m["t_vec"]
                rot_mat = pose_m["rot_mat"]
                quat = utils.rotmat_to_quat(rot_mat)
                pose_m["quat"] = quat
                vec = np.array([t_vec[0], t_vec[1], t_vec[2], quat[0], quat[1], quat[2], quat[3]])
                pose_m["vec"] = vec
                data_mat[:, index] = vec
            # Get \mu and \sigma using robust method based on MCD or not
            if robust:
                mcd = MinCovDet().fit(data_mat)
                mu = mcd.location_
                Sigma = mcd.covariance_
                Sigma_inv = np.linalg.inv(Sigma)
            else:
                mu = np.mean(data_mat, axis=0)
                Sigma = np.cov(data_mat, rowvar=False)
                Sigma_inv = np.linalg.inv(Sigma)
            # eliminate outliers
            for pose_m in pose:
                d = pose_m["vec"] - mu
                distance = d @ Sigma_inv @ d.T
                if distance < threshold:
                    pose_new.append(pose_m)
            return pose_new
            
    def __improved_calculate_weight(self, seg:dict) -> float:
        seg_error = seg["error"]
        seg_range = seg["range"] * seg["range"]
        seg_theta = seg["theta"]
        weight = (np.cos(seg_theta) / (seg_error * seg_range))
        return weight
    
    def __separate_calculate_weight(self, seg:dict) -> np.ndarray:
        '''
        w_i, where i is x,y,z,roll,pitch,yaw, are calculated separately.
        w_i = (F(theta) * G(i)) / (H(R) * error)
        F(theta) = cos(theta);
        G(i) = 1 if i = f(id) else 0.5;
        H(R) = {log_2(x+1), x < 3,
                x - 1, 3 <= x < 7,
                (x - 6)^2 + 5, x > 7};
        '''
        weight = np.zeros((3, 1), dtype=float)
        # 计算G(i)
        match seg["feature"]:
            case "X":
                G_i = np.array([[1], [0.5], [0.5]])
            case "Y":
                G_i = np.array([[0.5], [1], [0.5]])
            case "Z":
                G_i = np.array(([0.5], [0.5], [1]))
        # 计算F(theta)
        F_theta = np.cos(seg["theta"])
        # 计算H(R)
        r = seg["range"]
        if r < 3:
            H_r = np.log2(r + 1)
        elif r < 7:
            H_r = r - 1
        else:
            H_r = (r - 6) * (r - 6) + 5
        # 计算error
        error = seg["error"]
        weight = (F_theta * G_i) / (H_r * error)
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
                weight = self.__improved_calculate_weight(m)
                eluer = np.asarray(utils.rotmat_to_euler(m["rot_mat"])).reshape(3, 1)
                sum_weighted_euler = sum_weighted_euler + eluer * weight
                t_vec = np.asarray(m["t_vec"]).reshape(3, 1)
                sum_weighted_tvec = sum_weighted_tvec + t_vec * weight
                sum_weights += weight
            weighted_mean_euler = sum_weighted_euler / sum_weights
            weighted_mean_tvec = sum_weighted_tvec / sum_weights
            weighted_mean_rotmat = utils.euler_to_rotmat([weighted_mean_euler[0], weighted_mean_euler[1], weighted_mean_euler[2]])
            return {"rot_mat": weighted_mean_rotmat, "t_vec": weighted_mean_tvec}
        else:
            return {}
        
    def __separate_weighted_mean_fusion(self, pose: list[dict]) -> dict:
        if pose:
            sum_weighted_euler = np.zeros((3, 1))
            sum_weighted_tvec = np.zeros((3, 1))
            sum_weights = np.zeros((3, 1))
            for m in pose:
                weight = self.__separate_calculate_weight(m)
                euler = np.asarray(utils.rotmat_to_euler(m["rot_mat"])).reshape(3, 1)
                sum_weighted_euler = sum_weighted_euler + euler * weight
                t_vec = np.asarray(m["t_vec"]).reshape(3, 1)
                sum_weighted_tvec = sum_weighted_tvec + t_vec * weight
                sum_weights = sum_weights + weight
            weighted_mean_euler = sum_weighted_euler / sum_weights
            weighted_mean_tvec = sum_weighted_tvec / sum_weights
            weighted_mean_rotmat = utils.euler_to_rotmat([weighted_mean_euler[0], weighted_mean_euler[1], weighted_mean_euler[2]])
            return {"rot_mat": weighted_mean_rotmat, "t_vec": weighted_mean_tvec}
        else:
            return {}
            
    def __eliminate_outliers_separate_weight_mean_fusion(self, pose: list[dict]) -> dict:
        if pose:
            pose = self.__pre_process_data(pose, threshold=12.59)
            sum_weighted_tvec = np.zeros((3, 1))
            M = np.zeros((4, 4))
            q_ref = pose[0]["quat"]
            sum_weights = np.zeros((3, 1))
            for m in pose:
                weight = self.__separate_calculate_weight(m)
                t_vec = np.asarray(m["t_vec"]).reshape(3, 1)
                sum_weighted_tvec = sum_weighted_tvec + t_vec * weight
                sum_weights = sum_weights + weight
                if np.dot(m["quat"], q_ref) < 0:
                    quat = - m["quat"]
                else:
                    quat = m["quat"]
                M = M + weight * (quat.T @ quat)
            weighted_mean_tvec = sum_weighted_tvec / sum_weights
            eigenvalues, eigenvectors = np.linalg.eig(M)
            weighted_mean_quat = eigenvectors[:, np.argmax(eigenvalues)]
            weighted_mean_quat /= np.linalg.norm(weighted_mean_quat)
            weighted_mean_rotmat = utils.quat_to_rotmat(weighted_mean_quat)
            return {"rot_mat": weighted_mean_rotmat, "t_vec": weighted_mean_tvec}
        else:
            return {}
    