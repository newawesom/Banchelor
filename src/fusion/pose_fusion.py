import numpy as np
import utils
#from sklearn.covariance import MinCovDet

class Pose_Fusion():
    def __init__(self) -> None:
        self.SIMPLE_WEIGHT_MEAN_FUSION = 0
        self.IMPROVED_WEIGHT_MEAN_FUSION = 1
        self.SEPARATE_WEIGHT_MEAN_FUSION = 2
        self.ELIMINATE_OUTLIERS_SEPARATE_WEIGHT_MEAN_FUSION = 3

    def pose_fusion(self, pose: list[dict], method:int=0)->dict:
        match method:
            case self.SIMPLE_WEIGHT_MEAN_FUSION:
                return self.__simple_weighted_mean_fusion(pose)
            case self.IMPROVED_WEIGHT_MEAN_FUSION:
                return self.__improved_weighted_mean_fusion(pose)
            case self.SEPARATE_WEIGHT_MEAN_FUSION:
                return self.__separate_weighted_mean_fusion(pose)
            case self.ELIMINATE_OUTLIERS_SEPARATE_WEIGHT_MEAN_FUSION:
                pose = self.pre_process_data(pose, threshold=8)
                return self.__eliminate_outliers_separate_weight_mean_fusion(pose)
            case _:
                return self.__simple_weighted_mean_fusion(pose)
            
    def pre_process_data(self, pose: list[dict], threshold: float, robust=False) -> list[dict]:
        data_length = len(pose)
        MIN_DETECTED_MARKERS = 3
        if(data_length <= MIN_DETECTED_MARKERS):
        # Too few sample
            for pose_m in pose:
                rot_mat = pose_m["rot_mat"]
                quat = utils.rotmat_to_quat(rot_mat)
                pose_m["quat"] = quat
            return pose
        else:
            if data_length <= 7:
                data_mat = np.zeros((len(pose), 3))
            else:
                data_mat = np.zeros((len(pose), 7))
            pose_new = []
            # Filling data_mat with [t_vec, quat] or [t_vec] from pose
            for index, pose_m in enumerate(pose):
                t_vec = pose_m["t_vec"]
                rot_mat = pose_m["rot_mat"]
                quat = utils.rotmat_to_quat(rot_mat)
                pose_m["quat"] = quat
                if data_length <= 7:
                    vec = np.array([t_vec[0], t_vec[1], t_vec[2]])
                else:
                    vec = np.array([t_vec[0], t_vec[1], t_vec[2], quat[0], quat[1], quat[2], quat[3]])
                pose_m["vec"] = vec
                data_mat[index, :] = vec
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
        
    def __simple_calculate_weight(self, seg:dict) -> float:
        seg_error = seg["error"]
        weight = 1.0 / (seg_error * seg_error)
        return weight
            
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
        H(r) = {r, x < 3,
                r^2 - 6, 3 <= x < 6,
                e^r - e^6 + 30, x >= 6};
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
            H_r = r
        elif r < 6:
            H_r = r * r - 6
        else:
            H_r = np.exp(r) - np.exp(6) + 30
        # 计算error
        error = seg["error"]
        weight = (F_theta * G_i) / (H_r * error)
        return weight
    
    def __simple_weighted_mean_fusion(self, pose: list[dict])->dict:
        if pose:
            sum_weighted_roll = 0
            sum_weighted_pitch = 0
            sum_weighted_yaw = 0
            sum_weights = 0
            sum_weighted_vec = np.zeros((3, 1))
            for m in pose:
                weight = self.__simple_calculate_weight(m)
                roll, pitch, yaw = utils.rotmat_to_euler(m["rot_mat"])
                sum_weighted_roll += roll * weight
                sum_weighted_pitch += pitch * weight
                sum_weighted_yaw += yaw * weight
                sum_weighted_vec += np.asarray(m["t_vec"]).reshape(3,1) * weight
                sum_weights += weight
            weighted_mean_roll = sum_weighted_roll / sum_weights
            weighted_mean_pitch = sum_weighted_roll / sum_weights
            weighted_mean_yaw = sum_weighted_yaw / sum_weights
            weighted_mean_rotmat = utils.euler_to_rotmat([weighted_mean_roll,
                                                          weighted_mean_pitch,
                                                          weighted_mean_yaw])
            weighted_mean_tvec = sum_weighted_vec / sum_weights
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
                M = M + np.max(weight) * np.outer(quat, quat)
            weighted_mean_tvec = sum_weighted_tvec / sum_weights
            eigenvalues, eigenvectors = np.linalg.eig(M)
            weighted_mean_quat = eigenvectors[:, np.argmax(eigenvalues)]
            weighted_mean_quat /= np.linalg.norm(weighted_mean_quat)
            weighted_mean_rotmat = utils.quat_to_rotmat(weighted_mean_quat)
            return {"rot_mat": weighted_mean_rotmat, "t_vec": weighted_mean_tvec, "quat": weighted_mean_quat}
        else:
            return {}
    