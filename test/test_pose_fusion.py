import os, sys

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, ".."))
src_path = os.path.join(project_root, "src")
if src_path not in sys.path:
    sys.path.insert(0, src_path)

from pathlib import Path
from fusion import Pose_Fusion
import numpy as np

def test_weighted_mean_fusion()->None:
    pose_fusion = Pose_Fusion()
    test_one = {"id": 0,
                "rot_mat": np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
                "t_vec": np.array([[4.0], [5.0], [12.0]]), 
                "error": 0.3}
    test_two = {"id": 0,
                "rot_mat": np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
                "t_vec": np.array([[6.0], [5.3], [6.0]]),
                "error": 1.2}
    test_data = []
    test_data.append(test_one)
    test_data.append(test_two)
    result = pose_fusion.pose_fusion(test_data)
    print(result)
    print(result["t_vec"][0])

if __name__ == "__main__":
    test_weighted_mean_fusion()