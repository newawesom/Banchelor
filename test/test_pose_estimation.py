import os, sys

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, ".."))
src_path = os.path.join(project_root, "src")
if src_path not in sys.path:
    sys.path.insert(0, src_path)

from pathlib import Path
from estimation import Pose_Estimation
import datetime

PATH = Path.cwd()
DATA_PATH = Path(PATH, "run", "exp")

def test_visualize() ->None:
    pose_estimation = Pose_Estimation()
    pose_estimation.visualize()
def test_load_camera_config() -> None:
    pose_estimation = Pose_Estimation()
    pose_estimation.load_camera_config()
    print(pose_estimation.camera_down_config, pose_estimation.camera_front_config)
def test_fusion_loop()->None:
    run_path = Path(DATA_PATH, datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    run_path.mkdir(parents=True)
    pose_estimation = Pose_Estimation()
    pose_estimation.fusion_loop(str(run_path), 20*5)

if __name__ == "__main__":
    test_fusion_loop()