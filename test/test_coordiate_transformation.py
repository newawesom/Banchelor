import os, sys
from pathlib import Path

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, ".."))
src_path = os.path.join(project_root, "src")
if src_path not in sys.path:
    sys.path.insert(0, src_path)

from transformation import *

PATH = Path.cwd()
CONFIG_PATH = Path(PATH, "config")

def test_parse_config() -> None:
    transf = Coordinate_Transformation()
    transf.parse_config(str(Path(CONFIG_PATH, "Config.json")), str(Path(CONFIG_PATH, "install_markers.json")))
    print(transf.vision_sensors)
    print(transf.markers)

def test_rotmat_to_T() -> None:
    euler = [0.0, -90, -90]
    t_vec = [1, 2, 3]
    rot_mat = euler_to_rotmat(euler, degree=True)
    print(rot_mat)
    print(t_vec)
    T, T_inv = rotmat_to_T(rot_mat, t_vec)
    print(f"T:{T}\n T':{T_inv}")

def test_transform() -> None:
    transf = Coordinate_Transformation()
    transf.parse_config(str(Path(CONFIG_PATH, "Config.json")), str(Path(CONFIG_PATH, "install_markers.json")))


if __name__ == "__main__":
    test_rotmat_to_T()