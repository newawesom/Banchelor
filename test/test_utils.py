import os, sys,math

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, ".."))
src_path = os.path.join(project_root, "src")
if src_path not in sys.path:
    sys.path.insert(0, src_path)

from utils import *

def test_quat_rotmat() -> None:
    q = [0.7071, 0.7071, 0.0, 0.0]
    rotmat = utils.quat_to_rotmat(np.array(q))
    print(rotmat)

    rotmat = np.array([
        [0, -1, 0],
        [1, 0, 0],
        [0, 0, 1]
    ])
    quat = utils.rotmat_to_quat(rotmat)
    print(quat)

def test_angle_between_euler()->None:
    euler1 = [math.pi/2,0,0]
    euler2 = [math.pi/2, 0, 0]
    print(angle_between_euler(euler1, euler2))

def test() -> None:
    x = np.array([1, 2, 3, 4, 5, 6])
    mu = np.array([0, 0, 0, 0, 0, 0])
    Sigma = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1]
    ])
    Sigma_inv = np.linalg.inv(Sigma)
    d = (x - mu) @ Sigma_inv @ (x - mu).T
    print(d)

if __name__ == "__main__":
    test_angle_between_euler()