import os, sys

from click import password_option
from sympy import im
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, ".."))
src_path = os.path.join(project_root, "src")
if src_path not in sys.path:
    sys.path.insert(0, src_path)

import VisionCaptureApi, UE4CtrlAPI
import time,datetime
from pathlib import Path
import cv2
import csv
import numpy as np, math
import matplotlib

from detection import Aruco_Detection
from utils import *

PATH = Path.cwd()
CONFIG_PATH = Path(PATH, "config")
DATA_PATH = Path(PATH, "run", "exp2")
MAX_MARKERS = 15

class Experiment2:
    def __init__(self) -> None:
        self.detect = Aruco_Detection(cv2.aruco.DICT_4X4_50, 1)
        self.detect.load_arguments(str(Path(CONFIG_PATH, "camera.json")))
        self.ue = UE4CtrlAPI.UE4CtrlAPI()
        self.vis = VisionCaptureApi.VisionCaptureApi()
        self.origin = [4.5, 11.2, -1.21]
        self.vehicle_init_pos = [4.5, 11.2, -1.21 -1]
        self.vehicle_init_att = [0, 0, 0]
        self.marker_init_pos = [4.5 + 0.2, 11.2, -1.21 -1]
        self.marker_init_att = [math.pi/2, 0, math.pi/2]
        self.camera_matrix = np.ndarray((3,3), dtype=float)
        self.camera_distortion = np.ndarray((1,5), dtype=float)
        self.run_path = Path(DATA_PATH, datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
        self.run_path.mkdir(parents=True)
        self.csv_path = Path(self.run_path, "data.csv")
        self.csv_buffer = []
        self.count = 0

    def setup_env(self) -> None:
        # Use RflySim3D command to change map to Factory_drone
        self.ue.sendUE4Cmd('RflyChangeMapbyName Factory_drone')
        time.sleep(1)
        # Set up the initial position and attitude of drone and marker
        self.ue.sendUE4Pos(1, 0, 0, self.vehicle_init_pos, self.vehicle_init_att)
        time.sleep(1)
        self.ue.sendUE4Pos(100, 4400, 0, self.marker_init_pos, self.marker_init_att)
        time.sleep(1)
        # Load sensor configure file Config.json
        self.vis.jsonLoad()
        # Send image request to RflySim3D
        if not self.vis.sendReqToUE4():
            sys.exit(1)
        self.vis.startImgCap(True)
        time.sleep(1)

        if not self.vis.hasData[0]:
            sys.exit(1)

    def one_batch_sample(self, distance: float, angle: float, marker_id:int=-1) -> None:
        position = np.array(self.marker_init_pos) + np.array([distance, 0, 0])
        attitude1 = np.array(self.marker_init_att) + np.array([angle, 0, 0])
        attitude2 = np.array(self.marker_init_att) + np.array([0, 0, angle])
        if marker_id == -1:
            max_num = MAX_MARKERS
            buffer = []
            detected_count = 0.0
            sum_position_error =  0.0
            sum_attitude_error =  0.0
            sum_reprj_error = 0.0
            for i in range(0, max_num):
                type_id = 4400 + i
                # Att1
                result1 = self.process_one_frame(type_id, position, attitude1)
                self.process_data2buffer(buffer, distance, angle, i, result1, True)
                # Att2
                result2 = self.process_one_frame(type_id, position, attitude2)
                self.process_data2buffer(buffer, distance, angle, i, result2, False)
            for data in buffer:
                if(data[2]):
                    detected_count += 1
                    sum_position_error += data[3] * data[3]
                    sum_attitude_error += data[4]
                    sum_reprj_error += data[5] * data[5]
                else:
                    pass
            if(detected_count != 0):
                mean_detected_rate = detected_count / (max_num * 2.0)
                mean_position_error = np.sqrt(sum_position_error / detected_count)
                mean_attitude_error = sum_attitude_error / detected_count
                mean_reprj_error = np.sqrt(sum_reprj_error / detected_count)
                row = [
                    angle,
                    distance,
                    mean_detected_rate,
                    mean_position_error,
                    mean_attitude_error,
                    mean_reprj_error
                ]
                self.csv_buffer.append(row)
            else:
                row = [
                    angle,
                    distance,
                    0.0,
                    None,
                    None,
                    None,
                ]
                self.csv_buffer.append(row)
        else:
            type_id = 4400 + marker_id
            # Att1
            result1 = self.process_one_frame(type_id, position, attitude1)
            self.write_rawdata2buffer(distance, angle, marker_id, result1, True)
            # Att2
            result2 = self.process_one_frame(type_id, position, attitude2)
            self.write_rawdata2buffer(distance, angle, marker_id, result2, False)

    def process_one_frame(self, type_id:int, pos:np.ndarray, att:np.ndarray) -> tuple:
        self.ue.sendUE4Pos(100, type_id, 0, pos, att)
        time.sleep(0.25)
        # get one frame
        frame_raw = self.vis.Img[0]
        # detect
        if(self.detect.detect_marker(frame_raw)):
            (ids, rot_mats, t_vecs) = self.detect.estimate_pose()
            reprj_err = self.detect.calculate_reprojection_error()
            processed_frame = self.detect.draw_marker_axis()
            # get relative pos & att and re-projection error
            marker_id = ids[0][0]
            relative_pos = t_vecs[0]
            relative_att = rotmat_to_euler(rot_mats[0])
            return (marker_id, relative_pos, relative_att, reprj_err[0], processed_frame)
        else:
            return ()
        
    def write_rawdata2buffer(self, distance:float, angle:float, marker_id:int, result:tuple, is_roll:bool):
        if is_roll:
            real_attitude = [math.pi + angle, 0, 0]
        else:
            real_attitude = [math.pi, angle, 0]
        real_position = [0, 0, distance]
        if result != ():
            detected_id, position, attitude, error, frame = result
            row = [self.count,
                    angle,
                    distance,
                    marker_id,
                    detected_id,
                    position[0][0],
                    real_position[0],
                    position[1][0],
                    real_position[1],
                    position[2][0],
                    real_position[2],
                    attitude[0],
                    real_attitude[0],
                    attitude[1],
                    real_attitude[1],
                    attitude[2],
                    real_attitude[2],
                    error]
            self.csv_buffer.append(row)
            cv2.imwrite(str(Path(self.run_path, f"image{self.count}.jpg")), frame)
            self.count += 1
        else:
            row = [self.count,
                    angle,
                    distance,
                    marker_id,
                    None,
                    None,
                    real_position[0],
                    None,
                    real_position[1],
                    None,
                    real_position[2],
                    None,
                    real_attitude[0],
                    None,
                    real_attitude[1],
                    None,
                    real_attitude[2],
                    None]
            self.csv_buffer.append(row)
            self.count += 1

    def process_data2buffer(self, buffer:list, distance:float, angle:float, marker_id:int, result:tuple, is_roll:bool):
        if is_roll:
            real_attitude = [math.pi + angle, 0, 0]
        else:
            real_attitude = [math.pi, angle, 0]
        real_position = [0, 0, distance]
        if result != ():
            detected_id, position, attitude, error, frame = result
            is_detected = (detected_id == marker_id)
            position_error = np.sqrt((real_position[0] - position[0][0]) * (real_position[0] - position[1][0]) + (real_position[1] - position[1][0]) * (real_position[1] - position[1][0]) + (real_position[2] - position[2][0])*(real_position[2] - position[2][0]))
            attitude_error = angle_between_euler(attitude, real_attitude)
            row = [
                angle,
                distance,
                is_detected,
                position_error,
                attitude_error,
                error
            ]
            buffer.append(row)
            if self.count % 10 == 0:
                cv2.imwrite(str(Path(self.run_path, f"image{self.count}.png")), frame)
            self.count += 1
        else:
            row = [
                angle,
                distance,
                False,
                None,
                None,
                None
            ]
            buffer.append(row)
            self.count += 1

    def write_buffer2csv(self, raw:bool=False):
        if raw:
            header = ["Number", "Angle", "Distance", "MarkerID", "detected_id", "PosX", "RealX", "PosY", "RealY", "PosZ", "RealZ", "AttRoll", "RealRoll", "AttPitch", "RealPitch", "AttYaw", "RealYaw", "Re-Projection Error"]
        else:
            header = ["Angle", "Distance", "Rate", "Pos_Err", "Att_Err", "Re-prj_Err"]
        with self.csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(self.csv_buffer)

    def main_loop(self):
        angle = - math.pi * 5 / 12
        angle_end = math.pi * 5 / 12
        angle_intval = math.pi /12
        distance = 1
        distance_end = 14
        distance_intval = 1
        while(angle < angle_end + 0.001):
            distance = 2
            while(distance < distance_end + 0.01):
                self.one_batch_sample(distance, angle)
                distance += distance_intval
            angle += angle_intval
        print("Sample has been finished.")
        self.write_buffer2csv()
        sys.exit(0)

        
def test_one_batch_sample()->None:
    exp = Experiment2()
    exp.setup_env()
    exp.one_batch_sample(6, 0, -1)
    exp.write_buffer2csv()

def test_main_loop()->None:
    exp = Experiment2()
    exp.setup_env()
    exp.main_loop()


if __name__ == "__main__":
    test_main_loop()