import os, sys

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, ".."))
src_path = os.path.join(project_root, "src")
if src_path not in sys.path:
    sys.path.insert(0, src_path)

import UE4CtrlAPI as UE4CtrlAPI
import VisionCaptureApi

import numpy as np
import cv2
import time, json, csv, datetime
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor


from detection import Aruco_Detection
from transformation import *
from fusion import Pose_Fusion
from utils import *

PATH = Path.cwd()
CONFIG_PATH = Path(PATH, "config")
DATA_PATH = Path(PATH, "run", "exp3")

class Experiment3():
    def __init__(self, dictionary: int) -> None:
        self.aruco_detection_down = Aruco_Detection(dictionary, maker_length=1.0)
        self.aruco_detection_front = Aruco_Detection(dictionary, maker_length=1.0)
        self.aruco_detection_down.load_arguments(str(Path(CONFIG_PATH, "camera.json")))
        self.aruco_detection_front.load_arguments(str(Path(CONFIG_PATH, "camera.json")))
        self.coordinate_transformation = Coordinate_Transformation()
        self.coordinate_transformation.parse_config(
            str(Path(CONFIG_PATH, "Config.json")),
            #str(Path(CONFIG_PATH, "install_markers_deprecated.json"))
            str(Path(CONFIG_PATH, "install_markers.json"))
        )
        self.pose_fusion = Pose_Fusion()
        self.vis = VisionCaptureApi.VisionCaptureApi()
        self.vis.jsonLoad(jsonPath=str(Path(CONFIG_PATH, "Config.json")))
        is_suss = self.vis.sendReqToUE4()
        if not is_suss:
            print('[ERROR]Can not send request to UE4, please execute RflySim3D first.')
            sys.exit(1)
        self.vis.startImgCap()
        self.run_path = Path(DATA_PATH, datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
        self.run_path.mkdir(parents=True)
        self.raw_csv_path = Path(self.run_path, "raw.csv")
        self.fusion_csv_path = Path(self.run_path, "fusion.csv")
        self.filter_csv_path = Path(self.run_path, "filter.csv")
        self.raw_buffer = []
        self.fusion_buffer = []
        self.filter_buffer = []

    def process_frame(self, detector: Aruco_Detection, sensor_id: int, frame: np.ndarray) -> list:
        '''
        [取图]->[处理图像]->[识别]->[获取字段表]->[变换]->[输出变换后的字段表]

        输入：detector Aruco_Detection()类 frame 一帧图像
        输出：字段列表
        '''
        # [取图]传入参数frame
        # [处理图像]
        # [识别]
        detector.detect_marker(frame)
        # [PnP解算]
        detector.estimate_pose()
        # [计算重映射误差]
        detector.calculate_reprojection_error()
        # [获取字段表]
        markers = detector.pack()
        # [刚体变换]
        result = []
        for marker in markers:
            marker_res = self.coordinate_transformation.transform(marker, sensor_id)
            if marker_res != {}:
                result.append(marker_res)
        # [输出变换后字段表列表]
        return result
    
    def fusion_loop(self, max_sample_time:float)->None:
        start_time = time.time()
        last_time = start_time
        time_interval = 1.0/30.0
        try:
            with ThreadPoolExecutor(max_workers=2) as executor:
                while(self.vis.hasData[0] and self.vis.hasData[1]):
                    last_time = last_time + time_interval
                    sleep_time = last_time - time.time()
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                    else:
                        last_time = time.time()
                    # Get one frame from down/front camera
                    image_down = self.vis.Img[0]
                    image_front = self.vis.Img[1]
                    timestamp = time.time() - start_time
                    # Submit task threads
                    down_future = executor.submit(self.process_frame, self.aruco_detection_down, 0, image_down)
                    front_future = executor.submit(self.process_frame, self.aruco_detection_front, 1, image_front)
                    # Receive result
                    markers_down = down_future.result()
                    markers_front = front_future.result()
                    markers = markers_down + markers_front
                    # Write raw data to buffer
                    self.write_rawdata2buffer(markers, timestamp)
                    # [Exp1]process raw data in 3 policy
                    #simple_fused_marker = self.pose_fusion.pose_fusion(markers, method=self.pose_fusion.SIMPLE_WEIGHT_MEAN_FUSION)
                    #self.write_result2buffer(simple_fused_marker, timestamp, 1)
                    #improved_fused_marker = self.pose_fusion.pose_fusion(markers, method=self.pose_fusion.IMPROVED_WEIGHT_MEAN_FUSION)
                    #self.write_result2buffer(improved_fused_marker, timestamp, 2)
                    #separate_fused_marker = self.pose_fusion.pose_fusion(markers, method=self.pose_fusion.SEPARATE_WEIGHT_MEAN_FUSION)
                    #self.write_result2buffer(separate_fused_marker, timestamp, 3)
                    # [Exp2]pre_process
                    filtered_markers = self.pose_fusion.pre_process_data(markers, threshold=8)
                    self.write_filterdata2buffer(filtered_markers, timestamp)
                    # process data in 2 policy
                    separated_fused_marker = self.pose_fusion.pose_fusion(markers, method=self.pose_fusion.SEPARATE_WEIGHT_MEAN_FUSION)
                    self.write_result2buffer(separated_fused_marker, timestamp, 3)
                    final_fused_marker = self.pose_fusion.pose_fusion(markers, method=self.pose_fusion.ELIMINATE_OUTLIERS_SEPARATE_WEIGHT_MEAN_FUSION)
                    self.write_result2buffer(final_fused_marker, timestamp, 4)
                    if timestamp > max_sample_time:
                        break
        finally:
            print("Sampling has been finished.")
            self.write_buffer2csv()
                    

    def write_rawdata2buffer(self, data:list[dict], timestamp:float)->None:
        for m in data:
            roll, pitch, yaw = rotmat_to_euler(m["rot_mat"])
            row = [
                timestamp,
                m["id"],
                m["t_vec"][0],
                m["t_vec"][1],
                m["t_vec"][2],
                roll,
                pitch,
                yaw
            ]
            self.raw_buffer.append(row)

    def write_filterdata2buffer(self, filter_data:list[dict], timestamp:float)->None:
        for m in filter_data:
            roll, pitch, yaw = rotmat_to_euler(m["rot_mat"])
            row = [
                timestamp,
                m["id"],
                m["t_vec"][0],
                m["t_vec"][1],
                m["t_vec"][2],
                roll,
                pitch,
                yaw
            ]
            self.filter_buffer.append(row)

    def write_result2buffer(self, data:dict, timestamp:float, policy:int)->None:
        roll, pitch, yaw = utils.rotmat_to_euler(data["rot_mat"])
        if (isinstance(roll, np.ndarray) and isinstance(pitch, np.ndarray) and isinstance(yaw, np.ndarray)):
            roll = roll[0]
            pitch = pitch[0]
            yaw = yaw[0]
        row = [
            timestamp,
            policy,
            data["t_vec"][0][0],
            data["t_vec"][1][0],
            data["t_vec"][2][0],
            roll,
            pitch,
            yaw
        ]
        self.fusion_buffer.append(row)

    def write_buffer2csv(self)->None:
        header = ["Timestamp", "MarkerID", "PosX", "PosY", "PosZ", "Roll", "Pitch", "Yaw"]
        with self.raw_csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(self.raw_buffer)
        header = ["Timestamp", "Policy", "PosX", "PosY", "PosZ", "Roll", "Pitch", "Yaw"]
        with self.fusion_csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(self.fusion_buffer)
        header = ["Timestamp", "MarkerID", "PosX", "PosY", "PosZ", "Roll", "Pitch", "Yaw"]
        with self.filter_csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(self.filter_buffer)


def test_experiment3():
    exp = Experiment3(cv2.aruco.DICT_7X7_1000)
    exp.fusion_loop(10)

def test_experiment3b():
    exp = Experiment3(cv2.aruco.DICT_4X4_50)
    exp.fusion_loop(10)
if __name__ == "__main__":
    test_experiment3b()