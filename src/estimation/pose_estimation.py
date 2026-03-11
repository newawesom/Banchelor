import UE4CtrlAPI as UE4CtrlAPI
import ReqCopterSim
import VisionCaptureApi

import numpy as np
import cv2
import time, sys, json, csv
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor


from detection import Aruco_Detection
from transformation import *
from fusion import Pose_Fusion
from utils import *

PATH = Path.cwd()
CONFIG_PATH = Path(PATH, "config")


class Pose_Estimation():
    def __init__(self) -> None:
        self.aruco_detection_down = Aruco_Detection(cv2.aruco.DICT_7X7_1000, maker_length=1.0)
        self.aruco_detection_down.load_arguments(str(Path(CONFIG_PATH, "camera.json")))
        self.aruco_detection_front = Aruco_Detection(cv2.aruco.DICT_7X7_1000, maker_length=1.0)
        self.aruco_detection_front.load_arguments(str(Path(CONFIG_PATH, "camera.json")))
        self.coordinate_transformation = Coordinate_Transformation()
        self.coordinate_transformation.parse_config(
            str(Path(CONFIG_PATH, "Config.json")),
            str(Path(CONFIG_PATH, "install_markers.json"))
        )
        self.camera_down_config = {}
        self.camera_front_config = {}
        self.marker_history = {"down": [], "front": []}
        self.pose_fusion = Pose_Fusion()

        self.vis = VisionCaptureApi.VisionCaptureApi()
        self.vis.jsonLoad(-1, str(Path(CONFIG_PATH, "Config.json")))
        is_suss = self.vis.sendReqToUE4()
        if not is_suss:
            print('[ERROR]Can not send request to UE4, please execute RflySim3D first.')
            sys.exit(1)
        self.vis.startImgCap(True)
        time.sleep(1)

    def visualize(self) -> None:
        '''
        可视化ArUco码的检测

        注：此函数只用作调试
        
        :param self: 说明
        '''
        # 设置定时触发器
        last_time = time.time()
        time_interval = 1.0 / 30.0 # 触发的最小时间间隔，1s/30fps

        # 设置子线程池函数
        def process_frame(detector, image):
            detector.detect_marker(image)
            (marker_ids, r_vecs, t_vecs) = detector.estimate_pose()
            return detector.draw_marker_axis(), marker_ids, r_vecs, t_vecs

        # 当相机有信号时
        with ThreadPoolExecutor(max_workers=2) as executor:
            while(self.vis.hasData[0] and self.vis.hasData[1]):
                # [last_time]=======[now] less-> wait until
                # [last_time] + time_interval|
                # [last_time]==================[now] more-> trigger immediately and set [last_time] to now
                last_time = last_time + time_interval
                sleep_time = last_time - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    last_time = time.time()
                # 取图
                image_down = self.vis.Img[0]
                image_front = self.vis.Img[1]

                down_future = executor.submit(process_frame, self.aruco_detection_down, image_down)
                front_future = executor.submit(process_frame, self.aruco_detection_front, image_front)

                image_down, maker_ids_down, r_vecs_down, t_vecs_down = down_future.result()
                image_front, maker_ids_front, r_vecs_front, t_vecs_front = front_future.result()
                cv2.imshow("Camera-Down", image_down)
                print(maker_ids_down, r_vecs_down, t_vecs_down)
                cv2.waitKey(1)
                cv2.imshow("Camera-Front", image_front)
                print(maker_ids_front, r_vecs_front, t_vecs_front)
                cv2.waitKey(1)

    def load_camera_config(self) -> None:
        '''
        加载相机相对于无人机质心的位置和姿态
        处理位置和姿态角为旋转矩阵和转移向量
        
        :param self: 说明
        '''
        try:
            with open(str(Path(CONFIG_PATH, "Config.json"))) as f:
                data_raw = json.load(f)
                cameras_config = data_raw["VisionSensors"]
                for camera in cameras_config:
                    if camera["SeqID"] == 0:
                        self.camera_down_config["Pose"] = camera["SensorPosXYZ"]
                        self.camera_down_config["t_vec"] = camera["SensorPosXYZ"]
                        self.camera_down_config["Attitude"] = camera["SensorAngEular"]
                    elif camera["SeqID"] == 1:
                        self.camera_front_config["Pose"] = camera["SensorPosXYZ"]
                        self.camera_front_config["t_vec"] = camera["SensorPosXYZ"]
                        self.camera_front_config["Attitude"] = camera["SensorAngEular"]
        except:
            print("Can not load json file!")

    def fusion_loop(self, analysis_output_dir: str | None = None, max_frames: int | None = None) -> None:
        last_time = time.time()
        time_interval = 1.0 / 30.0
        frame_index = 0
        self.marker_history = {"down": [], "front": []}

        try:
            with ThreadPoolExecutor(max_workers=2) as executor:
                while(self.vis.hasData[0] and self.vis.hasData[1]):
                    last_time = last_time + time_interval
                    sleep_time = last_time - time.time()
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                    else:
                        last_time = time.time()

                    image_down = self.vis.Img[0]
                    image_front = self.vis.Img[1]

                    down_future = executor.submit(self.process_frame, self.aruco_detection_down, 0, image_down)
                    front_future = executor.submit(self.process_frame, self.aruco_detection_front, 1, image_front)

                    markers_down = down_future.result()
                    markers_front = front_future.result()
                    markers = markers_down + markers_front
                    fused_marker = self.pose_fusion.pose_fusion(markers)
                    fused_marker["id"] = 99
                    fused_marker["error"] = 0
                    timestamp = time.time()

                    self.collect_marker_snapshot("down", frame_index, timestamp, markers_down)
                    self.collect_marker_snapshot("front", frame_index, timestamp, markers_front)
                    self.collect_marker_snapshot("fused", frame_index, timestamp, [fused_marker])
                    

                    frame_index += 1
                    if max_frames is not None and frame_index >= max_frames:
                        break
        finally:
            if analysis_output_dir is not None:
                self.export_marker_analysis(analysis_output_dir)

    def process_frame(self, detector: Aruco_Detection, sensor_id: int, frame: np.ndarray) -> list:
        '''
        [取图]->[处理图像]->[识别]->[获取字段表]->[变换]->[输出变换后的字段表]

        输入：detector Aruco_Detection()类 frame 一帧图像
        输出：字段列表
        '''
        # [取图]传入参数frame
        # [处理图像]
        # TODO:预处理，以加速线程
        # [识别]
        detector.detect_marker(frame)
        # PnP解算
        detector.estimate_pose()
        # 计算重映射误差
        detector.calculate_reprojection_error()
        # [获取字段表]
        markers = detector.pack()
        # [刚体变换]
        result = []
        for marker in markers:
            marker_res = self.coordinate_transformation.transform(marker, sensor_id)
            result.append(marker_res)
        # [输出变换后字段表列表]
        return result

    def collect_marker_snapshot(self, camera_name: str, frame_index: int, timestamp: float, markers: list[dict]) -> None:
        records = self.marker_history.setdefault(camera_name, [])
        for marker in markers:
            t_vec = np.asarray(marker["t_vec"], dtype=float).reshape(3)
            roll, pitch, yaw = rotmat_to_euler(np.asarray(marker["rot_mat"], dtype=float), degree=True)
            records.append({
                "frame": frame_index,
                "timestamp": timestamp,
                "marker_id": int(marker["id"]),
                "error": float(marker["error"]),
                "x": float(t_vec[0]),
                "y": float(t_vec[1]),
                "z": float(t_vec[2]),
                "roll": float(roll),
                "pitch": float(pitch),
                "yaw": float(yaw),
            })

    def export_marker_analysis(self, output_dir: str) -> None:
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        self.save_marker_history_csv(output_path)
        self.plot_marker_history(output_path)

    def save_marker_history_csv(self, output_dir: Path) -> None:
        fieldnames = ["frame", "timestamp", "marker_id", "error", "x", "y", "z", "roll", "pitch", "yaw"]
        for camera_name, records in self.marker_history.items():
            if not records:
                continue
            csv_path = output_dir / f"markers_{camera_name}.csv"
            with csv_path.open("w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(records)

    def plot_marker_history(self, output_dir: Path) -> None:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("[WARN] matplotlib not installed, skip marker plots.")
            return

        for camera_name, records in self.marker_history.items():
            if not records:
                continue

            marker_ids = sorted({record["marker_id"] for record in records})
            fig, axes = plt.subplots(4, 1, figsize=(12, 14), sharex=True)
            axis_keys = ["x", "y", "z"]
            axis_labels = ["X Position", "Y Position", "Z Position"]

            for marker_id in marker_ids:
                marker_records = [record for record in records if record["marker_id"] == marker_id]
                frames = [record["frame"] for record in marker_records]
                for idx, key in enumerate(axis_keys):
                    axes[idx].plot(frames, [record[key] for record in marker_records], label=f"id={marker_id}")
                axes[3].plot(frames, [record["error"] for record in marker_records], label=f"id={marker_id}")

            for idx, label in enumerate(axis_labels):
                axes[idx].set_ylabel(label)
                axes[idx].grid(True, linestyle="--", alpha=0.4)
            axes[3].set_ylabel("Reproj Error")
            axes[3].set_xlabel("Frame")
            axes[3].grid(True, linestyle="--", alpha=0.4)

            handles, labels = axes[0].get_legend_handles_labels()
            if handles:
                axes[0].legend(handles, labels, loc="upper right")

            fig.suptitle(f"Marker History - {camera_name}")
            fig.tight_layout()
            fig.savefig(output_dir / f"markers_{camera_name}.png", dpi=200)
            plt.close(fig)
