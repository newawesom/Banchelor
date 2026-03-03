import numpy as np
import json
from pathlib import Path

PATH = Path.cwd()
CONFIG_PATH = Path(PATH, "config")

class Coordinate_transformation():
    def __init__(self) -> None:
        self.vision_sensors = []
        self.markers = []

    def parse_config(self, sensors_path:str, markers_path:str) -> None:
        with open(sensors_path, 'r', encoding='utf-8') as f:
            sensors_data = json.load(f)
            self.vision_sensors = sensors_data["VisionSensors"]

        with open(markers_path, 'r', encoding='utf-8') as f:
            markers_data = json.load(f)
            self.markers = markers_data["Markers"]
