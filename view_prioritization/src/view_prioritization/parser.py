"""File Parser Module"""

import json
from dataclasses import dataclass
from pathlib import Path
from typing import List


@dataclass
class CameraViewParameter:
    """Camera View Parameter Model Class"""

    camera_id: str
    orientation: float = 0.0


class SensorConfigParser:
    """Parses the sensor configuration file to get all camera views"""

    def __init__(self, sensor_file: Path, role_name: str):
        objects = json.loads(sensor_file.read_text())["objects"]
        ego_vehicle = [obj for obj in objects if obj["id"] == role_name][0]
        self.__sensor_definition = ego_vehicle["sensors"]

    def parse_camera_views(self) -> List[CameraViewParameter]:
        return [
            CameraViewParameter(
                camera_id=camera["id"], orientation=camera["spawn_point"]["yaw"]
            )
            for camera in self.__sensor_definition
            if camera["type"] == "sensor.camera.rgb"
        ]
