"""File Parser Test"""

import json
import unittest
from pathlib import Path
from typing import List

from pyfakefs.fake_filesystem_unittest import TestCase

from view_prioritization.parser import CameraViewParameter, SensorConfigParser


def create_valid_sensor_config(camera_views: List[str], vehicle_id: str) -> str:
    """Create test sensor configs for the given camera views"""
    return json.dumps(
        {
            "objects": [
                {
                    "id": vehicle_id,
                    "sensors": [
                        {
                            "type": "sensor.camera.rgb",
                            "id": view,
                            "spawn_point": {"yaw": 0.0},
                        }
                        for view in camera_views
                    ],
                }
            ]
        }
    )


class SensorConfigParserTest(TestCase):
    """Sensor config parser test"""

    def setUp(self) -> None:
        self.setUpPyfakefs()

    def test_parse_camera_views__with_empty_json__should_raise_key_error(self):
        self.fs.create_file("test.json", contents="{}")
        with self.assertRaises(KeyError):
            SensorConfigParser(Path("test.json"), "hero")

    def test_parse_camera_views__with_single_camera_config__should_be_one_camera(self):
        self.fs.create_file(
            "test.json", contents=create_valid_sensor_config(["front"], "hero")
        )
        unit = SensorConfigParser(Path("test.json"), "hero")
        self.assertListEqual([CameraViewParameter("front")], unit.parse_camera_views())

    def test_parse_camera_views__with_two_cameras_in_config__should_be_two_cameras(
        self,
    ):
        self.fs.create_file(
            "test.json", contents=create_valid_sensor_config(["front", "rear"], "hero")
        )
        unit = SensorConfigParser(Path("test.json"), "hero")
        self.assertListEqual(
            [CameraViewParameter("front"), CameraViewParameter("rear")],
            unit.parse_camera_views(),
        )


class ParserTestSuite(unittest.TestSuite):
    """Parser Test"""

    def __init__(self):
        super().__init__()
        self.addTest(SensorConfigParserTest())
