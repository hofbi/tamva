"""View Prioritization Test"""

import unittest

from carla_msgs.msg import CarlaEgoVehicleStatus

from view_prioritization.parser import CameraViewParameter
from view_prioritization.prioritization import ViewPrioritizationModel


class ViewPrioritizationModelTest(unittest.TestCase):
    """View Prioritization Model Test"""

    def test_calculate_camera_prioritization__with_one_camera__should_be_max(self):
        unit = ViewPrioritizationModel([CameraViewParameter("single")])
        self.assertDictEqual(
            {"single": 1.0},
            unit.calculate_camera_prioritization(CarlaEgoVehicleStatus()),
        )

    def test_calculate_camera_prioritization__with_three_cameras__should_be_one_in_total(
        self,
    ):
        unit = ViewPrioritizationModel(
            [
                CameraViewParameter("one"),
                CameraViewParameter("two"),
                CameraViewParameter("three"),
            ]
        )
        camera_prios = unit.calculate_camera_prioritization(CarlaEgoVehicleStatus())

        self.assertListEqual(["one", "two", "three"], list(camera_prios.keys()))
        self.assertEqual(1.0, sum(camera_prios.values()))

    def test_calculate_camera_prioritization__with_one_front_and_one_rear_camera__should_be_not_equally_distributed(
        self,
    ):
        unit = ViewPrioritizationModel(
            [
                CameraViewParameter("front", 0),
                CameraViewParameter("rear", 180),
            ]
        )
        camera_prio = unit.calculate_camera_prioritization(CarlaEgoVehicleStatus())
        self.assertGreater(camera_prio["front"], camera_prio["rear"])

    def test_calculate_camera_prioritization__with_two_front_cameras__should_be_equally_distributed(
        self,
    ):
        unit = ViewPrioritizationModel(
            [
                CameraViewParameter("front", 0),
                CameraViewParameter("front2", 0),
            ]
        )
        camera_prio = unit.calculate_camera_prioritization(CarlaEgoVehicleStatus())
        self.assertEqual(camera_prio["front"], camera_prio["front2"])

    def test_calculate_camera_prioritization__with_two_cameras__should_be_more_weight_for_higher_speed(
        self,
    ):
        unit = ViewPrioritizationModel(
            [CameraViewParameter("front", 0), CameraViewParameter("rear", 180)]
        )
        status_high_speed = CarlaEgoVehicleStatus()
        status_high_speed.velocity = 10.0

        camera_prio_low_speed = unit.calculate_camera_prioritization(
            CarlaEgoVehicleStatus()
        )
        camera_prio_high_speed = unit.calculate_camera_prioritization(status_high_speed)

        self.assertGreater(
            camera_prio_high_speed["front"], camera_prio_low_speed["front"]
        )

    def test_calculate_camera_prioritization__with_two_close_cameras_and_steering__should_be_different_after_steering(
        self,
    ):
        unit = ViewPrioritizationModel(
            [
                CameraViewParameter("front_left", 50),
                CameraViewParameter("front_right", -40),
            ]
        )
        camera_prio = unit.calculate_camera_prioritization(CarlaEgoVehicleStatus())
        self.assertGreater(camera_prio["front_right"], camera_prio["front_left"])

        status = CarlaEgoVehicleStatus()
        status.control.steer = -1  # steer max to the left
        camera_prio = unit.calculate_camera_prioritization(status)
        self.assertGreater(camera_prio["front_left"], camera_prio["front_right"])

    def test_steer_to_angle__for_steer_zero__angle_should_be_zero(self):
        angle = ViewPrioritizationModel.steer_to_angle(0.0)
        self.assertEqual(0.0, angle)

    def test_steer_to_angle__for_steer_one__angle_should_be_max_negative(self):
        angle = ViewPrioritizationModel.steer_to_angle(1.0)
        self.assertEqual(-60.0, angle)

    def test_steer_to_angle__for_steer_minus_one__angle_should_be_max_positive(self):
        angle = ViewPrioritizationModel.steer_to_angle(-1.0)
        self.assertEqual(60.0, angle)

    def test_get_camera_orientation__for_steer_zero__orientation_should_be_same(self):
        orientation = ViewPrioritizationModel.get_camera_orientation(30, 0.0)
        self.assertEqual(30, orientation)

    def test_get_camera_orientation__for_steer_negative__orientation_should_be_smaller(
        self,
    ):
        orientation = ViewPrioritizationModel.get_camera_orientation(30, -0.1)
        self.assertGreater(30, orientation)

    def test_get_camera_orientation__for_steer_positive__orientation_should_be_larger(
        self,
    ):
        orientation = ViewPrioritizationModel.get_camera_orientation(30, 0.1)
        self.assertGreater(orientation, 30)

    def test_calculate_camera_prioritization__while_reversing_with__rear_and__front_cameras__rear_should_be_prioritized(
        self,
    ):
        unit = ViewPrioritizationModel(
            [
                CameraViewParameter("front", 0),
                CameraViewParameter("rear", 180),
            ]
        )
        status = CarlaEgoVehicleStatus()
        status.control.reverse = True
        camera_prio = unit.calculate_camera_prioritization(status)
        self.assertGreater(camera_prio["rear"], camera_prio["front"])


class PrioritizationTestSuite(unittest.TestSuite):
    """View Test"""

    def __init__(self):
        super().__init__()
        self.addTest(ViewPrioritizationModelTest())
