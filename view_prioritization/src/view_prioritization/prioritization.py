"""View Prioritization Module"""

from typing import Dict, List

from carla_msgs.msg import CarlaEgoVehicleStatus

from view_prioritization.parser import CameraViewParameter


class ViewPrioritizationModel:
    """Camera View Prioritization Module"""

    MAX_STEERING_ANGLE_DEGREE = 60

    def __init__(self, camera_views: List[CameraViewParameter]):
        self.__camera_views = camera_views

    @staticmethod
    def get_camera_weight(orientation: float, velocity: float) -> int:
        if abs(orientation) < 45:
            speed_influence_weight = int(max(0.0, 0.1 * velocity))
            return 4 + speed_influence_weight
        if abs(orientation) < 90:
            return 2
        return 1

    @staticmethod
    def steer_to_angle(steer: float) -> float:
        """
        Positive steering value results in a negative orientation.
        """
        return ViewPrioritizationModel.MAX_STEERING_ANGLE_DEGREE * -steer

    @staticmethod
    def get_camera_orientation(orientation: float, steer: float) -> float:
        return orientation - ViewPrioritizationModel.steer_to_angle(steer)

    @staticmethod
    def get_orientation_from_car_direction(orientation: float, reverse: bool) -> float:
        if reverse:
            orientation = -orientation + 180
            orientation = orientation % 360 if orientation >= 360 else orientation
        return orientation

    def calculate_camera_prioritization(
        self, vehicle_status: CarlaEgoVehicleStatus
    ) -> Dict[str, float]:
        camera_orientations = [
            self.get_camera_orientation(
                self.get_orientation_from_car_direction(
                    camera_view.orientation, vehicle_status.control.reverse
                ),
                vehicle_status.control.steer,
            )
            for camera_view in self.__camera_views
        ]
        camera_weights = [
            self.get_camera_weight(orientation, vehicle_status.velocity)
            for orientation in camera_orientations
        ]
        normalized_wights = [weight / sum(camera_weights) for weight in camera_weights]
        return {
            camera_view.camera_id: weight
            for camera_view, weight in zip(self.__camera_views, normalized_wights)
        }
