#!/usr/bin/env python

"""Prioritize individual camera views"""

from pathlib import Path

import rospy
from carla_msgs.msg import CarlaEgoVehicleStatus

from view_adaptation_msgs.msg import Bandwidth, ViewPriority
from view_prioritization.parser import SensorConfigParser
from view_prioritization.prioritization import ViewPrioritizationModel


class ViewPrioritization:
    """Prioritize individual camera views"""

    def __init__(self):
        rospy.loginfo("Starting view prioritization...")
        role_name = rospy.get_param("~role_name", "ego_vehicle")
        sensor_definition_file = Path(rospy.get_param("~sensor_definition_file"))

        if not sensor_definition_file.is_file():
            rospy.logerr(f"Cannot find sensor definition file {sensor_definition_file}")
            rospy.signal_shutdown(
                f"Cannot find sensor definition file {sensor_definition_file}"
            )

        config_parser = SensorConfigParser(sensor_definition_file, role_name)
        camera_views = config_parser.parse_camera_views()

        self.__prio_model = ViewPrioritizationModel(camera_views)

        self.__camera_prio_publisher_dict = {
            camera_view.camera_id: rospy.Publisher(
                f"/carla/{role_name}/{camera_view.camera_id}/prioritization",
                ViewPriority,
                queue_size=1,
            )
            for camera_view in camera_views
        }
        self.__available_transmission_rate_kbits = 3000  # Initial guess
        rospy.Subscriber(
            f"/carla/{role_name}/vehicle_status",
            CarlaEgoVehicleStatus,
            self.vehicle_status_callback,
        )
        rospy.Subscriber(
            "/bandwidth",
            Bandwidth,
            self.bandwidth_callback,
        )
        rospy.loginfo("View prioritization running")

    def vehicle_status_callback(self, vehicle_status: CarlaEgoVehicleStatus):
        camera_weights = self.__prio_model.calculate_camera_prioritization(
            vehicle_status
        )
        for camera_view, camera_weight in camera_weights.items():
            prioritization = ViewPriority(
                camera_weight,
                int(self.__available_transmission_rate_kbits * camera_weight),
            )
            self.__camera_prio_publisher_dict[camera_view].publish(prioritization)

    def bandwidth_callback(self, bandwidth: Bandwidth):
        self.__available_transmission_rate_kbits = bandwidth.bandwidth_bps / 1000


def main():
    """Run the view prioritization"""
    rospy.init_node("view_prioritization", anonymous=True)
    ViewPrioritization()
    rospy.spin()


if __name__ == "__main__":
    main()
