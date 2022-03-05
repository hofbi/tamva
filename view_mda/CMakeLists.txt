cmake_minimum_required(VERSION 3.0)
project(view_mda)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             dynamic_reconfigure
             sensor_msgs
             cv_bridge
             roslaunch
             view_adaptation_msgs
             telecarla_msgs
             gstreaming)

catkin_package(DEPENDS dynamic_reconfigure)

add_subdirectory(source)

add_subdirectory(test)
