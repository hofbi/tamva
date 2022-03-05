cmake_minimum_required(VERSION 3.0.2)
project(view_roi_filter)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge roslaunch
                                        rosbag carla_msgs)
find_package(OpenCV REQUIRED)

catkin_package()

add_library(view_roi_filter src/roi_filter_mask.cpp include/roi_filter_mask.h)

target_link_libraries(view_roi_filter PUBLIC ${catkin_LIBRARIES}
                                             ${OpenCV_LIBRARIES})

target_include_directories(
  view_roi_filter
  PRIVATE include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
  INTERFACE include ${catkin_INCLUDE_DIRS})

target_compile_features(view_roi_filter PUBLIC cxx_std_14)

add_executable(view_roi_filter_node src/roi_filter_mask_node.cpp)
target_link_libraries(view_roi_filter_node PRIVATE view_roi_filter)

add_subdirectory(test)
