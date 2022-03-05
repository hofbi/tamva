# Library unittests
catkin_add_gtest(${PROJECT_NAME}_unittests main.cpp roi_filter_mask_test.cpp)

target_link_libraries(${PROJECT_NAME}_unittests view_roi_filter)

# ROS Tests

# Roslaunch Checks
roslaunch_add_file_check(${PROJECT_SOURCE_DIR}/launch DEPENDENCIES
                         view_roi_filter_node IGNORE_UNSET_ARGS)
