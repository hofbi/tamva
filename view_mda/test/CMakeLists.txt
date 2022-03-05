configure_file(${CMAKE_CURRENT_SOURCE_DIR}/resources/akiyo-0.png
               ${CMAKE_CURRENT_BINARY_DIR}/vqm/resources/akiyo-0.png COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/resources/akiyo-1.png
               ${CMAKE_CURRENT_BINARY_DIR}/vqm/resources/akiyo-1.png COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/resources/mobile-0.png
               ${CMAKE_CURRENT_BINARY_DIR}/vqm/resources/mobile-0.png COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/resources/mobile-1.png
               ${CMAKE_CURRENT_BINARY_DIR}/vqm/resources/mobile-1.png COPYONLY)

# Library unittests
catkin_add_gmock(
  ${PROJECT_NAME}_unittests main.cpp simple_test.cpp vqm/frame_history_test.cpp
  vqm/mdvqm_test.cpp model/mda_test.cpp)

target_include_directories(${PROJECT_NAME}_unittests PRIVATE include)

target_link_libraries(${PROJECT_NAME}_unittests model vqm)

# ROS Tests

# Roslaunch Checks
roslaunch_add_file_check(${PROJECT_SOURCE_DIR}/launch DEPENDENCIES
                         view_mda_node)
