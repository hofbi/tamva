add_subdirectory(vqm)
add_subdirectory(model)

# Executables
add_executable(view_mda_node src/view_mda_node.cpp src/view_mda.cpp
                             include/view_mda.h)

target_include_directories(
  view_mda_node
  PRIVATE include ${catkin_INCLUDE_DIRS}
  INTERFACE include ${catkin_INCLUDE_DIRS})

target_link_libraries(view_mda_node PRIVATE model ${catkin_LIBRARIES})
