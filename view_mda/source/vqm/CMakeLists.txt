find_package(OpenCV REQUIRED)

add_library(
  vqm
  src/frame_history.cpp
  include/frame_history.h
  src/mdvqm.cpp
  include/mdvqm.h
  include/mdvqm_param.h
  include/pixel_bitrate.h
  src/pixel_bitrate.cpp)

target_include_directories(
  vqm
  PRIVATE include ${OpenCV_INCLUDE_DIRS}
  INTERFACE include ${OpenCV_INCLUDE_DIRS})

target_link_libraries(vqm PUBLIC ${OpenCV_LIBRARIES})

target_compile_features(vqm PRIVATE cxx_std_17)
