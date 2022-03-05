add_library(
  model
  src/mda.cpp include/mda.h include/optimal_fps_estimator.h
  src/optimal_fps_estimator.cpp include/optimal_spatial_scale_estimator.h
  src/optimal_spatial_scale_estimator.cpp)

target_include_directories(
  model
  PRIVATE include
  INTERFACE include)

target_link_libraries(model PUBLIC vqm)

target_compile_features(model PRIVATE cxx_std_17)
