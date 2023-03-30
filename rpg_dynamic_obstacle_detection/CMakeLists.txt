cmake_minimum_required(VERSION 2.8.3)
project(rpg_dynamic_obstacle_detection)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message("Opencv Version ${OpenCV_INCLUDE_DIRS}")
#find_package(Armadillo REQUIRED)
#include_directories(${ARMADILLO_INCLUDE_DIRS})

#find_library(MLPACK_LIB mlpack)
#include_directories(${MLPACK_INCLUDE_DIRS})

find_package(OpenMP)
include_directories(${OpenMP_INCLUDE_DIRS})

SET(CMAKE_BUILD_TYPE RelWithDebInfo) # Release, RelWithDebInfo
SET(CMAKE_CXX_FLAGS "-std=c++14 -Wall -fPIC -pedantic -fopenmp -mtune=native")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(
  -std=c++14
  -Wall 
  -Wno-ignored-qualifiers
  -Wno-sign-compare
  -Wno-unknown-pragmas
  -Wno-pedantic
  -fPIC #-pedantic 
  -fopenmp 
  -mtune=native
  -O3
)

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON) 
set(Boost_USE_STATIC_RUNTIME ON)


##################################################

set(HEADERS
  include/rpg_dynamic_obstacle_detection/dvs_handler.hpp
  include/rpg_dynamic_obstacle_detection/imu_warp.hpp
  include/rpg_dynamic_obstacle_detection/normalized_mean_timestamp_image.hpp
  include/rpg_dynamic_obstacle_detection/dynamic_obstacle_detection.hpp
  include/rpg_dynamic_obstacle_detection/optical_flow.hpp
  include/rpg_dynamic_obstacle_detection/sort_vectors.hpp
  include/rpg_dynamic_obstacle_detection/common.hpp
  include/rpg_dynamic_obstacle_detection/average_timer.hpp
  include/rpg_dynamic_obstacle_detection/ellipsoid.hpp
  include/rpg_dynamic_obstacle_detection/image2world.hpp
  include/rpg_dynamic_obstacle_detection/clustering.hpp
)

set(SOURCES
  src/dvs_handler.cpp
  src/imu_warp.cpp
  src/normalized_mean_timestamp_image.cpp
  src/dynamic_obstacle_detection.cpp
  src/optical_flow.cpp
  src/clustering.cpp
  src/ellipsoid.cpp
  src/image2world.cpp
)

cs_add_library(rpg_dynamic_obstacle_detection ${SOURCES} ${HEADERS})

set(LIBRARIES
  ${OpenCV_LIBRARIES}
  ${rpg_dynamic_obstacle_detection_LIBRARIES}
  #${MLPACK_LIB}
  ${OpenMP_LINRARIES}
)

##################################################

#cs_add_executable(test_main src/main/main.cpp)
#cs_add_executable(dbscan_stack src/dbscan_stack.cpp)

#target_link_libraries(test_main ${OpenCV_LIBRARIES})
#target_link_libraries(dbscan_stack ${OpenCV_LIBRARIES})

#cs_add_executable(optitrack_camera_calibration src/optitrack_camera_calibration.cpp)
#target_link_libraries(optitrack_camera_calibration ${LIBRARIES})

cs_add_executable(dynamic_obstacle_detection src/dynamic_obstacle_detection.cpp)
target_link_libraries(dynamic_obstacle_detection ${LIBRARIES})

#cs_add_executable(step src/step.cpp)
#target_link_libraries(step ${LIBRARIES})

cs_add_executable(optical_flow_test src/tests/optical_flow_test.cpp)
target_link_libraries(optical_flow_test ${LIBRARIES})

cs_add_executable(ellipsoid_test src/tests/ellipsoid_test.cpp)
target_link_libraries(ellipsoid_test ${LIBRARIES})

cs_add_executable(testing src/tests/testing.cpp)
target_link_libraries(testing ${LIBRARIES})

cs_install()
cs_export()
