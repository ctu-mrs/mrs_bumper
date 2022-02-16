cmake_minimum_required(VERSION 3.1.2)
project(mrs_bumper)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  image_geometry
  pcl_ros
  pcl_conversions
  mrs_lib
  mrs_msgs
  )

# add_definitions("-fdiagnostics-color=always -O0 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options("-Wall")
add_compile_options("-Wextra")
add_compile_options(${PCL_COMPILE_OPTIONS})

find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED COMPONENTS)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

generate_dynamic_reconfigure_options(
  config/Bumper.cfg
  )

set(EXECUTABLES
  histogram_displayer
  )

set(LIBRARIES
  Bumper
  )

catkin_package(
  CATKIN_DEPENDS mrs_msgs pcl_ros message_runtime # image_transport roscpp rospy std_msgs
  LIBRARIES ${LIBRARIES}
  )

## --------------------------------------------------------------
## |                            Build                           |
## --------------------------------------------------------------

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  # ~/git/darknet_ocl/src
  )

# bumper

add_library(Bumper
  src/bumper_nodelet.cpp
  )

add_dependencies(Bumper
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(Bumper
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  )

# histogram displayer

add_executable(histogram_displayer
  src/histogram_displayer.cpp
  )

add_dependencies(histogram_displayer
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(histogram_displayer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(TARGETS ${EXECUTABLES}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
  )
