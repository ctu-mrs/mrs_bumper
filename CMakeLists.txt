cmake_minimum_required(VERSION 3.5)
project(mrs_bumper)

set(CATKIN_DEPENDENCIES
  cmake_modules
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_geometry
  image_transport
  message_runtime
  mrs_lib
  mrs_msgs
  nodelet
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

# add_definitions("-fdiagnostics-color=always -O0 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options("-Wall")
add_compile_options("-Wextra")
add_compile_options(${PCL_COMPILE_OPTIONS})

# remove warning info from PCL
set(PCL_FIND_QUIETLY INTERNAL)

find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED COMPONENTS)

generate_dynamic_reconfigure_options(
  config/Bumper.cfg
  )

set(EXECUTABLES
  histogram_displayer
  )

set(LIBRARIES
  MrsBumper_Bumper
  )

catkin_package(
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
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

add_library(MrsBumper_Bumper
  src/bumper_nodelet.cpp
  )

add_dependencies(MrsBumper_Bumper
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsBumper_Bumper
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

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY tmux/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tmux
  )
