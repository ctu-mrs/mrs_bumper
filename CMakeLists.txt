cmake_minimum_required(VERSION 2.8.3)
project(mrs_bumper)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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

add_definitions("-fdiagnostics-color=always -O0 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# we really need to replace the flags because
# we cannot build this with march=native
set(CMAKE_CXX_FLAGS "")

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

catkin_package(
  CATKIN_DEPENDS mrs_msgs pcl_ros message_runtime # image_transport roscpp rospy std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  # ~/git/darknet_ocl/src
)

add_library(Bumper src/bumper_nodelet.cpp)
add_dependencies(Bumper
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(Bumper
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(histogram_displayer src/histogram_displayer.cpp)
add_dependencies(histogram_displayer
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(histogram_displayer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
