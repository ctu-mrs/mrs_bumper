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

add_definitions("-fdiagnostics-color=always -O3 -g -Wall")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED)

generate_dynamic_reconfigure_options(
  config/Bumper.cfg
  )

catkin_package(
  CATKIN_DEPENDS mrs_msgs message_runtime # image_transport roscpp rospy std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
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
