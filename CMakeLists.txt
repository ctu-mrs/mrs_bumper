cmake_minimum_required(VERSION 2.8.3)
project(mrs_bumper)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  sensor_msgs
  cv_bridge
  image_transport
  image_geometry
  mrs_lib
  mrs_msgs
)

# set(CMAKE_CXX_FLAGS "")
# set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DEIGEN_INITIALIZE_MATRICES_BY_NAN -DDISABLE_PARALLELIZATION -DDEBUG_PARAMS -DDEBUG")
# set(CMAKE_CXX_FLAGS "-g -O0 -DEIGEN_INITIALIZE_MATRICES_BY_NAN -DDISABLE_PARALLELIZATION -DDEBUG_PARAMS -DDEBUG")
# set(CMAKE_CXX_FLAGS_RELEASE "-Werror -O3")
# add_compile_options("-DDEBUG_MEDIAN_FILTER" "-O0" "-g")

find_package(OpenCV REQUIRED)

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
