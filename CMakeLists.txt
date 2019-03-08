cmake_minimum_required(VERSION 2.8.3)
project(mrs_bumper)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  message_generation
  sensor_msgs
  cv_bridge
  image_transport
  image_geometry
  mrs_lib
)

set(CMAKE_CXX_FLAGS "")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DEIGEN_INITIALIZE_MATRICES_BY_NAN -DDISABLE_PARALLELIZATION -DDEBUG_PARAMS")
set(CMAKE_CXX_FLAGS_RELEASE "-Werror -O3")

add_compile_options("-std=c++17" "-Wall")

find_package(OpenCV REQUIRED)

add_message_files(DIRECTORY msgs FILES
  ObstacleSectors.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  config/BumperConfig.cfg
  )

catkin_package(
  CATKIN_DEPENDS message_runtime # image_transport roscpp rospy std_msgs
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
