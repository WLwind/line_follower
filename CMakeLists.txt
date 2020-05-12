cmake_minimum_required(VERSION 2.8.3)
project(line_follower)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
)
find_package(OpenCV)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES line_follower
  CATKIN_DEPENDS cv_bridge geometry_msgs roscpp rospy std_msgs sensor_msgs image_transport
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
message("OpenCV dir: ${OpenCV_DIR}; OpenCV include: ${OpenCV_INCLUDE_DIRS}; catkin include: ${catkin_INCLUDE_DIRS}")

add_library(LineDetect
  src/LineDetect.cpp
)
add_dependencies(LineDetect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(LineDetect
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_executable(detect
  src/detect.cpp
)
add_dependencies(detect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} LineDetect)
target_link_libraries(detect LineDetect
  ${catkin_LIBRARIES}
)
add_library(VelCmdRobot
  src/VelCmdRobot.cpp
)
add_dependencies(VelCmdRobot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(VelCmdRobot
  ${catkin_LIBRARIES}
)
add_executable(navig
  src/motion_node.cpp
)
add_dependencies(navig ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} VelCmdRobot)
target_link_libraries(navig VelCmdRobot
  ${catkin_LIBRARIES}
)
