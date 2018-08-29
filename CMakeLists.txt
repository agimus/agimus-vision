cmake_minimum_required(VERSION 2.8.3)
project(agimus_vision)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  message_generation
  cv_bridge
  visp_bridge
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
)

find_package(OpenCV REQUIRED)

find_package(VISP REQUIRED)

find_package(LibXml2 REQUIRED)


add_service_files(
  FILES
  EmptyService.srv
  AddAprilTagService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${LibXml2_INCLUDE_DIRS}
)

add_subdirectory(src/tracker_object)
##add_subdirectory(src/extrinsic_calibration)

## Mark cpp header files for installation
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY objects
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

