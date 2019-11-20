cmake_minimum_required(VERSION 2.8.3)

SET(PROJECT_NAME agimus_vision)
SET(PROJECT_DESCRIPTION "Computer vision for Agimus")
SET(PROJECT_URL https://github.com/agimus/agimus-vision)

SET(CXX_DISABLE_WERROR TRUE)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cxx11.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

## Compile as C++11, supported in ROS Kinetic and newer
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(NOT COMPILER_SUPPORTS_CXX11)
  message(FATAL "This project needs support for C++ 11.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  message_generation
  cv_bridge
  visp_bridge
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

find_package(OpenCV REQUIRED)

ADD_REQUIRED_DEPENDENCY(visp)

add_service_files(
  FILES
  AddAprilTagService.srv
  SetChessboardService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package()

add_subdirectory(src/tracker_object)
##add_subdirectory(src/extrinsic_calibration)

SET(${PROJECT_NAME}_HEADERS
  include/agimus_vision/extrinsic_calibration/extrinsic_calibration.hpp
  include/agimus_vision/extrinsic_calibration/node.hpp

  include/agimus_vision/tracker_object/detector.hpp
  include/agimus_vision/tracker_object/detector_apriltag.hpp
  include/agimus_vision/tracker_object/detector_chessboard.hpp
  include/agimus_vision/tracker_object/node.hpp
  )

install(DIRECTORY objects
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/simulate.py scripts/calibrate_real_joint.py scripts/publish_real_joint.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
