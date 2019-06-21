cmake_minimum_required(VERSION 2.8.3)

INCLUDE(cmake/base.cmake)

SET(CXX_DISABLE_WERROR TRUE)

# These variables have to be defined before running SETUP_PROJECT
SET(PROJECT_NAME agimus_vision)
SET(PROJECT_DESCRIPTION "Computer vision for Agimus")
SET(PROJECT_URL https://github.com/agimus/agimus-vision)

SETUP_PROJECT()

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

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
  tf
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

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
)

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

catkin_install_python(PROGRAMS scripts/simulate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

SETUP_PROJECT_FINALIZE()
