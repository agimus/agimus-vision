cmake_minimum_required(VERSION 2.8.3)

SET(PROJECT_NAME agimus_vision)
SET(PROJECT_DESCRIPTION "Computer vision for Agimus")
SET(PROJECT_URL https://github.com/agimus/agimus-vision)
SET(PROJECT_VERSION 0.1.0)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

SET(CXX_DISABLE_WERROR TRUE)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cxx11.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

OPTION(COMPILE_AND_INSTALL_BINARIES "Set this to OFF to install only message and service files" ON)

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
  image_transport
  dynamic_reconfigure
)

find_package(OpenCV REQUIRED)

ADD_REQUIRED_DEPENDENCY("visp >= 3.2.0")

add_message_files(
  FILES
  ImageDetectionResult.msg
  AprilTag.msg
)

add_service_files(
  FILES
  AddAprilTagService.srv
  AddObjectTracking.srv
  SetChessboardService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Tracker.cfg
)

catkin_package()

if(COMPILE_AND_INSTALL_BINARIES)
  add_subdirectory(src/tracker_object)
  ##add_subdirectory(src/extrinsic_calibration)
  #add_subdirectory(tests)

  SET(${PROJECT_NAME}_HEADERS
    include/agimus_vision/extrinsic_calibration/extrinsic_calibration.hpp
    include/agimus_vision/extrinsic_calibration/node.hpp

    include/agimus_vision/tracker_object/detector.hpp
    include/agimus_vision/tracker_object/detector_apriltag.hpp
    include/agimus_vision/tracker_object/detector_chessboard.hpp
    include/agimus_vision/tracker_object/node.hpp
    )

  catkin_install_python(PROGRAMS scripts/simulate.py scripts/calibrate_real_joint.py scripts/publish_real_joint.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  find_package(pybind11 CONFIG)
  if(pybind11_FOUND)
    pybind11_add_module(py_agimus_vision src/python.cpp)
    target_link_libraries(py_agimus_vision PRIVATE ${PROJECT_NAME}_tracker_object_lib)

    if(NOT DEFINED PYTHON_SITELIB)
      execute_process(
        COMMAND "${PYTHON_EXECUTABLE}" "-c"
        #"import sys, os; print(os.sep.join(['lib', 'python' + sys.version[:3], 'dist-packages']))"
        "from distutils import sysconfig; print(sysconfig.get_python_lib(prefix='', plat_specific=False))"
        OUTPUT_VARIABLE PYTHON_SITELIB
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
    endif()

    message(STATUS "Installing python module to ${PYTHON_SITELIB}")
    install(TARGETS py_agimus_vision
      DESTINATION ${PYTHON_SITELIB}/agimus_vision)
  endif()
endif(COMPILE_AND_INSTALL_BINARIES)

install(DIRECTORY objects launch config data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)