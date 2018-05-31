cmake_minimum_required(VERSION 2.8.3)
project(agimus-vision)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  cv_bridge
  visp_bridge
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
)

find_package(VISP REQUIRED)

find_package(LibXml2 REQUIRED)


catkin_package()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${LibXml2_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp src/node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  zbar # Necessary when the detectors are not used via pointers.
)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME tracker_box PREFIX "")


install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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
