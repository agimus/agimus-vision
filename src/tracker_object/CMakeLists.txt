add_executable(${PROJECT_NAME}_tracker_object main.cpp node.cpp detector.cpp detector_apriltag.cpp detector_chessboard.cpp) 
add_dependencies(${PROJECT_NAME}_tracker_object ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(${PROJECT_NAME}_tracker_object SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_tracker_object
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  zbar # Necessary when the detectors are not used via pointers.
)

message(${OpenCV_LIBS})

set_target_properties(${PROJECT_NAME}_tracker_object PROPERTIES OUTPUT_NAME tracker_object PREFIX "")


install(TARGETS ${PROJECT_NAME}_tracker_object
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

