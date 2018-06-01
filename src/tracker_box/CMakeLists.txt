add_executable(${PROJECT_NAME}_tracker_box main.cpp node.cpp)
add_dependencies(${PROJECT_NAME}_tracker_box ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_tracker_box
  ${catkin_LIBRARIES}
  zbar # Necessary when the detectors are not used via pointers.
)
set_target_properties(${PROJECT_NAME}_tracker_box PROPERTIES OUTPUT_NAME tracker_box PREFIX "")


install(TARGETS ${PROJECT_NAME}_tracker_box
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

