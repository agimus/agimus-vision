add_executable(${PROJECT_NAME}_camera_calibration main.cpp node.cpp extrinsic_calibration.cpp)
add_dependencies(${PROJECT_NAME}_camera_calibration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_options(${PROJECT_NAME}_camera_calibration -std=c++11)
target_include_directories(${PROJECT_NAME}_camera_calibration SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_camera_calibration
  ${catkin_LIBRARIES}
)
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME}_camera_calibration visp)
set_target_properties(${PROJECT_NAME}_camera_calibration PROPERTIES OUTPUT_NAME camera_calibration PREFIX "")


install(TARGETS ${PROJECT_NAME}_camera_calibration
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

