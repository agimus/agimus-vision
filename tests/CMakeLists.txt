list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/")
find_package(V4L2 REQUIRED)

add_executable(tracker-apriltag
  tracker-apriltag.cpp)
#PKG_CONFIG_USE_DEPENDENCY(apriltag-detector visp)
target_include_directories(tracker-apriltag SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS} ${V4L2_INCLUDE_DIRS}
)
target_link_libraries(tracker-apriltag
  ${catkin_LIBRARIES} ${V4L2_LIBRARIES} ${PROJECT_NAME}_tracker_object_lib
)
target_compile_options(tracker-apriltag
  PUBLIC -std=c++11)

add_test(tracker-apriltag tracker-apriltag)
