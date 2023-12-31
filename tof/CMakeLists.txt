cmake_minimum_required(VERSION 3.8)
project(tof)
set(CMAKE_VEBOSE_MAKEFILE TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)   
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hand_msgs REQUIRED) 
# find_package(PCL REQUIRED)
# CORE_INCLUDE_PATHS = "/uld-driver/inc"
# PLATFORM_INCLUDE_PATHS = "/uld-driver/platform"

include_directories("src/uld-driver/inc")
include_directories("src/uld-driver/platform")
include_directories("src/functions")
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})
# add_custom_target(tof
#                   /usr/bin/make -f "${CMAKE_CURRENT_SOURCE_DIR}/src/Makefile")

# add_executable(talker src/kyle_cpp_test.cpp
# src/uld-driver/src/vl53l7cx_api.c
# src/uld-driver/src/vl53l7cx_plugin_detection_thresholds.c
# src/uld-driver/src/vl53l7cx_plugin_motion_indicator.c
# src/uld-driver/src/vl53l7cx_plugin_xtalk.c
# src/uld-driver/platform/platform.c
# )
# ament_target_dependencies(talker rclcpp std_msgs hand_msgs)
add_executable(tof_publisher src/tof_publisher.cpp

src/functions/PointCalcs.cpp
src/functions/sensor_bringup.cpp
src/uld-driver/src/vl53l7cx_api.c
src/uld-driver/src/vl53l7cx_plugin_detection_thresholds.c
src/uld-driver/src/vl53l7cx_plugin_motion_indicator.c
src/uld-driver/src/vl53l7cx_plugin_xtalk.c
src/uld-driver/platform/platform.c
)

ament_target_dependencies(tof_publisher rclcpp std_msgs sensor_msgs hand_msgs)
install(TARGETS
  # talker
  tof_publisher
  DESTINATION lib/${PROJECT_NAME})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
