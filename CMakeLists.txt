cmake_minimum_required(VERSION 3.8)
project(uvdar_gazebo_plugin)

# Force CMake to look in the ROS 2 installation paths
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/jazzy")

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(mrs_msgs REQUIRED)
find_package(uvdar_core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-physics7 REQUIRED)

find_package(rosidl_default_generators REQUIRED)


# Generate ROS 2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LedInfo.msg"
  "msg/CamInfo.msg"
  "msg/LedMessage.msg"
  "srv/SetLedMessage.srv"
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${gz-sim8_INCLUDE_DIRS}
  ${rosidl_generated_headers_INCLUDE_DIRS}
)
# Link directories
link_directories(
  ${OpenCV_LIBRARY_DIRS}
)

# --------------------------------------------------------------
# |                          Libraries                         |
# --------------------------------------------------------------

## | ------------------------ uvdar_led ----------------------- |
add_library(uvdar_led SHARED
  src/uvled.cc
)

ament_target_dependencies(uvdar_led
  rclcpp
  std_msgs
  ament_index_cpp
  mrs_msgs
  gz-sim8
)

rosidl_get_typesupport_target(cpp_typesupport_target
  "${PROJECT_NAME}" "rosidl_typesupport_cpp")

target_link_libraries(uvdar_led
  "${cpp_typesupport_target}"
)



## | ------------------------ uvdar_occl_check ----------------------- |
add_library(uvdar_occl_check SHARED
  src/uvcam_ray.cc
  include/undistortFunctions/ocam_functions.cpp
)

ament_target_dependencies(uvdar_occl_check
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  ament_index_cpp
  mrs_msgs
  uvdar_core
  gz-sim8
  gz-physics7
)

target_link_libraries(uvdar_occl_check
    "${cpp_typesupport_target}"
    ${OpenCV_LIBRARIES}
)

# --------------------------------------------------------------
# |                           Testing                          |
# --------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# --------------------------------------------------------------
# |                           Install                          |
# --------------------------------------------------------------

install(TARGETS
  uvdar_led
  uvdar_occl_check
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY msg srv config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_export_include_directories(include)
ament_export_libraries(uvdar_led uvdar_occl_check)
ament_export_dependencies(
  rclcpp
  ament_index_cpp
  std_msgs
  mrs_msgs
  uvdar_core
  gz-sim8
  gz-physics7
  OpenCV
  rosidl_default_runtime
)

ament_package()
