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
find_package(mrs_msgs REQUIRED) 
find_package(uvdar_core_msgs REQUIRED) 
find_package(OpenCV REQUIRED)
find_package(gz-sim8 REQUIRED) 
find_package(gz-transport13 REQUIRED) 

find_package(rosidl_default_generators REQUIRED) 

# Generate ROS 2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LedInfo.msg"
  "msg/CamInfo.msg"
  "msg/LedMessage.msg"
  DEPENDENCIES std_msgs uvdar_core_msgs  #std_msgs uvdar_core_msgs mrs_msgs sensor_msgs
)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${gz-sim8_INCLUDE_DIRS}
  ${gz-transport13_INCLUDE_DIRS}
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
  mrs_msgs
  uvdar_core_msgs
  gz-sim8
  gz-transport13
)

rosidl_target_interfaces(uvdar_led
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

target_link_libraries(uvdar_led
  ${OpenCV_LIBRARIES}
)

## | ------------------------ uvdar_cam ----------------------- |
#add_library(uvdar_cam SHARED
#  src/uvcam.cc
#  include/undistortFunctions/ocam_functions.cpp
#  include/ObjectMgr/ObjectMgr.cpp
#  include/ObjectMgr/LedMgr.cpp
#)

#ament_target_dependencies(uvdar_cam
#  rclcpp
#  sensor_msgs
#  mrs_msgs
#  uvdar_core_msgs
#  gz-sim8
#  gz-transport13
#  OpenCV
#)

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
  #  uvdar_cam
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_export_include_directories(include)
#ament_export_libraries(uvdar_led uvdar_cam)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  mrs_msgs
  uvdar_core_msgs
  gz-sim8
  gz-transport13
  OpenCV
  rosidl_default_runtime
)

ament_package()
