cmake_minimum_required(VERSION 3.1.2 FATAL_ERROR)
project(uvdar_gazebo_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall)
add_definitions(-Wextra)

set(CATKIN_DEPENDENCIES
  cv_bridge
  image_transport
  message_generation
  message_runtime
  mrs_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  uvdar_core
  gazebo_plugins
  gazebo_ros
  mrs_uav_testing
  )

set(LIBRARIES
  uvdar_led
  uvdar_cam
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
)

add_message_files(DIRECTORY msg FILES
  LedInfo.msg
  CamInfo.msg
  LedMessage.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  uvdar_core
  )

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
   LIBRARIES ${LIBRARIES}
)

find_package(gazebo REQUIRED)

ign_import_target(UUID)

find_package(roscpp REQUIRED)

find_package(OpenCV REQUIRED HINTS /usr/local/lib)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
  )

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${OPENCV_LIBRARY_DIRS}
  )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

## | ------------------------ uvdar_led ----------------------- |

add_library(uvdar_led SHARED
  src/uvled.cc
  )

target_link_libraries(uvdar_led
  ${GAZEBO_LIBRARIES}
  )

add_dependencies(uvdar_led
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## | ------------------------ uvdar cam ----------------------- |

add_library(uvdar_cam SHARED
  src/uvcam.cc
  include/undistortFunctions/ocam_functions.cpp
  include/ObjectMgr/ObjectMgr.cpp
  include/ObjectMgr/LedMgr.cpp
  )

target_link_libraries(uvdar_cam
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${OpenCV_LIBRARIES}
  CameraPlugin
  )

add_dependencies(uvdar_cam
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## --------------------------------------------------------------
## |                           Testing                          |
## --------------------------------------------------------------

if(CATKIN_ENABLE_TESTING AND MRS_ENABLE_TESTING)

  message(WARNING "Testing enabled.")

  add_subdirectory(test)

endif()

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
