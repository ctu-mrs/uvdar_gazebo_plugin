cmake_minimum_required(VERSION 3.1.2 FATAL_ERROR)
project(uvdar_gazebo_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-diagnostics-color -fmessage-length=0 -std=c++17 -Wno-return-type -Wno-unused-result -Wno-unused-variable -Wno-maybe-uninitialized -g")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -fno-diagnostics-color -fmessage-length=0 -std=c++17 -Wno-return-type -Wno-unused-result -Wno-unused-variable -Wno-maybe-uninitialized -g")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf
  message_generation
  mrs_msgs
  uvdar_core
)

add_message_files(DIRECTORY msg FILES
  LedInfo.msg
  LedMessage.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  uvdar_core
  )

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs message_runtime uvdar_core
   LIBRARIES uvdar_led uvdar_cam
)


find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)
find_package(OpenCV REQUIRED HINTS /usr/local/lib)
include_directories(${GAZEBO_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include)
link_directories(${GAZEBO_LIBRARY_DIRS} ${OPENCV_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(uvdar_led SHARED src/uvled.cc)
target_link_libraries(uvdar_led ${GAZEBO_LIBRARIES})

# add_library(collisions SHARED include/collisions/collision_kernel.cpp  include/collisions/collision_space.cpp)

add_library(uvdar_cam SHARED src/uvcam.cc include/undistortFunctions/ocam_functions.cpp include/ObjectMgr/ObjectMgr.cpp include/ObjectMgr/LedMgr.cpp)
target_link_libraries(uvdar_cam ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES} CameraPlugin)
add_dependencies(uvdar_cam ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(uvdar_led ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# install(TARGETS uvdar_cam uvdar_led
#         LIBRARY DESTINATION $ENV{HOME}/gazebo_local_plugins/)
