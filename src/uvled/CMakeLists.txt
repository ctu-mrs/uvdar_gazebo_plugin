cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(uvled)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf
)
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES uvled uvcam
   catkin_depends cv_bridge image_transport roscpp sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
#   DEPENDS Eigen
)
find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)
find_package(OpenCV REQUIRED HINTS /usr/local/lib)
include_directories(${GAZEBO_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include)
link_directories(${GAZEBO_LIBRARY_DIRS} ${OPENCV_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(uvled SHARED src/uvled.cc)
target_link_libraries(uvled ${GAZEBO_LIBRARIES})

add_library(uvcam SHARED src/uvcam.cc include/undistortFunctions/ocam_functions.cpp)
target_link_libraries(uvcam ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS uvcam uvled
        LIBRARY DESTINATION $ENV{HOME}/gazebo_local_plugins/)
