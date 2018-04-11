cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)
find_package(OpenCV REQUIRED HINTS /usr/local/lib)
include_directories(${GAZEBO_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} include)
link_directories(${GAZEBO_LIBRARY_DIRS} ${OPENCV_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(uvled SHARED src/uvled.cc)
target_link_libraries(uvled ${GAZEBO_LIBRARIES})

add_library(uvcam SHARED src/uvcam.cc include/undistortFunctions/ocam_functions.cpp)
target_link_libraries(uvcam ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES})

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/libuvled.so"
  "${CMAKE_CURRENT_BINARY_DIR}/libuvcam.so"
  DESTINATION $ENV{GAZEBO_PLUGIN_PATH}
  )



