cmake_minimum_required(VERSION 3.8)
project(vpt_cameras)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_library(FREENECT_LIB freenect REQUIRED)
find_library(FREENECT_SYNC_LIB freenect_sync REQUIRED)

add_executable(kinect_face_node
  src/kinect_face_node.cpp
)

ament_target_dependencies(kinect_face_node rclcpp sensor_msgs)
target_link_libraries(kinect_face_node ${FREENECT_LIB} ${FREENECT_SYNC_LIB})

install(TARGETS kinect_face_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
