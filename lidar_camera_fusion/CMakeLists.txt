cmake_minimum_required(VERSION 2.8.12)
project(lidar_camera_fusion)

add_compile_options(-std=c++11)

#set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev)
set(OpenCV_DIR /usr/share/OpenCV)

find_package(catkin REQUIRED COMPONENTS
  OpenCV
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  roscpp
  tf
  semantic_msg
  std_msgs
)


# ROS Kinetic OpenCV-3.3.1
include_directories(${OpenCV_INCLUDE_DIRS})

# PCL
#find_package(PCL REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES ${PROJECT_NAME} semantic_msg
  CATKIN_DEPENDS cv_bridge image_transport pcl_conversions pcl_ros roscpp tf
)

# https://answers.ros.org/question/237183/the-library-is-neither-a-target-nor-builtinstalled-properly/

add_executable(lidar_camera_fusion
  src/lidar_camera_fusion.cpp
  src/main.cpp)

# 添加新的编译输出，后面也需要链接头文件和库文件，否则会提示找不到文件
add_executable(lidar_camera_fusion_no_msg
  src/lidar_camera_fusion_no_msg.cpp)

add_executable(lidar_camera_fusion_no_msg_sem
  src/lidar_camera_fusion_no_msg_sem.cpp)

target_include_directories(lidar_camera_fusion PRIVATE
  ${catkin_INCLUDE_DIRS}
  include)

target_include_directories(lidar_camera_fusion_no_msg PRIVATE
  ${catkin_INCLUDE_DIRS}
  include)

target_include_directories(lidar_camera_fusion_no_msg_sem PRIVATE
  ${catkin_INCLUDE_DIRS}
  include)


target_link_libraries(lidar_camera_fusion
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

target_link_libraries(lidar_camera_fusion_no_msg
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

target_link_libraries(lidar_camera_fusion_no_msg_sem
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}) 


add_dependencies(lidar_camera_fusion semantic_msg_gencpp)