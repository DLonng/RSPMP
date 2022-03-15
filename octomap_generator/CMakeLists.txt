cmake_minimum_required(VERSION 2.8)
project(octomap_generator)

add_compile_options(-std=c++11)

# SET OCTOMAP_OMP to disable/enable OPENMP (experimental for octomap, default disabled)
#SET(OCTOMAP_OMP TRUE CACHE BOOL "Enable/disable OpenMP parallelization")
#IF(DEFINED ENV{OCTOMAP_OMP})
#  SET(OCTOMAP_OMP $ENV{OCTOMAP_OMP})
#ENDIF(DEFINED ENV{OCTOMAP_OMP})
#IF(OCTOMAP_OMP)
#  FIND_PACKAGE( OpenMP REQUIRED)
#  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#ENDIF(OCTOMAP_OMP)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


set(PACKAGE_DEPENDENCIES
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  nav_msgs
  std_msgs
  std_srvs
  octomap_ros
  octomap_msgs
  nodelet
  scout_msgs
  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(octomap REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS OCTOMAP
)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(octomap_generator
 src/octomap_generator/octomap_generator.cpp
 src/octomap_generator/octomap_generator_ros.cpp
 src/semantic_octree/semantics_bayesian.cpp
 src/semantic_octree/semantics_max.cpp
)

target_link_libraries(octomap_generator ${LINK_LIBS})

add_dependencies(octomap_generator scout_msgs_gencpp)
