cmake_minimum_required(VERSION 3.0.2)
project(pcl_euclidean_cluster)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_euclidean_cluster
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs pcl_ros pcl_conversions
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cluster_extractor src/cluster_extractor.cpp)
add_dependencies(cluster_extractor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cluster_extractor ${catkin_LIBRARIES} ${PCL_LIBRARIES} -lpcl_filters)

add_executable(cluster_finder src/cluster_finder.cpp)
target_link_libraries(cluster_finder ${catkin_LIBRARIES} ${PCL_LIBRARIES} -lpcl_filters)

add_executable(conditional_cluster_finder src/conditional_cluster_finder.cpp)
target_link_libraries(conditional_cluster_finder ${catkin_LIBRARIES} ${PCL_LIBRARIES} -lpcl_filters)

