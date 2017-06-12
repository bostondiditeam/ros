cmake_minimum_required(VERSION 2.8.3)
project(sensor_fusion)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
  pcl_conversions
  pcl_ros
  message_filters
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES sensor_fusion
 CATKIN_DEPENDS 
 roscpp 
 rospy 
 sensor_msgs 
 std_msgs 
 image_transport 
 cv_bridge 
 pcl_conversions
 pcl_ros
 message_filters
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(image_pointcloud_sync src/image_pointcloud_sync.cpp)
target_link_libraries(image_pointcloud_sync 
                        ${catkin_LIBRARIES}
                        ${OpenCV_LIBRARIES}
                        )

