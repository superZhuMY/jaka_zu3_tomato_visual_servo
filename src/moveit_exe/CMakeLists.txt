cmake_minimum_required(VERSION 3.0.2)
project(moveit_exe)
SET(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  message_generation
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  sensor_msgs
  moveit_visual_tools
  cv_bridge
  image_transport
  roslib
  tf
)
find_package(Eigen3 REQUIRED)  # 查找Eigen3
include_directories(${EIGEN3_INCLUDE_DIR})  # 添加Eigen的包含目录



catkin_package(
   INCLUDE_DIRS include
   #LIBRARIES robot_jaka_acs
   #CATKIN_DEPENDS moveit_core moveit_ros_perception moveit_ros_planning_interface moveit_visual_tools roscpp rospy rviz_visual_tools
#  DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


