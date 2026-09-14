cmake_minimum_required(VERSION 2.8.3)
project(dianyunceshi)
set(CMAKE_BUILD_TYPE DEBUG)
set(SRC_LIST main.cpp)
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  pcl_conversions
  sensor_msgs
  rospy
  std_msgs
  message_generation
  cv_bridge
  actionlib
  actionlib_msgs
)



add_message_files(
  FILES
  posev.msg
  posevs.msg
  # Message2.msg
) 
add_action_files(
  FILES
  tomato_pose.action
)

generate_messages(
   DEPENDENCIES
   std_msgs 
   sensor_msgs
   actionlib_msgs)


find_package(PCL REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
catkin_package()






catkin_install_python(PROGRAMS
  scripts/biye_zuizhong_zuixin.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
