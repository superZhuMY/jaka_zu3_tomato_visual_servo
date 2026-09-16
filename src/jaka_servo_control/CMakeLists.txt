cmake_minimum_required(VERSION 3.0.2)
project(jaka_servo_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  jaka_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs std_srvs jaka_msgs
)

catkin_install_python(PROGRAMS
  scripts/servoj_bridge.py
  scripts/servoj_sine_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
