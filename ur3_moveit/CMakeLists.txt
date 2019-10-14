cmake_minimum_required(VERSION 3.1.3)
project(ur3_moveit)

find_package(catkin REQUIRED COMPONENTS
  moveit_msgs
  moveit_ros_planning
  moveit_ros_warehouse
  moveit_ros_manipulation
  moveit_ros_move_group
  geometry_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  eigenpy
  roscpp
  actionlib
  rospy
  rosconsole
  std_msgs
)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
