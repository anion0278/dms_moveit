#! /bin/bash

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

#roslaunch ur3_moveit start_rviz.launch
#rviz
rosrun rviz rviz -d `rospack find ur3_moveit`/launch/dms_rviz.rviz
