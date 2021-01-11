#! /bin/bash

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/catking_ws
catkin config  --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build --jobs 5
