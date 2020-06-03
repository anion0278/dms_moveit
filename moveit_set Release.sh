#! /bin/bash

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/ws_moveit
catkin clean 
ccache –C 
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
