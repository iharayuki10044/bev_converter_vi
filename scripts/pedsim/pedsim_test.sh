#!/bin/bash -x

ssh reydeoro 'export DISPLAY=:1.0; export ROS_MASTER_URI=http://192.168.0.201:11311; source /opt/ros/melodic/setup.bash; source ${HOME}/ros_catkin_ws/devel/setup.bash; roslaunch pedsim_teb pedsim_teb.launch'
