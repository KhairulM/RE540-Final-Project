#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/ubuntu/armpi_pro/devel/setup.bash
export ROS_HOSTNAME=192.168.10.11
export ROS_MASTER_URI=http://192.168.10.11:11311

roslaunch /home/ubuntu/armpi_pro/src/armpi_pro_bringup/launch/start_sensor.launch
