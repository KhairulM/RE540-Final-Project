#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/ubuntu/armpi_pro/devel/setup.bash
export ROS_HOSTNAME=192.168.10.11
export ROS_MASTER_URI=http://192.168.0.10:11311

sudo ip route add 192.168.0.0/24 via 192.168.10.10
sleep 5
roslaunch /home/ubuntu/armpi_pro/src/armpi_pro_bringup/launch/start_dependence.launch &
sleep 10
roslaunch /home/ubuntu/armpi_pro/src/armpi_pro_bringup/launch/start_camera.launch &
sleep 10
sudo /home/ubuntu/armpi_pro/src/armpi_pro_bringup/scripts/start_sensor_node.sh &
sleep 10
roslaunch /home/ubuntu/armpi_pro/src/hiwonder_servo_controllers/launch/start.launch &
sleep 10
roslaunch /home/ubuntu/armpi_pro/src/armpi_pro_bringup/launch/start_functions.launch
