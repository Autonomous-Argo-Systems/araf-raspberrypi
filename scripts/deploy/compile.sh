#!/bin/bash

# compilation of the code
echo "Starting compilation.."

source /opt/ros/noetic/setup.bash
source /home/araf/ds4_driver/devel/setup.bash

cd /home/araf/araf-raspberrypi 

/opt/ros/noetic/bin/catkin_make

# launch of the code
echo "Starting launch.."
rosnode list | grep -v rosout | xargs rosnode kill
echo araf | sudo -S systemctl stop my_robot_ros.service
echo araf | sudo -S systemctl start my_robot_ros.service

# succesfully flashed (-;
echo "Succesfully deployed!"