#!/bin/bash

# compilation of the code
echo "Starting compilation.."

source /opt/ros/noetic/setup.bash
source /home/araf/dependencies/devel/setup.bash

cd /home/araf/araf-raspberrypi 

/opt/ros/noetic/bin/catkin_make

# launch of the code
echo "Starting launch.."
sudo systemctl restart araf

# succesfully flashed (-;
echo "Tried deployement!"