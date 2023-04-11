#!/bin/bash

# compilation of the code
echo "Starting compilation.."

source /opt/ros/noetic/setup.bash
source /home/araf/araf-raspberrypi/devel/setup.bash
source /home/araf/ds4_driver/devel/setup.bash

cd /home/araf/araf-raspberrypi 

/opt/ros/noetic/bin/catkin_make

# launch of the code
echo "Starting launch.."
rosnode kill --all
roslaunch robotcontrol araf.launch

# succesfully flashed (-;
echo "Succesfully deployed!"