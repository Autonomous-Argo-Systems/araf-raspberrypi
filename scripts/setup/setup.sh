#!/bin/bash

# setting up ds4drv
echo "Setting up enviroment..."
sudo apt install python3 -y
sudo apt install python3-pip -y
pip install ds4drv
pip install pynmea2

# Installing rosserial-python
echo "Setting up rosserial-python..."
sudo apt install ros-noetic-rosserial ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs -y
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh 

# creating workspace
cd ..
echo "Setting up workspace..."
mkdir dependencies
cd dependencies
mkdir src
catkin_make
echo 'source '$PWD'/devel/setup.bash'>> ~/.bashrc

# downloading ds4_driver 
cd src
git clone "https://github.com/naoki-mizuno/ds4_driver.git"
cd ds4_driver
git checkout noetic-devel
cd ..

# building driver
echo "Building driver"
catkin_make

# echo done
echo "Setup process is done!"
echo "Please source ~/.bashrc"








