#!/bin/bash

# setting up dependencies
echo "Setting up enviroment..."
sudo apt update
sudo apt install python3 -y
sudo apt install python3-pip -y
sudo apt install libserial-dev -y
pip install ds4drv
pip install pynmea2

# Installing rosserial-python
echo "Setting up rosserial-python..."
sudo apt install ros-noetic-rosserial ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame -y
curl https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | sudo bash

# Installing mavproxy
pip3 install PyYAML mavproxy
echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc

# creating workspace
cd ..
echo "Setting up workspace..."
mkdir dependencies
cd dependencies
mkdir src
catkin_make
echo 'source '$PWD'/devel/setup.bash'>> ~/.bashrc

# downloading Lidar (Velodyne) driver
cd src
git clone "https://github.com/ros-drivers/velodyne.git"
cd ..
rosdep install --from-paths src --ignore-src --rosdistro noetic -y

# downloading DS4 (PS4 controller) driver 
cd src
git clone "https://github.com/naoki-mizuno/ds4_driver.git"
cd ds4_driver
git checkout noetic-devel
cd ../..

# building drivers
echo "Building driver"
catkin_make

# echo done
echo "Setup process is done!"
echo "Please source ~/.bashrc"








