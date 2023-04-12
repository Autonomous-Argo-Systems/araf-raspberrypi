#!/bin/bash

# setting up ds4drv
echo "Setting up enviroment..."
sudo apt install python3
sudo apt install python3-pip
pip install ds4drv

# creating workspace
cd ..
echo "Setting up workspace..."
mkdir ds4_driver
cd ds4_driver
mkdir src
catkin_make
echo 'source '$PWD'/devel/setup.bash'>> ~/.bashrc


# downloading driver 
cd src
git clone "https://github.com/naoki-mizuno/ds4_driver.git"
cd ds4_driver
git checkout noetic-devel
cd ../..

# building driver
echo "Building driver"
catkin_make

# echo done
echo "Setup process is done!"
source ~/.bashrc









