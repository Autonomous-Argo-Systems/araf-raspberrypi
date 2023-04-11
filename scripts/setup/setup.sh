#!/bin/bash
cd
git clone https://github.com/naoki-mizuno/ds4drv --branch devel
cd ds4drv
mkdir -p ~/.local/lib/python3.8/site-packages
python3 setup.py install --prefix ~/.local

# Note: udev directory is in the ds4drv repo, not ds4_driver (this repo)
sudo cp udev/50-ds4drv.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

cd 
cd araf-raspberrypi/src/ds4_driver
git checkout noetic-devel

