cd /home/araf/araf-raspberrypi
echo "Starting launch.."
rosnode kill -a
source /home/araf/.bashrc
source /home/araf/araf-raspberrypi/devel/setup.bash
sudo ifconfig eth0 192.168.3.100
sudo route add 192.168.1.201 eth0
nohup roslaunch robotcontrol araf.launch
