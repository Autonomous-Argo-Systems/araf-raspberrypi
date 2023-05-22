cd /home/araf/araf-raspberrypi
echo "Starting launch.."
rosnode kill -a
source /home/araf/.bashrc
source /home/araf/araf-raspberrypi/devel/setup.bash
nohup roslaunch robotcontrol araf.launch
