#!/bin/bash
echo "ARAF Deployment Script"

# copying files
echo "Starting clone..."
rsync ./ araf@araf.local:/home/araf/araf-raspberrypi -r -v --delete --chown=araf:araf --progress --exclude=.git --delete-excluded

# compilation of the code
echo "Starting compilation.."
catkin_make
if ["$?" -eq "0"]; then
  echo "Compilation succesfull!"
else; then
  echo "Compilation failed!"
  exit 1
fi

# deployment fo the code
echo "Starting deployment.."
# TODO roslaunch

# succesfully flashed (-;
echo "Succesfully deployed!"
exit 0