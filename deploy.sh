#!/bin/bash
echo "ARAF Deployment Script"
echo "Starting clone..."

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