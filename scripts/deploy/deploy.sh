#!/bin/bash
echo "ARAF Deployment Script"

# copying files
echo "Starting clone..."
rsync ./ araf@araf.local:/home/araf/araf-raspberrypi -r -v --delete --chown=araf:araf --progress --exclude=.git --delete-excluded --exclude-from=.gitignore --copy-unsafe-links

# calling build script
ssh araf@araf.local 'bash -s < /home/araf/araf-raspberrypi/scripts/deploy/compile.sh'

exit 0