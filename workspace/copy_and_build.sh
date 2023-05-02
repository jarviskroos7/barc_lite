#!/bin/bash

# Change this to where you mounted the directory containing your code
export MOUNT_DIRECTORY="/project_code"
# export MOUNT_DIRECTORY="/Users/jarvis/Library/Mobile Documents/com~apple~CloudDocs/Desktop/Spring 2023/ME236C/barc_lite/project_files/code"

# Copy contents of mounted directory to 
# /barc_lite/workspace/src/mpclab_controllers/mpclab_controlers/lib/mpclab_controllers
cp -r ${MOUNT_DIRECTORY}/* /barc_lite/workspace/src/mpclab_controllers/mpclab_controllers/lib/mpclab_controllers

# Change directory to ROS workspace
cd /barc_lite/workspace

# Delete existing packages
rm -rf build log install

# Rebuild ROS packages
colcon build --symlink-install