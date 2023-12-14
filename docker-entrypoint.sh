#!/bin/bash
set -e
source /root/.bashrc
source "/opt/ros/noetic/setup.bash"             #<--- TODO: change to your ROS version
# CONTAINER_INITIALIZED="CONTAINER_INITIALIZED_PLACEHOLDER"

CONTAINER_INITIALIZED="CONTAINER_INITIALIZED_PLACEHOLDER"
if [ ! -e $CONTAINER_INITIALIZED ]; then
    touch $CONTAINER_INITIALIZED
    echo "-- First container startup --"
    catkin init
    #catkin config
    catkin build
    source "/ws/devel/setup.bash"
else
    echo "-- Not first container startup --"
    source "/ws/devel/setup.bash"
fi


exec "$@"