#!/bin/bash

source /opt/ros/noetic/setup.bash
source /home/rocksat/catkin_ws/devel/setup.bash

# TODO: Elias put that one command that you have to do before launching ros here:
sudo chmod g+rw /dev/ttyTHS1

# TODO: Elias verify that this is the right launch file
roslaunch daedalus_core mission.launch
