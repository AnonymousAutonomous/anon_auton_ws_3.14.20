#!/usr/bin/env bash

cd ~/anon_auton_ws
source devel/setup.bash

# Start roscore if it's not already running
roscore &
/bin/bash ~/anon_auton_ws/src/launch_manager/launch/set_ports.sh

# wait until roscore is running
until rostopic list ; do sleep 1; done

sleep 5

roslaunch ~/anon_auton_ws/src/launch_manager/launch/components/antenna_chair.launch
