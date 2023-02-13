#!/usr/bin/env bash

cd ~/anon_auton_ws
source devel/setup.bash

/bin/bash ~/anon_auton_ws/src/launch_manager/launch/pre.sh

sleep 5

roslaunch ~/anon_auton_ws/src/robot_gui_bridge/launch/websocket.launch &
roslaunch ~/anon_auton_ws/src/launch_manager/launch/components/antenna_hub.launch
