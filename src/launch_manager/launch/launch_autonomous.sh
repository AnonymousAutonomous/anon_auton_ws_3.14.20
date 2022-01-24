#!/usr/bin/env bash

cd ~/anon_auton_ws
# cd /Users/felichri/Documents/AnonAuton/anon_auton_ws
source devel/setup.bash

/bin/bash ~/anon_auton_ws/src/launch_manager/launch/pre.sh

sleep 5

cd ~/anon_auton_ws/src/launch_manager/launch/components

if grep -iq "^antenna_port" ~/anon_auton_ws/src/config_manager/configs/ports/active.yaml; then
    echo "ANTENNA found"
    roslaunch antenna_chair.launch
else
    echo "!!! NO ANTENNA FOUND !!!"
fi

if grep -iq "^port" ~/anon_auton_ws/src/config_manager/configs/ports/active.yaml; then
    echo "ARDUINO found"
    roslaunch arduino.launch
else
    echo "!!! NO ARDUINO FOUND !!!"
fi

if grep -iq "^device_path" ~/anon_auton_ws/src/config_manager/configs/ports/active.yaml; then
    echo "CAMERA found"
    roslaunch camera.launch  
else
    echo "!!! NO CAMERA FOUND !!!"
fi

if grep -iq "^serial_port" ~/anon_auton_ws/src/config_manager/configs/ports/active.yaml; then
    echo "LIDAR found"
    roslaunch lidar.launch
else
    echo "!!! NO LIDAR FOUND !!!"
fi

# Launch the queue to start everything
roslaunch queue.launch
