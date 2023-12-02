#!/usr/bin/env bash

cd ~/anon_auton_ws
source devel/setup.bash

echo "stop" > /tmp/handwritten-input

./src/launch_manager/launch/kill_ros.sh
./src/launch_manager/launch/pre.sh &

sleep 5 

./src/launch_manager/scripts/chair_on_startup.bash &
