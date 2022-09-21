#!/usr/bin/env bash

cd ~/anon_auton_ws
source devel/setup.bash

./src/launch_manager/launch/kill_ros.sh
./src/launch_manager/launch/pre.sh &

./src/launch_manager/launch/launch_antenna_chair.sh &
