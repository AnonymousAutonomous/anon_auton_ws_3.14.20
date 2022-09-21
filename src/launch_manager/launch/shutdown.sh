#!/usr/bin/env bash

cd ~/anon_auton_ws
source devel/setup.bash

echo "stop" > /tmp/handwritten-input
./src/launch_manager/launch/kill_ros.sh

shutdown