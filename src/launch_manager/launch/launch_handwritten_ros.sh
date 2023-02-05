#!/usr/bin/env bash

cd ~/anon_auton_ws
source devel/setup.bash

roslaunch ~/anon_auton_ws/src/launch_manager/launch/handwritten.launch
