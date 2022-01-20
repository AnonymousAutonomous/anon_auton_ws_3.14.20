#!/usr/bin/env bash

cd ~/anon_auton_ws
source devel/setup.bash

python2 ~/anon_auton_ws/src/config_manager/scripts/set_ports.py

killall -9 roscore
killall -9 rosmaster

roscore &
