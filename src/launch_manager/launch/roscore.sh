#!/usr/bin/env bash

cd ~/anon_auton_ws
source devel/setup.bash

killall -9 roscore
killall -9 rosmaster

roscore &
