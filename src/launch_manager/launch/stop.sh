#!/usr/bin/env bash

cd ~/anon_auton_ws
source devel/setup.bash

rosnode kill -a
sleep 5

killall -9 roscore
killall -9 rosmaster

sleep 5
