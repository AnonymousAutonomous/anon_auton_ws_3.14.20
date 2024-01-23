#!/usr/bin/env bash

cd ~/anon_auton_ws
source devel/setup.bash

killall -9 roscore
killall -9 rosmaster

roscore &

until rostopic list; do sleep 1; done

rosparam set enable_statistics true

