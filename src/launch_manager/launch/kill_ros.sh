#!/usr/bin/env bash

cd ~/anon_auton_ws
source devel/setup.bash

rostopic pub -1 /shutdown_ros std_msgs/Empty

rosnode kill -a &
sleep 5

rosnode list | while read -r nodeid ; do
    kill $(ps aux | grep $nodeid | grep -v grep | awk '{print $2}')
done

killall -9 roscore
killall -9 rosmaster
