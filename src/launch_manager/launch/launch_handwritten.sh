#!/usr/bin/env bash

cd ~/anon_auton_ws
source devel/setup.bash

# Start roscore if it's not already running
roscore &
/bin/bash ~/anon_auton_ws/src/launch_manager/launch/set_ports.sh

# wait until roscore is running
until rostopic list ; do sleep 1; done

rm -rf /tmp/handwritten-input
mkfifo /tmp/handwritten-input
cat > /tmp/handwritten-input &
echo $! > /tmp/handwritten-input-pid

tail -f /tmp/handwritten-input | ./src/launch_manager/launch/launch_handwritten_ros.sh &
