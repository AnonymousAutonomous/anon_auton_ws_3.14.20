#!/usr/bin/env bash

cd ~/anon_auton_ws
source devel/setup.bash

rm -rf /tmp/handwritten-input
mkfifo /tmp/handwritten-input
cat > /tmp/handwritten-input &
echo $! > /tmp/handwritten-input-pid

tail -f /tmp/handwritten-input | ./src/launch_manager/launch/launch_handwritten_ros.sh &
