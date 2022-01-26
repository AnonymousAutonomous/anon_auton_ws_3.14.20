#!/usr/bin/env bash

cd ~/anon_auton_ws
source devel/setup.bash

./src/launch_manager/roscore.sh
./src/launch_manager/set_ports.sh
