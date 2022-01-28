#!/usr/bin/env bash

cd ~/anon_auton_ws
source devel/setup.bash

amixer cset numid=3 1

./src/launch_manager/launch/roscore.sh
./src/launch_manager/launch/set_ports.sh
