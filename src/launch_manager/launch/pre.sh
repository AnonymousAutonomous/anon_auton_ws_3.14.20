#!/usr/bin/env bash

cd ~/anon_auton_ws
source devel/setup.bash

amixer cset numid=3 1

./src/launch_manager/roscore.sh
./src/launch_manager/set_ports.sh
