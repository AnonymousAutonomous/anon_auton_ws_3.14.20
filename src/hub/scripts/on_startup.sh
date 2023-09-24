#!/usr/bin/env bash

cd ~/anon_auton_ws
source devel/setup.bash


python2 ~/anon_auton_ws/src/config_manager/scripts/set_ports.py

killall -9 roscore
killall -9 rosmaster

roscore &

until rostopic list; do sleep 1; done

if grep -iq "^antenna_port" ~/anon_auton_ws/src/config_manager/configs/ports/active.yaml; then
	echo "----------------- ANTENNA FOUND -----------------"
	notify-send "Antenna is connected."
	roslaunch --wait src/robot_gui_bridge/launch/websocket.launch &
	roslaunch --wait src/launch_manager/launch/components/antenna_hub.launch &
else
	notify-send -u critical -t 0 "!!! PLEASE CONNECT THE ANTENNA !!!"
	until grep -iq "^antenna_port" ~/anon_auton_ws/src/config_manager/configs/ports/active.yaml;
	do
		./src/launch_manager/launch/set_ports.sh
		sleep 5
	done
	notify-send "Antenna is connected."
	roslaunch --wait src/robot_gui_bridge/launch/websocket.launch &
	roslaunch --wait src/launch_manager/launch/components/antenna_hub.launch &
fi
