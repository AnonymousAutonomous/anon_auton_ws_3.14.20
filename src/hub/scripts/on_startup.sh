#!/usr/bin/env bash

cd ~/anon_auton_ws
source devel/setup.bash

./src/launch_manager/launch/pre.sh

until rostopic list; do sleep 1; done

if grep -iq "^antenna_port" ~/anon_auton_ws/src/config_manager/configs/ports/active.yaml; then
	echo "----------------- ANTENNA FOUND -----------------"
	notify-send "Antenna is connected."
	roslaunch --wait src/launch_manager/launch/components/antenna_hub.launch &
else
	notify-send -u critical -t 0 "!!! PLEASE CONNECT THE ANTENNA !!!"
	until grep -iq "^antenna_port" ~/anon_auton_ws/src/config_manager/configs/ports/active.yaml;
	do
		./src/launch_manager/launch/pre.sh
		sleep 5
	done
	notify-send "Antenna is connected."
	roslaunch --wait src/launch_manager/launch/components/antenna_hub.launch &
fi
