#!/usr/bin/env bash

cd ~/anon_auton_ws
source devel/setup.bash

./src/launch_manager/launch/pre.sh

until rostopic list; do sleep 1; done

if grep -iq "^antenna_port" ~/anon_auton_ws/src/config_manager/configs/ports/active.yaml; then
	echo "----------------- ANTENNA FOUND -----------------"
	notify-send "Antenna is connected."
	roslaunch --wait src/launch_manager/launch/components/antenna_chair.launch &
else
	notify-send -u critical -t 0 "!!! PLEASE CONNECT THE ANTENNA !!!"
	until grep -iq "^antenna_port" ~/anon_auton_ws/src/config_manager/configs/ports/active.yaml;
	do
		./src/launch_manager/launch/set_ports.sh
		sleep 5
	done
	notify-send "Antenna is connected."
	roslaunch --wait src/launch_manager/launch/components/antenna_chair.launch &
fi


until rostopic list | grep 'chair_transmitter'; do sleep 1; done
    rostopic pub -1 /from_chair std_msgs/String "chair registered"
