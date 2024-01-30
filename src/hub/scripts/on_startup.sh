#!/usr/bin/env bash

cd ~/anon_auton_ws
source devel/setup.bash

amixer cset numid=1
modprobe snd_bcm2835

rosnode list | while read -r nodeid ; do
    kill $(ps aux | grep $nodeid | grep -v grep | awk '{print $2}')
done

killall -9 roscore
killall -9 rosmaster

python2 ~/anon_auton_ws/src/config_manager/scripts/set_ports.py

# Clear out port we will be using
kill $(lsof -t -i:9090)

roscore &

until rostopic list; do sleep 1; done

rosparam load ~/anon_auton_ws/src/config_manager/configs/hub/active.yaml

if grep -iq "^antenna_port" ~/anon_auton_ws/src/config_manager/configs/ports/active.yaml; then
	echo "----------------- ANTENNA FOUND -----------------"
	notify-send "Antenna is connected."
	roslaunch --wait src/launch_manager/launch/components/antenna_hub.launch &
	roslaunch --wait src/robot_gui_bridge/launch/websocket.launch
else
	notify-send -u critical -t 0 "!!! PLEASE CONNECT THE ANTENNA !!!"
	until grep -iq "^antenna_port" ~/anon_auton_ws/src/config_manager/configs/ports/active.yaml;
	do
		./src/launch_manager/launch/set_ports.sh
		sleep 5
	done
	notify-send "Antenna is connected."
	roslaunch --wait src/launch_manager/launch/components/antenna_hub.launch &
	roslaunch --wait src/robot_gui_bridge/launch/websocket.launch
fi
