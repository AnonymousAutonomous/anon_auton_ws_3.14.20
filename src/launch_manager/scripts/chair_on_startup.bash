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


if grep -iq "^port" ~/anon_auton_ws/src/config_manager/configs/ports/active.yaml; then
    echo "---------------- ARDUINO found ----------------"
    if rosnode list | grep "arduino"; then
        echo "Arduino already running"
    else
        roslaunch --wait arduino.launch &
    fi
else
    echo "!!! NO ARDUINO FOUND !!!"
fi

if grep -iq "^device_path" ~/anon_auton_ws/src/config_manager/configs/ports/active.yaml; then
    echo "---------------- CAMERA found ----------------"
    if rosnode list | grep "eyes"; then
        echo "Camera already running"
    else
        roslaunch --wait camera.launch &
    fi
else
    echo "!!! NO CAMERA FOUND !!!"
fi

if grep -iq "^serial_port" ~/anon_auton_ws/src/config_manager/configs/ports/active.yaml; then
    echo "---------------- LIDAR found ----------------"
    if rosnode list | grep "tart"; then
        echo "Lidar already running"
    else
        roslaunch --wait lidar.launch &
    fi
else
    echo "!!! NO LIDAR FOUND !!!"
fi

if rosnode list | grep "queue"; then
    echo "Queue already running"
else
    roslaunch --wait queue.launch &
fi

rm -rf /tmp/handwritten-input
mkfifo /tmp/handwritten-input
cat > /tmp/handwritten-input &
echo $! > /tmp/handwritten-input-pid

tail -f /tmp/handwritten-input | ./src/launch_manager/launch/launch_handwritten_ros.sh &
