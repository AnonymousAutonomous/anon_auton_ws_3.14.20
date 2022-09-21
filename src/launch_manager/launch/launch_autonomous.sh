#!/usr/bin/env bash

cd ~/anon_auton_ws
# cd /Users/felichri/Documents/AnonAuton/anon_auton_ws
source devel/setup.bash

# Start roscore if it's not already running
roscore &
/bin/bash ~/anon_auton_ws/src/launch_manager/launch/set_ports.sh

# wait until roscore is running
until rostopic list ; do sleep 1; done

cd ~/anon_auton_ws/src/launch_manager/launch/components

if grep -iq "^antenna_port" ~/anon_auton_ws/src/config_manager/configs/ports/active.yaml; then
    echo "---------------- ANTENNA found ----------------"
    if rosnode list | grep "chair_manager"; then
        echo "Antenna already running"
    else
        roslaunch --wait antenna_chair.launch &
    fi
else
    echo "!!! NO ANTENNA FOUND !!!"
fi

if grep -iq "^port" ~/anon_auton_ws/src/config_manager/configs/ports/active.yaml; then
    echo "---------------- ARDUINO found ----------------"
    roslaunch --wait arduino.launch &
else
    echo "!!! NO ARDUINO FOUND !!!"
fi

if grep -iq "^device_path" ~/anon_auton_ws/src/config_manager/configs/ports/active.yaml; then
    echo "---------------- CAMERA found ----------------"
    roslaunch --wait camera.launch &
else
    echo "!!! NO CAMERA FOUND !!!"
fi

if grep -iq "^serial_port" ~/anon_auton_ws/src/config_manager/configs/ports/active.yaml; then
    echo "---------------- LIDAR found ----------------"
    roslaunch --wait lidar.launch &
else
    echo "!!! NO LIDAR FOUND !!!"
fi


# Set up the fifo input for handwritten
#rm -rf /tmp/handwritten-input
#mkfifo /tmp/handwritten-input
#cat > /tmp/handwritten-input &
#echo $! > /tmp/handwritten-input-pid

# Launch handwritten to read from the fifo
#cat /tmp/handwritten-input | ../launch_handwritten.sh &

# Launch the queue to start everything
roslaunch --wait queue.launch
