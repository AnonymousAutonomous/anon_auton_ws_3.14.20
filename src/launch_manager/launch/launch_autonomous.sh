#!/usr/bin/env bash

cd ~/anon_auton_ws
source devel/setup.bash

# Start roscore if it's not already running
if rostopic list | grep "rosout"; then
    echo "Roscore already running"
else
    roscore &
    # wait until roscore is running
    until rostopic list ; do sleep 1; done
fi

# Set the ports
/bin/bash ~/anon_auton_ws/src/launch_manager/launch/set_ports.sh

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


# Set up the fifo input for handwritten
#rm -rf /tmp/handwritten-input
#mkfifo /tmp/handwritten-input
#cat > /tmp/handwritten-input &
#echo $! > /tmp/handwritten-input-pid

# Launch handwritten to read from the fifo
#cat /tmp/handwritten-input | ../launch_handwritten.sh &

# Launch the queue to start everything
if rosnode list | grep "queue"; then
    echo "Queue already running"
else
    roslaunch --wait queue.launch &
fi
