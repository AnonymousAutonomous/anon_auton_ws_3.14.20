#!/bin/bash

DEV_TO_PORT_FILE="dev_to_port.txt"

# clear file contents
> $DEV_TO_PORT_FILE

for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
	(
		syspath="${sysdevpath%/dev}"
		devname="$(udevadm info -q name -p $syspath)"
		[[ "$devname" == "bus/"* ]] && exit
		eval "$(udevadm info -q property --export -p $syspath)"
		[[ -z "$ID_SERIAL" ]] && exit
		echo "/dev/$devname - $ID_SERIAL"

		if [[ "$ID_SERIAL" == *"Arduino"* || "$ID_SERIAL" == *"1a86_USB2.0"* ]]; then
			echo "arduino /dev/$devname" >> $DEV_TO_PORT_FILE
			export ARDUINO_PORT="/dev/$devname"
		elif [[ "$ID_SERIAL" == *"Basic_UART"* ]]; then
			echo "antenna /dev/$devname" >> $DEV_TO_PORT_FILE
			export ANTENNA_PORT="/dev/$devname"
		elif [[ "$ID_SERIAL" == *"Silicon_Labs"* ]]; then
			echo "lidar /dev/$devname" >> $DEV_TO_PORT_FILE
			export LIDAR_PORT="/dev/$devname"
		elif [[ "$devname" == *"video"* ]]; then
			echo "camera /dev/$devname" >> $DEV_TO_PORT_FILE
			export CAMERA_PORT="/dev/$devname"
		fi
	)
done
