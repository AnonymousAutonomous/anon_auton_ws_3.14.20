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
			echo "ARDUINO_PORT /dev/$devname" >> $DEV_TO_PORT_FILE
		elif [[ "$ID_SERIAL" == *"Basic_UART"* ]]; then
			echo "ANTENNA_PORT /dev/$devname" >> $DEV_TO_PORT_FILE
		elif [[ "$ID_SERIAL" == *"Silicon_Labs"* ]]; then
			echo "LIDAR_PORT /dev/$devname" >> $DEV_TO_PORT_FILE
		elif [[ "$devname" == *"video"* ]]; then
			echo "CAMERA_PORT /dev/$devname" >> $DEV_TO_PORT_FILE
		fi
	)
done

while IFS= read -r line
do
	echo "$line"
	arr=($line)
	echo ${arr[0]}
	echo ${arr[1]}
	export ${arr[0]}=${arr[1]}
done < "$DEV_TO_PORT_FILE"

echo $ARDUINO_PORT
