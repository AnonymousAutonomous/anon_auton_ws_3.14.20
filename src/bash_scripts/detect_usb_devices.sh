#!/bin/bash

DEV_TO_PORT_FILE="dev_to_port.txt"

for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
	(
		syspath="${sysdevpath%/dev}"
		devname="$(udevadm info -q name -p $syspath)"
		[[ "$devname" == "bus/"* ]] && exit
		eval "$(udevadm info -q property --export -p $syspath)"
		[[ -z "$ID_SERIAL" ]] && exit
		echo "/dev/$devname - $ID_SERIAL"
		echo $ID_SERIAL
		[[$ID_SERIAL == *'Arduino'*]] && is_arduino=true || is_arduino=false
		[[grep -q "Arduino" <<< $ID_SERIAL]] && is_arduino=true || is_arduino=false

		echo $is_arduino

		if [["$ID_SERIAL" == *"Arduino"*]] || [[$ID_SERIAL == *"arduino"*]] || [[$devname == *"ACM"* ]]
		then
			echo "ARDUINO"
		fi

		if [["$ID_SERIAL" =~ "Arduino" ]]
		then
			echo "arduino is here"
		fi 
	)
done
