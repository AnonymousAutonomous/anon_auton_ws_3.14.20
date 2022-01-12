#!/usr/bin/env python

import serial
from enum import Enum
import os
import subprocess

# Find all USB devices
df = subprocess.check_output("find /sys/bus/usb/devices/usb*/ -name dev", shell=True)
devices = {}

antenna_port = "/dev/ttyUSB2"

class Command(Enum):
    LAUNCH = 1
    STOP = 2
    HANDWRITTEN = 3
    PRE = 4

cmd_to_script = {
    Command.LAUNCH: 'launch_autonomous.sh',
    Command.STOP: 'stop.sh',
    Command.HANDWRITTEN: '.sh',
    Command.PRE: 'pre.sh',
}

def cleanStr(msg):
    return msg.lower().strip().rstrip()

def launch_script_cmd(filename):
    return ["~/anon_auton_ws/src/launch_manager/launch/" + filename]

if __name__ == '__main__':
    ser = serial.Serial(antenna_port, 57600)

    while True:
        str_msg = ser.readline()[:-1]
        str_msg = str_msg.decode('ascii')
        str_msg = cleanStr(str_msg)
        print(cleanStr(str_msg))

        cmd = None
        if str_msg == 'launch':
            cmd = Command.LAUNCH
        elif str_msg == 'stop':
            cmd = Command.STOP
        elif str_msg == 'handwritten':
            cmd = Command.HANDWRITTEN
        elif str_msg == 'pre':
            cmd = Command.PRE

        print(f"Running command for: {{str_msg}}")

        if cmd is not None: 
            subprocess.Popen(launch_script_cmd(cmd_to_script[cmd]), shell=True, executable="/bin/bash")

