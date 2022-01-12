#!/usr/bin/env python

import serial

antenna_port = "/dev/ttyUSB2"

if __name__ == '__main__':
    ser = serial.Serial(antenna_port, 57600)

    while True:
        str_msg = ser.readline()[:-1]
        print(str_msg.decode('ascii'))

