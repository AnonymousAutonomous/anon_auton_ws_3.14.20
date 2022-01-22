#!/usr/bin/env python

import serial

antenna_port = "/dev/ttyUSB0"

if __name__ == '__main__':
    ser = serial.Serial(antenna_port, 57600)

    while True:
        msg = input('Message:\n')
        print(msg)
        ser.write((msg + '\n').encode())
        

