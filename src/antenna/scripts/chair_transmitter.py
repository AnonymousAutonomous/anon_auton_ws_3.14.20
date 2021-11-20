#!/usr/bin/env python
import rospy
import serial
from std_msgs.msg import String

ser = serial.Serial('/dev/ttyUSB0', 57600)

def callback(data):
    rospy.loginfo(data.data)
    ser.write(data.data + '\n')

def chair_transmitter():
    rospy.init_node('chair_transmitter', anonymous=True)
    rospy.Subscriber('from_chair', String, callback)
    rospy.spin()

if __name__ == '__main__':
    chair_transmitter()