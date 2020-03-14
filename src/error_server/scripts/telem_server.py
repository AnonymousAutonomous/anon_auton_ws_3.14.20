#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import serial

def telem_server():
    pub = rospy.Publisher('error', String, queue_size=10)
    rospy.init_node('telem_server', anonymous=True)
    rate = rospy.Rate(10)

    ser = serial.Serial("/dev/ttyUSB0", 57600)

    while not rospy.is_shutdown():

        data = ser.readline()
        rospy.loginfo(data)

        rate.sleep()

if __name__ == '__main__':
    try:
        telem_server()
    except rospy.ROSInterruptException:
        pass


