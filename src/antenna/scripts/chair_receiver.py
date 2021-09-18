#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import String

def chair_receiver():
    pub = rospy.Publisher("from_chair_receiver", String, queue_size=10)
    rospy.init_node("chair_receiver", anonymous=True)
    ser = serial.Serial("/dev/ttyUSB0", 57600)
    while not rospy.is_shutdown():
        str_msg = ser.readline()[:-1]
        rospy.loginfo(str_msg)
        pub.publish(str_msg)

if __name__ == '__main__':
    try:
        chair_receiver()
    except rospy.ROSInterruptException:
        pass
