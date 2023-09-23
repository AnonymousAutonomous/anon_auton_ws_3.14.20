#!/usr/bin/env python

import rospy
import serial
import base64
from std_msgs.msg import String

antenna_port = rospy.get_param('antenna_port')


def hub_receiver():
    pub = rospy.Publisher("from_hub_receiver", String, queue_size=10)
    rospy.init_node("hub_receiver", anonymous=True)
    ser = serial.Serial(antenna_port, 57600)
    while not rospy.is_shutdown():
        str_msg = base64.decodestring(ser.readline().strip())
        # [2:].decode("utf-8").strip()
        rospy.loginfo(str_msg)
        pub.publish(str_msg)


if __name__ == '__main__':
    try:
        hub_receiver()
    except rospy.ROSInterruptException:
        pass
