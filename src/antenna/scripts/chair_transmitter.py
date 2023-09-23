#!/usr/bin/env python
import rospy
import serial
import time
import base64
from std_msgs.msg import String

antenna_port = rospy.get_param('antenna_port')
chair_num = rospy.get_param('chair_num')
ser = serial.Serial(antenna_port, 57600)


# def encode(str):
#     return str.encode('unicode')


def callback(data):
    rospy.loginfo(data.data)
    ser.write(base64.encodestring(str(chair_num) + data.data))


def chair_transmitter():
    rospy.init_node('chair_transmitter', anonymous=True)
    rospy.Subscriber('from_chair', String, callback)
    time.sleep(0.5)
    rospy.spin()


if __name__ == '__main__':
    chair_transmitter()
