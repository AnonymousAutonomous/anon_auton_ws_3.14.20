#!/usr/bin/env python
import rospy
import serial
import time
from std_msgs.msg import String, Empty

antenna_port = rospy.get_param('antenna_port')
chair_num = rospy.get_param('chair_num')
ser = serial.Serial(antenna_port, 57600)


def encode(str):
    return str.encode('utf-8')


def callback(data):
    rospy.loginfo(data.data)
    ser.write(encode(chair_num + data.data + '\n'))


def close_callback(data):
    ser.close()



def chair_transmitter():
    rospy.init_node('chair_transmitter', anonymous=False)
    rospy.Subscriber('from_chair', String, callback)
    rospy.Subscriber('shutdown_ros', Empty, close_callback)
    time.sleep(0.5)
    rospy.spin()


if __name__ == '__main__':
    chair_transmitter()
