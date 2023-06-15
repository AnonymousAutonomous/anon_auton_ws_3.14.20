#!/usr/bin/env python
import rospy
import serial
from std_msgs.msg import String

antenna_port = rospy.get_param('antenna_port')
ser = serial.Serial(antenna_port, 57600)

def callback(data):
    print(data.data)
    rospy.loginfo(data.data)
    ser.write(data.data.replace('\n', '') + '\n')

def hub_transmitter():
    rospy.init_node('hub_transmitter', anonymous=True)
    rospy.Subscriber('from_hub', String, callback)
    rospy.spin()

if __name__ == '__main__':
    hub_transmitter()
