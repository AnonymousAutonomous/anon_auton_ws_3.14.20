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

def close_serial():
    rospy.logerr("SHUTTING DOWN HUB TRANSMITTER")
    ser.close()

def hub_transmitter():
    rospy.init_node('hub_transmitter', anonymous=False)
    rospy.Subscriber('from_hub', String, callback)
    rospy.on_shutdown(close_serial)

    rospy.spin()

if __name__ == '__main__':
    hub_transmitter()
