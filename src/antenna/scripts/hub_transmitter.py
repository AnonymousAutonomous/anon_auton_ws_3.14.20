#!/usr/bin/env python
import rospy
import serial
from std_msgs.msg import String

antenna_port = rospy.get_param('antenna_port')
ser = serial.Serial(antenna_port, 57600)

def callback(data):
    print(data.data)
    try:
        rospy.loginfo(data.data)
        ser.write(encode(chair_num + data.data + '\n'))
    except serial.SerialException as e:
        ser.close()
        rospy.signal_shutdown("Antenna disconnected.")


def hub_transmitter():
    rospy.init_node('hub_transmitter', anonymous=True)
    rospy.Subscriber('from_hub', String, callback)
    rospy.spin()

if __name__ == '__main__':
    hub_transmitter()
