#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import socket

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    TCP_IP = '192.168.1.1'
    TCP_PORT = 5005
    BUFFER_SIZE = 1024
    MESSAGE = data.data

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    s.send(MESSAGE)
    echo = s.recv(BUFFER_SIZE)
    s.close()

    print "echo:", echo

def tcp_client():
    rospy.init_node('tcp_client', anonymous=True)

    rospy.Subscriber("error", String, callback)

    rospy.spin()

if __name__ == '__main__':
    tcp_client()
