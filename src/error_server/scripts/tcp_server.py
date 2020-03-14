#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import socket

def tcp_server():
    pub = rospy.Publisher('error', String, queue_size=10)
    rospy.init_node('tcp_server', anonymous=True)
    rate = rospy.Rate(10)

    TCP_IP = '192.168.1.1'
    TCP_PORT = 5005
    BUFFER_SIZE = 20

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((TCP_IP, TCP_PORT))
    s.listen(1)

    while not rospy.is_shutdown():
        conn, addr = s.accept()
        print 'Connection address:', addr
        while 1:
            data = conn.recv(BUFFER_SIZE)
            if not data: break
            print "received data:", data
            conn.send(data)
            pub.publish(data)
        conn.close()

        rate.sleep()

if __name__ == '__main__':
    try:
        tcp_server()
    except rospy.ROSInterruptException:
        pass
