#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import String

antenna_port = rospy.get_param('antenna_port')
chair_num = rospy.get_param('chair_num')


def chair_receiver():
    pub = rospy.Publisher("from_chair_receiver", String, queue_size=10)
    debug_pub = rospy.Publisher("chair_receiver_debug", String, queue_size=10)

    rospy.init_node("chair_receiver", anonymous=True)

    ser = serial.Serial(antenna_port, 57600)
    while not rospy.is_shutdown():
        try:
            str_msg = ser.readline()[:-1]
            rospy.loginfo(str_msg)
            debug_pub.publish(str_msg)
            if str_msg[0] == chair_num or str_msg[0] == "0":
                pub.publish(str_msg[1:])
        except serial.SerialException as e:
            ROS_INFO("Chair receiver is sad %s", e)
            # ser.close()
            # rospy.signal_shutdown("Antenna disconnected.")




if __name__ == '__main__':
    try:
        chair_receiver()
    except rospy.ROSInterruptException:
        pass
