#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import String, Empty

antenna_port = rospy.get_param('antenna_port')
chair_num = rospy.get_param('chair_num')
ser = serial.Serial(antenna_port, 57600)


# def close_callback(data, ser):
#     rospy.loginfo("SHUTTING DOWN!")
#     rospy.signal_shutdown("Received shutdown ")


def close_serial():
    rospy.logerror("SHUTTING DOWN ANTENNA RECEIVER")
    ser.close()

def chair_receiver():
    pub = rospy.Publisher("from_chair_receiver", String, queue_size=10)
    debug_pub = rospy.Publisher("chair_receiver_debug", String, queue_size=10)
    # rospy.Subscriber('shutdown_ros', Empty, close_callback, (ser))
    rospy.init_node("chair_receiver", anonymous=False)

    rospy.on_shutdown(close_serial)

    while not rospy.is_shutdown():
        str_msg = ser.readline().strip()
        rospy.loginfo(str_msg)
        debug_pub.publish(str_msg)
        if str_msg[0] == chair_num or str_msg[0] == "0":
            pub.publish(str_msg[1:])


if __name__ == '__main__':
    try:
        chair_receiver()
    except rospy.ROSInterruptException as err:
        rospy.logerror(err)
