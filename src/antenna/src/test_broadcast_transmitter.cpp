#include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>

int main(int argc, char** argv) {
	ros::init(argc, argv, "test_broadcast_transmitter");
	ros::NodeHandle nh;
	ros::Publisher test_pub = nh.advertise<std_msgs::String>("to_hub_manager", 1000);

	std_msgs::String msg;
	msg.data = "0Bf100f100t4";
	test_pub.publish(msg);
	msg.data = "0Bf000f000t2";
	test_pub.publish(msg);
}
