#include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>

int main(int argc, char** argv) {
	ROS_INFO("START OF PROGRAM...");

	ros::init(argc, argv, "test_broadcast_transmitter");
	ros::NodeHandle nh;
	ros::Publisher test_pub = nh.advertise<std_msgs::String>("to_hub_manager", 1000);

	while (!test_pub.getNumSubscribers()) {
		// pass
	}

	std_msgs::String msg;
	msg.data = "0Bf100f100t4";
	test_pub.publish(msg);
	msg.data = "0Bf000f000t2";
	test_pub.publish(msg);
	msg.data = "0Br100r100t4";
	test_pub.publish(msg);
	msg.data = "0Br000r000t2";
	test_pub.publish(msg);
	msg.data = "0Bf100r100t4";
	test_pub.publish(msg);
	msg.data = "0Bf000r000t2";
	test_pub.publish(msg);
	msg.data = "0Br100f100t4";
	test_pub.publish(msg);
	msg.data = "0Br000f000t2";
	test_pub.publish(msg);
	msg.data = "0Bend";
	test_pub.publish(msg);

	ROS_INFO("END OF PROGRAM...");

	return 0;
}
