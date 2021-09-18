#include "ros/ros.h"
#include "std_msgs/String.h"

#include <ros/spinner.h>
#include <queue>
#include <vector>
#include <string>

// probably not necessary
#define NUMBER_OF_CHAIRS 1

// probably not necessary
enum chair_broadcast_status : char {ready, success, failure};
enum chair_stuck_status : char {stuck, not_stuck};

ros::Publisher chair_manager_pub;

void receive_callback(const std_msgs::String& msg) {
	chair_manager_pub.publish(msg);
	ROS_INFO("PUBLISHING %s", msg.data.c_str());
}

int main (int argc, char** argv) {
	// initialize node and node handle
	ros::init(argc, argv, "chair_manager");
	ros::NodeHandle nh;

	// initialize spinner
	ros::AsyncSpinner spinner(0);
	spinner.start();

	// initialize subscribers
	ros::Subscriber sub = nh.subscribe("from_chair_receiver", 1000, receive_callback);

	// initialize publishers
	chair_manager_pub = nh.advertise<std_msgs::String>("driver_output", 1000);

	while (ros::ok()) {
		// pass
	}
}
