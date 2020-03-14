#include "ros/ros.h"
#include "std_msgs/String.h"

void callback(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("%s", msg->data.c_str());
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "ping_pong");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("bounceback", 1000, callback);
	ros::spin();

	return 0;
}
