#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char** argv) {
	ros::init(argc, argv, "ping");
	ros::NodeHandle nh;
	ros::Publisher paddle = nh.advertise<std_msgs::String>("bounce", 1000);
	ros::Rate loop_rate(10);

	while (ros::ok()) {
		std_msgs::String msg;
		std::stringstream ss;
		ss << "ECHO";
		msg.data = ss.str();

		paddle.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

// Clock skew lol
