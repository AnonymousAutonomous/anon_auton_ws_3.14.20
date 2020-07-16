#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char** argv) {
	ros::init(argc, argv, "source");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<std_msgs::String>("driver_output", 0);
	ros::Rate loop_rate(10);

	int count = 0;

	while (ros::ok()) {
		std_msgs::String msg;
		std::stringstream ss;
		ss << "A";
		ss << count;
		msg.data = ss.str();

		++count;

		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
