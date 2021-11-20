// nice

#include "ros/ros.h"
#include "../../../constants/str_cmds.h"
#include "std_msgs/String.h"
#include <string>
#include <sstream>

ros::Publisher driver_pub;

int main(int argc, char** argv) {
    ros::init(argc, argv, "baby_pseudo");

    ros::NodeHandle nh;

    driver_pub = nh.advertise<std_msgs::String>("driver_output" , 1000);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
    	std_msgs::String msg;
	std::stringstream ss;
	ss << "0Af2.5f2.5";
	msg.data = ss.str();

	driver_pub.publish(msg);
	ros::spinOnce();
	loop_rate.sleep();
    }

    return 0;
}
