#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../../../constants/str_cmds.h"
#include <string>
#include <sstream>
#include <map>
#include <unordered_map>

std::map<std::string, std::string> commands_in;
std::unordered_map<AutonomousCmd, std::string> commands;

int main(int argc, char** argv) {
	ros::init(argc, argv, "source");
	ros::NodeHandle nh;
	if (nh.getParam("autonomous", commands_in)) {
        for (auto i = commands_in.begin(); i != commands_in.end(); i++)
            commands[AUTOCMD_STRING_TO_ENUM[i->first]] = i->second;
        }
        ROS_INFO("Autonomous commands have been loaded for ping arduino source.");
    }
    else {
        ROS_INFO("You must load autonomous commands before using ping arduino source.");
        return 1;
    }

	ros::Publisher pub = nh.advertise<std_msgs::String>("driver_output", 0);
	ros::Rate loop_rate(10);

	// int count = 0;

	while (ros::ok()) {
		std_msgs::String msg;
		std::stringstream ss;
		ss << commands[FWD];
		// ss << count;
		msg.data = ss.str();

		// ++count;

		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
