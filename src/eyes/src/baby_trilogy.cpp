// nice

#include "ros/ros.h"
#include "../../../constants/str_cmds.h"
#include "std_msgs/String.h"
#include <string>
#include <sstream>
#include <map>
#include <unordered_map>

ros::Publisher driver_pub;

void camera_callback(const std_msgs::String& commands); 
void lidar_callback(const std_msgs::String& commands);

std::pair<std_msgs::String, std_msgs::String> command_pair;
// first command is camera
// second command is lidar

bool favor_right = true;

// All available autonomous commands
std::map<std::string, std::string> commands_in;
std::unordered_map<AutonomousCmd, std::string> commands;


int main(int argc, char** argv) {
    ros::init(argc, argv, "baby_trilogy");

    ros::NodeHandle nh;
	if (nh.getParam("autonomous", commands_in)) {
        for (auto i = commands_in.begin(); i != commands_in.end(); i++) {
            commands[AUTOCMD_STRING_TO_ENUM[i->first]] = i->second;
        }
        ROS_INFO("Autonomous commands have been loaded for baby trilogy.");
    }
    else {
        ROS_INFO("You must load autonomous commands before using baby trilogy.");
        return 1;
    }

    ros::Subscriber camera_sub = nh.subscribe("cameron", 1000, camera_callback);
    ros::Subscriber lidar_sub = nh.subscribe("larry", 1000, lidar_callback);
    driver_pub = nh.advertise<std_msgs::String>("driver_output" , 1000);

    ros::spin();
}

void command_compare() {
    ROS_INFO("%s OR %s", command_pair.first.data.c_str(), command_pair.second.data.c_str());
    if (command_pair.first.data.empty() && command_pair.second.data.empty()) {
        std_msgs::String low_prio;
	low_prio.data = "EAf2.0f2.0";
	driver_pub.publish(low_prio);
	ROS_INFO("SELECTED: %s", low_prio.data.c_str());
    }
    else if (command_pair.first.data.empty()) { // camera command has yet to arrive
        std_msgs::String& priority = command_pair.second;
	if (priority.data[1] == 'C') {
	    if (priority.data[2] == 'Z') {
	    	return;
	    }
	    else {
		if (priority.data == commands[RREVERSE] && !favor_right) {
		    priority.data = commands[LREVERSE];
		}
	    	driver_pub.publish(priority);
		ROS_INFO("SELECTED: %s", priority.data.c_str());
		priority.data[2] = 'Z';
	    }
	}
	else {
	    driver_pub.publish(priority);
	    ROS_INFO("SELECTED: %s", priority.data.c_str());
	}
    }
    else if (command_pair.second.data.empty()) { // lidar command has yet to arrive
        std_msgs::String& priority = command_pair.first; // get camera command
	if (priority.data[1] == 'C') {
	    if (priority.data[2] == 'Z') {
	        return;
	    }
	    else {
	        driver_pub.publish(priority);
		ROS_INFO("SELECTED: %s", priority.data.c_str());
		priority.data[2] = 'Z';
	    }
	}
	else {
	    driver_pub.publish(priority);
	    ROS_INFO("SELECTED: %s", priority.data.c_str());
	}
    }
    else {
    	std_msgs::String& priority = (command_pair.first.data[0] < command_pair.second.data[0] ? command_pair.first : command_pair.second);
	if (priority.data[1] == 'C') {
	    if (priority.data[2] == 'Z') {
	    	return;
	    }
	    else {
		if (priority.data == commands[RREVERSE] && !favor_right) {
		    priority.data = commands[LREVERSE];
		}
	    	driver_pub.publish(priority);
		ROS_INFO("SELECTED: %s", priority.data.c_str());
		priority.data[2] = 'Z';
	    }
	}
	else {
	    driver_pub.publish(priority);
	    ROS_INFO("SELECTED: %s", priority.data.c_str());
	}
    }
}

void camera_callback(const std_msgs::String& commands) {
    if (commands.data == commands[PIVOTR] ||
	commands.data == commands[RCP] ||
	commands.data == commands[VEERR] ||
	commands.data == commands[FPIVOTR]) {
    	favor_right = true;
    }
    else if (commands.data == commands[PIVOTL] ||
        commands.data == commands[LCP] ||
	commands.data == commands[VEERL] ||
	commands.data == commands[FPIVOTL]) {
    	favor_right = false;
    }
    else {
        // do nothing
    }
    command_pair.first = commands;
    command_compare();
}

void lidar_callback(const std_msgs::String& commands) {
    command_pair.second = commands;
    command_compare();
}
