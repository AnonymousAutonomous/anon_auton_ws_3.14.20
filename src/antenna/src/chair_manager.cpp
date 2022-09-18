#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <stdlib.h>
#include <unordered_map>

#include <ros/spinner.h>
#include <queue>
#include <vector>
#include <string>

enum Command
{
	START,
	STOP,
	LAUNCH,
	SHUTDOWN,
	HANDWRITTEN
};

const std::unordered_map<std::string, Command>
	cmd_to_case{
		{"start", START},
		{"stop", STOP},
		{"launch", LAUNCH},
		{"shutdown", SHUTDOWN},
		{"handwritten", HANDWRITTEN}};

char LAUNCH_AUTONOMOUS_SCRIPT[] = "~/anon_auton_ws/src/launch_manager/launch/launch_autonomous.sh";
char LAUNCH_HANDWRITTEN_SCRIPT[] = "~/anon_auton_ws/src/launch_manager/launch/launch_handwritten.sh";
char SHUTDOWN_SCRIPT[] = "~/anon_auton_ws/src/launch_manager/launch/stop.sh";

// probably not necessary
// #define NUMBER_OF_CHAIRS 1

// for reference
// enum class chair_broadcast_status : char {ready, success, failure};
// enum class chair_stuck_status : char {stuck, not_stuck};
// enum class chair_trapped_status : char {trapped, not_trapped};

ros::Publisher chair_manager_pub;
// ros::Publisher test_pub;

void handle_start()
{
	system("echo \"toggle\" > /tmp/handwritten-input");
}

void handle_stop()
{
	system("echo \"stop\" > /tmp/handwritten-input");
}

void handle_launch()
{
	system(LAUNCH_AUTONOMOUS_SCRIPT);
	system(LAUNCH_HANDWRITTEN_SCRIPT);
}

void handle_shutdown()
{
	system("echo \"stop\" > /tmp/handwritten-input");
	sleep(1);
	system(SHUTDOWN_SCRIPT);
}

void handle_handwritten(char handwritten_cmd[])
{
	std::string prefix = "echo \"";
	char suffix[] = "\" > /tmp/handwritten-input";
	system((prefix + handwritten_cmd + suffix).c_str());
}

void receive_callback(const std_msgs::String &msg)
{
	// chair_manager_pub.publish(msg);
	ROS_INFO("PUBLISHING %s", msg.data.c_str());

	char msg_copy[30];
	strcpy(msg_copy, msg.data.c_str());

	char *cmd = strtok(msg_copy, " ");

	Command command = cmd_to_case.find(std::string(cmd))->second;

	switch (command)
	{
	case START:
		handle_start();
		break;
	case STOP:
		handle_stop();
		break;
	case LAUNCH:
		handle_launch();
		break;
	case SHUTDOWN:
		handle_shutdown();
		break;
	case HANDWRITTEN:
		handle_handwritten(strtok(NULL, " "));
		break;
	default:
		ROS_INFO("Invalid command:  %s", cmd);
		break;
	}
}

int main(int argc, char **argv)
{
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
	// test_pub = nh.advertise<std_msgs::String>("from_chair", 1000);

	while (ros::ok())
	{
		/*
		std_msgs::String msg;
		msg.data = "0B" + (char)(chair_broadcast_status::ready);
		test_pub.publish(msg);
		*/
	}
}
