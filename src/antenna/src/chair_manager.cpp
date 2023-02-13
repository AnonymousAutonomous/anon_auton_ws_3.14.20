#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <stdlib.h>
#include <unordered_map>
#include "../../../constants/str_cmds.h"

#include <ros/spinner.h>
#include <queue>
#include <vector>
#include <string>

// All available autonomous hand_cmds
std::map<std::string, std::string> hand_cmds_in;
std::unordered_map<AutonomousCmd, std::string> hand_cmds;

enum Command
{
	ANTENNA_START,
	ANTENNA_STOP,
	ANTENNA_LAUNCH,
	ANTENNA_RESET,
	ANTENNA_SHUTDOWN,
	ANTENNA_HANDWRITTEN,
	ANTENNA_FFWD,
	ANTENNA_FWD,
	ANTENNA_BWD,
	ANTENNA_FBWD,
	ANTENNA_PIVOTL,
	ANTENNA_PIVOTR,
	ANTENNA_VEERL,
	ANTENNA_VEERR,
};

const std::unordered_map<std::string, Command> cmd_to_case = {
	{"start", ANTENNA_START},
	{"stop", ANTENNA_STOP},
	{"launch", ANTENNA_LAUNCH},
	{"reset", ANTENNA_RESET},
	{"shutdown", ANTENNA_SHUTDOWN},
	{"ffwd", ANTENNA_FFWD},
	{"fwd", ANTENNA_FWD},
	{"bwd", ANTENNA_BWD},
	{"fbwd", ANTENNA_FBWD},
	{"veerl", ANTENNA_VEERL},
	{"veerr", ANTENNA_VEERR},
	{"pivotl", ANTENNA_PIVOTL},
	{"pivotr", ANTENNA_PIVOTR},
	{"handwritten", ANTENNA_HANDWRITTEN}};

char LAUNCH_AUTONOMOUS_SCRIPT[] = "~/anon_auton_ws/src/launch_manager/launch/launch_autonomous.sh &";
char LAUNCH_HANDWRITTEN_SCRIPT[] = "~/anon_auton_ws/src/launch_manager/launch/launch_handwritten.sh &";
char SHUTDOWN_SCRIPT[] = "~/anon_auton_ws/src/launch_manager/launch/shutdown.sh &";
char RESET_SCRIPT[] = "~/anon_auton_ws/src/launch_manager/launch/shutdown.sh &";

// probably not necessary
// #define NUMBER_OF_CHAIRS 1

// for reference
enum class chair_broadcast_status : char
{
	ready = 'r',
	exclude = 'e',
	success = 's',
	failure = 'f'
};
// enum class chair_stuck_status : char {stuck, not_stuck};
// enum class chair_trapped_status : char {trapped, not_trapped};

ros::Publisher chair_manager_pub;
ros::Publisher test_pub;

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

void handle_reset()
{
	system(RESET_SCRIPT);
}

void handle_shutdown()
{
	system("echo \"stop\" > /tmp/handwritten-input");
	sleep(1);
	system(SHUTDOWN_SCRIPT);
}

// TODO: handle custom handwritten vs. from the standard set
void handle_handwritten(char handwritten_cmd[])
{
	std::string prefix = "echo \"";
	char suffix[] = "\" > /tmp/handwritten-input";
	system((prefix + handwritten_cmd + suffix).c_str());
}

void send_as_handwritten(AutonomousCmd cmd)
{
	std::string prefix = "echo \"";
	char suffix[] = "\" > /tmp/handwritten-input";
	system((prefix + hand_cmds[cmd] + suffix).c_str());
}

/* Message format:
   #cmd
   where
   # = number of the chair the command is for. If # is 0, it's for all chairs.
   cmd = either one of the cmds above, or one of the handwritten hand_cmds.
*/

void receive_callback(const std_msgs::String &msg)
{
	// chair_manager_pub.publish(msg);
	ROS_INFO("PUBLISHING %s", msg.data.c_str());

	char msg_copy[30];
	strcpy(msg_copy, msg.data.c_str());

	char *cmd = strtok(msg_copy, " ");

	auto command_ptr = cmd_to_case.find(std::string(cmd));

	if (command_ptr != cmd_to_case.end())
	{
		switch (command_ptr->second)
		{
		case ANTENNA_START:
			handle_start();
			break;
		case ANTENNA_STOP:
			handle_stop();
			break;
		case ANTENNA_LAUNCH:
			handle_launch();
			break;
		case ANTENNA_SHUTDOWN:
			handle_shutdown();
			break;
		case ANTENNA_RESET:
			handle_reset();
			break;
		case ANTENNA_HANDWRITTEN:
			handle_handwritten(strtok(NULL, " "));
			break;
		case ANTENNA_FWD:
			send_as_handwritten(FWD);
			break;
		case ANTENNA_BWD:
			send_as_handwritten(BWD);
			break;
		case ANTENNA_FFWD:
			send_as_handwritten(FFWD);
			break;
		case ANTENNA_FBWD:
			send_as_handwritten(FBWD);
			break;
		case ANTENNA_PIVOTL:
			send_as_handwritten(PIVOTL);
			break;
		case ANTENNA_PIVOTR:
			send_as_handwritten(PIVOTR);
			break;
		case ANTENNA_VEERL:
			send_as_handwritten(VEERL);
			break;
		case ANTENNA_VEERR:
			send_as_handwritten(VEERR);
			break;
		default:
			ROS_INFO("Invalid command:  %s", cmd);
			break;
		}
	}
	else
	{
		ROS_INFO("Invalid command:  %s", cmd);
	}
}

int main(int argc, char **argv)
{
	// initialize node and node handle
	ros::init(argc, argv, "chair_manager");
	ros::NodeHandle nh;

	// load speeds
	if (nh.getParam("/handwritten", hand_cmds_in))
	{
		for (auto i = hand_cmds_in.begin(); i != hand_cmds_in.end(); i++)
		{
			hand_cmds[AUTOCMD_STRING_TO_ENUM[i->first]] = i->second;
		}
		ROS_INFO("Handwritten hand_cmds have been loaded for chair manager.");
	}
	else
	{
		ROS_INFO("You must load handwritten hand_cmds before using chair manager.");
		return 1;
	}

	// initialize spinner
	ros::AsyncSpinner spinner(0);
	spinner.start();

	// initialize subscribers
	ros::Subscriber sub = nh.subscribe("from_chair_receiver", 1000, receive_callback);

	// initialize publishers
	chair_manager_pub = nh.advertise<std_msgs::String>("driver_output", 1000);
	test_pub = nh.advertise<std_msgs::String>("from_chair", 1000);

	while (ros::ok())
	{
		std_msgs::String msg;
		msg.data = (char)(chair_broadcast_status::ready);
		test_pub.publish(msg);
	}
}
