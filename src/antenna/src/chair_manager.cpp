#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/Image.h"
#include <stdio.h>
#include <stdlib.h>
#include <unordered_map>
#include "../../../constants/str_cmds.h"

#include <ros/spinner.h>
#include <queue>
#include <vector>
#include <string>

enum Command
{
	ANTENNA_LAUNCH,
	ANTENNA_RESET,
	ANTENNA_SHUTDOWN,
	ANTENNA_START,
	ANTENNA_TOGGLE,
	ANTENNA_STOP,
	ANTENNA_FFWD,
	ANTENNA_FWD,
	ANTENNA_BWD,
	ANTENNA_FBWD,
	ANTENNA_FWDL,
	ANTENNA_FWDR,
	ANTENNA_BWDL,
	ANTENNA_BWDR,
	ANTENNA_PIVOTL,
	ANTENNA_PIVOTR,
	ANTENNA_HANDWRITTEN,
	ANTENNA_CONFIG,
	ANTENNA_SEND_IMAGE,
};

const std_msgs::Empty empty_msg;

const std::unordered_map<std::string, Command> cmd_to_case = {
	// launching entire chair
	{"launch", ANTENNA_LAUNCH},
	{"reset", ANTENNA_RESET},
	{"shutdown", ANTENNA_SHUTDOWN},
	{"start", ANTENNA_START},
	// handwritten
	{"toggle", ANTENNA_TOGGLE},
	{"stop", ANTENNA_STOP},
	{"ffwd", ANTENNA_FFWD},
	{"fwd", ANTENNA_FWD},
	{"bwd", ANTENNA_BWD},
	{"fbwd", ANTENNA_FBWD},
	{"fwdl", ANTENNA_FWDL},
	{"fwdr", ANTENNA_FWDR},
	{"bwdl", ANTENNA_BWDL},
	{"bwdr", ANTENNA_BWDR},
	{"pivotl", ANTENNA_PIVOTL},
	{"pivotr", ANTENNA_PIVOTR},
	// for custom handwritten commands
	{"hand", ANTENNA_HANDWRITTEN},
	// config
	{"config", ANTENNA_CONFIG},
	{"send_image", ANTENNA_SEND_IMAGE},
};

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
ros::Publisher request_image_pub;

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

void update_config(const std_msgs::String &msg)
{
	std::string stringmsg = std::string(msg.data.c_str());

	auto end_of_cmd = stringmsg.find(' ');
	auto end_of_path = stringmsg.find(' ', end_of_cmd + 1);
	std::string filename = stringmsg.substr(end_of_cmd + 1, end_of_path - end_of_cmd - 1);
	std::string config = stringmsg.substr(end_of_path + 1);
	std::string root = "~/anon_auton_ws/src/config_manager/configs/live/";
	std::string fullpath = root + filename;

	ROS_INFO("FILENAME %s", filename);
	ROS_INFO("CONFIG %s", config);

	system(("mkdir " + root + " & echo '" + config + "' > " + fullpath).c_str());
}

// TODO: handle custom handwritten vs. from the standard set
void handle_handwritten(char handwritten_cmd[])
{
	std::string prefix = "echo \"";
	char suffix[] = "\" > /tmp/handwritten-input";
	system((prefix + handwritten_cmd + suffix).c_str());
}

void handle_send_image()
{
	request_image_pub.publish(empty_msg);
}

void receive_image_callback(const sensor_msgs::Image &view)
{
	test_pub.publish("image " + String(view.height) + " " + String(view.width) + " " + view.data).c_str());
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
	ROS_ERROR("PUBLISHING %s", msg.data.c_str());

	char msg_copy[30];
	strcpy(msg_copy, msg.data.c_str());

	char *cmd = strtok(msg_copy, " ");

	auto command_ptr = cmd_to_case.find(std::string(cmd));

	if (command_ptr != cmd_to_case.end())
	{
		switch (command_ptr->second)
		{
		case ANTENNA_LAUNCH:
			handle_launch();
			break;
		case ANTENNA_SHUTDOWN:
			handle_shutdown();
			break;
		case ANTENNA_RESET:
			handle_reset();
			break;
		case ANTENNA_START:
			handle_start();
			break;
		case ANTENNA_TOGGLE:
		case ANTENNA_STOP:
		case ANTENNA_FWD:
		case ANTENNA_BWD:
		case ANTENNA_FFWD:
		case ANTENNA_FBWD:
		case ANTENNA_PIVOTL:
		case ANTENNA_PIVOTR:
		case ANTENNA_FWDL:
		case ANTENNA_FWDR:
		case ANTENNA_BWDL:
		case ANTENNA_BWDR:
			handle_handwritten(cmd);
			break;
		case ANTENNA_HANDWRITTEN:
			handle_handwritten(strtok(NULL, " "));
			break;
		case ANTENNA_CONFIG:
			update_config(msg);
			break;
		case ANTENNA_SEND_IMAGE:
			handle_send_image();
			break;
		default:
			ROS_ERROR("Invalid command:  %s", cmd);
			break;
		}
	}
	else
	{
		ROS_ERROR("Invalid command:  %s", cmd);
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
	ros::Subscriber image_sub = nh.subscribe("sample_image", 1000, receive_image_callback);

	// initialize publishers
	chair_manager_pub = nh.advertise<std_msgs::String>("driver_output", 1000);
	request_image_pub = nh.advertise<std_msgs::Empty>("send_image", 1000);
	test_pub = nh.advertise<std_msgs::String>("from_chair", 1000);

	ros::Rate delay_rate(5);

	while (ros::ok())
	{
		std_msgs::String msg;
		msg.data = (char)(chair_broadcast_status::ready);
		test_pub.publish(msg);
	}
}
