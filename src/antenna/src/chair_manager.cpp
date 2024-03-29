#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Char.h"
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
};

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
	{"config", ANTENNA_CONFIG}};

char LAUNCH_AUTONOMOUS_SCRIPT[] = "~/anon_auton_ws/src/launch_manager/launch/launch_autonomous.sh &";
char LAUNCH_HANDWRITTEN_SCRIPT[] = "~/anon_auton_ws/src/launch_manager/launch/launch_handwritten.sh &";
char SHUTDOWN_SCRIPT[] = "~/anon_auton_ws/src/launch_manager/launch/shutdown.sh &";
char RESET_SCRIPT[] = "~/anon_auton_ws/src/launch_manager/launch/reset.sh &";

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

enum class chair_stuck_status : char
{
	stuck = 's',
	not_stuck = 'n'
};
enum class chair_trapped_status : char
{
	trapped = 't',
	not_trapped = 'm'
};

ros::Publisher chair_manager_pub;
ros::Publisher from_chair_pub;

chair_stuck_status chair_stuck_state = chair_stuck_status::not_stuck;
chair_trapped_status chair_trapped_state = chair_trapped_status::not_trapped;
char camera_online = 'n';
char lidar_online = 'n';

std::string chair_flags = "";

ros::Time startTime;
ros::Duration heartbeatDuration(0.5); // 0.5 seconds

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

	// Broadcast = send along to the queue
	if (strlen(msg.data.c_str()) > 2 && msg.data[1] == 'B')
	{
		chair_manager_pub.publish(msg);

		// std_msgs::String sendmsg;
		// sendmsg.data = "0" + std::string(msg_copy);
		// chair_manager_pub.publish(sendmsg);
		return;
	}

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

void chair_flags_callback(const std_msgs::String &flags_in)
{
	ROS_ERROR("HERE ARE THE FLAGS: %s", flags_in.data.c_str());
	// Order [A][B][C][H][T][D][S][EOC][SOB][EOB]
	// y/n
	chair_flags = flags_in.data;
}

void stuck_or_trapped_callback(const std_msgs::Char state_in)
{
	// Stuck!
	if (state_in.data == 'S')
	{
		// send first time
		// if (chair_stuck_state == chair_stuck_status::not_stuck)
		// {
		// 	std_msgs::String msg;
		// 	msg.data = "Ss";
		// 	from_chair_pub.publish(msg);
		// }
		chair_stuck_state = chair_stuck_status::stuck;
	}
	// Trapped!
	else if (state_in.data == 'T')
	{
		// send first time
		// if (chair_trapped_state == chair_trapped_status::not_trapped)
		// {
		// 	std_msgs::String msg;
		// 	msg.data = "Tt";
		// 	from_chair_pub.publish(msg);
		// }
		chair_trapped_state = chair_trapped_status::trapped;
	}
	// Not stuck anymore
	else if (state_in.data == 's')
	{
		// send first time
		// if (chair_stuck_state == chair_stuck_status::stuck)
		// {
		// 	std_msgs::String msg;
		// 	msg.data = "Sn";
		// 	from_chair_pub.publish(msg);
		// }
		chair_stuck_state = chair_stuck_status::not_stuck;
	}
	// Not trapped anymore
	else if (state_in.data == 't')
	{
		// send first time
		// if (chair_trapped_state == chair_trapped_status::trapped)
		// {
		// 	std_msgs::String msg;
		// 	msg.data = "Tm";
		// 	from_chair_pub.publish(msg);
		// }
		chair_trapped_state = chair_trapped_status::not_trapped;
	}
	{
		return;
	}
}

void camera_status_callback(const std_msgs::Bool msg)
{
	ROS_ERROR("IN CAMERA STATUS CALLBACK");
	if (msg.data == true)
	{
		camera_online = 'y';
	}
	else
	{
		camera_online = 'n';
	}
}

void lidar_status_callback(const std_msgs::Bool msg)
{
	ROS_ERROR("IN LIDAR STATUS CALLBACK");

	if (msg.data == true)
	{
		lidar_online = 'y';
	}
	else
	{
		lidar_online = 'n';
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

	// TODO: delete this when actually running!
	ros::Subscriber chair_flags_sub = nh.subscribe("queue_to_manager", 1000, chair_flags_callback);
	ros::Subscriber trapped_stuck_sub = nh.subscribe("stuck_or_trapped_alert", 1000, stuck_or_trapped_callback);

	ros::Subscriber camera_online_sub = nh.subscribe("camera_online_status", 1000, camera_status_callback);
	ros::Subscriber lidar_online_sub = nh.subscribe("lidar_online_status", 1000, lidar_status_callback);

	// initialize publishers
	chair_manager_pub = nh.advertise<std_msgs::String>("driver_output", 1000);
	from_chair_pub = nh.advertise<std_msgs::String>("from_chair", 1000);

	// ros::Timer timer = nh.createTimer(ros::Duration(0.1), onHeartbeat);
	// ros::Rate delay_rate(5); // 5 cycles per second
	startTime = ros::Time::now();

	while (ros::ok())
	{
		if (ros::Time::now() >= startTime + heartbeatDuration)
		{
			// Send heartbeat with statuses
			std::string msgs;
			msgs = "";
			if (chair_flags.size() >= 1)
			{
				msgs += chair_flags.substr(0, 1); // get autonomous, broadcast, etc. from chair flags
			}
			else
			{
				msgs += 'o';
			}
			msgs += camera_online;
			msgs += lidar_online;
			msgs += static_cast<char>(chair_stuck_state);
			msgs += static_cast<char>(chair_trapped_state);
			if (chair_flags.size() >= 2)
			{
				msgs += chair_flags.substr(1); // rest of flags
			}

			ROS_ERROR("SENDING HEARTBEAT: %s", msgs.c_str());
			std_msgs::String msg;
			msg.data = msgs;
			from_chair_pub.publish(msg);

			// // send stuck or not
			// std_msgs::String stuck_msg;
			// stuck_msg.data = "S" + static_cast<char>(chair_stuck_state);
			// from_chair_pub.publish(stuck_msg);

			// // send trapped or not
			// std_msgs::String trapped_msg;
			// trapped_msg.data = "T" + static_cast<char>(chair_trapped_state);
			// from_chair_pub.publish(trapped_msg);

			startTime = ros::Time::now();
		}
		// 	std_msgs::String msg;
		// 	msg.data = 'h'; // heartbeat!
		// 	from_chair_pub.publish(msg);
		// 	delay_rate.sleep(); // runs out duration is remaining
	}
	ros::waitForShutdown();
}
