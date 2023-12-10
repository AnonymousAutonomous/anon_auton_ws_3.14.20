#include "ros/ros.h"
#include "std_msgs/String.h"
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

enum class state : char
{
	autonomous = 'A',
	choreo = 'C',
	custom = 'H',
	broadcast = 'B'
};
// enum class chair_stuck_status : char {stuck, not_stuck};
// enum class chair_trapped_status : char {trapped, not_trapped};

ros::Publisher chair_manager_pub;
ros::Publisher from_chair_pub;

state chair_state = state::autonomous;
std::string chair_flags = "";

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

void chair_state_callback(const std_msgs::Char state_in)
{
	chair_state = static_cast<state>(state_in.data);
}

void chair_flags_callback(const std_msgs::String flags_in)
{
	ROS_ERROR("HERE ARE THE FLAGS: %s", flags_in.data.c_str());
	// Order [A][B][C][H][T][D][S][EOC][SOB][EOB]
	// y/n
	chair_flags = flags_in.data;
}

void onHeartbeat(const ros::TimerEvent &event)
{
	ROS_ERROR("<3");
	std_msgs::String msg;
	msg.data = static_cast<char>(chair_state) + chair_flags; // heartbeat!
	from_chair_pub.publish(msg);
}

int main(int argc, char **argv)
{
	// initialize node and node handle
	ros::init(argc, argv, "chair_manager");
	ros::NodeHandle nh;

	// initialize subscribers
	ros::Subscriber sub = nh.subscribe("from_chair_receiver", 1000, receive_callback);
	ros::Subscriber chair_state_sub = nh.subscribe("queue_to_lidar", 1000, chair_state_callback);

	// TODO: delete this when actually running!
	ros::Subscriber chair_flags_sub = nh.subscribe("queue_to_manager", 1000, chair_flags_callback);

	// initialize publishers
	chair_manager_pub = nh.advertise<std_msgs::String>("driver_output", 1000);
	from_chair_pub = nh.advertise<std_msgs::String>("from_chair", 1000);

	ros::Timer timer = nh.createTimer(ros::Duration(0.1), onHeartbeat);
	// ros::Rate delay_rate(5); // 5 cycles per second

	// initialize spinner
	ros::AsyncSpinner spinner(0);
	spinner.start();
	ros::waitForShutdown();

	// while (ros::ok())
	// {
	// 	std_msgs::String msg;
	// 	msg.data = 'h'; // heartbeat!
	// 	from_chair_pub.publish(msg);
	// 	delay_rate.sleep(); // runs out duration is remaining
	// 	spin.once()
	// }
}
