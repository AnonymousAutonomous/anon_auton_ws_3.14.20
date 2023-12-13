// nice

#include "ros/ros.h"
#include "../../../constants/str_cmds.h"
#include "std_msgs/String.h"
#include "std_msgs/Char.h"
#include "std_msgs/Empty.h"
#include <string>
#include <sstream>
#include <map>
#include <unordered_map>
#include <queue>
#include <boost/circular_buffer.hpp>
#include <boost/range/algorithm.hpp>

ros::Publisher driver_pub;
ros::Publisher to_chair_manager_pub;

void camera_callback(const std_msgs::String &commands);
void lidar_callback(const std_msgs::String &commands);

// Keeps track of last N pivots. If we've done N pivots in the past M minutes, then we are stuck.
// N = 2; M = 3
// Contains unix walltime as double
int lidar_stuck_max_choreos = 1;
boost::circular_buffer<ros::WallTime> lidar_stuck_pq(lidar_stuck_max_choreos);
ros::WallDuration lidar_stuck_duration(1.0 * 60); // 1 min in seconds

// Keeps track of last N pivots. If we've done N pivots in the past M minutes, then we are stuck.
// N = 2; M = 3
int camera_trapped_max_choreos = 2;
boost::circular_buffer<ros::WallTime> camera_trapped_pq(camera_trapped_max_choreos);
ros::WallDuration camera_trapped_duration(1.0 * 60); // 1 mins

std::pair<std_msgs::String, std_msgs::String> command_pair;
// first command is camera
// second command is lidar

bool favor_right = true;

bool is_stuck = false;
bool is_trapped = false;
bool please_clear_queue = false;

// All available autonomous commands
std::map<std::string, std::string> auto_commands_in;
std::unordered_map<AutonomousCmd, std::string> auto_commands;

void clear_queues_callback(const std_msgs::Empty empty_msg)
{
	please_clear_queue = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "baby_trilogy");

	ros::NodeHandle nh;
	if (nh.getParam("/autonomous", auto_commands_in))
	{
		for (auto i = auto_commands_in.begin(); i != auto_commands_in.end(); i++)
		{
			auto_commands[AUTOCMD_STRING_TO_ENUM[i->first]] = i->second;
		}
		ROS_INFO("Autonomous commands have been loaded for baby trilogy.");
	}
	else
	{
		ROS_INFO("You must load autonomous commands before using baby trilogy.");
		return 1;
	}

	ros::Subscriber camera_sub = nh.subscribe("cameron", 1000, camera_callback);
	ros::Subscriber lidar_sub = nh.subscribe("larry", 1000, lidar_callback);
	ros::Subscriber clear_queues = nh.subscribe("clear_stuck_trapped", 1000, clear_queues_callback);
	driver_pub = nh.advertise<std_msgs::String>("driver_output", 1000);
	to_chair_manager_pub = nh.advertise<std_msgs::Char>("stuck_or_trapped_alert", 1000);

	// while (ros::ok) {
	// 	updateStuckStatus();
	// 	updateTrappedStatus();
	// }
	ros::spin();
}

void updateStuckStatus()
{
	if (lidar_stuck_pq.size() == lidar_stuck_max_choreos)
	{
		ros::WallTime earliest_choreo = lidar_stuck_pq[0];
		ros::WallTime now = ros::WallTime::now();
		// ros::WallTime now = ros::WallTime::now();
		// double now = ros::WallTime::now().toSec();
		ROS_ERROR("LIDAR DIFF: %d\t%d\t%d", static_cast<double>(ros::WallTime::now().toSec()), static_cast<double>(earliest_choreo.toSec()), static_cast<double>((ros::WallTime::now() - earliest_choreo).toSec()));
		if (now - earliest_choreo <= camera_trapped_duration)
		{
			if (!is_stuck)
			{
				std_msgs::Char msg;
				msg.data = 'S'; // Stuck!
				to_chair_manager_pub.publish(msg);
			}
			is_stuck = true;
		}
		else
		{
			if (is_stuck)
			{
				std_msgs::Char msg;
				msg.data = 's'; // not stuck
				to_chair_manager_pub.publish(msg);
			}
			is_stuck = false;
		}
	}
}

void updateTrappedStatus()
{
	if (camera_trapped_pq.size() == camera_trapped_max_choreos)
	{

		ros::WallTime earliest_choreo = camera_trapped_pq[0];
		ros::WallTime now = ros::WallTime::now();
		// ROS_ERROR("CAMERA DIFF: %d", now - earliest_choreo);

		if (now - earliest_choreo <= camera_trapped_duration)
		{
			if (!is_trapped)
			{
				std_msgs::Char msg;
				msg.data = 'T'; // Trapped!
				to_chair_manager_pub.publish(msg);
			}
			is_trapped = true;
		}
		else
		{
			if (is_trapped)
			{
				std_msgs::Char msg;
				msg.data = 't'; // not trapped
				to_chair_manager_pub.publish(msg);
			}
			is_trapped = false;
		}
	}
}

void command_compare()
{
	ROS_ERROR("lidar pq size: %d\tcam pq size: %d", lidar_stuck_pq.size(), camera_trapped_pq.size());
	if (please_clear_queue)
	{
		lidar_stuck_pq = boost::circular_buffer<ros::WallTime>(lidar_stuck_max_choreos);
		camera_trapped_pq = boost::circular_buffer<ros::WallTime>(camera_trapped_max_choreos);
		is_stuck = false;
		is_trapped = false;
		std_msgs::Char msg;
		msg.data = 's'; // not stuck
		to_chair_manager_pub.publish(msg);
		std_msgs::Char tmsg;
		tmsg.data = 't'; // not stuck
		to_chair_manager_pub.publish(tmsg);
		please_clear_queue = false;
	}
	else
	{
		updateStuckStatus();
		updateTrappedStatus();
	}

	ROS_INFO("%s OR %s", command_pair.first.data.c_str(), command_pair.second.data.c_str());
	if (command_pair.first.data.empty() && command_pair.second.data.empty())
	{
		std_msgs::String low_prio;
		low_prio.data = "EAf2.0f2.0";
		driver_pub.publish(low_prio);
		ROS_INFO("SELECTED: %s", low_prio.data.c_str());
	}
	else if (command_pair.first.data.empty())
	{ // camera command has yet to arrive
		std_msgs::String &priority = command_pair.second;
		if (priority.data[1] == 'C')
		{
			if (priority.data[2] == 'Z')
			{
				return;
			}
			else
			{
				if (priority.data == auto_commands[RREVERSE] && !favor_right)
				{
					priority.data = auto_commands[LREVERSE];
				}
				driver_pub.publish(priority);
				ROS_INFO("SELECTED: %s", priority.data.c_str());
				priority.data[2] = 'Z';
			}
		}
		else
		{
			driver_pub.publish(priority);
			ROS_INFO("SELECTED: %s", priority.data.c_str());
		}
	}
	else if (command_pair.second.data.empty())
	{													 // lidar command has yet to arrive
		std_msgs::String &priority = command_pair.first; // get camera command
		if (priority.data[1] == 'C')
		{
			if (priority.data[2] == 'Z')
			{
				return;
			}
			else
			{
				driver_pub.publish(priority);
				ROS_INFO("SELECTED: %s", priority.data.c_str());
				priority.data[2] = 'Z';
			}
		}
		else
		{
			driver_pub.publish(priority);
			ROS_INFO("SELECTED: %s", priority.data.c_str());
		}
	}
	else
	{
		std_msgs::String &priority = (command_pair.first.data[0] < command_pair.second.data[0] ? command_pair.first : command_pair.second);
		if (priority.data[1] == 'C')
		{
			if (priority.data[2] == 'Z')
			{
				return;
			}
			else
			{
				if (priority.data == auto_commands[RREVERSE] && !favor_right)
				{
					priority.data = auto_commands[LREVERSE];
				}
				driver_pub.publish(priority);
				ROS_INFO("SELECTED: %s", priority.data.c_str());
				priority.data[2] = 'Z';
			}
		}
		else
		{
			driver_pub.publish(priority);
			ROS_INFO("SELECTED: %s", priority.data.c_str());
		}
	}
}

void camera_callback(const std_msgs::String &commands)
{
	if (commands.data == auto_commands[PIVOTR] ||
		commands.data == auto_commands[RCP] ||
		commands.data == auto_commands[VEERR] ||
		commands.data == auto_commands[FPIVOTR])
	{
		favor_right = true;
	}
	else if (commands.data == auto_commands[PIVOTL] ||
			 commands.data == auto_commands[LCP] ||
			 commands.data == auto_commands[VEERL] ||
			 commands.data == auto_commands[FPIVOTL])
	{
		favor_right = false;
	}
	else
	{
		// do nothing
	}

	// If choreo, then count towards trapped
	if (commands.data[1] == 'C')
	{
		camera_trapped_pq.push_back(ros::WallTime::now());
	}
	command_pair.first = commands;
	command_compare();
}

void lidar_callback(const std_msgs::String &commands)
{
	// If choreo, then count towards stuck
	if (commands.data[1] == 'C')
	{
		lidar_stuck_pq.push_back(ros::WallTime::now());
	}
	command_pair.second = commands;
	command_compare();
}
