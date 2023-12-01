#include "ros/ros.h"
#include "std_msgs/String.h"

#include <ros/spinner.h>
#include <queue>
#include <vector>
#include <algorithm>

// active_chair_nums :

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

struct chair_status
{
	chair_broadcast_status cbs = chair_broadcast_status::success;
	chair_stuck_status css = chair_stuck_status::not_stuck;
	chair_trapped_status cts = chair_trapped_status::not_trapped;
	// being user-controlled? low battery? surrounded? current command? connection break?
};

std::vector<int> active_chair_nums;
std::map<int, chair_status> chair_status_map;

ros::Time startTime;
ros::Duration waitDurationBeforeCheckingAgain(1.0); // 1.0 seconds
ros::Publisher hub_manager_pub;

bool all_chairs_are_ready()
{
	for (const auto &p : chair_status_map)
	{
		if (p.second.cbs != chair_broadcast_status::ready && p.second.cbs != chair_broadcast_status::exclude)
		{
			return false;
		}
	}
	return true;
}

void overwrite_excluded_chairs()
{
	for (auto &p : chair_status_map)
	{
		if (p.second.cbs != chair_broadcast_status::success)
		{
			p.second.cbs = chair_broadcast_status::failure;
		}
	}
}

void overwrite_trapped_chairs()
{
	for (auto &p : chair_status_map)
	{
		if (p.second.cts == chair_trapped_status::trapped)
		{
			p.second.cts = chair_trapped_status::not_trapped;
		}
	}
}

bool all_chairs_are_done()
{
	for (const auto &p : chair_status_map)
	{
		if (p.second.cbs == chair_broadcast_status::ready)
		{
			return false;
		}
	}
	return true;
}

bool a_chair_is_trapped()
{
	for (const auto &p : chair_status_map)
	{
		if (p.second.cts == chair_trapped_status::trapped)
		{
			return true;
		}
	}
	return false;
}

enum class state : char
{
	outside,
	awaiting_confirmation,
	awaiting_status
};
state mode = state::outside;

std::queue<std_msgs::String> transmit_queue;

void receive_callback(const std_msgs::String &msg)
{
	if (strlen(msg.data.c_str()) > 2)
	{
		ROS_ERROR("hub manager callback for: %s", msg.data.c_str());
	}

	// ignore malformed messages and heartbeats
	if (strlen(msg.data.c_str()) != 3)
	{
		return;
	}

	std::string stringmsg = std::string(msg.data.c_str());

	ROS_ERROR("getting chair number");

	// update chair status vector
	// format of str msg is {chair number}{chair status indicator}{new value}
	int chair_number = stringmsg[0] - 48;

	// return early if chair number isn't active
	// TODO: should we instead just check that it is a real number?
	if (std::find(active_chair_nums.begin(), active_chair_nums.end(), chair_number) == active_chair_nums.end())
	{
		return;
	}

	ROS_ERROR("getting chair property");

	char chair_property = stringmsg[1];

	// ignore malformed
	if (!(chair_property == 'B' || chair_property == 'S' || chair_property == 'T'))
	{
		return;
	}

	ROS_ERROR("getting property value");

	char property_value = stringmsg[2];

	switch (chair_property)
	{
	case 'B':
	{
		// TODO: error handle here if it's not in the enum?
		chair_status_map[chair_number].cbs = static_cast<chair_broadcast_status>(property_value);
		if (chair_status_map[chair_number].cbs == chair_broadcast_status::ready)
		{
			ROS_ERROR("CHAIR %d IS READY TO RECEIVE BROADCAST", chair_number);
		}
		if (chair_status_map[chair_number].cbs == chair_broadcast_status::exclude)
		{
			ROS_ERROR("CHAIR %d IS EXCLUDED FROM THIS BROADCAST", chair_number);
		}
		if (chair_status_map[chair_number].cbs == chair_broadcast_status::success)
		{
			ROS_ERROR("CHAIR %d SUCCESSFULLY COMPLETED BROADCAST", chair_number);
		}
		if (chair_status_map[chair_number].cbs == chair_broadcast_status::failure)
		{
			ROS_ERROR("CHAIR %d FAILED TO COMPLETE BROADCAST", chair_number);
		}
		break;
	}
	case 'S':
	{
		chair_status_map[chair_number].css = static_cast<chair_stuck_status>(property_value);
		if (chair_status_map[chair_number].css == chair_stuck_status::stuck)
		{
			ROS_ERROR("CHAIR %d IS STUCK", chair_number);
		}
		if (chair_status_map[chair_number].css == chair_stuck_status::not_stuck)
		{
			ROS_ERROR("CHAIR %d IS NOT STUCK", chair_number);
		}
		break;
	}
	case 'T':
	{
		chair_status_map[chair_number].cts = static_cast<chair_trapped_status>(property_value);
		if (chair_status_map[chair_number].cts == chair_trapped_status::trapped)
		{
			ROS_ERROR("CHAIR %d IS TRAPPED", chair_number);
		}
		if (chair_status_map[chair_number].cts == chair_trapped_status::not_trapped)
		{
			ROS_ERROR("CHAIR %d IS NOT TRAPPED", chair_number);
		}
		break;
	}
	default:
	{
		ROS_ERROR("INVALID CHAIR PROPERTY %s", chair_property);
		break;
	}
	}
}

void clean_up_after_broadcast_done()
{
	transmit_queue = std::queue<std_msgs::String>();
	mode = state::outside;
	std_msgs::String msg;
	msg.data = "00Bfinish";
	hub_manager_pub.publish(msg);
	ROS_ERROR("BROADCAST IS FINISHED");
	// clear transmit queue
	overwrite_trapped_chairs();
	overwrite_excluded_chairs();
}

void broadcast_callback(const std_msgs::String &msg)
{
	ROS_ERROR("BROADCAST CALLBACK FOR %s", msg.data.c_str());
	if (msg.data == "clear")
	{
		ROS_ERROR("CLEEEEEEEEEEEEEEEEEEAR");
		clean_up_after_broadcast_done();
	}
	transmit_queue.push(msg);
}

int main(int argc, char **argv)
{
	// initialize node and node handle
	ros::init(argc, argv, "hub_manager");
	ros::NodeHandle nh;

	// initialize chair map
	nh.getParam("active_chair_nums", active_chair_nums);

	for (int num : active_chair_nums)
	{
		chair_status_map[num] = chair_status();
	}

	// initialize spinner
	ros::AsyncSpinner spinner(0);
	spinner.start();

	// initialize subscribers
	ros::Subscriber sub1 = nh.subscribe("from_hub_receiver", 1000, receive_callback);
	ros::Subscriber sub2 = nh.subscribe("to_hub_manager", 1000, broadcast_callback);

	// initialize publishers
	hub_manager_pub = nh.advertise<std_msgs::String>("from_hub", 1000);

	mode = state::outside;
	while (ros::ok())
	{
		switch (mode)
		{
		case state::outside:
		{
			if (a_chair_is_trapped())
			{
				ROS_ERROR("A chair is trapped!");
				std_msgs::String msg;
				msg.data = "00Bf2.5r2.5t5";
				transmit_queue = std::queue<std_msgs::String>();
				transmit_queue.push(msg);
				transmit_queue.push(msg);
				transmit_queue.push(msg);
				transmit_queue.push(msg);
				msg.data = "00Bend";
				transmit_queue.push(msg);
			}
			if (!transmit_queue.empty())
			{
				// ROS_ERROR("NUM COMMANDS: %d", transmit_queue.size());
				// // Wait until entire broadcast is in the queue
				// if (transmit_queue.back().data == "00Bend")
				// {
				// 	mode = state::awaiting_confirmation;
				// }
				mode = state::awaiting_confirmation;
				// also transmit start of broadcast
				std_msgs::String msg;
				msg.data = "00Bstart";
				hub_manager_pub.publish(msg);
				startTime = ros::Time::now();
			}
			break;
		}
		case state::awaiting_confirmation:
		{
			// ROS_ERROR("awaiting confirmation");

			// // also transmit start of broadcast
			// std_msgs::String msg;
			// msg.data = "00Bstart";
			// hub_manager_pub.publish(msg);
			// wait until cbs is ready for all chairs
			// then transmit until end of broadcast stage
			if (!all_chairs_are_ready())
			{
				// Waited long enough, so check again
				if (ros::Time::now() >= startTime + waitDurationBeforeCheckingAgain)
				{
					std_msgs::String msg;
					msg.data = "00Bstart";
					hub_manager_pub.publish(msg);
					startTime = ros::Time::now();
				}
				break;
			}
			ROS_ERROR("ALL CHAIRS ARE READY");
			mode = state::awaiting_status;
			while (!transmit_queue.empty())
			{
				bool break_out = transmit_queue.front().data == "00Bend";
				hub_manager_pub.publish(transmit_queue.front());
				transmit_queue.pop();
				// if (break_out)
				// {
				// 	mode = state::awaiting_status;
				// 	break;
				// }
			}
			break;
		}
		case state::awaiting_status:
		{
			// wait until cbs is success / failure for all chairs
			// change state to outside
			// transmit end of broadcast message
			while (!all_chairs_are_done())
			{
				// pass
			}
			ROS_ERROR("ALL CHAIRS ARE DONE");
			clean_up_after_broadcast_done();
			break;
		}
		default:
		{
			ROS_ERROR("How did we get here? Hub manager edition");
			break;
		}
		}
	}
}
