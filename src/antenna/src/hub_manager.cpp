#include "ros/ros.h"
#include "std_msgs/String.h"

#include <ros/spinner.h>
#include <queue>
#include <vector>

#define NUMBER_OF_CHAIRS 1

enum class chair_broadcast_status : char {ready ='r', exclude='e', success='s', failure='f'};
enum class chair_stuck_status : char {stuck, not_stuck};
enum class chair_trapped_status : char {trapped, not_trapped};

struct chair_status {
	chair_broadcast_status cbs = chair_broadcast_status::success;
	chair_stuck_status css = chair_stuck_status::not_stuck;
	chair_trapped_status cts = chair_trapped_status::not_trapped;
	// being user-controlled? low battery? surrounded? current command? connection break?
};

std::vector<chair_status> chair_status_vector(NUMBER_OF_CHAIRS);

bool all_chairs_are_ready() {
	for (chair_status& status : chair_status_vector) {
		if (status.cbs != chair_broadcast_status::ready && status.cbs != chair_broadcast_status::exclude) {
			return false;
		}
	}
	return true;
}

void overwrite_excluded_chairs() {
	for (chair_status& status : chair_status_vector) {
		if (status.cbs == chair_broadcast_status::exclude) {
			status.cbs = chair_broadcast_status::failure;
		}
	}
}

void overwrite_trapped_chairs() {
	for (chair_status& status : chair_status_vector) {
		if (status.cts == chair_trapped_status::trapped) {
			status.cts = chair_trapped_status::not_trapped;
		}
	}
}

bool all_chairs_are_done() {
	for (chair_status& status : chair_status_vector) {
		if (status.cbs == chair_broadcast_status::ready) {
			return false;
		}
	}
	return true;
}

bool a_chair_is_trapped() {
	for (chair_status& status : chair_status_vector) {
		if (status.cts == chair_trapped_status::trapped) {
			return true;
		}
	}
	return false;
}

enum class state : char {outside, awaiting_confirmation, awaiting_status};
state mode = state::outside;

std::queue<std_msgs::String> transmit_queue;

void receive_callback(const std_msgs::String& msg) {
	// update chair status vector
	// format of str msg is {chair number}{chair status indicator}{new value}
	int chair_number = msg.data[0] - 48 - 1;
	ROS_INFO("UPDATING STATUS OF CHAIR %d", chair_number);

	char chair_property = msg.data[1];
	char property_value = msg.data[2];

	switch (chair_property) {
		case 'B':
		{
			chair_status_vector[chair_number].cbs = static_cast<chair_broadcast_status>(property_value);
			if (chair_status_vector[chair_number].cbs == chair_broadcast_status::ready) {
				ROS_INFO("CHAIR %d IS READY TO RECEIVE BROADCAST", chair_number);
			}
			if (chair_status_vector[chair_number].cbs == chair_broadcast_status::exclude) {
				ROS_INFO("CHAIR %d IS EXCLUDED FROM THIS BROADCAST", chair_number);
			}
			if (chair_status_vector[chair_number].cbs == chair_broadcast_status::success) {
				ROS_INFO("CHAIR %d SUCCESSFULLY COMPLETED BROADCAST", chair_number);
			}
			if (chair_status_vector[chair_number].cbs == chair_broadcast_status::failure) {
				ROS_INFO("CHAIR %d FAILED TO COMPLETE BROADCAST", chair_number);
			}
			break;
		}
		case 'S':
		{
			chair_status_vector[chair_number].css = static_cast<chair_stuck_status>(property_value);
			if (chair_status_vector[chair_number].css == chair_stuck_status::stuck) {
				ROS_INFO("CHAIR %d IS STUCK", chair_number);
			}
			if (chair_status_vector[chair_number].css == chair_stuck_status::not_stuck) {
				ROS_INFO("CHAIR %d IS NOT STUCK", chair_number);
			}
			break;
		}
		case 'T':
		{
			chair_status_vector[chair_number].cts = static_cast<chair_trapped_status>(property_value);
			if (chair_status_vector[chair_number].cts == chair_trapped_status::trapped) {
				ROS_INFO("CHAIR %d IS TRAPPED", chair_number);
			}
			if (chair_status_vector[chair_number].cts == chair_trapped_status::not_trapped) {
				ROS_INFO("CHAIR %d IS NOT TRAPPED", chair_number);
			}
			break;
		}
		default:
		{
			ROS_INFO("INVALID CHAIR PROPERTY");
			break;
		}
	}
}

void broadcast_callback(const std_msgs::String& msg) {
	transmit_queue.push(msg);
}

ros::Publisher hub_manager_pub;

int main (int argc, char** argv) {
	// initialize node and node handle
	ros::init(argc, argv, "hub_manager");
	ros::NodeHandle nh;

	// initialize spinner
	ros::AsyncSpinner spinner(0);
	spinner.start();

	// initialize subscribers
	ros::Subscriber sub1 = nh.subscribe("from_hub_receiver", 1000, receive_callback);
	ros::Subscriber sub2 = nh.subscribe("to_hub_manager", 1000, broadcast_callback);

	// initialize publishers
	hub_manager_pub = nh.advertise<std_msgs::String>("from_hub", 1000);

	mode = state::outside;
	while (ros::ok()) {
		switch (mode) {
			case state::outside:
			{
				if (a_chair_is_trapped()) {
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
				if (!transmit_queue.empty()) {
					mode = state::awaiting_confirmation;
					// also transmit start of broadcast
					std_msgs::String msg;
					msg.data = "00Bstart";
					hub_manager_pub.publish(msg);
				}
				break;
			}
			case state::awaiting_confirmation:
			{
				// wait until cbs is ready for all chairs
				// then transmit until end of broadcast stage
				while (!all_chairs_are_ready()) {
					// pass
				}
				ROS_INFO("ALL CHAIRS ARE READY");
				mode = state::awaiting_status;
				while (!transmit_queue.empty()) {
					bool break_out = transmit_queue.front().data == "00Bend";
					hub_manager_pub.publish(transmit_queue.front());
					transmit_queue.pop();
					if (break_out) break;
				}
				break;
			}
			case state::awaiting_status:
			{
				// wait until cbs is success / failure for all chairs
				// change state to outside
				// transmit end of broadcast message
				while (!all_chairs_are_done()) {
					// pass
				}
				ROS_INFO("ALL CHAIRS ARE DONE");
				mode = state::outside;
				std_msgs::String msg;
				msg.data = "00Bfinish";
				hub_manager_pub.publish(msg);
				ROS_INFO("BROADCAST IS FINISHED");
				overwrite_trapped_chairs();
				overwrite_excluded_chairs();
				break;
			}
			default:
			{
				ROS_INFO("How did we get here? Hub manager edition");
				break;
			}
		} 
	}
}
