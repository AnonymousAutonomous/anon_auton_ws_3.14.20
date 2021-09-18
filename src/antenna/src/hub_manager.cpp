#include "ros/ros.h"
#include "std_msgs/String.h"

#include <ros/spinner.h>
#include <queue>
#include <vector>

#define NUMBER_OF_CHAIRS 1

enum chair_broadcast_status : char {ready, success, failure};
enum chair_stuck_status : char {stuck, not_stuck};

struct chair_status {
	chair_broadcast_status cbs = chair_broadcast_status::success;
	chair_stuck_status css = chair_stuck_status::not_stuck;
	// being user-controlled? low battery? surrounded? current command? connection break?
};

std::vector<chair_status> chair_status_vector(5);

enum state : char {outside, awaiting_confirmation, awaiting_status};
state mode = state::outside;

std::queue<std_msgs::String> transmit_queue;

void receive_callback(const std_msgs::String& msg) {
	// update chair status vector
	// format of str msg is {chair number}{chair status indicator}{new value}
	int chair_number = msg.data[0] - 48;
	ROS_INFO("UPDATING STATUS OF CHAIR %d", chair_number);

	char chair_property = msg.data[1];
	char property_value = msg.data[2];

	switch (chair_property) {
		case 'B':
		{
			chair_status_vector[chair_number].cbs = (chair_broadcast_status)(property_value);
			if (chair_status_vector[chair_number].cbs == chair_broadcast_status::ready) {
				ROS_INFO("CHAIR %d IS READY TO RECEIVE BROADCAST", chair_number);
			}
			break;
		}
		case 'S':
		{
			chair_status_vector[chair_number].css = (chair_stuck_status)(property_value);
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
				if (!transmit_queue.empty()) {
					mode = state::awaiting_confirmation;
					// also transmit start of broadcast
					std_msgs::String msg;
					msg.data = "0Bstart";
					hub_manager_pub.publish(msg);
				}
				break;
			}
			case state::awaiting_confirmation:
			{
				// wait until cbs is ready for all chairs
				// then transmit until end of broadcast stage
				break;
			}
			case state::awaiting_status:
			{
				// wait until cbs is success / failure for all chairs
				// change state to outside
				// transmit end of broadcast message
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
