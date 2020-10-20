#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "eyes/Generic.h"

#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <string>
#include <queue>
#include <sstream>

enum state : char {autonomous, choreo, custom};
state mode = state::autonomous;

std::queue<eyes::Generic> autonomous_queue;
std::queue<eyes::Generic> choreo_queue;
std::queue<eyes::Generic> custom_queue;

ros::CallbackQueue side_queue;
bool notified = false;

void wait_for_notification() {
	while (!notified) {
		side_queue.callOne();
	}
	notified = false;
}

void callback(const std_msgs::String& command) {
	// code coming soon
	// think sorting hat from Harry Potter
	char identifier = command.data[0];
	switch (identifier) {
		case 'H':
		{
			// parse handwritten command here (PLACEHOLDER)
			eyes::Generic generic_message;
			generic_message.identifier = 'h';
			generic_message.timed = false; // inconsequential
			generic_message.duration = 0; // inconsequential

			if (command.data == "Hfwd") {
				ROS_INFO("FORWARD RECEIVED");
				generic_message.left_forward = true;
				generic_message.right_forward = true;
				generic_message.left_speed = 255;
				generic_message.right_speed = 255;
			}
			else if (command.data == "Hbwd") {
				ROS_INFO("BACKWARD RECEIVED");
				generic_message.left_forward = false;
				generic_message.right_forward = false;
				generic_message.left_speed = 255;
				generic_message.right_speed = 255;
			}
			else if (command.data == "Hstop") {
				ROS_INFO("STOP RECEIVED");
				generic_message.left_forward = true;
				generic_message.right_forward = true;
				generic_message.left_speed = 0;
				generic_message.right_speed = 0;
			}
			else { // command.data == "Htoggle"
				ROS_INFO("DEFAULT TO TOGGLE");
				generic_message.identifier = 't';
				generic_message.left_forward = true; // inconsequential
				generic_message.right_forward = true; // inconsequential
				generic_message.left_speed = 0; // inconsequential
				generic_message.right_speed = 0; // inconsequential
			}

			custom_queue.push(generic_message);
			break;
		}
		case 'A':
		{
			// parse autonomous command here (PLACEHOLDER)
			eyes::Generic generic_message;
			generic_message.identifier = 'a';
			generic_message.left_forward = true;
			generic_message.right_forward = true;
			generic_message.left_speed = 255;
			generic_message.right_speed = 255;
			generic_message.timed = false; // inconsequential
			generic_message.duration = 0; // inconsequential

			autonomous_queue.push(generic_message);
			break;
		}
		case 'C':
		{
			// parse choreo command here (PLACEHOLDER)
			eyes::Generic generic_message;
			generic_message.identifier = 'c';
			generic_message.left_forward = false;
			generic_message.right_forward = false;
			generic_message.left_speed = 255;
			generic_message.right_speed = 255;
			generic_message.timed = false;
			generic_message.duration = 20000;

			choreo_queue.push(generic_message);

			generic_message.left_speed = 0;
			generic_message.right_speed = 0;
			generic_message.timed = true;
			generic_message.duration = 5;

			choreo_queue.push(generic_message);

			generic_message.left_speed = 255;
			generic_message.right_speed = 255;
			generic_message.timed = false;
			generic_message.duration = 20000;

			choreo_queue.push(generic_message);

			generic_message.left_speed = 0;
			generic_message.right_speed = 0;
			generic_message.timed = true;
			generic_message.duration = 5;

			choreo_queue.push(generic_message);
			break;
		}
		default:
		{
			// parse safety(?) command here (PLACEHOLDER)
			break;
		}
	}
}

void side_callback(const std_msgs::Empty& notification) {
	notified = true;
}

ros::Publisher generic_pub;

int main(int argc, char** argv) {
	ros::init(argc, argv, "queue_two");	
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(0);
	spinner.start();

	ros::Subscriber sub = nh.subscribe("driver_output", 1000, callback);

	ros::NodeHandle side;
	side.setCallbackQueue(&side_queue);
	ros::Subscriber side_sub = side.subscribe("notifications", 1000, side_callback);

	generic_pub = nh.advertise<eyes::Generic>("queue_output", 1000);

	mode = state::autonomous;
	while (ros::ok()) {
		switch (mode) {
			case state::autonomous:
			{
				if (!autonomous_queue.empty()) {
					generic_pub.publish(autonomous_queue.front());
					autonomous_queue.pop();
					ROS_INFO("PUBLISHING AUTONOMOUS COMMAND");
				}

				if (!custom_queue.empty()) mode = state::custom;
				else if (!choreo_queue.empty()) mode = state::choreo;
				else mode = state::autonomous;

				break;
			}
			case state::choreo:
			{
				generic_pub.publish(choreo_queue.front());
				choreo_queue.pop();
				ROS_INFO("PUBLISHING CHOREO COMMAND");
				wait_for_notification();
				autonomous_queue = std::queue<eyes::Generic>();

				if (!custom_queue.empty()) mode = state::custom;
				else if (!choreo_queue.empty()) mode = state::choreo;
				else mode = state::autonomous;

				break;
			}
			case state::custom:
			{
				if (!custom_queue.empty()) {
					if (custom_queue.front().identifier == 't') {
						custom_queue.pop();
						mode = state::autonomous;
						autonomous_queue = std::queue<eyes::Generic>();
						choreo_queue = std::queue<eyes::Generic>();
						custom_queue = std::queue<eyes::Generic>();
						ROS_INFO("TOGGLE");
					}
					else {
						generic_pub.publish(custom_queue.front());
						custom_queue.pop();
						ROS_INFO("PUBLISHING CUSTOM COMMAND");
					}
				}
				break;
			}
			default:
			{
				ROS_INFO("How did we get here?");
				break;
			}
		}
	}
}
