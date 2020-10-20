#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "eyes/Generic.h"
#include "../../../constants/choreos.h"

#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <string>
#include <queue>
#include <sstream>

enum state : char {autonomous, choreo, custom, broadcast};
state mode = state::autonomous;

std::queue<eyes::Generic> autonomous_queue;
std::queue<eyes::Generic> choreo_queue;
std::queue<eyes::Generic> custom_queue;
std::queue<eyes::Generic> broadcast_queue;

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
	char identifier = command.data[1];
	switch (identifier) {
		case 'H':
		{
			// parse handwritten command here (PLACEHOLDER)
			eyes::Generic generic_message;
			generic_message.identifier = 'h';
			generic_message.timed = false; // inconsequential
			generic_message.duration = 0; // inconsequential

			if (command.data == "0Htoggle") {
				ROS_INFO("DEFAULT TO TOGGLE");
				generic_message.identifier = 't';
				generic_message.left_forward = true; // inconsequential
				generic_message.right_forward = true; // inconsequential
				generic_message.left_speed = 0; // inconsequential
				generic_message.right_speed = 0; // inconsequential
			}
			else {
				generic_message.left_forward = (command.data[2] == 'f' ? true : false);
				generic_message.right_forward =	(command.data[6] == 'f' ? true : false);
				std::string ls = command.data.substr(3,3);
				generic_message.left_speed = atoi(ls.c_str());
				std::string rs = command.data.substr(7,3);
				generic_message.right_speed = atoi(rs.c_str());
			}

			custom_queue.push(generic_message);
			break;
		}
		case 'A':
		{
			// parse autonomous command here (PLACEHOLDER)
			eyes::Generic generic_message;
			generic_message.identifier = 'a';
			generic_message.left_forward = (command.data[2] == 'f' ? true : false);
			generic_message.right_forward = (command.data[6] == 'f' ? true : false);
			std::string ls = command.data.substr(3,3);
			generic_message.left_speed = atoi(ls.c_str());
			std::string rs = command.data.substr(7,3);
			generic_message.right_speed = atoi(rs.c_str());
			generic_message.timed = false; // inconsequential
			generic_message.duration = 0; // inconsequential

			autonomous_queue.push(generic_message);
			break;
		}
		case 'C':
		{
			// parse choreo command here (PLACEHOLDER)
			switch (command.data[2]) {
				case 'C':
				{
					choreo_queue.push(DANCE_C[0]);
					choreo_queue.push(DANCE_C[1]);
					choreo_queue.push(DANCE_C[2]);
					choreo_queue.push(DANCE_C[3]);
					choreo_queue.push(DANCE_C[4]);
					break;
				}
				case 'D':
				{
					choreo_queue.push(RREVERSE_C[0]);
					choreo_queue.push(RREVERSE_C[1]);
					choreo_queue.push(RREVERSE_C[2]);
					choreo_queue.push(RREVERSE_C[3]);
					break;
				}
				case 'E':
				{
					choreo_queue.push(LREVERSE_C[0]);
					choreo_queue.push(LREVERSE_C[1]);
					choreo_queue.push(LREVERSE_C[2]);
					choreo_queue.push(LREVERSE_C[3]);
					break;
				}
				case 'H':
				{
					choreo_queue.push(SPIN_C[0]);
					choreo_queue.push(SPIN_C[1]);
					choreo_queue.push(SPIN_C[2]);
					break;
				}
				case 'I':
				{
					choreo_queue.push(LCP_C[0]);
					choreo_queue.push(LCP_C[1]);
					break;
				}
				case 'J':
				{
					choreo_queue.push(RCP_C[0]);
					choreo_queue.push(RCP_C[1]);
					break;
				}
				default:
				{
					// haha choreo go brrrr
					ROS_INFO("INVALID CHOREO SHORTCODE");
					break;
				}
			}
			break;
		}
		case 'B':
		{
			// parse broadcast command here (PLACEHOLDER)
			eyes::Generic generic_message;
			generic_message.identifier = 'b';
			generic_message.left_forward = (command.data[2] == 'f' ? true : false);
			generic_message.right_forward = (command.data[6] == 'f' ? true : false);
			std::string ls = command.data.substr(3,3);
			generic_message.left_speed = atoi(ls.c_str());
			std::string rs = command.data.substr(7,3);
			generic_message.right_speed = atoi(rs.c_str());
			generic_message.timed = (command.data[10] == 't' ? true : false);
			std::string dur = command.data.substr(11);
			generic_message.duration = atoi(dur.c_str());

			broadcast_queue.push(generic_message);
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
ros::Publisher eoc_pub;

int main(int argc, char** argv) {
	ros::init(argc, argv, "queue_four");	
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(0);
	spinner.start();

	ros::Subscriber sub = nh.subscribe("driver_output", 1000, callback);

	ros::NodeHandle side;
	side.setCallbackQueue(&side_queue);
	ros::Subscriber side_sub = side.subscribe("notifications", 1000, side_callback);

	generic_pub = nh.advertise<eyes::Generic>("queue_output", 1000);
	eoc_pub = nh.advertise<std_msgs::Empty>("end_of_choreo", 1000);
	std_msgs::Empty empty_msg;

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
				if (choreo_queue.front().identifier == 'e') {
					ROS_INFO("END OF CHOREO");
					eoc_pub.publish(empty_msg);
					choreo_queue = std::queue<eyes::Generic>();
				}
				else {
					generic_pub.publish(choreo_queue.front());
					choreo_queue.pop();
					ROS_INFO("PUBLISHING CHOREO COMMAND");
					wait_for_notification();
				}
				
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
			case state::broadcast:
			{
				generic_pub.publish(broadcast_queue.front());
				broadcast_queue.pop();
				ROS_INFO("PUBLISHING BROADCAST COMMAND");
				// wait for notification of end of broadcast stage
				
				autonomous_queue = std::queue<eyes::Generic>();
				if (!custom_queue.empty()) mode = state::custom;
				else if (!broadcast_queue.empty()) mode = state::broadcast;
				else if (!choreo_queue.empty()) mode = state::choreo;
				else mode = state::autonomous;

				/*** BAD CODE FOLLOWS ***
				if (!broadcast_queue.empty()) {
					generic_pub.publish(broadcast_queue.front());
					broadcast_queue.pop();
					ROS_INFO("PUBLISHING BROADCAST COMMAND");
				}
				else {
					mode = state::autonomous;
					autonomous_queue = std::queue<eyes::Generic>();
					choreo_queue = std::queue<eyes::Generic>();

					ROS_INFO("END OF BROADCAST");
				}
				*** END OF BAD CODE ***/

				// chair should choose handwritten command if present
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
