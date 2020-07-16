#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "eyes/Custom.h"
#include "eyes/Choreo.h"
#include "eyes/Autonomous.h"

#include <ros/callback_queue.h>
#include <string>
#include <queue>
#include <sstream>

enum state : char {autonomous, choreo, custom};
state mode = state::autonomous;

std::queue<eyes::Autonomous> autonomous_queue;
std::queue<eyes::Choreo> choreo_queue;
std::queue<eyes::Custom> custom_queue;

ros::CallbackQueue side_queue;
bool notified = false;

void callback(const std_msgs::String& command) {
	// code coming soon
	// think sorting hat from Harry Potter
	char identifier = command.data[0];
	switch (identifier) {
		case 'H':
		{
			// parse handwritten command here (PLACEHOLDER)
			eyes::Custom custom_message;
			custom_message.command = command.data;
			custom_queue.push(custom_message);
			
			break;
		}
		case 'A':
		{
			// parse autonomous command here (PLACEHOLDER)
			eyes::Autonomous autonomous_message;
			autonomous_message.safety = false;
			autonomous_message.left_forward = true;
			autonomous_message.right_forward = true;
			autonomous_message.left_speed = 255;
			autonomous_message.right_speed = 255;
			autonomous_queue.push(autonomous_message);

			break;
		}
		case 'C':
		{
			// parse choreo command here (PLACEHOLDER)
			eyes::Choreo choreo_message;
			choreo_message.timed = false;
			choreo_message.duration = 20000;
			choreo_message.left_forward = false;
			choreo_message.right_forward = false;
			choreo_message.left_speed = 255;
			choreo_message.right_speed = 255;
			choreo_queue.push(choreo_message);

			choreo_message.timed = true;
			choreo_message.duration = 5;
			choreo_message.left_forward = false;
			choreo_message.right_forward = false;
			choreo_message.left_speed = 0;
			choreo_message.right_speed = 0;
			choreo_queue.push(choreo_message);

			choreo_message.timed = false;
			choreo_message.duration = 20000;
			choreo_message.left_forward = false;
			choreo_message.right_forward = false;
			choreo_message.left_speed = 255;
			choreo_message.right_speed = 255;
			choreo_queue.push(choreo_message);

			choreo_message.timed = true;
			choreo_message.duration = 5;
			choreo_message.left_forward = false;
			choreo_message.right_forward = false;
			choreo_message.left_speed = 0;
			choreo_message.right_speed = 0;
			choreo_queue.push(choreo_message);

			break;
		}
		default:
		{
			// parse safety(?) command here
			ROS_INFO("SAFETY PLACEHOLDER");
			break;
		}
	}
}

void side_callback(const std_msgs::Empty& notification) {
	notified = true;
}

ros::Publisher autonomous_pub;
ros::Publisher choreo_pub;
ros::Publisher custom_pub;

int main(int argc, char** argv) {
	ros::init(argc, argv, "queue_up");
	
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("driver_output", 1000, callback);

	ros::NodeHandle side;
	side.setCallbackQueue(&side_queue);
	ros::Subscriber side_sub = side.subscribe("notifications", 1000, side_callback);

	custom_pub = nh.advertise<eyes::Custom>("custom_feed", 1000);
	choreo_pub = nh.advertise<eyes::Choreo>("choreo_feed", 1000);
	autonomous_pub = nh.advertise<eyes::Autonomous>("autonomous_feed", 1000);

	while (ros::ok()) {
		if (!custom_queue.empty()) mode = state::custom;
		else if (!choreo_queue.empty()) mode = state::choreo;
		else mode = state::autonomous;

		switch (mode) {
			case state::autonomous:
			{
				if (!autonomous_queue.empty()) {
					autonomous_pub.publish(autonomous_queue.front());
					autonomous_queue.pop();
					ROS_INFO("PUBLISHING AUTONOMOUS COMMAND");
				}
				// else do nothing, wait

				break;
			}
			case state::choreo:
			{
				choreo_pub.publish(choreo_queue.front());
				choreo_queue.pop();
				ROS_INFO("PUBLISHING CHOREO COMMAND");
				while (!notified) {
					side_queue.callOne();
					ros::spinOnce(); // potential source of instability in this line
				}
				notified = false;
				autonomous_queue = std::queue<eyes::Autonomous>();

				break;
			}
			case state::custom:
			{
				while (!notified) {
					if (!custom_queue.empty()) {
						custom_pub.publish(custom_queue.front());
						custom_queue.pop();
						ROS_INFO("PUBLISHING CUSTOM COMMAND");
					}
					side_queue.callOne();
					ros::spinOnce(); // potential source of instability in this line
				}
				notified = false;
				autonomous_queue = std::queue<eyes::Autonomous>();
				choreo_queue = std::queue<eyes::Choreo>();

				break;
			}
			default:
			{
				ROS_INFO("How did we get here?");
				break;
			}
		}
		ros::spinOnce();
	}
}
