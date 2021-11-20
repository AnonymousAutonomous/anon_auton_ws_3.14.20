// nice, seems good to me? I think so, but just in case

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int32.h"
#include "eyes/Generic.h"
#include "../../../constants/choreos.h"

#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <string>
#include <queue>
#include <sstream>
#include <ctime>
#include <cmath>

enum class state : char {autonomous, choreo, custom, broadcast};
state mode = state::autonomous;

enum class broadcast_state : char {outside, ready, wait};
broadcast_state broadcast_mode = broadcast_state::outside;

// mirror chair status enums in hub manager
enum class chair_broadcast_status : char {ready, exclude, success, failure};
enum class chair_stuck_status : char {stuck, not_stuck};

#define flag_A !autonomous_queue.empty()
#define flag_B !broadcast_queue.empty()
#define flag_C !choreo_queue.empty()
#define flag_H !custom_queue.empty()

bool flag_S = false; // stuck

bool flag_EOC = false; // end of choreo
bool flag_SOB = false; // start of broadcast
bool flag_EOB = false; // end of broadcast

std::queue<eyes::Generic> autonomous_queue;
std::queue<eyes::Generic> choreo_queue;
std::queue<eyes::Generic> custom_queue;
std::queue<eyes::Generic> broadcast_queue;

std::queue<eyes::Generic> choreo_buffer;
std::queue<eyes::Generic> broadcast_buffer;

volatile int encoder_count_R = 0;
volatile int encoder_count_L = 0;

time_t initial_time;
time_t final_time;

int initial_encoder_value;
int final_encoder_value;

void update_encoder_values_R(const std_msgs::Int32& int32_msg_R) {
	encoder_count_R = int32_msg_R.data;
	// ROS_INFO("RIGHT: %d", encoder_count_R);
}

void update_encoder_values_L(const std_msgs::Int32& int32_msg_L) {
	encoder_count_L = int32_msg_L.data;
	// ROS_INFO("LEFT: %d", encoder_count_L);
}

void callback(const std_msgs::String& command) {
	// code coming soon
	// think sorting hat from Harry Potter
	char identifier = command.data[1];
	switch (identifier) {
		case 'H':
		{
			// parse handwritten command here (PLACEHOLDER)
			if (command.data == "0Htoggle") {
				if (mode == state::custom) {
					ROS_INFO("TOGGLE");
					mode = state::autonomous;
					autonomous_queue = std::queue<eyes::Generic>();
					choreo_queue = std::queue<eyes::Generic>();
					custom_queue = std::queue<eyes::Generic>();
					broadcast_queue = std::queue<eyes::Generic>();
					flag_SOB = false;
					flag_EOB = false;
				}
			}
			else {
				eyes::Generic generic_message;
				generic_message.identifier = 'h';
				generic_message.timed = false; // inconsequential
				generic_message.duration = 0; // inconsequential
				generic_message.left_forward = (command.data[2] == 'f' ? true : false);
				generic_message.right_forward =	(command.data[6] == 'f' ? true : false);
				std::string ls = command.data.substr(3,3);
				generic_message.left_speed = atoi(ls.c_str());
				std::string rs = command.data.substr(7,3);
				generic_message.right_speed = atoi(rs.c_str());
				custom_queue.push(generic_message);
			}
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
				case 'K':
				{
					choreo_queue.push(AUDIO_C[0]);
					choreo_queue.push(AUDIO_C[1]);
					choreo_queue.push(AUDIO_C[2]);
					choreo_queue.push(AUDIO_C[3]);
					choreo_queue.push(AUDIO_C[4]);
					choreo_queue.push(AUDIO_C[5]);
					choreo_queue.push(AUDIO_C[6]);
					choreo_queue.push(AUDIO_C[7]);
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
			// parse broadcast command here (PLACEHOLDER), need end of broadcast somehow
			if (command.data == "0Bstart") {
				ROS_INFO("START OF BROADCAST");
				flag_SOB = true;
			}
			else if (command.data == "0Bfinish") {
				ROS_INFO("FINISHED BROADCAST");
				flag_EOB = true;
			}
			else if (command.data == "0Bend") {
				// ROS_INFO("END OF BROADCAST");
				eyes::Generic generic_message;
				generic_message.identifier = 'e';
				generic_message.left_forward = true;
				generic_message.right_forward = true;
				generic_message.left_speed = 0;
				generic_message.right_speed = 0;
				generic_message.timed = true;
				generic_message.duration = 0;
				broadcast_queue.push(generic_message);
			}
			else if (command.data[2] == 'A') {
				eyes::Generic generic_message;
				generic_message.identifier = command.data[3];
				generic_message.left_forward = true;
				generic_message.right_forward = true;
				generic_message.left_speed = 0;
				generic_message.right_speed = 0;
				generic_message.timed = true;
				generic_message.duration = 0;
				broadcast_queue.push(generic_message);
			}
			else {
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
			}
			break;
		}
		default:
		{
			// parse safety(?) command here (PLACEHOLDER)
			break;
		}
	}
}

ros::Publisher generic_pub;
ros::Publisher eoc_pub;
ros::Publisher update_hub_pub;
ros::Publisher audio_pub;

int main(int argc, char** argv) {
	ros::init(argc, argv, "queue_six");	
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(0);
	spinner.start();

	ros::Subscriber sub = nh.subscribe("driver_output", 1000, callback);
	ros::Subscriber sub_R = nh.subscribe("encoder_value_R", 10, update_encoder_values_R);
	ros::Subscriber sub_L = nh.subscribe("encoder_value_L", 10, update_encoder_values_L);

	generic_pub = nh.advertise<eyes::Generic>("generic_feed", 1000);
	eoc_pub = nh.advertise<std_msgs::Empty>("end_of_choreo", 1000);
	update_hub_pub = nh.advertise<std_msgs::String>("from_chair", 1000);
	audio_pub = nh.advertise<std_msgs::String>("audio_channel", 1000);

	mode = state::autonomous;
	while (ros::ok()) {
		switch (mode) {
			case state::autonomous: // matches FSM
			{
				if (flag_A) {
					generic_pub.publish(autonomous_queue.front());
					autonomous_queue.pop();
					// ROS_INFO("PUBLISHING AUTONOMOUS COMMAND");
				}

				// state transition logic (WIP)
				if (flag_H) {
					mode = state::custom;
					// flag_SOB = false;
				}
				else if (flag_SOB) {
					mode = state::broadcast;
					flag_SOB = false;
					flag_EOB = false;
				}
				else if (flag_C) mode = state::choreo;
				else mode = state::autonomous;

				break;
			}
			case state::choreo: // potential issue if choreo queue is empty, only possible if choreo stages are not received fast enough following transition from autonomous state
			{
				if (choreo_queue.front().identifier == 'e') {
					flag_EOC = true;
					ROS_INFO("END OF CHOREO");
					std_msgs::Empty empty_msg;
					eoc_pub.publish(empty_msg);
					choreo_queue = std::queue<eyes::Generic>();
				}
				else if (choreo_queue.front().identifier == 'p') {
					flag_EOC = false;
					// beep
					std_msgs::String str_msg;
					str_msg.data = "beep";
					audio_pub.publish(str_msg);
					ROS_INFO("BEEP");
				}
				else if (choreo_queue.front().identifier == 'k') {
					flag_EOC = false;
					// honk
					std_msgs::String str_msg;
					str_msg.data = "honk";
					audio_pub.publish(str_msg);
					ROS_INFO("HONK");
				}
				else if (choreo_queue.front().identifier == 't') {
					flag_EOC = false;
					// low battery
					std_msgs::String str_msg;
					str_msg.data = "batt";
					audio_pub.publish(str_msg);
					ROS_INFO("BATT");
				}
				else {
					flag_EOC = false;
					// generic_pub.publish(choreo_queue.front());
					// ROS_INFO("PUBLISHING CHOREO COMMAND");
					// duration, replaces wait_for_notification();
					// if choreo stage uses encoder motors
					if (choreo_queue.front().timed == false) {
						if (choreo_buffer.empty()) {
							// populate choreo buffer and record initial_value
							choreo_buffer.push(choreo_queue.front());
							initial_encoder_value = encoder_count_R;
							generic_pub.publish(choreo_buffer.front());
							ROS_INFO("STARTING (ENCODER)");
							ROS_INFO("LEFT SPEED: %d", choreo_buffer.front().left_speed);
							ROS_INFO("RIGHT SPEED: %d", choreo_buffer.front().right_speed);
							ROS_INFO("DURATION: %d", choreo_buffer.front().duration);
						}
						else {
							final_encoder_value = encoder_count_R;
							int difference = abs(final_encoder_value - initial_encoder_value);
							// ROS_INFO("ENCODER DIF: %d", difference);
							if (difference > choreo_buffer.front().duration) choreo_buffer.pop();
						}
					}
					// if choreo stage uses a timer
					else {
						if (choreo_buffer.empty()) {
							// populate choreo buffer and record initial_value
							choreo_buffer.push(choreo_queue.front());
							time(&initial_time);
							generic_pub.publish(choreo_buffer.front());
							ROS_INFO("STARTING (TIMER)");
							ROS_INFO("LEFT SPEED: %d", choreo_buffer.front().left_speed);
							ROS_INFO("RIGHT SPEED: %d", choreo_buffer.front().right_speed);
							ROS_INFO("DURATION: %d", choreo_buffer.front().duration);
						}
						else {
							time(&final_time);
							int difference = difftime(final_time, initial_time);
							// ROS_INFO("TIMER DIF: %f", difference);
							if (difference > choreo_buffer.front().duration) choreo_buffer.pop();
						}
					}
				}
				
				// state transition logic (WIP)
				if (choreo_buffer.empty() && !flag_EOC) {
					// ROS_INFO("JUST FINISHED");
					// ROS_INFO("RIGHT SPEED: %d", choreo_queue.front().right_speed);
					choreo_queue.pop();
					// ROS_INFO("POP!");
				}
				if (flag_H) {
					mode = state::custom;
					choreo_buffer = std::queue<eyes::Generic>(); // <-- exit case, resetting choreo buffer for next time
					// flag_SOB = false;
				}
				else if (flag_SOB) {
					mode = state::broadcast;
					choreo_buffer = std::queue<eyes::Generic>(); // <-- exit case, resetting choreo buffer for next time
					flag_SOB = false;
					flag_EOB = false;
				}
				else if (flag_C) {
					mode = state::choreo;
				}
				else {
					mode = state::autonomous;
					autonomous_queue = std::queue<eyes::Generic>();
				}

				break;
			}
			case state::custom:
			{
				if (flag_SOB) {
					std_msgs::String to_hub;
					to_hub.data = "0B";
					to_hub.data.push_back(static_cast<char>(chair_broadcast_status::exclude));
					update_hub_pub.publish(to_hub);
					ROS_INFO("CHAIR 0 IS EXCLUDED FROM BROADCAST");
					flag_SOB = false;
				}
				else if (flag_H) {
					generic_pub.publish(custom_queue.front());
					custom_queue.pop();
					ROS_INFO("PUBLISHING CUSTOM COMMAND");
				}
				break;
			}
			case state::broadcast: // WIP! WIP! WIP!
			{
				// ROS_INFO("IN BROADCAST STATE");
				switch (broadcast_mode) {
					case broadcast_state::outside:
					{
						// ROS_INFO("ARRIVED IN BROADCAST STATE");
						broadcast_mode = broadcast_state::ready;
						eyes::Generic stop;
						stop.identifier = 'b';
						stop.left_forward = true;
						stop.right_forward = true;
						stop.left_speed = 0;
						stop.right_speed = 0;
						stop.timed = false; // inconsequential
						stop.duration = 0; // inconsequential
						generic_pub.publish(stop);
						// TODO: PUBLISH INDICATION THAT CHAIR IS READY TO GO TO INFORM HUB
						std_msgs::String to_hub;
						to_hub.data = "0B";
						to_hub.data.push_back(static_cast<char>(chair_broadcast_status::ready));
						update_hub_pub.publish(to_hub);
						ROS_INFO("CHAIR 0 IS READY");
						break;
					}
					case broadcast_state::ready: // absorbed performing
					{
						// ROS_INFO("AWAITING BROADCAST");
						if (flag_B) {
							if (broadcast_queue.front().identifier == 'e') {
								ROS_INFO("LAST STAGE OF BROADCAST");
								broadcast_mode = broadcast_state::wait;
								// safety stop
								generic_pub.publish(broadcast_queue.front());
								broadcast_queue = std::queue<eyes::Generic>();
								// TODO: PUBLISH INDICATION THAT CHAIR COMPLETED BROADCAST SUCCESSFULLY
								std_msgs::String to_hub;
								to_hub.data = "0B";
								to_hub.data.push_back(static_cast<char>(chair_broadcast_status::success));
								update_hub_pub.publish(to_hub);
								ROS_INFO("CHAIR 0 SUCCESSFULLY COMPLETED BROADCAST");
							}
							else if (broadcast_queue.front().identifier == 'p') {
								// beep
								std_msgs::String str_msg;
								str_msg.data = "beep";
								audio_pub.publish(str_msg);
								// pop
								broadcast_queue.pop();
							}
							else if (broadcast_queue.front().identifier == 'k') {
								// honk
								std_msgs::String str_msg;
								str_msg.data = "honk";
								audio_pub.publish(str_msg);
								// pop
								broadcast_queue.pop();
							}
							else if (broadcast_queue.front().identifier == 't') {
								// low battery
								std_msgs::String str_msg;
								str_msg.data = "batt";
								audio_pub.publish(str_msg);
								// pop
								broadcast_queue.pop();
							}
							else {
								// duration, replaces wait_for_notification();
								// if broadcast stage uses encoder motors
								if (broadcast_queue.front().timed == false) {
									if (broadcast_buffer.empty()) {
										// populate broadcast buffer and record initial_value
										broadcast_buffer.push(broadcast_queue.front());
										initial_encoder_value = encoder_count_R;
										generic_pub.publish(broadcast_buffer.front());
									}
									else {
										final_encoder_value = encoder_count_R;
										int difference = abs(final_encoder_value - initial_encoder_value);
										ROS_INFO("ENCODER DIF: %d", difference);
										if (difference > broadcast_buffer.front().duration) broadcast_buffer.pop();
									}
								}
								// if broadcast stage uses a timer
								else {
									if (broadcast_buffer.empty()) {
										// populate broadcast buffer and record initial_value
										broadcast_buffer.push(broadcast_queue.front());
										time(&initial_time);
										generic_pub.publish(broadcast_buffer.front());
										ROS_INFO("STARTING");
										ROS_INFO("RIGHT SPEED: %d", broadcast_buffer.front().right_speed);
									}
									else {
										time(&final_time);
										int difference = difftime(final_time, initial_time);
										// ROS_INFO("TIMER DIF: %f", difference);
										if (difference > broadcast_buffer.front().duration) broadcast_buffer.pop();
									}
								}
							}
							// if block should never execute if broadcast_queue is empty
							if (broadcast_buffer.empty() && broadcast_mode == broadcast_state::ready) {
								ROS_INFO("JUST FINISHED");
								ROS_INFO("RIGHT SPEED: %d", broadcast_queue.front().right_speed);
								broadcast_queue.pop();
								ROS_INFO("POP!");
							}
						}
						break;
					}
					case broadcast_state::wait:
					{
						// wait until EOB flag
						break;
					}
					default:
					{
						ROS_INFO("How did we get here? Broadcast edition");
						break;
					}
				}

				// state transition logic (WIP)
				if (flag_H) {
					mode = state::custom;
					broadcast_mode = broadcast_state::outside;
					flag_SOB = false; // shouldn't be necessary, but just to be safe to avoid double update
					flag_EOB = false;
					broadcast_buffer = std::queue<eyes::Generic>(); // <-- exit case, resetting broadcast buffer for next time
					std_msgs::String to_hub;
					to_hub.data = "0B";
					to_hub.data.push_back(static_cast<char>(chair_broadcast_status::exclude));
					update_hub_pub.publish(to_hub);
					ROS_INFO("CHAIR 0 IS YANKED FROM BROADCAST");
				}
				// TODO: ADD CHECK FOR flag_SOB TO ENABLE SEQUENTIAL BROADCASTS?
				else if (flag_EOB) {
					mode = state::autonomous;
					broadcast_mode = broadcast_state::outside;
					autonomous_queue = std::queue<eyes::Generic>();
					choreo_queue = std::queue<eyes::Generic>();
					custom_queue = std::queue<eyes::Generic>();
					broadcast_queue = std::queue<eyes::Generic>();
					ROS_INFO("RESUMING AUTONOMOUS BEHAVIOR");
					flag_SOB = false;
					flag_EOB = false;
					broadcast_buffer = std::queue<eyes::Generic>(); // <-- exit case, resetting broadcast buffer for next time
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