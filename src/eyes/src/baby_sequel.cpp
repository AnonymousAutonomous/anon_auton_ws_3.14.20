#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <ros/callback_queue.h>
#include <string>
#include <sstream>

const std::string FWD		= "f255";
const std::string BWD		= "r255";
const std::string STOP		= "f000";

ros::Publisher pub;
void callback(const std_msgs::String& command);

bool reset = false;

class Encoder {
	public:
		Encoder(int c) : init_val(0), counter(c), started(false), finished(false) {
			ROS_INFO("CONSTRUCTOR");
			nh.setCallbackQueue(&my_queue);
			sub = nh.subscribe("encoder_val", 10, &Encoder::callback, this);
		}

		void callback(const std_msgs::Int32::ConstPtr& msg) {
			if (!started) {
				init_val = msg->data;
				started = true;
			}
			ROS_INFO("INFO RECEIVED: %d", msg->data);
			if (counter >= 0) {
				if (msg->data >= init_val + counter) finished = true;
			}
			else {
				if (msg->data <= init_val + counter) finished = true;
			}
		}

		void time() {
			ROS_INFO("TIMER START");

			ROS_INFO("WAITING...");
			while (!finished) {
				// do nothing
				my_queue.callOne();
			}

		}

	private:
		int init_val;
		int counter;

		bool started;
		bool finished;

		ros::NodeHandle nh;
		ros::CallbackQueue my_queue;
		ros::Subscriber sub;
};

ros::CallbackQueue global_queue;

int main(int argc, char** argv) {
	ros::init(argc, argv, "baby_sequel");
	ros::NodeHandle oi;
	oi.setCallbackQueue(&global_queue);

	ros::Subscriber sub = oi.subscribe("stream", 1000, callback);
	pub = oi.advertise<std_msgs::String>("feed", 1000);

	while (ros::ok()) {
		global_queue.callOne();
		if (reset) {
			sub = oi.subscribe("stream", 1000, callback);
			ROS_INFO("QUEUE CLEARED");
			reset = false;
		}

	}
}

void callback(const std_msgs::String& command) {
	std::stringstream ss;
	if (command.data[0] == 'f') {
		ROS_INFO("MESSAGE: %s", command.data.c_str());
		ss << FWD;
		std_msgs::String msg;
		msg.data = ss.str();

		pub.publish(msg);
	}
	else {
		ss << BWD;
		std_msgs::String msg;
		msg.data = ss.str();
		pub.publish(msg);
		Encoder e(-20000);
		e.time();
		reset = true;
	}
}
