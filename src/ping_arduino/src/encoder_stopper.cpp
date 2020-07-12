#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include <sstream>

class Encoder {
	public:
		Encoder(int c) : init_val(0), counter(c), started(false), finished(false) {
			sub = nh.subscribe("encoder_val", 1000, &Encoder::callback, this);
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

			ros::Rate loop_rate(10);

			ROS_INFO("WAITING...");
			while (!finished) {
				// do nothing
				ros::spinOnce();
			}

		}

	private:
		int init_val;
		int counter;

		bool started;
		bool finished;

		ros::NodeHandle nh;
		ros::Subscriber sub;
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "encoder_stopper");
	ROS_INFO("PROGRAM START");
	Encoder e(20000);
	e.time();
	ROS_INFO("PROGRAM END");
}
