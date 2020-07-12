#include "ros/ros.h"
#include "std_msgs/String.h"
#include "eyes/Custom.h"
#include "eyes/Choreo.h"
#include "eyes/Autonomous.h"

#include <ros/callback_queue.h>
#include <string>
#include <sstream>

void callback(const std_msgs::String& command) {
	// code coming soon
	// think sorting hat from Harry Potter
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "queue_up");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("driver_output", 1000, callback);

	ros::Publisher pub1 = nh.advertise<eyes::Custom>("custom_feed", 1000);
	ros::Publisher pub2 = nh.advertise<eyes::Choreo>("choreo_feed", 1000);
	ros::Publisher pub3 = nh.advertise<eyes::Autonomous>("autonomous_feed", 1000);
	eyes::Custom msg;
	msg.command = "fwd";
	while (ros::ok()) {
		pub1.publish(msg);
		ros::spinOnce();
	}
}
