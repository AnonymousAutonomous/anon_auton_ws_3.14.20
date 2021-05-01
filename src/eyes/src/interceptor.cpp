#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "eyes/Generic.h"

#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <bcm2835.h>
#include <string>

enum filter : char {on, off};
filter mode = filter::off;

ros::CallbackQueue side_queue;
ros::Publisher pub;
ros::Publisher override_pub;

bool high_level;

void side_callback(const std_msgs::Empty& empty_msg) {
	eyes::Generic safety_stop;
	safety_stop.identifier = 'h';
	safety_stop.left_forward = true;
	safety_stop.right_forward = true;
	safety_stop.left_speed = 0;
	safety_stop.right_speed = 0;
	safety_stop.timed = false; // inconsequential
	safety_stop.duration = 0; // inconsequential
	pub.publish(safety_stop);

	if (high_level) {
		high_level = false;
		bcm2835_gpio_write(26, LOW);
	}
	else {
		high_level = true;
		bcm2835_gpio_write(26, HIGH);
	}

	mode = filter::on;
}

void callback(const eyes::Generic& generic_msg) {
	if (generic_msg.identifier != 'c') {
		mode = filter::off;
	}
	if (mode == filter::on) {
		std_msgs::Empty empty_msg;
		override_pub.publish(empty_msg);
		// ROS_INFO("NOTIFICATIONS PUB");
	}
	else {
		pub.publish(generic_msg);
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "interceptor");
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(0);
	spinner.start();

	ros::Subscriber sub = nh.subscribe("safety_output", 1000, callback);

	ros::NodeHandle side;
	side.setCallbackQueue(&side_queue);
	ros::Subscriber side_sub = side.subscribe("interceptor_ping", 1000, side_callback);

	pub = nh.advertise<eyes::Generic>("generic_feed", 1000);
	override_pub = nh.advertise<std_msgs::Empty>("notifications", 1000);

	bcm2835_init();
	bcm2835_gpio_fsel(26, BCM2835_GPIO_FSEL_OUTP);
	high_level = false;
	bcm2835_gpio_write(26, LOW);

	mode = filter::off;
	while (ros::ok()) {
		side_queue.callOne();
	}

	bcm2835_close();
}
