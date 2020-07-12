#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"

#define RAD2DEG(x) ((x) * 180. / M_PI)

ros::Publisher pub;

std::vector<double> distances(360, 0); // does nothing?
double diff_squared = 0; // does nothing?

double prev_range = 0;

int sample_rate = 2;
int sample = 0;

void callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
	double diff = 1; // garbage placeholder value

	std_msgs::String str_msg;
	std::stringstream ss;

	int count = msg->scan_time / msg->time_increment;
	for (int i = 0; i < count; ++i) {
		size_t degree = floor(RAD2DEG(msg->angle_min + msg->angle_increment * i)) + 180;
		if (degree == 358 || degree == 1) ROS_INFO(": [%d, %f]", degree, msg->ranges[i]);
		if (degree == 358 && sample >= sample_rate) {
			diff = msg->ranges[i] - prev_range;
			prev_range = msg->ranges[i];
			sample = 0;
		}
	}
	++sample;

	if (abs(diff) > 0.08) ss << "E";
	else if (abs(diff) > 0.03) ss << "Y";
	else ss << "N";

	if (sample == 1) {
		str_msg.data = ss.str();
		pub.publish(str_msg);
	}

	ros::spinOnce();
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "analyzer");
	ros::NodeHandle nh;

	pub = nh.advertise<std_msgs::String>("analysis", 1000);
	ros::Subscriber sub = nh.subscribe("scan_filtered", 1000, callback);

	ros::spin();

	return 0;
}
