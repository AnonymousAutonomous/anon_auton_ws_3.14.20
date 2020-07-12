#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <sstream>

double rsum = 0;
double rcount = 0;
int limit = 10;

int ncount = 0;
int nsmalls = 0;
double small = 3;

ros::Publisher pub;

void callback(const std_msgs::Float64::ConstPtr& msg);

int main(int argc, char** argv) {
	ros::init(argc, argv, "stale");
	ros::NodeHandle nh;
	pub = nh.advertise<std_msgs::String>("emove", 1000);
	ros::Subscriber sub = nh.subscribe("fresh", 1000, callback);

	ros::spin();
	return 0;
}

void callback(const std_msgs::Float64::ConstPtr& msg) {
	double num = msg->data;
/*	
	if (rcount < limit) {
		rsum += num;
		++rcount;
	}
	else {
		double avg = rsum / rcount;
		ROS_INFO("%f", avg);
		rsum = 0;
		rcount = 0;
	}
*/
	if (num < small) ++nsmalls;
	++ncount;

	if (ncount == limit) {
		ROS_INFO("%d", nsmalls);

		std_msgs::String msg;
		std::stringstream ss;

		if (nsmalls < 4) ss << "y";
		else ss << "n";

		msg.data = ss.str();
		pub.publish(msg);

		ros::spinOnce();

		nsmalls = 0;
		ncount = 0;
	}
}

// time skew sanity check
