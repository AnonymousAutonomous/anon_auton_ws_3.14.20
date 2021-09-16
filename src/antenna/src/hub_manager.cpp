#include "ros/ros.h"
#include "std_msgs/String.h"

enum state : char {outside, awaiting_confirmation, transmit, awaiting_status};

ros::Publisher hub_manager_pub;

void callback(const std_msgs::String::ConstPtr& msg) {
  // pass
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "hub_manager");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("from_hub_receiver", 1000, callback);
  hub_manager_pub = nh.advertise<std_msgs::String>("from_hub", 1000);

  ROS_INFO("Up and running...");

  ros::spin();

  return 0;
}
