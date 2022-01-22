// nice???

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <sstream>
#include <unordered_map>

std::unordered_map<std::string, std::string> commands;

int main(int argc, char** argv) {
  ros::init(argc, argv, "new_trigger");
  ros::NodeHandle nh;

  if (nh.getParam("handwritten", commands)) {
    ROS_INFO("Handwritten commands have been loaded.");
  }
  else {
    ROS_INFO("You must load handwritten commands first.");
    return 1;
  }

  ros::Publisher trigger_pub = nh.advertise<std_msgs::String>("driver_output", 1000);

  ros::Rate delay_rate(10);

  std::string cmd;

  while (ros::ok() && std::cin >> cmd && cmd != "exit") {
    std_msgs::String msg;
    if (commands.find(cmd) != commands.end()) {
      msg.data = commands[cmd];
    }
    else {
    	continue;
    }

    trigger_pub.publish(msg);
    ros::spinOnce();

    delay_rate.sleep();
  }

  return 0;
}
