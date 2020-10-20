#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../../../constants/user_cmds.h"
#include <string>
#include <sstream>

int main(int argc, char** argv) {
  ros::init(argc, argv, "new_trigger");
  ros::NodeHandle nh;

  ros::Publisher trigger_pub = nh.advertise<std_msgs::String>("driver_output", 1000);

  ros::Rate delay_rate(10);

  std::string cmd;

  while (ros::ok() && std::cin >> cmd && cmd != "exit") {
    std_msgs::String msg;
    if (cmd == "left") {
    	msg.data = LEFT;
    }
    else if (cmd == "right") {
    	msg.data = RIGHT;
    }
    else if (cmd == "fwd") {
    	msg.data = FWD;
    }
    else if (cmd == "ffwd") {
    	msg.data = FFWD;
    }
    else if (cmd == "fwdl") {
    	msg.data = FWDL;
    }
    else if (cmd == "fwdr") {
    	msg.data = FWDR;
    }
    else if (cmd == "bwd") {
    	msg.data = BWD;
    }
    else if (cmd == "fbwd") {
    	msg.data = FBWD;
    }
    else if (cmd == "bwdl") {
    	msg.data = BWDL;
    }
    else if (cmd == "bwdr") {
    	msg.data = BWDR;
    }
    else if (cmd == "pivotl") {
    	msg.data = PIVOTL;
    }
    else if (cmd == "pivotr") {
    	msg.data = PIVOTR;
    }
    else if (cmd == "toggle") {
    	msg.data = TOGGLE;
    }
    else if (cmd == "stop") {
    	msg.data = STOP;
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
