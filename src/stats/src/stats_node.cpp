#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Char.h"
#include "rosgraph_msgs/TopicStatistics.h"
#include <stdio.h>
#include <stdlib.h>
#include <unordered_map>
#include "../../../constants/str_cmds.h"

#include <ros/spinner.h>
#include <queue>
#include <vector>
#include <string>

const std::unordered_map<std::string, ros::Time> cmd_to_case = {
    // launching entire chair
    {
        "launch",
    },
};

ros::Time startTime;
ros::Duration heartbeatDuration(0.5); // 0.5 seconds

ros::Publisher stats_debug_pub;

void statistics_callback(const rosgraph_msgs::TopicStatistics &stats_msg)
{
    rosgraph_msgs::TopicStatistics msg_copy;
    msg_copy.data = stats_msg.data;
    stats_debug_pub.publish(msg_copy);
}

int main(int argc, char **argv)
{
    // initialize node and node handle
    ros::init(argc, argv, "stats");
    ros::NodeHandle nh;

    // initialize spinner
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // initialize subscribers
    ros::Subscriber statistics_sub = nh.subscribe("statistics", 1000, statistics_callback);

    // initialize publishers
    stats_debug_pub = nh.advertise<rosgraph_msgs::TopicStatistics>("stats_debug", 1000);

    // ros::Timer timer = nh.createTimer(ros::Duration(0.1), onHeartbeat);
    // ros::Rate delay_rate(5); // 5 cycles per second
    startTime = ros::Time::now();

    while (ros::ok())
    {
        if (ros::Time::now() >= startTime + heartbeatDuration)
        {
        }
    }
}