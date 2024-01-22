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

const std::vector<std::string> topics_we_care_about = [ "image_mono", "raw_obstacles" ];
ros::Duration timeBeforeOfflineSec(2); // 2 seconds

const std::unordered_map<std::string, ros::Time> topic_to_last_start_time = {};

ros::Time startTime;

ros::Publisher stats_debug_pub;

void statistics_callback(const rosgraph_msgs::TopicStatistics &stats_msg)
{
    topic_to_last_start_time[stats_msg.topic] = stats_msg.window_start;
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
    stats_debug_pub = nh.advertise<std_msgs::String>("stats_debug", 1000);

    // ros::Timer timer = nh.createTimer(ros::Duration(0.1), onHeartbeat);
    // ros::Rate delay_rate(5); // 5 cycles per second
    startTime = ros::Time::now();

    for (int i = 0; i < topics_we_care_about.size(); i++)
    {
        topic_to_last_start_time[topics_we_care_about[i]] = startTime;
    }

    while (ros::ok())
    {
        for (auto i = m.begin(); i != m.end(); i++)
        {
            if (ros::Time::now() >= i->second + timeBeforeOfflineSec)
            {
                std::String msg;
                if (i->first == "image_mono")
                {
                    msg.data = "CAMERA OFFLINE";
                }
                else if (i->first == "raw_obstacles")
                {
                    msg.data = "LIDAR OFFLINE";
                }
                else
                {
                }
                stats_debug_pub.publish(msg);
            }
        }
    }
}