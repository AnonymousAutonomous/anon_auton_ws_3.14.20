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

std::vector<std::string> topics_we_care_about{"/cv_camera/image_mono", "raw_obstacles"};
ros::Duration timeBeforeOfflineSec(10); // 10 seconds

std::unordered_map<std::string, ros::Time> topic_to_last_start_time = {};

ros::Time startTime;

ros::Publisher stats_debug_pub;

bool camera_online = true;
bool lidar_online = true;

void statistics_callback(const rosgraph_msgs::TopicStatistics &stats_msg)
{
    std::string topic(stats_msg.topic);
    ROS_INFO("Callback for: %s", topic.c_str());
    ros::Time startTime = stats_msg.window_start;
    topic_to_last_start_time[topic] = startTime;
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
        for (auto i = topic_to_last_start_time.begin(); i != topic_to_last_start_time.end(); i++)
        {
            if (ros::Time::now() >= i->second + timeBeforeOfflineSec)
            {
                std::string msg;
                if (i->first == "/cv_camera/image_mono")
                {
                    camera_online = false;
                }
                else if (i->first == "raw_obstacles")
                {
                    lidar_online = false;
                }
                else
                {
                }
                msg += "Camera: ";
                msg += camera_online ? "on" : "OFF";
                msg += "\tLidar: ";
                msg += lidar_online ? "on" : "OFF";
                std_msgs::String msgs;
                msgs.data = msg;
                stats_debug_pub.publish(msgs);
            }
        }
    }
}