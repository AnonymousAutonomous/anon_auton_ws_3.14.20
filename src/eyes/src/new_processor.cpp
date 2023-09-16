// nice

#include <iostream>
#include <chrono>
#include <future>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_msgs/UInt8.h"
#include "../../../constants/str_cmds.h"
#include "../../../constants/camera_variables.h"
#include <string>
#include <map>
#include <unordered_map>

ros::Publisher george; // the variable formerly known as signal
ros::Publisher debug;
ros::Publisher notify_lidar;
ros::Publisher send_sample_image;

bool data_read = false;

bool inTop(int counter);
bool inLeft(int counter);
bool inRight(int counter);
bool inMiddle(int counter);
unsigned int topBandWidth = 120;
unsigned int sideBandWidth = 180;
unsigned int imageWidth = 640; // 640 INSTEAD OF 720 IT'S LYING DON'T TRUST IT
unsigned int imageHeight = 480;
unsigned int brightnessThreshold = 175;
double sidePercentThreshold = 0.20;
double topPercentThreshold = 0.20;
bool favorRight = false;
double numMiddlePixels = 100800;

bool listening = true;
bool send_image = false;

void chatterCallBack(const sensor_msgs::Image &view);
void pauseCallback(const std_msgs::Empty empty_msg);
void sendImageCallback(const std_msgs::Empty empty_msg);

std::map<std::string, std::string> commands_in;
std::map<std::string, double> variables_in;
std::unordered_map<AutonomousCmd, std::string> commands;

int main(int argc, char **argv)
{
    ROS_ERROR("INITING EVERYTHING");
    ros::init(argc, argv, "camera_process");

    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::NodeHandle Iris; // subscriber
    if (Iris.getParam("/autonomous", commands_in))
    {
        for (auto i = commands_in.begin(); i != commands_in.end(); i++)
        {
            commands[AUTOCMD_STRING_TO_ENUM[i->first]] = i->second;
        }
        ROS_ERROR("Autonomous commands have been loaded for camera.");
    }
    else
    {
        ROS_ERROR("You must load autonomous commands before using camera.");
        return 1;
    }

    if (Iris.getParam("/camera", variables_in))
    {
        for (auto i = variables_in.begin(); i != variables_in.end(); i++)
        {
            ROS_ERROR("%s", i->first.c_str());
            ROS_ERROR("%s", std::to_string(i->second).c_str());
            switch (CAMERA_VARIABLE_STRING_TO_ENUM[i->first])
            {
            case TOP_BAND_WIDTH:
                topBandWidth = int(i->second);
                break;
            case SIDE_BAND_WIDTH:
                sideBandWidth = int(i->second);
                break;
            case IMAGE_WIDTH:
                imageWidth = int(i->second);
                break;
            case IMAGE_HEIGHT:
                imageHeight = int(i->second);
                break;
            case BRIGHTNESS_THRESHOLD:
                brightnessThreshold = int(i->second);
                break;
            case SIDE_PERCENT_THRESHOLD:
                sidePercentThreshold = i->second;
                break;
            case TOP_PERCENT_THRESHOLD:
                topPercentThreshold = i->second;
                break;
            case NUM_MIDDLE_PIXELS:
                numMiddlePixels = i->second;
                break;
            default:
                std::string infostr = "Unexpected variable: " + i->first;
                ROS_ERROR("%s", infostr.c_str());
            }

            std::string infostr = "Loaded parameter: " + i->first + " - " + std::to_string(i->second);
            ROS_ERROR("%s", infostr.c_str());
        }
    }
    else
    {
        ROS_ERROR("Unable to load camera parameters.");
    }

    ros::NodeHandle Cornea;                                                                // publisher
    ros::Subscriber Handle = Iris.subscribe("/cv_camera/image_mono", 10, chatterCallBack); // 10 was 1000
    ros::Subscriber eoc_sub = Iris.subscribe("end_of_choreo", 1000, pauseCallback);
    ros::Subscriber send_image_sub = Iris.subscribe("send_image", 1000, sendImageCallback);
    // george = Cornea.advertise<std_msgs::String>("cameron", 1000);
    george = Iris.advertise<std_msgs::String>("cameron", 10); // 10 was 1000
    // ros::spin();
    debug = Iris.advertise<std_msgs::String>("cam_debug", 10);
    notify_lidar = Iris.advertise<std_msgs::UInt8>("camera_to_lidar", 1000);
    send_sample_image = Iris.advertise<sensor_msgs::Image>("sample_image", 1000);

    // Fee copied over from simple_motors trigger
    ros::Rate delay_rate(5);

    while (ros::ok())
    {
    }
}

void pauseCallback(const std_msgs::Empty empty_msg)
{
    listening = true;
}

void sendImageCallback(const std_msgs::Empty empty_msg)
{
    send_image = true;
}

void chatterCallBack(const sensor_msgs::Image &view)
{
    if (send_image)
    {
        send_sample_image.publish(view);
        send_image = false;
    }

    double topCount = 0;
    double leftCount = 0;
    double rightCount = 0;
    double middleCount = 0;

    imageWidth = view.width;
    imageHeight = view.height;

    for (int i = 0; i < view.data.size(); i++) // grab top
    {
        // std::string command = "i: " + std::to_string(i);

        if (inTop(i))
        {
            // command += "T";
            topCount += (view.data[i] > brightnessThreshold);
        }
        if (inLeft(i))
        {
            // command += "L";
            leftCount += (view.data[i] > brightnessThreshold);
        }
        if (inRight(i))
        {
            // command += "R";
            rightCount += (view.data[i] > brightnessThreshold);
        }
        if (inMiddle(i))
        {
            middleCount += (view.data[i] > brightnessThreshold);
        }
    }
    double leftAverage = leftCount / (imageHeight * sideBandWidth);
    double rightAverage = rightCount / (imageHeight * sideBandWidth);
    double topAverage = topCount / (imageWidth * topBandWidth);
    double middleAverage = middleCount / numMiddlePixels;

    std::string result;

    /*
        forward = f
        backward = b
        stop = s
        pivotr = R
        pivotl = L
        fwdl = l
        fwdr = r
        bwdl = e
        bwdr = i

    */

    std_msgs::UInt8 camera_to_lidar_msg;
    camera_to_lidar_msg.data = 4 * (topAverage > topPercentThreshold) + 2 * (rightAverage > sidePercentThreshold) + 1 * (leftAverage > sidePercentThreshold);
    notify_lidar.publish(camera_to_lidar_msg);

    if (!listening)
        return;

    if (topAverage > topPercentThreshold && leftAverage > sidePercentThreshold && rightAverage > sidePercentThreshold) // white in top, left, right
    {
        if (favorRight)
        {
            result = commands[PIVOTR];
        }
        else
        {
            result = commands[PIVOTL];
        }
    }
    else if (topAverage > topPercentThreshold && leftAverage > sidePercentThreshold && rightAverage <= sidePercentThreshold) // white in top, left
    {
        result = commands[RCP]; // PIVOTR;
        listening = false;
        favorRight = true;
    }
    else if (topAverage > topPercentThreshold && leftAverage <= sidePercentThreshold && rightAverage > sidePercentThreshold) // white in top, right
    {
        result = commands[LCP]; // PIVOTL;
        listening = false;
        favorRight = false;
    }
    else if (topAverage > topPercentThreshold && leftAverage <= sidePercentThreshold && rightAverage <= sidePercentThreshold) // white in top
    {
        if (favorRight)
        {
            result = commands[PIVOTR];
        }
        else
        {
            result = commands[PIVOTL];
        }
    }
    else if (topAverage <= topPercentThreshold && leftAverage > sidePercentThreshold && rightAverage > sidePercentThreshold) // white in left, right
    {
        result = commands[FWD];
    }
    else if (topAverage <= topPercentThreshold && leftAverage > sidePercentThreshold && rightAverage <= sidePercentThreshold) // white in left
    {
        result = commands[VEERR];
        favorRight = true;
    }
    else if (topAverage <= topPercentThreshold && leftAverage <= sidePercentThreshold && rightAverage > sidePercentThreshold) // white in right
    {
        result = commands[VEERL];
        favorRight = false;
    }
    else // no white
    {
        result = commands[GO]; // f = fwd but with lower priority
    }

    if (middleAverage > topPercentThreshold)
    {                     // crossed over line --> backup and then pivot
        listening = true; // added in case choreo is overwritten by FPIVOTX
        if (favorRight)
        {
            result = commands[FPIVOTR];
        }
        else
        {
            result = commands[FPIVOTL];
        }
    }

    std::string d;
    // TODO: comment out when ready to send to queue
    //
    d = "Top: ";
    d += std::to_string(topAverage);
    d += " Left: ";
    d += std::to_string(leftAverage);
    d += " Right: ";
    d += std::to_string(rightAverage);
    d += " Middle: ";
    d += std::to_string(middleAverage);
    d += "\nThreshold: ";
    d += std::to_string(brightnessThreshold);
    d += " Top Threshold: ";
    d += std::to_string(topPercentThreshold);
    d += " Side Threshold: ";
    d += std::to_string(sidePercentThreshold);

    std_msgs::String stuff;
    stuff.data = result;
    george.publish(stuff);

    std_msgs::String other;
    other.data = d;
    debug.publish(other);

    data_read = true; // now compare values after the first frame
}

bool inTop(int counter)
{
    return counter < (imageWidth * topBandWidth);
}

bool inLeft(int counter)
{
    return (counter % imageWidth) < sideBandWidth;
}

bool inRight(int counter)
{
    return (counter % imageWidth) > (imageWidth - sideBandWidth);
}

bool inMiddle(int counter)
{
    return !(inTop(counter) || inLeft(counter) || inRight(counter));
}
