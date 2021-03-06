// nice

#include <iostream>
#include <chrono> 
#include <future>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "../../../constants/str_cmds.h"
#include <string>

ros::Publisher george; //the variable formerly known as signal
ros::Publisher debug;

bool data_read = false;

bool inTop(int counter);
bool inLeft(int counter);
bool inRight(int counter);
bool inMiddle(int counter);
unsigned int topBandWidth = 120;
unsigned int sideBandWidth = 180;
unsigned int imageWidth = 640; // 640 INSTEAD OF 720 IT'S LYING DON'T TRUST IT
unsigned int imageHeight = 480;
unsigned int threshold = 175;
double side_percent_threshold = 0.20;
double top_percent_threshold = 0.20;
bool favorRight = false; 
double num_middle_pixels = 100800;

bool listening = true;

void chatterCallBack(const sensor_msgs::Image& view);
void pauseCallback(const std_msgs::Empty empty_msg);

std::string GetLineFromCin() {
    std::string line; 
    std::getline(std::cin, line); 
    return line;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_process");

    ros::AsyncSpinner spinner(0);
    spinner.start();
    
    ros::NodeHandle Iris; //subscriber
    ros::NodeHandle Cornea; //publisher
    ros::Subscriber Handle = Iris.subscribe("/cv_camera/image_mono", 10, chatterCallBack); // 10 was 1000
    ros::Subscriber eoc_sub = Iris.subscribe("end_of_choreo", 1000, pauseCallback);
    //george = Cornea.advertise<std_msgs::String>("cameron", 1000);
    george = Iris.advertise<std_msgs::String>("cameron", 10); // 10 was 1000
    //ros::spin();
    debug = Iris.advertise<std_msgs::String>("cam_debug", 10);

    // Fee copied over from simple_motors trigger 
    ros::Rate delay_rate(5);

    std::string cmd = "";
    //int value; 

    auto future = std::async(std::launch::async, GetLineFromCin);

    while (ros::ok() &&cmd != "exit") {

        if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            auto cmd = future.get();

            future = std::async(std::launch::async, GetLineFromCin);
            std::cout << "CMD: " << cmd << std::endl;
            
if (cmd[0] == 't') {
                std::string value = "";
                value += cmd[1];
                value += cmd[2];
                value += cmd[3];
                threshold = std::stoi(value);
             } else if (cmd[0] == 'p') {
                std::string value = "";
                value += cmd[1];
                value += cmd[2];
                value += cmd[3];
                top_percent_threshold = std::stod(value) / 100.0; 
             } else if (cmd[0] == 's') {
                std::string value = "";
                value += cmd[1];
                value += cmd[2];
                value += cmd[3];
                side_percent_threshold = std::stod(value) / 100.0;
             } else {}

        // delay_rate.sleep(); // need this? 
}
            std::cout << "THRESHOLD: " <<   std::to_string(threshold) << std::endl;        
std::cout << "SIDE: " << std::to_string(side_percent_threshold) << std::endl;
std::cout << "TOP: " << std::to_string(top_percent_threshold) << std::endl;



            

            // ros::spinOnce(); ros::spin(); // uses async spinner now 

//delay_rate.sleep();

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void pauseCallback(const std_msgs::Empty empty_msg) {
	listening = true;
}

void chatterCallBack(const sensor_msgs::Image& view)
{
    if (!listening) return;

    double topCount = 0;
    double leftCount = 0;
    double rightCount = 0;
    double middleCount = 0;
    
    imageWidth = view.width;
    imageHeight = view.height;

    for(int i = 0; i < view.data.size(); i++) //grab top
    {
    //std::string command = "i: " + std::to_string(i);
    
        if(inTop(i))
        {
            //command += "T";
            topCount += (view.data[i] > threshold); 
        }
        if(inLeft(i))
        {
        //command += "L";
            leftCount += (view.data[i] > threshold);
        }
        if(inRight(i))
        {
        //command += "R";
            rightCount += (view.data[i] > threshold);
        }
        if (inMiddle(i)) {
            middleCount += (view.data[i] > threshold);
        }
    }
    double leftAverage = leftCount/ (imageHeight*sideBandWidth);
    double rightAverage = rightCount / (imageHeight*sideBandWidth);
    double topAverage = topCount / (imageWidth*topBandWidth);
    double middleAverage = middleCount / num_middle_pixels;

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

    if(topAverage > top_percent_threshold && leftAverage > side_percent_threshold && rightAverage > side_percent_threshold) // white in top, left, right
    {
        if (favorRight) {
            result = PIVOTR;
        } else {
            result = PIVOTL;
        }
        
    }
    else if(topAverage > top_percent_threshold && leftAverage > side_percent_threshold && rightAverage <= side_percent_threshold) // white in top, left
    {
        result = RCP; // PIVOTR;
	listening = false;
        favorRight = true;
    }
    else if(topAverage > top_percent_threshold && leftAverage <= side_percent_threshold && rightAverage > side_percent_threshold) // white in top, right
    {
        result = LCP; // PIVOTL;
	listening = false;
        favorRight = false;
    }
    else if(topAverage > top_percent_threshold && leftAverage <= side_percent_threshold && rightAverage <= side_percent_threshold) // white in top
    {
        if (favorRight) {
            result = PIVOTR;
        }
        else {
            result = PIVOTL;
        }
    }
    else if(topAverage <= top_percent_threshold && leftAverage > side_percent_threshold && rightAverage > side_percent_threshold) // white in left, right
    {
        result = FWD;
    }
    else if(topAverage <= top_percent_threshold && leftAverage > side_percent_threshold && rightAverage <= side_percent_threshold) // white in left
    {
        result = VEERR;
        favorRight = true;
    }
    else if(topAverage <= top_percent_threshold && leftAverage <= side_percent_threshold && rightAverage > side_percent_threshold) // white in right
    {
        result = VEERL;
        favorRight = false;
    }
    else // no white
    {
        result = GO; //f = fwd but with lower priority
    }
     
    if (middleAverage > top_percent_threshold) { // crossed over line --> backup and then pivot
	listening = true; // added in case choreo is overwritten by FPIVOTX
        if (favorRight) {
            result = FPIVOTR;
        }
        else {
            result = FPIVOTL;
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
    d += std::to_string(threshold);
    d += " Top Threshold: "; 
    d += std::to_string(top_percent_threshold);
    d += " Side Threshold: "; 
    d += std::to_string(side_percent_threshold);

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
    return counter < (imageWidth*topBandWidth);
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
