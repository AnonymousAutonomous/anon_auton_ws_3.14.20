#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include <string>
#include <sstream>

const std::string BWD =     "cBstomr055r055";
const std::string FAST_BWD =     "cBstomr070r070";
const std::string PIVOTR =  "cBstomf060r060";
const std::string PIVOTL =  "cBstomr060f060";
const std::string FAST_PIVOTR   = "cBstomf100r100"; 
const std::string FAST_PIVOTL   = "cBstomr100f100";

ros::Publisher chairles;
void chatterCallBackLidar(const std_msgs::String& commands); 
std::pair<std_msgs::String, std_msgs::String> jimothy;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "baby_driver");

    ros::NodeHandle bill; //subscriber
    ros::NodeHandle ted; //publisher

    ros::Subscriber Rope = bill.subscribe("larry", 1000, chatterCallBackLidar);
    chairles = ted.advertise<std_msgs::String>("timmy" , 1000);
    ros::spin();
}

void chatterCallBackLidar(const std_msgs::String& commands)
{
    std::stringstream ss;
    ss << commands.data;
    
    std_msgs::String msg;
    msg.data = ss.str();
    chairles.publish(msg);

    ros::spinOnce();
}
