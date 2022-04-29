#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <eyes/Autonomous.h>
#include <eyes/Choreo.h>
#include <eyes/Custom.h>
#include <eyes/Big_Boi.h>

                          //encoder motor
const int dirPin1 = 6;     //purple
const int speedPin1 = 7;   //grey

const int motorA = 3;
const int motorB = 4;

ros::NodeHandle nh;

void autonomous_callback(const eyes::Autonomous& autonomous_msg);
void choreo_callback(const eyes::Choreo& choreo_msg);
void custom_callback(const eyes::Custom& custom_msg);

void big_boi_callback(const eyes::Big_Boi& big_boi_msg);

extern ros::Subscriber<eyes::Autonomous>* autonomous_sub;
// ros::Subscriber<eyes::Choreo> choreo_sub("choreo_feed", &choreo_callback);
// ros::Subscriber<eyes::Custom> custom_sub("custom_feed", &custom_callback);

// ros::Subscriber<eyes::Big_Boi> big_boi_sub("big_boi_feed", &big_boi_callback);

void setup() {
  nh.initNode();

  autonomous_sub = new ros::Subscriber<eyes::Autonomous>("autonomous_feed", &autonomous_callback);
}

void loop() {
  // put your main code here, to run repeatedly:

}
