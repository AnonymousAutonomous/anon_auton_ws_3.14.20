#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher paddle("bounceback", &str_msg);

char echo[13] = "arduino ECHO";

void callback( const std_msgs::String& ball ) {
  str_msg.data = echo;
  paddle.publish( &str_msg );
  nh.spinOnce();
  delay(1);
}

ros::Subscriber<std_msgs::String> sub("bounce", &callback);

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(paddle);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(1);
}
