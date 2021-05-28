#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <eyes/Autonomous.h>
#include <eyes/Choreo.h>
#include <eyes/Custom.h>

                           //encoder motor
const int dirPin1 = 5;     //purple
const int speedPin1 = 6;   //grey

const int motorA = 3;
const int motorB = 4;

volatile long count = 0;

ros::NodeHandle nh;

std_msgs::Empty empty_msg;
ros::Publisher pub("notifications", &empty_msg);

void autonomous_callback(const eyes::Autonomous& autonomous_msg) {
  if (autonomous_msg.left_forward) {
    digitalWrite(dirPin1, HIGH);
  }
  else {
    digitalWrite(dirPin1, LOW);
  }
  analogWrite(speedPin1, autonomous_msg.left_speed);
}

void choreo_callback(const eyes::Choreo& choreo_msg) {
  if (choreo_msg.left_forward) {
    digitalWrite(dirPin1, HIGH);
  }
  else {
    digitalWrite(dirPin1, LOW);
  }
  analogWrite(speedPin1, choreo_msg.left_speed);
  
  if (choreo_msg.timed) {
    delay(choreo_msg.duration * 1000);
  }
  else {
    long initial_count = count;
    while (abs(count - initial_count) < choreo_msg.duration) {
      // do nothing, wait
    }
  }

  pub.publish(&empty_msg);
}

void custom_callback(const eyes::Custom& custom_msg) {
  if (custom_msg.command == "Hfwd") {
    digitalWrite(dirPin1, HIGH);
    analogWrite(speedPin1, 255);
  }
  if (custom_msg.command == "Hbwd") {
    digitalWrite(dirPin1, LOW);
    analogWrite(speedPin1, 255);
  }
  if (custom_msg.command == "Hstop") {
    digitalWrite(dirPin1, HIGH);
    analogWrite(speedPin1, 0);
  }
  if (custom_msg.command == "Htoggle") {
    pub.publish(&empty_msg);  
  }
}

ros::Subscriber<eyes::Autonomous> autonomous_sub("autonomous_feed", &autonomous_callback);
ros::Subscriber<eyes::Choreo> choreo_sub("choreo_feed", &choreo_callback);
ros::Subscriber<eyes::Custom> custom_sub("custom_feed", &custom_callback);

void setup() {
  nh.initNode();

  nh.subscribe(autonomous_sub);
  nh.subscribe(choreo_sub);
  nh.subscribe(custom_sub);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
}

void EncoderEvent() {
   if (digitalRead(motorA) == HIGH) {
     if (digitalRead(motorB) == LOW) {
       ++count; 
     } else {
       --count;
     }
   } else {
     if (digitalRead(motorB) == LOW) {
       --count;
     } else {
       ++count;
     }
   }
 }
