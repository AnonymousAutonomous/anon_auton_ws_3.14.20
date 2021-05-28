#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <eyes/Generic.h>

                           //encoder motor
const int dirPin1 = 5;     //purple
const int speedPin1 = 6;   //grey

const int motorA = 3;
const int motorB = 4;

volatile long count = 0;

ros::NodeHandle nh;

std_msgs::Empty empty_msg;
ros::Publisher pub("notifications", &empty_msg);

void generic_callback(const eyes::Generic& generic_msg) {
  if (generic_msg.left_forward) {
    digitalWrite(dirPin1, HIGH);
  }
  else {
    digitalWrite(dirPin1, LOW);
  }
  analogWrite(speedPin1, generic_msg.left_speed);

  if (generic_msg.identifier == 'c') {
    if (generic_msg.timed) {
      delay(generic_msg.duration * 1000);
      pub.publish(&empty_msg);
    }
    else {
      long initial = count;
      while (abs(count - initial) < generic_msg.duration) {
        delay(1);
      }
      pub.publish(&empty_msg);
    }
  }
  
  return;
}

ros::Subscriber<eyes::Generic> generic_sub("generic_feed", &generic_callback);

void setup() {
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(generic_sub);

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(motorA, INPUT);
  pinMode(motorB, INPUT);

  attachInterrupt(digitalPinToInterrupt(3), EncoderEvent, CHANGE);

  pinMode(dirPin1, OUTPUT);
  pinMode(speedPin1, OUTPUT);
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
