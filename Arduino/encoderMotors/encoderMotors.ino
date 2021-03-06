/*
 * encoder motor test
 */

 #include <ros.h>
 #include <std_msgs/String.h>
 #include <std_msgs/Int32.h>

 ros::NodeHandle nh;
 std_msgs::Int32 int_msg;
 ros::Publisher pub("encoder_val", &int_msg);

 char message[20] = "arduino PLACEHOLDER";

 volatile long count = 42;

 void callback(const std_msgs::String& ball) {
   int_msg.data = count;
   pub.publish(&int_msg);
   nh.spinOnce();
   delay(1);
 }

 ros::Subscriber<std_msgs::String> sub("request", &callback);
 
                            //encoder motor
 const int dirPin1 = 5;     //purple
 const int speedPin1 = 6;   //grey

 const int motorA = 3;
 const int motorB = 4;

 bool forward = true;

 void setup() {
   nh.initNode();
   nh.subscribe(sub);
   nh.advertise(pub);
  
   pinMode(motorA, INPUT);
   pinMode(motorB, INPUT);

   attachInterrupt(digitalPinToInterrupt(3), EncoderEvent, CHANGE);

   pinMode(dirPin1, OUTPUT);
   pinMode(speedPin1, OUTPUT);

   // Serial.begin(9600);
 }

 void loop() {
   if (forward) {
     digitalWrite(dirPin1, HIGH);
     analogWrite(speedPin1, 0);
   }
   else {
     digitalWrite(dirPin1, LOW);
     analogWrite(speedPin1, 0);
   }

   // Serial.print("Count: ");
   // Serial.println(count);
   // Serial.println();

   if (count > 50000) forward = false;
   if (count < -50000) forward = true;

   nh.spinOnce();
   delay(1);
 }

 void EncoderEvent() {
   if (digitalRead(motorA) == HIGH) {
     if (digitalRead(motorB) == LOW) {
       count++; 
     } else {
       count--;
     }
   } else {
     if (digitalRead(motorB) == LOW) {
       count--;
     } else {
       count++;
     }
   }
 }
 
