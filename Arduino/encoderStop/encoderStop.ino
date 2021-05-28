 /*
  *  encoder stop test
  */

 #include <ros.h>
 #include <std_msgs/String.h>

 ros::NodeHandle nh;                           
                            //encoder motor
 const int dirPin1 = 6;     //purple
 const int speedPin1 = 7;   //grey

 const int motorA = 3;
 const int motorB = 4;

 volatile long count = 0;

 bool forward = true;

 void setup() {
   pinMode(motorA, INPUT);
   pinMode(motorB, INPUT);

   attachInterrupt(digitalPinToInterrupt(3), EncoderEvent, CHANGE);

   pinMode(dirPin1, OUTPUT);
   pinMode(speedPin1, OUTPUT);

   Serial.begin(9600);
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

   Serial.print("Count: ");
   Serial.println(count);
   Serial.println();

   if (count > 50000) forward = false;
   if (count < -50000) forward = true;

   delay(500);
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
