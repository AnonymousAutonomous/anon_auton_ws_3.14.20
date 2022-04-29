/*
 * encoder motor async spinner test
 */

 #include <ros.h>
 #include <std_msgs/String.h>
 #include <std_msgs/Int32.h>

 ros::NodeHandle nh;
 std_msgs::Int32 int_msg;
 ros::Publisher pub("encoder_val", &int_msg);

                            //encoder motor
 const int dirPin1 = 6;     //purple
 const int speedPin1 = 7;   //grey

 const int motorA = 3;
 const int motorB = 4;

 bool forward = true;

 void act(const std_msgs::String& ball) {
   // *** CODE GOES HERE ***
   digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));

   String speedy = ""; // speed

   auto directiony = HIGH; // direction

   if (ball.data[0] == 'r') {
     directiony = LOW;
   }
   
   speedy += ball.data[1];
   speedy += ball.data[2];
   speedy += ball.data[3];

   int custom_speed = speedy.toInt();

   digitalWrite(dirPin1, directiony); // replace value with directiony
   analogWrite(speedPin1, custom_speed); // replace value with custom_speed
 }

 ros::Subscriber<std_msgs::String> consumer("feed", &act);

 void setup() {
   nh.initNode();
   nh.subscribe(consumer);
   nh.advertise(pub);
  
   pinMode(motorA, INPUT);
   pinMode(motorB, INPUT);

   attachInterrupt(digitalPinToInterrupt(3), EncoderEvent, CHANGE);

   pinMode(dirPin1, OUTPUT);
   pinMode(speedPin1, OUTPUT);

   int_msg.data = 0;
 }

 void loop() {
   /*
   if (forward) {
     digitalWrite(dirPin1, HIGH);
     analogWrite(speedPin1, 255); // *** CHANGE ME TO SPIN ***
   }
   else {
     digitalWrite(dirPin1, LOW);
     analogWrite(speedPin1, 255); // *** CHANGE ME TO SPIN ***
   }

   if (count > 50000) forward = false;
   if (count < -50000) forward = true;
   */
   
   pub.publish(&int_msg);
   
   nh.spinOnce();
   delay(100);
 }

 void EncoderEvent() {
   if (digitalRead(motorA) == HIGH) {
     if (digitalRead(motorB) == LOW) {
       ++int_msg.data; 
     } else {
       --int_msg.data;
     }
   } else {
     if (digitalRead(motorB) == LOW) {
       --int_msg.data;
     } else {
       ++int_msg.data;
     }
   }
 }
 
