/*
 * runs motors on callback
 */

 #include <ros.h>
 #include <std_msgs/String.h>
                            //right motor
 const int dirPin1 = 5;     //red
 const int speedPin1 = 6;   //orange
                            //left motor
 const int dirPin2 = 9;     //yellow
 const int speedPin2 = 10;  //green

 const int interruptPin = 13;

 const int HIGHER_MAX_SPEED = 255;
 const int LOWER_MAX_SPEED = 160;

 int custom_right_speed = 122;
 int custom_left_speed = 80;

 ros::NodeHandle nh;
 
 void messageCb( const std_msgs::String& timmy_msg) {
   digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));
   
   String speed1 = ""; //right speed
   String speed2 = ""; //left speed

   auto direction1 = HIGH; //right direction
   auto direction2 = HIGH; //left direction

   if (timmy_msg.data[6] == 'r') {
     direction2 = LOW;
   }

   if (timmy_msg.data[10] == 'r') {
     direction1 = LOW;
   }
      
      
   speed2 += timmy_msg.data[7];
   speed2 += timmy_msg.data[8];
   speed2 += timmy_msg.data[9];
   speed1 += timmy_msg.data[11];
   speed1 += timmy_msg.data[12];
   speed1 += timmy_msg.data[13];
      
   custom_left_speed = speed2.toInt();
   custom_right_speed = speed1.toInt();
      
   digitalWrite(dirPin1, direction1);
   analogWrite(speedPin1, custom_right_speed);
   digitalWrite(dirPin2, direction2);
   analogWrite(speedPin2, custom_left_speed);
 }

 ros::Subscriber<std_msgs::String> sub("timmy", &messageCb );

 void setup() {
   pinMode(LED_BUILTIN, OUTPUT);

   pinMode(dirPin1, OUTPUT);
   pinMode(speedPin1, OUTPUT);
   pinMode(dirPin2, OUTPUT);
   pinMode(speedPin2, OUTPUT);

   nh.initNode();
   nh.subscribe(sub);
 }

 void loop() {
   nh.spinOnce();
   delay(1);
 }
