/*
 * toggle motor controls on command
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

 bool commands = false;

 ros::NodeHandle nh;

 std_msgs::String str_msg;
 ros::Publisher pub("tmove", &str_msg);
 char holder[2] = "n";

 void messageCb( const std_msgs::String& timmy_msg) {
   digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));

   if (strlen(timmy_msg.data) != 14 || timmy_msg.data[0] != 'c') {
     commands = true;
   }

   if (!commands) {
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

     if (custom_left_speed == 0 && custom_right_speed == 0) holder[0] = 'n';
     else holder[0] = 'y';
      
     digitalWrite(dirPin1, direction1);
     analogWrite(speedPin1, custom_right_speed);
     digitalWrite(dirPin2, direction2);
     analogWrite(speedPin2, custom_left_speed);
   }
   else {
     if (strcmp(timmy_msg.data, "left") == 0) {
       digitalWrite(dirPin2, HIGH);
       analogWrite(speedPin2, 255);
       holder[0] = 'y';
     }
     if (strcmp(timmy_msg.data, "right") == 0) {
       digitalWrite(dirPin1, HIGH);
       analogWrite(speedPin1, 255);
       holder[0] = 'y';
     }
     if (strcmp(timmy_msg.data, "fwd") == 0) {
       digitalWrite(dirPin1, HIGH);
       analogWrite(speedPin1, 160); // lower max
       digitalWrite(dirPin2, HIGH);
       analogWrite(speedPin2, 255); // higher max
       holder[0] = 'y';
     }
     if (strcmp(timmy_msg.data, "ffwd") == 0) {
       digitalWrite(dirPin1, HIGH);
       analogWrite(speedPin1, 255);
       digitalWrite(dirPin2, HIGH);
       analogWrite(speedPin2, 255);
       holder[0] = 'y';
     }
     if (strcmp(timmy_msg.data, "fwdl") == 0) {
       digitalWrite(dirPin1, HIGH);
       analogWrite(speedPin1, 160);
       digitalWrite(dirPin2, HIGH);
       analogWrite(speedPin2, 128);
       holder[0] = 'y';
     }
     if (strcmp(timmy_msg.data, "fwdr") == 0) {
       digitalWrite(dirPin1, HIGH);
       analogWrite(speedPin1, 80);
       digitalWrite(dirPin2, HIGH);
       analogWrite(speedPin2, 255);
       holder[0] = 'y';
     }
     if (strcmp(timmy_msg.data, "bwd") == 0) {
       digitalWrite(dirPin1, LOW);
       analogWrite(speedPin1, 255); // higher max
       digitalWrite(dirPin2, LOW);
       analogWrite(speedPin2, 160); // lower max
       holder[0] = 'y';
     }
     if (strcmp(timmy_msg.data, "bbwd") == 0) {
       digitalWrite(dirPin1, LOW);
       analogWrite(speedPin1, 255);
       digitalWrite(dirPin2, LOW);
       analogWrite(speedPin2, 255);
       holder[0] = 'y';
     }
     if (strcmp(timmy_msg.data, "bwdl") == 0) {
       digitalWrite(dirPin1, LOW);
       analogWrite(speedPin1, 255);
       digitalWrite(dirPin2, LOW);
       analogWrite(speedPin2, 80);
       holder[0] = 'y';
     }
     if (strcmp(timmy_msg.data, "bwdr") == 0) {
       digitalWrite(dirPin1, LOW);
       analogWrite(speedPin1, 128);
       digitalWrite(dirPin2, LOW);
       analogWrite(speedPin2, 160);
       holder[0] = 'y';
     }
     if (strcmp(timmy_msg.data, "pivotl") == 0) {
       digitalWrite(dirPin1, HIGH);
       analogWrite(speedPin1, 255);
       digitalWrite(dirPin2, LOW);
       analogWrite(speedPin2, 255);
       holder[0] = 'y';
     }
     if (strcmp(timmy_msg.data, "pivotr") == 0) {
       digitalWrite(dirPin1, LOW);
       analogWrite(speedPin1, 255);
       digitalWrite(dirPin2, HIGH);
       analogWrite(speedPin2, 255);
       holder[0] = 'y';
     }
     if (strcmp(timmy_msg.data, "stop") == 0) {
       digitalWrite(dirPin1, HIGH);
       analogWrite(speedPin1, 0);
       digitalWrite(dirPin2, HIGH);
       analogWrite(speedPin2, 0);
       holder[0] = 'n';
     }
     if (strcmp(timmy_msg.data, "toggle") == 0) {
       commands = false;
     }
   }
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
   nh.advertise(pub);
 }

 void loop() {
   str_msg.data = holder;
   pub.publish(&str_msg);
   
   nh.spinOnce();
   
   delay(100);
 }
