/*
 * runs motors on callback
 */

 #include <ros.h>
 #include <std_msgs/String.h>
 #include <std_msgs/Empty.h>
                            //right motor
 const int dirPin1 = 5;     //red
 const int speedPin1 = 6;   //orange
                            //left motor
 const int dirPin2 = 9;     //yellow
 const int speedPin2 = 10;  //green

 const int HIGHER_MAX_SPEED = 255;
 const int LOWER_MAX_SPEED = 160;

 int custom_right_speed = 122;
 int custom_left_speed = 80;

 ros::NodeHandle nh;

 int x, y, z;
 int Xrest, Yrest, Zrest;
 int dull;

 int vx, vy, vz;

 std_msgs::String str_msg;
 ros::Publisher pub("info", &str_msg);
 
 void messageCb( const std_msgs::String& timmy_msg) {
   digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));

   if (strcmp(timmy_msg.data, "left") == 0) {
     digitalWrite(dirPin2, HIGH);
     analogWrite(speedPin2, 255); 
   }
   if (strcmp(timmy_msg.data, "right") == 0) {
     digitalWrite(dirPin1, HIGH);
     analogWrite(speedPin1, 255); 
   }
   if (strcmp(timmy_msg.data, "fwd") == 0) {
     digitalWrite(dirPin1, HIGH);
     analogWrite(speedPin1, 160); // lower max
     digitalWrite(dirPin2, HIGH);
     analogWrite(speedPin2, 255); // higher max
   }
   if (strcmp(timmy_msg.data, "ffwd") == 0) {
     digitalWrite(dirPin1, HIGH);
     analogWrite(speedPin1, 255);
     digitalWrite(dirPin2, HIGH);
     analogWrite(speedPin2, 255);
   }
   if (strcmp(timmy_msg.data, "fwdl") == 0) {
     digitalWrite(dirPin1, HIGH);
     analogWrite(speedPin1, 160);
     digitalWrite(dirPin2, HIGH);
     analogWrite(speedPin2, 128);
   }
   if (strcmp(timmy_msg.data, "fwdr") == 0) {
     digitalWrite(dirPin1, HIGH);
     analogWrite(speedPin1, 80);
     digitalWrite(dirPin2, HIGH);
     analogWrite(speedPin2, 255);
   }
   if (strcmp(timmy_msg.data, "bwd") == 0) {
     digitalWrite(dirPin1, LOW);
     analogWrite(speedPin1, 255); // higher max
     digitalWrite(dirPin2, LOW);
     analogWrite(speedPin2, 160); // lower max
   }
   if (strcmp(timmy_msg.data, "bbwd") == 0) {
     digitalWrite(dirPin1, LOW);
     analogWrite(speedPin1, 255);
     digitalWrite(dirPin2, LOW);
     analogWrite(speedPin2, 255);
   }
   if (strcmp(timmy_msg.data, "bwdl") == 0) {
     digitalWrite(dirPin1, LOW);
     analogWrite(speedPin1, 255);
     digitalWrite(dirPin2, LOW);
     analogWrite(speedPin2, 80);
   }
   if (strcmp(timmy_msg.data, "bwdr") == 0) {
     digitalWrite(dirPin1, LOW);
     analogWrite(speedPin1, 128);
     digitalWrite(dirPin2, LOW);
     analogWrite(speedPin2, 160);
   }
   if (strcmp(timmy_msg.data, "pivotl") == 0) {
     digitalWrite(dirPin1, LOW);
     analogWrite(speedPin1, 255);
     digitalWrite(dirPin2, HIGH);
     analogWrite(speedPin2, 255);
   }
   if (strcmp(timmy_msg.data, "pivotr") == 0) {
     digitalWrite(dirPin1, HIGH);
     analogWrite(speedPin1, 255);
     digitalWrite(dirPin2, LOW);
     analogWrite(speedPin2, 255);
   }
   if (strcmp(timmy_msg.data, "stop") == 0) {
     digitalWrite(dirPin1, HIGH);
     analogWrite(speedPin1, 0);
     digitalWrite(dirPin2, HIGH);
     analogWrite(speedPin2, 0);
   }
   if (strlen(timmy_msg.data) == 14 && timmy_msg.data[0] == 'c') { // custom speed
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
 }

 ros::Subscriber<std_msgs::String> sub("timmy", &messageCb );
 ros::Subscriber<std_msgs::Empty> esub("request", &info);

 void setup() {
   pinMode(LED_BUILTIN, OUTPUT);

   pinMode(dirPin1, OUTPUT);
   pinMode(speedPin1, OUTPUT);
   pinMode(dirPin2, OUTPUT);
   pinMode(speedPin2, OUTPUT);

   nh.initNode();
   nh.subscribe(sub);
   nh.subscribe(esub);

   delay(1000);
   Xrest = (analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0)) / 10;
   Yrest = (analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1)) / 10;
   Zrest = (analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2)) / 10;

   dull = 10; 

   vx = 0;
   vy = 0;
   vz = 0;
 }

 void loop() {
   nh.spinOnce();
  
   x = ((analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0)) / 10 - Xrest) / dull;
   y = ((analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1)) / 10 - Yrest) / dull;
   z = ((analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2)) / 10 - Zrest) / dull;

   if (x >= 10) moving[0] = 'f';
   else if (x > 0) moving[0] = 'm';
   else moving[0] = 's';

   if (y >= 10) moving[1] = 'f';
   else if (y > 0) moving[1] = 'm';
   else moving[1] = 's';

   if (z >= 10) moving[2] = 'f';
   else if (z > 0) moving[2] = 'm';
   else moving[2] = 's';

   str_msg.data = moving;
   pub.publish( &str_msg );

/*
   vx += x;
   vy += y;
   vz += z;

   Serial.print("velocities are x, y, z: ");
   Serial.print(vx, DEC);
   Serial.print(" ");
   Serial.print(vy, DEC);
   Serial.print(" ");
   Serial.print(vz, DEC);
   Serial.println(" ");
*/
 
   delay(100);
 }
