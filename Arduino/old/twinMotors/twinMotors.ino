#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <eyes/Generic.h>

                          // right motor
const int dirPin1 = 5;
const int speedPin1 = 6;

                          // left motor
const int dirPin2 = 9;
const int speedPin2 = 10;

/****************************/
volatile bool interrupted;
int pin_value;
/****************************/

ros::NodeHandle nh;

std_msgs::Empty empty_msg;
ros::Publisher pub("notifications", &empty_msg);

void generic_callback(const eyes::Generic& generic_msg) {
  interrupted = false;
  if (generic_msg.left_forward) {
    digitalWrite(dirPin2, HIGH);
  }
  else {
    digitalWrite(dirPin2, LOW);
  }
  if (generic_msg.right_forward) {
    digitalWrite(dirPin1, HIGH);
  }
  else {
    digitalWrite(dirPin1, LOW);
  }
  analogWrite(speedPin2, generic_msg.left_speed);
  analogWrite(speedPin1, generic_msg.right_speed);

  if (generic_msg.identifier == 'c') {
    // interrupted = false;
    if (generic_msg.timed) {
      long length_of_time = millis() + generic_msg.duration * 1000;
      while (length_of_time > millis() && !interrupted) {
        // do nothing
      }
      interrupted = false;
      pub.publish(&empty_msg);
    }
    else {
      // do nothing, placeholder for error state
    }
  }
  
  return;
}

ros::Subscriber<eyes::Generic> generic_sub("generic_feed", &generic_callback);

/********************************/
void pciSetup(byte pin) {
  *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));
  PCIFR |= bit(digitalPinToPCICRbit(pin));
  PCICR |= bit(digitalPinToPCICRbit(pin));
}

ISR(PCINT0_vect) {
  int new_value = digitalRead(8);
  if (new_value != pin_value) {
    pin_value = new_value;
    digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));
    interrupted = true;
  }
}
/********************************/

void setup() {
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(generic_sub);

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(dirPin1, OUTPUT);
  pinMode(speedPin1, OUTPUT);                                                                                       

  pinMode(dirPin2, OUTPUT);
  pinMode(speedPin2, OUTPUT);

  /*****************************/
  interrupted = false;
  pinMode(8, INPUT);
  digitalWrite(8, HIGH);
  pciSetup(8);
  pin_value = digitalRead(8);
  /*****************************/
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
}
