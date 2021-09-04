#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <eyes/Generic.h>

// right motor
const int dirPinR = 5;
const int speedPinR = 6;

const int motorRA = 3; // yellow, INTERRUPT
const int motorRB = 4; // white

// left motor
const int dirPinL = 9;
const int speedPinL = 10;

const int motorLA = 2; // yellow, INTERRUPT
const int motorLB = 7; // white

volatile long countR = 0;
volatile long countL = 0;

ros::NodeHandle nh;

std_msgs::Int32 int32_msg_R;
std_msgs::Int32 int32_msg_L;
ros::Publisher pubR("encoder_value_R", &int32_msg_R);
ros::Publisher pubL("encoder_value_L", &int32_msg_L);

void generic_callback(const eyes::Generic& generic_msg) {
  if (generic_msg.left_forward) {
    digitalWrite(dirPinL, HIGH);
  }
  else {
    digitalWrite(dirPinL, LOW);
  }
  if (generic_msg.right_forward) {
    digitalWrite(dirPinR, HIGH);
  }
  else {
    digitalWrite(dirPinR, LOW);
  }
  analogWrite(speedPinL, generic_msg.left_speed);
  analogWrite(speedPinR, generic_msg.right_speed);

  return;
}

ros::Subscriber<eyes::Generic> generic_sub("generic_feed", &generic_callback);

void setup() {
  nh.initNode();
  nh.advertise(pubR);
  nh.advertise(pubL);
  nh.subscribe(generic_sub);

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(motorRA, INPUT);
  pinMode(motorRB, INPUT);

  pinMode(motorLA, INPUT);
  pinMode(motorLB, INPUT);

  attachInterrupt(digitalPinToInterrupt(3), EncoderEventR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), EncoderEventL, CHANGE);

  pinMode(dirPinR, OUTPUT);
  pinMode(speedPinR, OUTPUT);

  pinMode(dirPinL, OUTPUT);
  pinMode(speedPinL, OUTPUT);

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  // Serial.begin(9600);

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
}

void loop() {
  // put your main code here, to run repeatedly:

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  // digitalWrite(dirPinR, LOW);
  // analogWrite(speedPinR, 0);

  // Serial.print("Right Count: ");
  // Serial.println(countR);
  // Serial.println();

  // digitalWrite(dirPinL, HIGH);
  // analogWrite(speedPinL, 255);

  // Serial.print("Left Count: ");
  // Serial.println(countL);
  // Serial.println();

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  int32_msg_R.data = countR;
  int32_msg_L.data = countL;
  pubR.publish(&int32_msg_R);
  pubL.publish(&int32_msg_L);

  nh.spinOnce();
  delay(10);
}

void EncoderEventR() {
  if (digitalRead(motorRA) == HIGH) {
    if (digitalRead(motorRB) == LOW) {
      ++countR;
    } else {
      --countR;
    }
  } else {
    if (digitalRead(motorRB) == LOW) {
      --countR;
    } else {
      ++countR;
    }
  }
}

void EncoderEventL() {
  if (digitalRead(motorLA) == HIGH) {
    if (digitalRead(motorLB) == LOW) {
      --countL;
    } else {
      ++countL;
    }
  } else {
    if (digitalRead(motorLB) == LOW) {
      ++countL;
    } else {
      --countL;
    }
  }
}
