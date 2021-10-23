#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <eyes/Generic.h>

// right motor
#define IN1 9
#define IN2 8
#define ENA 6
#define RIGHT_MOTOR ENA

#define MOTOR_RA 3 // yellow, INTERRUPT
#define MOTOR_RB 4 // white

// left motor
#define IN3 12
#define IN4 13
#define ENB 5
#define LEFT_MOTOR ENB

#define MOTOR_LA 2 // yellow, INTERRUPT
#define MOTOR_LB 7 // white

volatile long countR = 0;
volatile long countL = 0;

ros::NodeHandle nh;

std_msgs::Int32 int32_msg_R;
std_msgs::Int32 int32_msg_L;
ros::Publisher pubR("encoder_value_R", &int32_msg_R);
ros::Publisher pubL("encoder_value_L", &int32_msg_L);

const char FWD = 'f';
const char BWD = 'b';

void setRightMotorDir(char dir) {
  if (dir == BWD) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
}

void setLeftMotorDir(char dir) {
  if (dir == BWD) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}

void generic_callback(const eyes::Generic& generic_msg) {
  if (generic_msg.left_forward) {
    setLeftMotorDir(FWD);
  }
  else {
    setLeftMotorDir(BWD);
  }
  if (generic_msg.right_forward) {
    setRightMotorDir(FWD);
  }
  else {
    setRightMotorDir(BWD);
  }
  analogWrite(LEFT_MOTOR, generic_msg.left_speed);
  analogWrite(RIGHT_MOTOR, generic_msg.right_speed);

  return;
}

ros::Subscriber<eyes::Generic> generic_sub("generic_feed", &generic_callback);

void setup() {
  nh.initNode();
  nh.advertise(pubR);
  nh.advertise(pubL);
  nh.subscribe(generic_sub);

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(MOTOR_RA, INPUT);
  pinMode(MOTOR_RB, INPUT);

  pinMode(MOTOR_LA, INPUT);
  pinMode(MOTOR_LB, INPUT);

  attachInterrupt(digitalPinToInterrupt(MOTOR_RA), EncoderEventR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_LA), EncoderEventL, CHANGE);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

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
  if (digitalRead(MOTOR_RA) == HIGH) {
    if (digitalRead(MOTOR_RB) == LOW) {
      ++countR;
    } else {
      --countR;
    }
  } else {
    if (digitalRead(MOTOR_RB) == LOW) {
      --countR;
    } else {
      ++countR;
    }
  }
}

void EncoderEventL() {
  if (digitalRead(MOTOR_LA) == HIGH) {
    if (digitalRead(MOTOR_LB) == LOW) {
      --countL;
    } else {
      ++countL;
    }
  } else {
    if (digitalRead(MOTOR_LB) == LOW) {
      ++countL;
    } else {
      --countL;
    }
  }
}
