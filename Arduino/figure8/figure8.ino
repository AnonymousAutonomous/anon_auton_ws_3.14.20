                          // right motor
const int dirPin1 = 5;
const int speedPin1 = 6;

const int motor1A = 3; // yellow, INTERRUPT
const int motor1B = 4; // white

                          // left motor
const int dirPin2 = 9;
const int speedPin2 = 10;

const int motor2A = 2; // yellow, INTERRUPT
const int motor2B = 7; // white

volatile long leftEncoderValue = 0;
volatile long rightEncoderValue = 0;

const int clockwiseLeftSpeed = 160; /* PLACEHOLDER */
const int clockwiseRightSpeed = 0; /* PLACEHOLDER */
const long clockwiseLoopDuration = 42000; /* PLACEHOLDER */

const int counterClockwiseLeftSpeed = clockwiseRightSpeed; /* PLACEHOLDER */
const int counterClockwiseRightSpeed = clockwiseLeftSpeed; /* PLACEHOLDER */
const long counterClockwiseLoopDuration = clockwiseLoopDuration; /* PLACEHOLDER */

const int forwardLeftSpeed = 60; /* PLACEHOLDER */
const int forwardRightSpeed = 60; /* PLACEHOLDER */
const int forwardOffset = 10; /* PLACEHOLDER */

const int straightDistance1 = 4900; /* PLACEHOLDER */
const int straightDistance2 = straightDistance1; /* PLACEHOLDER */

void goStraight(int distance) {
  leftEncoderValue = 0;
  rightEncoderValue = 0;

  digitalWrite(dirPin1, HIGH);
  digitalWrite(dirPin2, HIGH);

  while (leftEncoderValue < distance) {
    if (leftEncoderValue < rightEncoderValue) {
      analogWrite(speedPin1, forwardRightSpeed - forwardOffset);
      analogWrite(speedPin2, forwardLeftSpeed + forwardOffset);
    }
    else if (leftEncoderValue > rightEncoderValue) {
      analogWrite(speedPin1, forwardRightSpeed + forwardOffset);
      analogWrite(speedPin2, forwardLeftSpeed - forwardOffset);
    }
    else {
      analogWrite(speedPin1, forwardRightSpeed);
      analogWrite(speedPin2, forwardLeftSpeed);
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(dirPin1, OUTPUT);
  pinMode(speedPin1, OUTPUT);
  
  pinMode(dirPin2, OUTPUT);
  pinMode(speedPin2, OUTPUT);
  
  pinMode(motor1A, INPUT);
  pinMode(motor1B, INPUT);

  pinMode(motor2A, INPUT);
  pinMode(motor2B, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(3), EncoderEvent1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), EncoderEvent2, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // clockwise rotation
  
  leftEncoderValue = 0;
  rightEncoderValue = 0;

  digitalWrite(dirPin1, HIGH);
  digitalWrite(dirPin2, HIGH);
  
  while (leftEncoderValue < clockwiseLoopDuration) {
    analogWrite(speedPin1, clockwiseRightSpeed);
    analogWrite(speedPin2, clockwiseLeftSpeed);
  }
//
//  while(1) {
//    analogWrite(speedPin1, 0 );
//    analogWrite(speedPin2, 0);
//  }
  
  // go straight
  
  goStraight(straightDistance1);
  
  // counter clockwise rotation
  
  leftEncoderValue = 0;
  rightEncoderValue = 0;
  
  digitalWrite(dirPin1, HIGH);
  digitalWrite(dirPin2, HIGH);
  
  while (rightEncoderValue < counterClockwiseLoopDuration) {
    analogWrite(speedPin1, counterClockwiseRightSpeed);
    analogWrite(speedPin2, counterClockwiseLeftSpeed);
  }
  
  // go straight
  
  goStraight(straightDistance2);
//  while(1) {
//    analogWrite(speedPin1, 0 );
//    analogWrite(speedPin2, 0);
//  }
}

void EncoderEvent1() {
  if (digitalRead(motor1A) == HIGH) {
    if (digitalRead(motor1B) == LOW) {
      ++rightEncoderValue; 
    } else {
      --rightEncoderValue;
    }
  } else {
    if (digitalRead(motor1B) == LOW) {
      --rightEncoderValue;
    } else {
      ++rightEncoderValue;
    }
  }
}

void EncoderEvent2() {
  if (digitalRead(motor2A) == HIGH) {
    if (digitalRead(motor2B) == LOW) {
      --leftEncoderValue; 
    } else {
      ++leftEncoderValue;
    }
  } else {
    if (digitalRead(motor2B) == LOW) {
      ++leftEncoderValue;
    } else {
      --leftEncoderValue;
    }
  }
}
