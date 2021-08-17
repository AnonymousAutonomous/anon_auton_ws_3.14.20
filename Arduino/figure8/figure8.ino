//define motor 1 related pins (right)
#define IN1 9
#define IN2 8
#define ENA 10

#define MOTOR_1A 3 // yellow, INTERRUPT
#define MOTOR_1B 4 // white

//define motor 2 related pins (left)
#define IN3 12
#define IN4 13
#define ENB 5                          

#define MOTOR_2A 2 // yellow, INTERRUPT
#define MOTOR_2B 7 // white

//#define 


volatile long leftEncoderValue = 0;
volatile long rightEncoderValue = 0;

const int clockwiseLeftSpeed = 160; /* PLACEHOLDER */
const int clockwiseRightSpeed = 40; /* PLACEHOLDER */
const long clockwiseLoopDuration = 42000; /* PLACEHOLDER */

const int counterClockwiseLeftSpeed = clockwiseRightSpeed; /* PLACEHOLDER */
const int counterClockwiseRightSpeed = clockwiseLeftSpeed; /* PLACEHOLDER */
const long counterClockwiseLoopDuration = clockwiseLoopDuration; /* PLACEHOLDER */

const int forwardLeftSpeed = 60; /* PLACEHOLDER */
const int forwardRightSpeed = 60; /* PLACEHOLDER */
const int forwardOffset = 10; /* PLACEHOLDER */

const int straightDistance1 = 3500; /* PLACEHOLDER */
const int straightDistance2 = straightDistance1; /* PLACEHOLDER */

int state = 0;


void right_motor_fwd() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void right_motor_bwd() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void left_motor_fwd() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void left_motor_bwd() {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}


void goStraight(int distance) {
  leftEncoderValue = 0;
  rightEncoderValue = 0;

  right_motor_fwd();
  left_motor_fwd();

  while (leftEncoderValue < distance) {
    if (leftEncoderValue < rightEncoderValue) {
      analogWrite(ENA, forwardRightSpeed - forwardOffset);
      analogWrite(ENB, forwardLeftSpeed + forwardOffset);
    }
    else if (leftEncoderValue > rightEncoderValue) {
      analogWrite(ENA, forwardRightSpeed + forwardOffset);
      analogWrite(ENB, forwardLeftSpeed - forwardOffset);
    }
    else {
      analogWrite(ENA, forwardRightSpeed);
      analogWrite(ENB, forwardLeftSpeed);
    }
  }
}

void setup() {
  Serial.begin(115200); // open the serial port at 115200 bps:
  
  //set output for motor 1 related pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  
  //set output for motor 2 related pins
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  right_motor_fwd();
  left_motor_fwd();

  //set encoder input from motor 1
  pinMode(MOTOR_1A, INPUT);
  pinMode(MOTOR_1B, INPUT);

  //set encoder input from motor 2
  pinMode(MOTOR_2A, INPUT);
  pinMode(MOTOR_2B, INPUT);

  attachInterrupt(digitalPinToInterrupt(MOTOR_1A), EncoderEvent1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_2A), EncoderEvent2, CHANGE);
  
}

void loop() {
  // clockwise rotation
  leftEncoderValue = 0;
  rightEncoderValue = 0;

  left_motor_fwd();
  right_motor_bwd();
  
  while (leftEncoderValue < clockwiseLoopDuration) {
    analogWrite(ENA, clockwiseRightSpeed);
    analogWrite(ENB, clockwiseLeftSpeed);
    Serial.print(leftEncoderValue, DEC);
    Serial.print('\t');
    Serial.println(rightEncoderValue, DEC); 
  }
  
  // go straight
  
  goStraight(straightDistance1);
  
  // counter clockwise rotation
  
  leftEncoderValue = 0;
  rightEncoderValue = 0;
  
  left_motor_bwd();
  right_motor_fwd();
  
  while (rightEncoderValue < counterClockwiseLoopDuration) {
    analogWrite(ENA, counterClockwiseRightSpeed);
    analogWrite(ENB, counterClockwiseLeftSpeed);
  }
  
  // go straight 
  goStraight(straightDistance2);
  
}

void EncoderEvent1() {
  if (digitalRead(MOTOR_1A) == HIGH) {
    if (digitalRead(MOTOR_1B) == LOW) {
      ++rightEncoderValue; 
    } else {
      --rightEncoderValue;
    }
  } else {
    if (digitalRead(MOTOR_1B) == LOW) {
      --rightEncoderValue;
    } else {
      ++rightEncoderValue;
    }
  }
}

void EncoderEvent2() {
  if (digitalRead(MOTOR_2A) == HIGH) {
    if (digitalRead(MOTOR_2B) == LOW) {
      --leftEncoderValue; 
    } else {
      ++leftEncoderValue;
    }
  } else {
    if (digitalRead(MOTOR_2B) == LOW) {
      ++leftEncoderValue;
    } else {
      --leftEncoderValue;
    }
  }
}
