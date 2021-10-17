//define motor 1 related pins (right)
#define IN1 9
#define IN2 8
#define ENA 6
#define RIGHT_MOTOR 6

#define MOTOR_1A 3 // yellow, INTERRUPT
#define MOTOR_1B 4 // white

//define motor 2 related pins (left)
#define IN3 12
#define IN4 13
#define ENB 5
#define LEFT_MOTOR 5
                          

#define MOTOR_2A 2 // yellow, INTERRUPT
#define MOTOR_2B 7 // white


#define FULL_REV_ENCODER_TICKS 4200
//#define 

// run at 65 in the hallway



// encoder ticks: 4200 for full revolution

volatile long leftEncoderValue = 0;
volatile long rightEncoderValue = 0;

const int clockwiseLeftFwdSpeed = 150; /* PLACEHOLDER */
const int clockwiseRightSpeed = 100; /* PLACEHOLDER */
const long clockwiseLoopDuration = 42000; /* PLACEHOLDER */

const int counterclockwiseLeftSpeed = 130; /* PLACEHOLDER */
const int counterClockwiseRightFwdSpeed = 122; /* PLACEHOLDER */
const long counterClockwiseLoopDuration = clockwiseLoopDuration; /* PLACEHOLDER */

const int forwardLeftSpeed = 130; /* PLACEHOLDER */
const int forwardRightSpeed = 105; /* PLACEHOLDER */
const int forwardOffset = 0; /* PLACEHOLDER */

const int straightDistance1 = 5000; /* PLACEHOLDER */
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

  int rightspeed = forwardRightSpeed;
  int leftspeed = forwardLeftSpeed;

//  Serial.println(mtrspeed, DEC); 
//
//
//  int rightspeed = mtrspeed;
//  int leftspeed = mtrspeed;
  
  
  leftEncoderValue = 0;
  rightEncoderValue = 0;

  right_motor_fwd();
  left_motor_fwd();

  

  while (leftEncoderValue < distance) {
    if (leftEncoderValue < rightEncoderValue) {
      analogWrite(ENA, rightspeed - forwardOffset);
      analogWrite(ENB, leftspeed + forwardOffset);
    }
    else if (leftEncoderValue > rightEncoderValue) {
      analogWrite(ENA, rightspeed + forwardOffset);
      analogWrite(ENB, leftspeed - forwardOffset);
    }
    else {
      analogWrite(ENA, rightspeed);
      analogWrite(ENB, leftspeed);
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


void testmotorspeeds() {
  right_motor_fwd();
  left_motor_fwd();
  
  for (int i = 70; i < 200; i+=10) {
    Serial.println(i, DEC); 
    analogWrite(ENA, i);
    analogWrite(ENB, i);
    
    delay(5000);
    
  }
}


void testspeed(int mtrspeed) {

  left_motor_fwd();
  right_motor_fwd();
  
  leftEncoderValue = 0;
  
  unsigned long startTime = millis();
  while (leftEncoderValue < 5 * FULL_REV_ENCODER_TICKS) {
      analogWrite(LEFT_MOTOR, mtrspeed);
  }
  unsigned long endTime = millis();
  
  analogWrite(LEFT_MOTOR, 0);

  unsigned long left_duration = endTime - startTime; 


  rightEncoderValue = 0;
  startTime = millis();
  
  while (rightEncoderValue < 5 * FULL_REV_ENCODER_TICKS) {
      analogWrite(RIGHT_MOTOR, mtrspeed);
  }
  
  endTime = millis();

  analogWrite(RIGHT_MOTOR, 0);

  unsigned long right_duration = endTime - startTime; 

  Serial.print(mtrspeed, DEC);
  Serial.print(',');
  Serial.print(left_duration, DEC);
  Serial.print(',');
  Serial.println(right_duration, DEC); 
  
}


void loop() {

//  Serial.print(leftEncoderValue, DEC);
//  Serial.print('\t');
//  Serial.println(rightEncoderValue, DEC); 
//
  for (int i = 50; i <= 200; i += 5) {
    testspeed(i);
  }


//   testmotorspeeds();

  // 150 to overcome stall?



  /*
  right_motor_fwd();
  left_motor_fwd();
  // analogWrite(ENA, 255);
  // analogWrite(ENB, 255);

  analogWrite(ENA, 140);
  analogWrite(ENB, 140);

  delay(500);

  for (int i = 140; i >= 100; i--) {
    analogWrite(ENA, i);
    analogWrite(ENB, i);
  }

  analogWrite(ENA, 100);
  analogWrite(ENB, 110);

  delay(10000);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  delay(2000);
*/



  /* 
  // go straight 
  goStraight(straightDistance2);

  // clockwise rotation
  leftEncoderValue = 0;
  rightEncoderValue = 0;

  left_motor_fwd();
  right_motor_bwd();
  
  while (leftEncoderValue < clockwiseLoopDuration) {
    analogWrite(ENA, clockwiseRightSpeed);
    analogWrite(ENB, clockwiseLeftFwdSpeed);
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
    analogWrite(ENA, counterClockwiseRightFwdSpeed);
    analogWrite(ENB, counterclockwiseLeftSpeed);
  }
  */


  
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
