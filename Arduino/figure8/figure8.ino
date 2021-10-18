//define motor 1 related pins (right)
#define IN1 9
#define IN2 8
#define ENA 6
#define RIGHT_MOTOR ENA

#define MOTOR_1A 3 // yellow, INTERRUPT
#define MOTOR_1B 4 // white

//define motor 2 related pins (left)
#define IN3 12
#define IN4 13
#define ENB 5
#define LEFT_MOTOR ENB
                          
#define MOTOR_2A 2 // yellow, INTERRUPT
#define MOTOR_2B 7 // white

// define constants
const int FULL_REV_ENCODER_TICKS = 4200;
const char FWD = 'f';
const char BWD = 'b';

volatile long leftEncoderValue = 0;
volatile long rightEncoderValue = 0;

const int clockwiseLeftFwdSpeed = 150; /* PLACEHOLDER */
const int clockwiseRightSpeed = 130; /* PLACEHOLDER */
const long clockwiseLoopDuration = 24500; /* in encoder ticks PLACEHOLDER */

const int counterClockwiseRightFwdSpeed = clockwiseLeftFwdSpeed; /* PLACEHOLDER */
const int counterClockwiseLeftSpeed = clockwiseRightSpeed; /* PLACEHOLDER */
const long counterClockwiseLoopDuration = clockwiseLoopDuration; /* in encoder ticks PLACEHOLDER */

const int forwardLeftSpeed = 110; /* PLACEHOLDER */
const int forwardRightSpeed = forwardLeftSpeed; /* PLACEHOLDER */
const int forwardOffset = 0; /* PLACEHOLDER */

const int straightDistance1 = 5000; /* in encoder ticks PLACEHOLDER */
const int straightDistance2 = straightDistance1; /* in encoder ticks PLACEHOLDER */

int state = 0;


void setRightMotorDir(char dir) {
  if (dir == BWD) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
}

void setLeftMotorDir(char dir) {
  if (dir == BWD) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}


// Allow mtrspeed to be overwritten, but set default to -1, which is a flag to use defined speeds up top
void goStraight(int distance, int mtrspeed = -1) {
  int rightspeed = mtrspeed == -1 ? forwardRightSpeed : mtrspeed;
  int leftspeed  = mtrspeed == -1 ? forwardLeftSpeed : mtrspeed;

  Serial.println(rightspeed, DEC);
  Serial.println(leftspeed, DEC);
  leftEncoderValue = 0;
  rightEncoderValue = 0;

  setLeftMotorDir(FWD);
  setRightMotorDir(FWD);

  while (leftEncoderValue < distance) {
    analogWrite(RIGHT_MOTOR, rightspeed);
    analogWrite(LEFT_MOTOR, leftspeed);
    Serial.print(leftEncoderValue, DEC);
    Serial.print('\t');
    Serial.println(rightEncoderValue, DEC); 
  }
//
//  while (leftEncoderValue < distance) {
//    if (leftEncoderValue < rightEncoderValue) {
//      analogWrite(RIGHT_MOTOR, rightspeed - forwardOffset);
//      analogWrite(LEFT_MOTOR, leftspeed + forwardOffset);
//    }
//    else if (leftEncoderValue > rightEncoderValue) {
//      analogWrite(RIGHT_MOTOR, rightspeed + forwardOffset);
//      analogWrite(LEFT_MOTOR, leftspeed - forwardOffset);
//    }
//    else {
//      analogWrite(RIGHT_MOTOR, rightspeed);
//      analogWrite(LEFT_MOTOR, leftspeed);
//    }
//  }
}



void rampUp() {
  Serial.println("Ramping motor speeds"); 
  setLeftMotorDir(FWD);
  setRightMotorDir(FWD);

  int startSpeed = 50;
  int endSpeed = 100;
  int increment = 50;
  
  for (int mtrspeed = startSpeed; mtrspeed <= endSpeed; mtrspeed+=increment) {
    Serial.println(mtrspeed, DEC); 
    analogWrite(LEFT_MOTOR, mtrspeed);
    analogWrite(RIGHT_MOTOR, mtrspeed);
    
    delay(5000);   
  }

  analogWrite(LEFT_MOTOR, 0);
  analogWrite(RIGHT_MOTOR, 0);
  delay(5000);
}


void testspeed(int mtrspeed, char mtrdir) {
  setRightMotorDir(mtrdir);
  setLeftMotorDir(mtrdir);
  
  leftEncoderValue = 0;
  rightEncoderValue = 0;
  
  // Test left motor
  unsigned long startTime = millis();

  if (mtrdir == FWD) {
    while (leftEncoderValue < 5 * FULL_REV_ENCODER_TICKS) {
      analogWrite(LEFT_MOTOR, mtrspeed);
    }
  } else {
    while (leftEncoderValue >= -5 * FULL_REV_ENCODER_TICKS) {
      analogWrite(LEFT_MOTOR, mtrspeed);
    }
  }
  
  unsigned long endTime = millis();
  analogWrite(LEFT_MOTOR, 0);

  unsigned long left_duration = endTime - startTime; 


  // Test right motor
  startTime = millis();
  
  if (mtrdir == FWD) {
    while (rightEncoderValue < 5 * FULL_REV_ENCODER_TICKS) {
      analogWrite(RIGHT_MOTOR, mtrspeed);
    }
  } else {
    while (rightEncoderValue >= -5 * FULL_REV_ENCODER_TICKS) {
      analogWrite(RIGHT_MOTOR, mtrspeed);
    }
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

void go_fwd_bwd(char dir, long duration, int maxLeftSpeed, int maxRightSpeed) {

  leftEncoderValue = 0;
  rightEncoderValue = 0;

  Serial.println(String(dir) + " " + String(maxLeftSpeed) + " " + String(maxRightSpeed));

  int gooseSpeed = 140;
  
  setRightMotorDir(dir);
  setLeftMotorDir(dir);

  // goose
  analogWrite(RIGHT_MOTOR, gooseSpeed);
  analogWrite(LEFT_MOTOR, gooseSpeed);

  delay(500);

  // go forward 10 seconds
  int leftSpeed = gooseSpeed;
  int rightSpeed = gooseSpeed;

  while (leftSpeed > maxLeftSpeed || rightSpeed > maxRightSpeed) {
    analogWrite(LEFT_MOTOR, leftSpeed);
    analogWrite(RIGHT_MOTOR, rightSpeed);

    leftSpeed = max(leftSpeed - 1, maxLeftSpeed);
    rightSpeed = max(rightSpeed - 1, maxRightSpeed);

//    Serial.println(String(leftSpeed) + "," + String(rightSpeed));
  }

  leftEncoderValue = 0;
  rightEncoderValue = 0;

  leftEncoderValue = 0;
  rightEncoderValue = 0;

  Serial.println("ENCODERS");

  if (dir == FWD) {
    while (leftEncoderValue < duration) {
      Serial.println(String(leftEncoderValue) + "," + String(rightEncoderValue));
    }
  } else {
    while (leftEncoderValue > -1 * duration) {
       Serial.println(String(leftEncoderValue) + "," + String(rightEncoderValue));
    }
  }
  

  // // slow down and stop one second
  // while (leftSpeed > 0 && rightSpeed > 0) {
  //   analogWrite(LEFT_MOTOR, leftSpeed);
  //   analogWrite(RIGHT_MOTOR, rightSpeed);

  //   leftSpeed = max(leftSpeed - 1, 0);
  //   rightSpeed = max(rightSpeed - 1, 0);
  // }

  analogWrite(RIGHT_MOTOR, 0);
  analogWrite(LEFT_MOTOR, 0);

  delay(1000);

}


void test() {
  int startSpeed = 50;
  int endSpeed = 100;
  int increment = 50;
  
  Serial.println("fwd_speed,left_ms,right_ms");
  for (int i = startSpeed; i <= endSpeed; i += increment) {
    testspeed(i, FWD);
  }
  
  Serial.println("bwd_speed,left_ms,right_ms");
  for (int i = startSpeed; i <= endSpeed; i += increment) {
    testspeed(i, BWD);
  }

  delay(5000);
}

/**********************************************************/
/*                        FIGURE 8                        */  
/**********************************************************/

void figureEight() {

  setRightMotorDir(FWD);
  setLeftMotorDir(FWD);

  // go straight 
  goStraight(straightDistance2);

  // clockwise rotation
  leftEncoderValue = 0;
  rightEncoderValue = 0;

  setLeftMotorDir(FWD);
  setRightMotorDir(BWD);
  
  while (leftEncoderValue < clockwiseLoopDuration) {
    analogWrite(RIGHT_MOTOR, clockwiseRightSpeed);
    analogWrite(LEFT_MOTOR, clockwiseLeftFwdSpeed);
    Serial.print(leftEncoderValue, DEC);
    Serial.print('\t');
    Serial.println(rightEncoderValue, DEC); 
  }
  
  // go straight
  goStraight(straightDistance1);

  // counter clockwise rotation
  leftEncoderValue = 0;
  rightEncoderValue = 0;
  
  setLeftMotorDir(BWD);
  setRightMotorDir(FWD);
  
  while (rightEncoderValue < counterClockwiseLoopDuration) {
    analogWrite(RIGHT_MOTOR, counterClockwiseRightFwdSpeed);
    analogWrite(LEFT_MOTOR, counterClockwiseLeftSpeed);
  }
}


/**********************************************************/
/*                        SETUP & LOOP                    */  
/**********************************************************/


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
  
  setLeftMotorDir(FWD);
  setRightMotorDir(FWD);

  //set encoder input from motor 1
  pinMode(MOTOR_1A, INPUT);
  pinMode(MOTOR_1B, INPUT);

  //set encoder input from motor 2
  pinMode(MOTOR_2A, INPUT);
  pinMode(MOTOR_2B, INPUT);

  attachInterrupt(digitalPinToInterrupt(MOTOR_1A), EncoderEvent1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_2A), EncoderEvent2, CHANGE);


  leftEncoderValue = 0;
  rightEncoderValue = 0;

//  setLeftMotorDir(BWD);
//  setRightMotorDir(FWD);
//  
//  while (rightEncoderValue < 2500) {
//    analogWrite(RIGHT_MOTOR, counterClockwiseRightFwdSpeed);
//    analogWrite(LEFT_MOTOR, counterClockwiseLeftSpeed);
//    Serial.print(leftEncoderValue, DEC);
//    Serial.print('\t');
//    Serial.println(rightEncoderValue, DEC); 
//  }
}


void loop() {
  // TODO: set mode

  // modes:
  //    t: test       (collect time for each wheel to complete 5 revolutions at diff speeds)
  //    f: figure 8   (run figure 8)
  //    r: ramp up    (ramp up speed with robot going fwd)
  //    1: go fwd/bwd
  //    2: go clockwise circle
  //    3: go counterclockwise circle
  char mode = '1';

  if (mode == 't') {
    test();
  } else if (mode == 'f') {
    figureEight();
  } else if (mode == 'r') {
    rampUp();
  } else if (mode == '1') {
    leftEncoderValue = 0;
    rightEncoderValue = 0;
    go_fwd_bwd(FWD, 8000, 115, 100);
    leftEncoderValue = 0;
    rightEncoderValue = 0;
    go_fwd_bwd(BWD, 9500, 100, 100);
  } else {
    Serial.println("Unsupported loop mode. Must be one of: [t, f, r, '1', '2', '3']."); 
  }
}



/**********************************************************/
/*                          ENCODERS                      */  
/**********************************************************/

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
