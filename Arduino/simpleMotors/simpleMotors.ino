// left and right motor swapped!!!!!!!! -- changing all the values here 

//define motor 1 related pins (right)
#define IN1 12         // was 9
#define IN2 13         // was 8
#define ENA 10        // was 11
#define RIGHT_MOTOR ENA

#define MOTOR_1A 2 // yellow, INTERRUPT // was 3
#define MOTOR_1B 7 // white             // was 4

//define motor 2 related pins (left)
#define IN3 9        // was 12
#define IN4 8        // was 13
#define ENB 11        // was 10
#define LEFT_MOTOR ENB
                          
#define MOTOR_2A 3 // yellow, INTERRUPT // was 2
#define MOTOR_2B 4 // white             // was 7

// define constants
const int FULL_REV_ENCODER_TICKS = 4200;
const char FWD = 'f';
const char BWD = 'b';

char dir_left = FWD;
char dir_right = FWD;
int speed_left = 0;
int speed_right = 0;


volatile long leftEncoderValue = 0;
volatile long rightEncoderValue = 0;

const unsigned int MAX_INPUT = 50;


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



void goRight(char dir, int mtrspeed) {
    setRightMotorDir(dir);
    analogWrite(RIGHT_MOTOR, mtrspeed);
}

void goLeft(char dir, int mtrspeed) {
    setLeftMotorDir(dir);
    analogWrite(LEFT_MOTOR, mtrspeed);
}

void setRightMotorDir(char dir) {
  if (dir == BWD) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
}

void setLeftMotorDir(char dir) {
  if (dir == BWD) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
}


void rampUp() {
  Serial.println("Ramping motor speeds"); 
  setLeftMotorDir(FWD);
  setRightMotorDir(FWD);

  int startSpeed = 50;
  int endSpeed = 100;
  int increment = 5;
  
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

  int gooseSpeed = 100;
  
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
    while (leftEncoderValue > -1 * duration) {
      Serial.println(String(leftEncoderValue) + "," + String(rightEncoderValue));
    }
  } else {
    while (leftEncoderValue < duration) {
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

void spin(char dir_left, char dir_right, int leftSpeed, int rightSpeed) {
  setLeftMotorDir(dir_left);
  setRightMotorDir(dir_right);

  while(true) {
    analogWrite(LEFT_MOTOR, leftSpeed);
    analogWrite(RIGHT_MOTOR, rightSpeed);
  }
}

void goClockwise() {
  spin(FWD, FWD, 200, 0);
}

void goCounterClockwise() {
  spin(FWD, FWD, 0, 200);
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
}


void process_data (const char* data) {
  Serial.print("Processing: "); Serial.print(data); Serial.print("\n");
  
  if (data[0] == 's') { // stop
      dir_left = FWD;
      dir_right = FWD;
      speed_left = 0;
      speed_right = 0;
  }

  String sub = "";
  bool left = true;

 for (int i = 0; i < strlen(data); ++i) {
  if (data[i] == ' ' || data[i] == '\n') {
  }
  else if (data[i] == FWD || data[i] == BWD) {
    if (left) {
      dir_left = data[i];
      left = false;
    }
    else {
      dir_right = data[i];
      speed_left = sub.toInt();
      sub = "";
    }
  }
  else {
    sub += data[i];
  }
  }
         speed_right = sub.toInt();


  Serial.print("Running: "); Serial.print(dir_left); Serial.print(' '); Serial.print(speed_left);
    Serial.print(" "); Serial.print(dir_right); Serial.print(' '); Serial.print(speed_right); Serial.print('\n');

}

void processIncomingByte(const byte inByte) {
  static char input_line [MAX_INPUT];
  static unsigned int input_pos = 0;

  switch (inByte) {
    case '\n':
      input_line[input_pos] = 0; // terminating null byte
      process_data(input_line);
      // reset
      input_pos = 0;
      break;
    case '\r':
      break; 
    default:
      if (input_pos < (MAX_INPUT - 1))
        input_line [input_pos++] = inByte;
      break;
  }
}



void loop() {
  while (Serial.available() > 0) {
    processIncomingByte(Serial.read());
  }

  goLeft(dir_left, speed_left);
  goRight(dir_right, speed_right);

  // modes:
  //    t: test       (collect time for each wheel to complete 5 revolutions at diff speeds)
  //    f: figure 8   (run figure 8)
  //    r: ramp up    (ramp up speed with robot going fwd)
  //    1: go fwd/bwd
  //    2: go clockwise circle
  //    3: go counterclockwise circle
  
//   char mode = '1';

//   if (mode == 't') {
//     test();
//   } else if (mode == 'f') {
//     figureEight();
//   } else if (mode == 'r') {
//     rampUp();
//   } else if (mode == '1') {
//     leftEncoderValue = 0;
//     rightEncoderValue = 0;
//     go_fwd_bwd(FWD, 12000, 80, 80);
//     leftEncoderValue = 0;
//     rightEncoderValue = 0;
//     go_fwd_bwd(BWD, 12000, 80, 80);
//   } else if (mode == '2') {
//     goClockwise();
//   } else if (mode == '3') {
//     goCounterClockwise();
//   } else {
//     Serial.println("Unsupported loop mode. Must be one of: [t, f, r, '1', '2', '3']."); 
//   }
}
