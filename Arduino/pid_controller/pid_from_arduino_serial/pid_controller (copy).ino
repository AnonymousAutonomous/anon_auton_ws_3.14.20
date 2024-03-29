#include "motors_and_pins.h"
#include "pid_serial.h"
//#include "ros_serial.h"
#include <PID_v1.h>

boolean DEBUG = true;

// Motor timing
unsigned long nowTime = 0;       // updated on every loop
unsigned long startTimeA = 0;    // start timing A interrupts
unsigned long startTimeB = 0;    // start timing B interrupts
unsigned long countIntA = 0;     // count the A interrupts
unsigned long countIntB = 0;     // count the B interrupts
double periodA = 0;              // motor A period
double periodB = 0;              // motor B period

// PID 
const unsigned long SAMPLE_TIME = 10;  // time between PID updates
const unsigned long INT_COUNT = 100;     // 100 encoder ticks for accurate timing


// !!!!!!!!!!!!! SETPOINTS MUST BE POSITIVE !!!!!!!!!!!!!!
double setpointA = 3.0;         // setpoint is inches / second
double setpointB = setpointA;   // setpoint is inches / second

double inputA = 0;              // input is inches / second
double outputA = 0;             // output is PWM to motors
int FEEDFWDA = 60;
int a_adjust = 0;

double inputB = 0;              // input is inches / second
double outputB = 0;             // output is PWM to motors
int FEEDFWDB = 60;
int b_adjust = 0;

double KpA = 10.0, KiA = 25.0, KdA = 0.0;
double KpB = KpA, KiB = KiA, KdB = KdA;
PID motorA(&inputA, &outputA, setpointA, KpA, KiA, KdA, DIRECT);
PID motorB(&inputB, &outputB, setpointB, KpB, KiB, KdB, DIRECT);
double storeB = 0;               // used for debug print

boolean CONNECTED_TO_ROS = false;

void initEncoders(){
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(ENCA == 2 ? 0 : 1, isr_A, CHANGE);  // TODO: fix this for the updated arduino board
  attachInterrupt(ENCB == 2 ? 0 : 1, isr_B, CHANGE);
};

void initPWM(){
  startTimeA = millis();
  startTimeB = millis();
  motorA.SetOutputLimits(MIN_PWM, MAX_PWM);
  motorB.SetOutputLimits(MIN_PWM, MAX_PWM);
  motorA.SetSampleTime(SAMPLE_TIME);
  motorB.SetSampleTime(SAMPLE_TIME);
  motorA.SetMode(AUTOMATIC);
  motorB.SetMode(AUTOMATIC);
  setNewSetpointMotorA(setpointA, FWD);
  setNewSetpointMotorB(setpointB, FWD);
};


void setNewSetpointMotorA(float setpoint, char dir) {
  if (dir == STOP) {
    a_adjust = 0;
  }
  else {
    a_adjust = FEEDFWDA;
  }
  setpointA = setpoint;
  motorA.SetSetpoint(setpoint);
  setADir(dir);
}

void setNewSetpointMotorB(float setpoint, char dir) {
  if (dir == STOP) {
    b_adjust = 0;
  }
  else {
    b_adjust = FEEDFWDB;
  }
  setpointB = setpoint;
  motorB.SetSetpoint(setpoint);
  setBDir(dir);
}

void processIncomingByte(const byte inByte) {
  static char input_line [MAX_INPUT];
  static unsigned int input_pos = 0;

  switch (inByte) {
    case '\n':
      input_line[input_pos] = 0; // terminating null byte
      processData(input_line);
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

void processData(const char* data) {
  char changeVar = data[0];
  if (changeVar == 'p') {
    KpA = KpB = strToFloat(String(data).substring(1));
  }
  else if (changeVar == 'i') {
    KiA = KiB = strToFloat(String(data).substring(1));
  }
  else if (changeVar == 'd') {
    KdA = KdB = strToFloat(String(data).substring(1));
  }
  else if (changeVar == 'b') {
        standbyMotors(true);
  }
  else if (changeVar == 's') {
    parseNewSetpoints(String(data).substring(1));
  }
  motorA.SetTunings(KpA, KiA, KdA);
    motorB.SetTunings(KpB, KiB, KdB);


     printPID(KpA, KiA, KdA, setpointA, FEEDFWDA, a_adjust);
}

void parseNewSetpoints(String setpointsIn) {
  char ADir = '\0';
  char BDir = '\0';
  float aSpeed = -1;
  float bSpeed = -1;

  String temp = "";
  
  // Go through string
  for (char c : setpointsIn) {
    if (isalpha(c)) {
      // first alpha = A direction
      if (ADir == '\0') {
        ADir = c;
      }
      // second alpha = B direction. Set A speed
      else {
        BDir = c;
        aSpeed = strToFloat(temp);
        temp = "";
        if (aSpeed < 0.0001) {
          aSpeed = 0;
          ADir = STOP;
        }
      }
    }
    // add to temp 
    else {
      temp += c;
    }
  }
  // set B Speed
  bSpeed = strToFloat(temp);
  if (bSpeed < 0.0001) {
    bSpeed = 0;
    BDir = STOP;
  }
// update setpoints
  setNewSetpointMotorA(aSpeed, ADir);
  setNewSetpointMotorB(bSpeed, BDir);
  };


/***********************************************************
 * SETUP & LOOP                                            *
 ***********************************************************/
 
void setup() {
  if (CONNECTED_TO_ROS) {
//    initROSSerial();
  } else {
    initPIDSerial();
  }

    // Initialize pins and value ranges
   initMotors();
   initEncoders();
   initPWM();
  
  if (DEBUG) {
    printForPID("MOTOR A:\n");
     printPIDHeader();
     printPID(KpA, KiA, KdA, setpointA, FEEDFWDA, a_adjust);
     
     printForPID("MOTOR B:\n");
     printPIDHeader();
     printPID(KpB, KiB, KdB, setpointB, FEEDFWDB, b_adjust);
  }
    
}

void loop() {
  nowTime = millis();
  motorA.Compute();
  motorB.Compute();

  moveA(max(0, min(255, (int)outputA + a_adjust)));
  moveB(max(0, min(255, (int)outputB + b_adjust)));


  if (!CONNECTED_TO_ROS) {
    while (Serial.available() > 0) {
      processIncomingByte(Serial.read());
    }
  }
  
  if (DEBUG) {
    if (storeB != outputB){
        storeB = outputB;
        printPIDUpdate(inputA, outputA, a_adjust);
    }
  }

 if (CONNECTED_TO_ROS) {
//   int32_msg_R.data = countR;
//   int32_msg_L.data = countL;
//   pubR.publish(&int32_msg_R);
//   pubL.publish(&int32_msg_L);
//
//   // TODO: check if this will cause issues
//   nh.spinOnce();
   delay(10);
 }

  
}

/***********************************************************
 * ENCODER CALLBACKS                                       *
 ***********************************************************/
void isr_A(){
  // count sufficient interrupts to get accurate timing
  countIntA++;
  if (countIntA == INT_COUNT){
    inputA = (float) ENCODER_CONVERSION * (1.0 / (float)(nowTime - startTimeA));
    startTimeA = nowTime;
    countIntA = 0;
  }
}

void isr_B(){
  // count sufficient interrupts to get accurate timing
  countIntB++;
  if (countIntB == INT_COUNT){
    inputB = (float) ENCODER_CONVERSION * (1.0 / (float)(nowTime - startTimeB));
    startTimeB = nowTime;
    countIntB = 0;
  }
}
