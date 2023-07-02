#include "motors_and_pins.h"
#include "pid_serial.h"
#include "ros_serial.h"
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <eyes/Generic.h>

void initEncoders()
{
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  // attachInterrupt(ENCA == 2 ? 0 : 1, isr_A, CHANGE); // TODO: fix this for the updated arduino board
  // attachInterrupt(ENCB == 2 ? 0 : 1, isr_B, CHANGE);
};

// void initPWM()
// {
//   startTimeA = millis();
//   startTimeB = millis();
//   motorA.SetOutputLimits(MIN_PWM, MAX_PWM);
//   motorB.SetOutputLimits(MIN_PWM, MAX_PWM);
//   motorA.SetSampleTime(SAMPLE_TIME);
//   motorB.SetSampleTime(SAMPLE_TIME);
//   motorA.SetMode(AUTOMATIC);
//   motorB.SetMode(AUTOMATIC);
// //  setNewSetpointMotorA(setpointA, FWD);
// //  setNewSetpointMotorB(setpointB, FWD);
// };

/***********************************************************
 * SETUP & LOOP                                            *
 ***********************************************************/

void setup()
{
  // Initialize pins and value ranges
  initMotors();
  initEncoders();

  setADir(FWD);
  setBDir(FWD);

  moveA(255);
  moveB(255);
  delay(5000);

  moveA(0);
  moveB(0);
  delay(5000);

  moveA(255);
  moveB(255);
  delay(5000);
}

String info = "";
void loop()
{
  moveA(255);
  moveB(255);
  delay(5000);

  moveA(0);
  moveB(0);
  delay(5000);

  moveA(255);
  moveB(255);
  delay(5000);

  moveA(0);
  moveB(0);
  delay(5000);
}

/***********************************************************
 * ENCODER CALLBACKS                                       *
 ***********************************************************/