#include "motors_and_pins.h"
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <eyes/Generic.h>
#include <util/atomic.h>

// Motor timing
unsigned long nowTime = 0;    // updated on every loop
unsigned long startTimeA = 0; // start timing A interrupts
unsigned long startTimeB = 0; // start timing B interrupts
double periodA = 0;           // motor A period
double periodB = 0;           // motor B period

// PID
const unsigned long SAMPLE_TIME = 100; // time between PID updates, in ms
const unsigned long INT_COUNT = 100;  // 100 encoder ticks for accurate timing

// !!!!!!!!!!!!! SETPOINTS MUST BE POSITIVE !!!!!!!!!!!!!!
double setpointA = 3.0;       // setpoint is inches / second
double setpointB = setpointA; // setpoint is inches / second

double inputA = 0;  // input is inches / second
double outputA = 0; // output is PWM to motors
int FEEDFWDA = 60;
int a_adjust = 0;

double inputB = 0;  // input is inches / second
double outputB = 0; // output is PWM to motors
int FEEDFWDB = FEEDFWDA;
int b_adjust = 0;

double KpA = .7, KiA = 0, KdA = 0.0;
double KpB = KpA, KiB = KiA, KdB = KdA;
PID motorA(&inputA, &outputA, setpointA, KpA, KiA, KdA, DIRECT);
PID motorB(&inputB, &outputB, setpointB, KpB, KiB, KdB, DIRECT);
double storeB = 0; // used for debug print

unsigned long prevTime = 0; // updated on every loop

volatile long vcountR = 0;
volatile long vcountL = 0;
volatile int vcountInterrA = 0; // count the A interrupts
volatile int vcountInterrB = 0; // count the B interrupts

long countR = 0;
long countL = 0;
long prevCountL = 0;
int countInterrA = 0; // count the A interrupts
int countInterrB = 0; // count the B interrupts

boolean CONNECTED_TO_ROS = true;

ros::NodeHandle nh;

// init to zero and update with each encoder tick
std_msgs::Int32 int32_msg_R;
std_msgs::Int32 int32_msg_L;
// ros::Publisher pubR("encoder_value_R", &int32_msg_R);
// ros::Publisher pubL("encoder_value_L", &int32_msg_L);

void initEncoders()
{
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  attachInterrupt(ENCA == 2 ? 0 : 1, isr_A, CHANGE); // TODO: fix this for the updated arduino board
  attachInterrupt(ENCB == 2 ? 0 : 1, isr_B, CHANGE);
};

void initPWM()
{
  startTimeA = millis();
  startTimeB = millis();
  motorA.SetOutputLimits(0, MAX_PWM);
  motorA.SetSampleTime(SAMPLE_TIME);
  motorA.SetMode(AUTOMATIC);
  motorA.SetSetpoint(100);
  setADir(FWD);
  setBDir(FWD);
};

void setup()
{
  Serial.begin(57600);

  // Initialize pins and value ranges
  initMotors();
  initEncoders();
  initPWM();
  // standbyMotors(true); // This could cause real issues -- then encoder just goes between 0 and -1
  setADir(FWD);
  moveA(0);
}

String info = "";
void loop()
{
  nowTime = millis();
  if (nowTime - prevTime >= SAMPLE_TIME)
  {

    prevCountL = countL;
    // Copy volatile variables so they don't change while we compute
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      // countInterrA = vcountInterrA;
      // countInterrB = vcountInterrB;
      countL = vcountL;
      countR = vcountR;

      // if (countInterrA >= INT_COUNT)
      // {
      //   vcountInterrA = 0;
      // }
      // if (countInterrB >= INT_COUNT)
      // {
      //   vcountInterrB = 0;
      // }
    }
    inputA = countL - prevCountL;

    Serial.print(countL - prevCountL);
    Serial.print("\t");
    Serial.print(outputA);
    Serial.print("\n");
    prevTime = nowTime;

  }
  if (outputA < 0) {
    setADir(BWD);
  } else {
    setADir(FWD);
  }
  moveA(outputA);

  motorA.Compute();
  moveA(outputA);
  // Serial.print(",");
  // Serial.print(countR);
}

/***********************************************************
 * ENCODER CALLBACKS                                       *
 ***********************************************************/
void isr_A()
{
  // count sufficient interrupts to get accurate timing
  // vcountInterrA++;

  if (digitalRead(ENCA) == digitalRead(STBYA))
  {
    vcountL++;
  }
  else
  {
    vcountL--;
  }
}

void isr_B()
{
  // count sufficient interrupts to get accurate timing
  // vcountInterrB++;

  if (digitalRead(ENCB) != digitalRead(STBYB))
  {
    vcountR++;
  }
  else
  {
    vcountR--;
  }
}
