#include "motors_and_pins.h"
#include "pid_serial.h"
#include "ros_serial.h"
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
const unsigned long SAMPLE_TIME = 25; // time between PID updates
const unsigned long INT_COUNT = 100;  // 100 encoder ticks for accurate timing

// !!!!!!!!!!!!! SETPOINTS MUST BE POSITIVE !!!!!!!!!!!!!!
double setpointA = 0.0;       // setpoint is inches / second
double setpointB = setpointA; // setpoint is inches / second

double setpointAAsTicksPerSampleTime = 0;
double setpointBAsTicksPerSampleTime = 0;

double inputA = 0;  // input is encoder ticks / SAMPLE_TIME
double outputA = 0; // output is PWM to motors
int FEEDFWDA = 40;
int a_adjust = 0;

double inputB = 0;  // input is encoder ticks / SAMPLE_TIME
double outputB = 0; // output is PWM to motors
int FEEDFWDB = FEEDFWDA;
int b_adjust = 0;

double KpA = 10.0, KiA = 25.0, KdA = 0.0;
double KpB = KpA, KiB = KiA, KdB = KdA;
PID motorA(&inputA, &outputA, setpointAAsTicksPerSampleTime, KpA, KiA, KdA, DIRECT);
PID motorB(&inputB, &outputB, setpointBAsTicksPerSampleTime, KpB, KiB, KdB, DIRECT);
double storeB = 0; // used for debug print

unsigned long prevTime = 0; // updated on every loop

volatile long vcountR = 0;
volatile long vcountL = 0;
volatile int vcountInterrA = 0; // count the A interrupts
volatile int vcountInterrB = 0; // count the B interrupts

long countL = 0;
long prevCountL = 0;
long diffL = 0;

long countR = 0;
long prevCountR = 0;
long diffR = 0;

boolean CONNECTED_TO_ROS = false;

ros::NodeHandle nh;

// init to zero and update with each encoder tick
std_msgs::Int32 int32_msg_R;
std_msgs::Int32 int32_msg_L;
// ros::Publisher pubR("encoder_value_R", &int32_msg_R);
// ros::Publisher pubL("encoder_value_L", &int32_msg_L);

void generic_callback(const eyes::Generic &generic_msg)
{
  char left_dir = generic_msg.left_forward ? FWD : BWD;
  char right_dir = generic_msg.right_forward ? FWD : BWD;

  if (generic_msg.left_speed < 0.001)
  {
    left_dir = STOP;
  }
  if (generic_msg.right_speed < 0.001)
  {
    right_dir = STOP;
  }
  setNewSetpointMotorA(generic_msg.left_speed, left_dir);
  setNewSetpointMotorB(generic_msg.right_speed, right_dir);
  return;
};

ros::Subscriber<eyes::Generic> generic_sub("generic_feed", &generic_callback);

void initROSSerial()
{
  nh.initNode();
  nh.subscribe(generic_sub);
};

void initEncoders()
{
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  attachInterrupt(ENCA == 2 ? 0 : 1, isr_A, CHANGE); // TODO: fix this for the updated arduino board
  attachInterrupt(ENCB == 2 ? 0 : 1, isr_B, CHANGE);
};

void initPWM()
{
  startTimeA, startTimeB = millis();
  motorA.SetOutputLimits(MIN_PWM, MAX_PWM);
  motorB.SetOutputLimits(MIN_PWM, MAX_PWM);
  motorA.SetSampleTime(SAMPLE_TIME);
  motorB.SetSampleTime(SAMPLE_TIME);
  motorA.SetMode(AUTOMATIC);
  motorB.SetMode(AUTOMATIC);
  setNewSetpointMotorA(setpointA, FWD);
  setNewSetpointMotorB(setpointB, FWD);
};

bool AreSame(double a, double b)
{
  return fabs(a - b) < 0.001;
}

void setNewSetpointMotorA(float setpoint, char dir)
{
  float signed_setpoint = dir == BWD ? -1 * setpoint : setpoint;

  if (setpoint < 0.001 || dir == STOP)
  {
    a_adjust = 0;
  }
  else
  {
    a_adjust = FEEDFWDA;
  }
  if (!AreSame(signed_setpoint, setpointA))
  {
    setpointA = signed_setpoint;
    setpointAAsTicksPerSampleTime = (setpoint * 4200.0 * SAMPLE_TIME) / (1000 * 13.25);
    motorA.SetSetpoint(setpointAAsTicksPerSampleTime);
    setADir(dir);
  }
}

void setNewSetpointMotorB(float setpoint, char dir)
{
  float signed_setpoint = dir == BWD ? -1 * setpoint : setpoint;

  if (setpoint < 0.001 || dir == STOP)
  {
    b_adjust = 0;
  }
  else
  {
    b_adjust = FEEDFWDB;
  }
  if (!AreSame(signed_setpoint, setpointB))
  {
    setpointB = signed_setpoint;
    setpointBAsTicksPerSampleTime = (setpoint * 4200.0 * SAMPLE_TIME) / (1000 * 13.25);
    motorB.SetSetpoint(setpointBAsTicksPerSampleTime);
    setBDir(dir);
  }
}

void processIncomingByte(const byte inByte)
{
  static char input_line[MAX_INPUT];
  static unsigned int input_pos = 0;

  switch (inByte)
  {
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
      input_line[input_pos++] = inByte;
    break;
  }
}

void processData(const char *data)
{
  char changeVar = data[0];
  if (changeVar == 'p')
  {
    KpA = KpB = strToFloat(String(data).substring(1));
  }
  else if (changeVar == 'i')
  {
    KiA = KiB = strToFloat(String(data).substring(1));
  }
  else if (changeVar == 'd')
  {
    KdA = KdB = strToFloat(String(data).substring(1));
  }
  else if (changeVar == 'b')
  {
    standbyMotors(true);
  }
  else if (changeVar == 's')
  {
    parseNewSetpoints(String(data).substring(1));
  }
  motorA.SetTunings(KpA, KiA, KdA);
  motorB.SetTunings(KpB, KiB, KdB);

  // printPID(KpA, KiA, KdA, setpointA, FEEDFWDA, a_adjust);
}

void parseNewSetpoints(String setpointsIn)
{
  char ADir = '\0';
  char BDir = '\0';
  float aSpeed = -1;
  float bSpeed = -1;

  String temp = "";

  // Go through string
  for (char c : setpointsIn)
  {
    if (isalpha(c))
    {
      // first alpha = A direction
      if (ADir == '\0')
      {
        ADir = c;
      }
      // second alpha = B direction. Set A speed
      else
      {
        BDir = c;
        aSpeed = strToFloat(temp);
        temp = "";
        if (aSpeed < 0.0001)
        {
          aSpeed = 0;
          ADir = STOP;
        }
      }
    }
    // add to temp
    else
    {
      temp += c;
    }
  }
  // set B Speed
  bSpeed = strToFloat(temp);
  if (bSpeed < 0.0001)
  {
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

void setup()
{
  if (CONNECTED_TO_ROS)
  {
    Serial.begin(57600);
    nh.getHardware()->setBaud(57600);
    initROSSerial();
  }
  else
  {
    initPIDSerial();
    Serial.print("diffL");
    Serial.print("\t");
    Serial.print("setpointAAsTicksPerSampleTime");
    Serial.print("\t");
    Serial.print("inputA");
    Serial.print("\t");
    Serial.print("outputA");
    Serial.print("\t");
    Serial.print("nowTime - prevTime");
    Serial.print("\n");

  }

  // Initialize pins and value ranges
  initMotors();
  initEncoders();
  initPWM();
}

String info = "";
void loop()
{
  nowTime = millis();
  // TODO: handle checking for very long time gap and ignore;
  if (nowTime - prevTime >= SAMPLE_TIME)
  {
//    prevCountL = countL;
//    prevCountR = countR;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      // Reset each loop
      countL = vcountL;
      countR = vcountR;
      vcountL = 0;
      vcountR = 0;
    }

    diffL = setpointA >= 0 ? countL : 0 - countL;
    diffR = setpointB >= 0 ? countR : 0 - countR;

    inputA = max(0, diffL);
    inputB = max(0, diffR);

    if (!CONNECTED_TO_ROS) {
      Serial.print(diffL);
    Serial.print("\t");
    Serial.print(setpointAAsTicksPerSampleTime);
    Serial.print("\t");
    Serial.print(inputA);
    Serial.print("\t");
    Serial.print(outputA);
    Serial.print("\t");
    Serial.print(nowTime - prevTime);
    Serial.print("\n");
    } else {
      info = String(int(nowTime - prevTime)) + "\t" + String(int(setpointAAsTicksPerSampleTime)) + "\t" + String(int(diffL));
      nh.loginfo(info.c_str());
    }
    

    prevTime = nowTime;
  }
  // Copy volatile variables so they don't change while we compute

  motorA.Compute();
  motorB.Compute();

  moveA(max(0, min(255, (int)outputA + a_adjust)));
  moveB(max(0, min(255, (int)outputB + b_adjust)));

  if (!CONNECTED_TO_ROS)
  {
    while (Serial.available() > 0)
    {
      processIncomingByte(Serial.read());
    }
  }

  if (CONNECTED_TO_ROS)
  {
    nh.spinOnce();
  }
}

/***********************************************************
 * ENCODER CALLBACKS                                       *
 ***********************************************************/
void isr_A()
{
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
  if (digitalRead(ENCB) != digitalRead(STBYB))
  {
    vcountR++;
  }
  else
  {
    vcountR--;
  }
}
