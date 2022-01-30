/* Robot_SimpleMotor_Drive_V0 with encoders
 * 
 * Adafruit Feather M4 using Pololu TB6612FNG motor controller
 * Drives two motors at fixed speed with PI control
 * 
 * Motor Control Table
 * XIN1   XIN2    Effect
 * Low    Low     Brake
 * Low    High    Forward
 * High   Low     Reverse
 * 
 * Free to use for all
 * F Milburn, January 2020
 */
 #include <PID_v1.h>

// ROS connection to pi
#include <ros.h>
#include <std_msgs/Int32.h>
#include <eyes/Generic.h>

ros::NodeHandle nh;

// init to zero and update with each encoder tick
std_msgs::Int32 int32_msg_R;
std_msgs::Int32 int32_msg_L;
ros::Publisher pubR("encoder_value_R", &int32_msg_R);
ros::Publisher pubL("encoder_value_L", &int32_msg_L);

 
// Output pins used to control motors

// A is left motor
const uint8_t PWMA = 11;         // Motor A PWM control     BLUE 
const uint8_t AIN1 = 9;         // Motor A input 1         YELLOW
const uint8_t AIN2 = 8;         // Motor A input 2         GREEN
const uint8_t STBYA = 4;        // Standby                 

// B is right motor
const uint8_t BIN1 = 12;        // Motor B input 1         ORANGE
const uint8_t BIN2 = 13;        // Motor B input 2         RED
const uint8_t PWMB = 10;        // Motor B PWM control     BROWN
const uint8_t STBYB = 7;        // Standby                 

// Motor encoder external interrupt pins
const uint8_t ENCA = 3;        // Encoder A input         
const uint8_t ENCB = 2;        // Encoder B input    

// PWM
const uint8_t ANALOG_WRITE_BITS = 8;
const uint8_t MAX_PWM = pow(2, ANALOG_WRITE_BITS)-1;
const uint8_t MIN_PWM = 0;    // Let motors stop

// Motor timing
unsigned long nowTime = 0;       // updated on every loop
unsigned long prevTime = 0;       // updated on every loop
unsigned long prevTime2 = 0;     // updated on every loop
unsigned long startTimeA = 0;    // start timing A interrupts
unsigned long startTimeB = 0;    // start timing B interrupts
unsigned long countIntA = 0;     // count the A interrupts
unsigned long countIntB = 0;     // count the B interrupts


// PID 
const unsigned long SAMPLE_TIME = 10;  // time between PID updates
const unsigned long INT_COUNT = 100;     // 100 encoder ticks for accurate timing

// !!!!!!!!!!!!! SETPOINTS MUST BE POSITIVE !!!!!!!!!!!!!!
double setpointA = 0.0;         // setpoint is inches / second
double setpointB = setpointA;   // setpoint is inches / second

double inputA = 0;              // input is inches / second
double outputA = 0;             // output is PWM to motors
uint8_t FEEDFWDA = 60;
uint8_t a_adjust = 0;

double inputB = 0;              // input is inches / second
double outputB = 0;             // output is PWM to motors
uint8_t FEEDFWDB = 60;
uint8_t b_adjust = 0;

//double KpA = 2.0, KiA = 7.0, KdA = 2.0;
// TODO: diff tunings for fwd or bwd? how to even out between the two motors? -- send avg to both? OR diff FEEDFWD terms for the two?
double KpA = 10.0, KiA = 25.0, KdA = 0.0;
double KpB = KpA, KiB = KiA, KdB = KdA;
PID motorA(&inputA, &outputA, setpointA, KpA, KiA, KdA, DIRECT);
PID motorB(&inputB, &outputB, setpointB, KpB, KiB, KdB, DIRECT);
double storeB = 0;               // used for debug print

double encoderConversion = 1300000.0 / 4200.0; // (10 ticks *  1000 milliseconds * 1 rev * 13 in) / (x millisecond * 1 second * 4200 ticks * 1 rev)

const char FWD = 'f';
const char BWD = 'r';
const char STOP = 's';

volatile long countR = 0;
volatile long countL = 0;

float strToFloat(String str) {
  char buffer[10];
  str.toCharArray(buffer, 10);
  return atof(buffer);
}

/***********************************************************
 * MOTOR HELPER FUNCTIONS                                  *
 ***********************************************************/
void setADir(char dir) {
  if (dir == BWD) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  else if (dir == FWD) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else { // STOP / BRAKE
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
}

void setBDir(char dir) {
  if (dir == BWD) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  else if (dir == FWD) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  else { // STOP / BRAKE
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
}

void moveA(uint16_t pwm){
  analogWrite(PWMA, pwm);
}

void moveB(uint16_t pwm){
  analogWrite(PWMB, pwm);
}

void standbyMotors(bool standby){
  if (standby == true){
    digitalWrite(STBYA, LOW);
    digitalWrite(STBYB, LOW);
  }
  else{
    digitalWrite(STBYA, HIGH);
    digitalWrite(STBYB, HIGH);
  }
}

// TODO: was this working? reset I term for every new setpoint? or just for specific ones?
          // Reset I terms
          // motorA.SetOutputLimits(0.0, 1.0);
          // motorB.SetOutputLimits(0.0, 1.0);

          // motorA.SetOutputLimits(-1.0, 0.0);
          // motorB.SetOutputLimits(-1.0, 0.0);

          // motorA.SetOutputLimits(MIN_PWM, MAX_PWM);
          // motorB.SetOutputLimits(MIN_PWM, MAX_PWM);
          // }

bool AreSame(double a, double b)
{
    return fabs(a - b) < 0.001;
}

void setNewSetpointMotorA(float setpoint, char dir) {
  float signed_setpoint = dir == BWD ? -1 * setpoint : setpoint;
  if (setpoint < 0.001) {
    dir = STOP;
  }
  if (dir == STOP) {
    a_adjust = 0;
  }
  else {
    a_adjust = FEEDFWDA;
  }
  if (!AreSame(signed_setpoint, setpointA)) {
    motorA.SetSetpoint(setpoint);
    setADir(dir);
    setpointA = signed_setpoint;
  }
  
}

void setNewSetpointMotorB(float setpoint, char dir) {
  float signed_setpoint = dir == BWD ? -1 * setpoint : setpoint;

  if (setpoint < 0.001) {
    dir = STOP;
  }
  if (dir == STOP) {
    b_adjust = 0;
  }
  else {
    b_adjust = FEEDFWDB;
  }
    if (!AreSame(signed_setpoint, setpointB)) {
      setpointB = signed_setpoint;
      motorB.SetSetpoint(setpoint);
      setBDir(dir);
    }  
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
}

void initMotors(){
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBYA, OUTPUT);
  pinMode(STBYB, OUTPUT);
  // analogWriteResolution(ANALOG_WRITE_BITS);
  standbyMotors(false);
}

void initEncoders(){
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(ENCA == 2 ? 0 : 1, isr_A, CHANGE);
  attachInterrupt(ENCB == 2 ? 0 : 1, isr_B, CHANGE);
}

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
} 


/***********************************************************
 * ROS                                                     *
 ***********************************************************/
void generic_callback(const eyes::Generic& generic_msg) {
  char leftdir = generic_msg.left_forward ? FWD : BWD;
  char rightdir = generic_msg.right_forward ? FWD : BWD;
  String info = "Callback for: " + leftdir + String(generic_msg.left_speed) + rightdir + String(generic_msg.right_speed);
  nh.loginfo(info.c_str());

  if (generic_msg.left_speed < 0.001) {
    leftdir = STOP;
  }
  if (generic_msg.right_speed < 0.001) {
    rightdir = STOP;
  }
  setNewSetpointMotorA(generic_msg.left_speed, leftdir);
  setNewSetpointMotorB(generic_msg.right_speed, rightdir);

  return;
}

/***********************************************************
 * SETUP & LOOP                                            *
 ***********************************************************/
ros::Subscriber<eyes::Generic> generic_sub("generic_feed", &generic_callback);

void setup(){
  // Set up ROS
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.loginfo("Node initialized");
  nh.advertise(pubR);
  nh.advertise(pubL);
  nh.subscribe(generic_sub);

  // Initialize pins and value ranges
 initMotors();
 initEncoders();
 initPWM();

  // Connect to Arduino serial
//  Serial.begin(115200);
//  while(!Serial){
//   // wait for serial to start
//  }
 
//setADir(FWD);
// setBDir(FWD);
//
// for (int i = 100; i >= 0; i-=10) {
//   Serial.print(i); Serial.print("\n");
//   moveA(i);
//   moveB(i);
//   delay(5000);
// }

// printPID();

}

void loop(){
  nowTime = millis();
  motorA.Compute();
  motorB.Compute();

  moveA(max(0, min(255, (int)outputA + a_adjust)));
  moveB(max(0, min(255, (int)outputB + b_adjust)));

  // while (Serial.available() > 0) {
  //   processIncomingByte(Serial.read());
  // }

  // publish every 200 ms
  if (nowTime - prevTime >= 200) {
    int32_msg_R.data = countR;
    int32_msg_L.data = countL;
    pubR.publish(&int32_msg_R);
    pubL.publish(&int32_msg_L);
    prevTime = nowTime;
  }
 
  // // TODO: check if this will cause issues
  if (nowTime - prevTime2 >= 200) {
    nh.spinOnce();
//    nh.loginfo("Arduino has spun");
    prevTime2 = nowTime;
  }

  // Delay for PID
  delay(10);

  // PID is stuck. Tell it it's stuck.
  if (nowTime - startTimeA >= 250) {
    inputA = 0;
    startTimeA = nowTime;
    countIntA = 0;
  }
  if (nowTime - startTimeB >= 250) {
    inputB = 0;
    startTimeB = nowTime;
    countIntB = 0;
  }


  if (storeB != outputB){
    storeB = outputB;
//    Serial.print("OUT: "); Serial.print(outputA); Serial.print(" "); Serial.print(outputB); Serial.print("\n");
//    Serial.print("IN:  "); Serial.print(inputA); Serial.print(" "); Serial.print(inputB); Serial.print("\n");
//
//  }
//    Serial.println("input A, inputB, errorA, errorB");
//    Serial.print(inputA); Serial.print("  ");
//    Serial.print(inputB); Serial.print("  ");
//    Serial.print(100*(setpointA-inputA)/setpointA); Serial.print("  ");
//    Serial.print(100*(setpointB-inputB)/setpointB); Serial.println("");
  }
//   Serial.println("inputA  inputB  outputA  outputB  errorA  errorB");
//   Serial.print(inputA); Serial.print("  ");

}



/***********************************************************
 * ENCODER CALLBACKS                                       *
 ***********************************************************/
void isr_A(){
  // count sufficient interrupts to get accurate timing
  countIntA++;
  if (countIntA == INT_COUNT){
    inputA = (float) encoderConversion * (1.0 / (float)(nowTime - startTimeA));
    startTimeA = nowTime;
    countIntA = 0;
  }
  
  if (digitalRead(ENCA) == HIGH) {
    if (digitalRead(STBYA) == LOW) {
      --countL;
    } else {
      ++countL;
    }
  } else {
    if (digitalRead(STBYA) == LOW) {
      ++countL;
    } else {
      --countL;
    }
  }
}

void isr_B(){
  // count sufficient interrupts to get accurate timing
  countIntB++;
  if (countIntB == INT_COUNT){
    inputB = (float) encoderConversion * (1.0 / (float)(nowTime - startTimeB));
    startTimeB = nowTime;
    countIntB = 0;
  }
  
  if (digitalRead(ENCB) == HIGH) {
    if (digitalRead(STBYB) == LOW) {
      ++countR;
    } else {
      --countR;
    }
  } else {
    if (digitalRead(STBYB) == LOW) {
      --countR;
    } else {
      ++countR;
    }
  }
}
