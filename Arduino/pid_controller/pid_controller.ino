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
#include <std_msgs/String.h>
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
const uint16_t PWMA = 10;         // Motor A PWM control     Orange // was 11
const uint16_t AIN2 = 13;         // Motor A input 2         Brown   // was 8
const uint16_t AIN1 = 12;         // Motor A input 1         Green   // was 9
const uint16_t STBYA = 7;        // Standby                 Brown   // was 4

// B is right motor
const uint16_t BIN1 = 9;        // Motor B input 1         Yellow  // was 12
const uint16_t BIN2 = 8;        // Motor B input 2         Purple  // was 13
const uint16_t PWMB = 11;        // Motor B PWM control     White   // was 10
const uint16_t STBYB = 4;        // Standby                 Brown   // was 7

// Motor encoder external interrupt pins
const uint16_t ENCA = 2;        // Encoder A input         Yellow   // was 3
const uint16_t ENCB = 3;        // Encoder B input         Green    // was 2

// PWM
const uint16_t ANALOG_WRITE_BITS = 8;
const uint16_t MAX_PWM = pow(2, ANALOG_WRITE_BITS)-1;
const uint16_t MIN_PWM = 0;    // Let motors stop

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
double setpointA = 0.0;         // setpoint is inches / second
double setpointB = setpointA;   // setpoint is inches / second

double inputA = 0;              // input is inches / second
double outputA = 0;             // output is PWM to motors
int FEEDFWDA = 60;
int a_adjust = 0;

double inputB = 0;              // input is inches / second
double outputB = 0;             // output is PWM to motors
int FEEDFWDB = 60;
int b_adjust = 0;

//double KpA = 2.0, KiA = 7.0, KdA = 2.0;
// TODO: diff tunings for fwd or bwd? how to even out between the two motors? -- send avg to both? OR diff FEEDFWD terms for the two?
double KpA = 10.0, KiA = 25.0, KdA = 0.0;
double KpB = KpA, KiB = KiA, KdB = KdA;
PID motorA(&inputA, &outputA, setpointA, KpA, KiA, KdA, DIRECT);
PID motorB(&inputB, &outputB, setpointB, KpB, KiB, KdB, DIRECT);
double storeB = 0;               // used for debug print

double encoderConversion = 1300000.0 / 4200.0; // (10 ticks *  1000 milliseconds * 1 rev * 13 in) / (x millisecond * 1 second * 4200 ticks * 1 rev)

const unsigned int MAX_INPUT = 50;

const char FWD = 'f';
const char BWD = 'r';
const char STOP = 's';

volatile long countR = 0;
volatile long countL = 0;

/***********************************************************
 * SERIAL / PRINTING HELPER FUNCTIONS                      *
 ***********************************************************/
 void printPID() {
//   Serial.print("KpA"); Serial.print(","); Serial.print("KiA"); Serial.print(","); Serial.print("KdA"); Serial.print(","); Serial.print("setpointA"); Serial.print(","); Serial.print("FEEDFWDA"); Serial.print(",");
//   Serial.print("KpB"); Serial.print(","); Serial.print("KiB"); Serial.print(","); Serial.print("KdB"); Serial.print(","); Serial.print("setpointB"); Serial.print(","); Serial.print("FEEDFWDB"); Serial.print("\n");
//   Serial.print(KpA); Serial.print(","); Serial.print(KiA); Serial.print(","); Serial.print(KdA); Serial.print(","); Serial.print(setpointA); Serial.print(","); Serial.print(FEEDFWDA); Serial.print(",");
//   Serial.print(KpB); Serial.print(","); Serial.print(KiB); Serial.print(","); Serial.print(KdB); Serial.print(","); Serial.print(setpointB); Serial.print(","); Serial.print(FEEDFWDB); Serial.print("\n");
 
//  Serial.print("inputA"); Serial.print(","); Serial.print("outputA"); Serial.print(","); Serial.print("a_adjust"); Serial.print(","); 
//   Serial.print("inputB"); Serial.print(","); Serial.print("outputB"); Serial.print(","); Serial.print("b_adjust"); Serial.print("\n"); 
 }

 void printUpdates() {
 Serial.print(inputA); Serial.print(","); Serial.print(outputA); Serial.print(","); Serial.print(a_adjust); Serial.print(","); 
  Serial.print(inputB); Serial.print(","); Serial.print(outputB); Serial.print(","); Serial.print(b_adjust); Serial.print("\n"); 
 
 }
 
 void process_data (const char* data) {
  char changeVar = data[0];
  if (changeVar == 'p') {
    KpA = KpB = String(data).substring(1).toFloat();
  }
  else if (changeVar == 'i') {
    KiA = KiB = String(data).substring(1).toFloat();
  }
  else if (changeVar == 'd') {
    KdA = KdB = String(data).substring(1).toFloat();
  }
  else if (changeVar == 'b') {
        standbyMotors(true);
  }
  else if (changeVar == 's') {
    parseNewSetpoints(String(data).substring(1));
  }
    motorA.SetTunings(KpA, KiA, KdA);
    motorB.SetTunings(KpB, KiB, KdB);

    printPID();

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

/***********************************************************
 * MOTOR HELPER FUNCTIONS                                  *
 ***********************************************************/
void setADir(char dir) {
  if (dir == BWD) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else if (dir == FWD) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  else { // STOP / BRAKE
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
}

void setBDir(char dir) {
  if (dir == BWD) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  else if (dir == FWD) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
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
    return fabs(a - b) < EPSILON;
}

void setNewSetpointMotorA(float setpoint, char dir) {
  if (dir == STOP) {
    a_adjust = 0;
  }
  else {
    a_adjust = FEEDFWDA;
  }
  if (!AreSame(setpoint, setpointA)) {
    setpointA = setpoint;
    motorA.SetSetpoint(setpoint);
    setADir(dir);
  }
  
}

void setNewSetpointMotorB(float setpoint, char dir) {
  if 
  if (dir == STOP) {
    b_adjust = 0;
  }
  else {
    b_adjust = FEEDFWDB;
  }
    if (!AreSame(setpoint, setpointB)) {

      setpointB = setpoint;
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
        aSpeed = temp.toFloat();
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
  bSpeed = temp.toFloat();
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
  attachInterrupt(digitalPinToInterrupt(ENCA), isr_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB), isr_B, CHANGE);
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
  nh.initNode();
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

  // printUpdates();
 
  int32_msg_R.data = countR;
  int32_msg_L.data = countL;
  pubR.publish(&int32_msg_R);
  pubL.publish(&int32_msg_L);

  // // TODO: check if this will cause issues
  nh.spinOnce();
  delay(1);


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
