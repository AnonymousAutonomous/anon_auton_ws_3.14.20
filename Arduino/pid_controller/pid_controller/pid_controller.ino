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
const uint16_t MIN_PWM = MAX_PWM / 4;    // Make sure motor turns
// Motor timing
unsigned long nowTime = 0;       // updated on every loop
unsigned long startTimeA = 0;    // start timing A interrupts
unsigned long startTimeB = 0;    // start timing B interrupts
unsigned long countIntA = 0;     // count the A interrupts
unsigned long countIntB = 0;     // count the B interrupts
double periodA = 0;              // motor A period
double periodB = 0;              // motor B period
// PID 
const unsigned long SAMPLE_TIME = 100;  // time between PID updates
const unsigned long INT_COUNT = 20;     // sufficient interrupts for accurate timing
double setpointA = 50;         // setpoint is rotational speed in Hz
double inputA = 0;              // input is PWM to motors
double outputA = 0;             // output is rotational speed in Hz
double setpointB = 50;         // setpoint is rotational speed in Hz
double inputB = 0;              // input is PWM to motors
double outputB = 0;             // output is rotational speed in Hz
double KpA = 0.50, KiA = 0.00, KdA = 0.0;
double KpB = 0.50, KiB = 0.00, KdB = 0.0;
PID motorA(&inputA, &outputA, &setpointA, KpA, KiA, KdA, DIRECT);
PID motorB(&inputB, &outputB, &setpointB, KpB, KiB, KdB, DIRECT);
double storeB = 0;               // used for debug print
void setup(){
 initMotors();
 initEncoders();
 initPWM();
 Serial.begin(115200);
 while(!Serial){
  // wait for serial to start
 }

// for (int i = 0; i < 40000; i++) {
//  forwardA(150);
// }

// for (int i = 0; i < 40000; i++) {
//  reverseB(150);
// }
// 
 Serial.print(KpA); Serial.print("  "); Serial.print(KiA); Serial.print("  "); Serial.print(KdA); Serial.print("  ");
 Serial.print(KpB); Serial.print("  "); Serial.print(KiB); Serial.print("  "); Serial.print(KdB); Serial.println("  ");
 Serial.println("inputA  inputB  outputA  outputB  errorA  errorB");

}
void loop(){
  nowTime = millis();
  motorA.Compute();
  motorB.Compute();
  forwardA((int)outputA);
  forwardB((int)outputB);

//  Serial.print(countIntA); Serial.print("  ");
//    Serial.print(countIntB); Serial.println("  ");

//  Serial.print(storeB); Serial.print("  "); Serial.print(outputB); Serial.print("  ");
  if (storeB != outputB){
    storeB = outputB;
    Serial.print(outputA); Serial.print(" "); Serial.print(outputB); Serial.print("\n");
    
  }
//    Serial.print(inputA); Serial.print("  ");
//    Serial.print(inputB); Serial.print("  ");
//    Serial.print(outputA); Serial.print("  ");
//    Serial.print(outputB); Serial.print("  ");
//    Serial.print(100*(setpointA-inputA)/setpointA); Serial.print("  ");
//    Serial.print(100*(setpointB-inputB)/setpointB); Serial.println("");
//  }
}
void forwardA(uint16_t pwm){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, pwm);
}
void forwardB(uint16_t pwm){
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, pwm);
}
void reverseA(uint16_t pwm){
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, pwm);
}
void reverseB(uint16_t pwm){
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);  
  analogWrite(PWMB, pwm);
}
void brakeA(){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}
void brakeB(){
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
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
void initMotors(){
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBYA, OUTPUT);
  pinMode(STBYB, OUTPUT);
//  analogWriteResolution(ANALOG_WRITE_BITS);
  standbyMotors(false);
}
void initEncoders(){
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), isr_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB), isr_B, RISING);
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
}
void isr_A(){
//  Serial.print("A"); Serial.print(" "); Serial.println(countIntA);
//  Serial.println("A ENCODER");
  // count sufficient interrupts to get accurate timing
  // inputX is the encoder frequency in Hz
  countIntA++;
  if (countIntA == INT_COUNT){
    inputA = (float) INT_COUNT * 1000 / (float)(nowTime - startTimeA);
    startTimeA = nowTime;
    countIntA = 0;
  }
}
void isr_B(){
//    Serial.print("B"); Serial.print(" "); Serial.println(countIntB);
  // count sufficient interrupts to get accurate timing
  // inputX is the encoder frequency in Hz
  countIntB++;
  if (countIntB == INT_COUNT){
    inputB = (float) INT_COUNT * 1000 / (float)(nowTime - startTimeB);
    startTimeB = nowTime;
    countIntB = 0;
  }
}
