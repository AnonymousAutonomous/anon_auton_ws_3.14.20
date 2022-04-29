////////////// LEFT MOTOR //////////////
// from motor
const uint8_t STBYA = 4;  // Motor A standby  // white
const uint8_t ENCA = 3;   // Motor A encoder  // yellow // MUST BE 2 OR 3
                                              // blue --> 5V
                                              // green --> ground

// from motor controller
const uint8_t PWMA = 11;  // Motor A PWM control
const uint8_t AIN1 = 9;  // Motor A PWM control
const uint8_t AIN2 = 8;  // Motor A PWM control

//////////////////////////////////////////


////////////// RIGHT MOTOR //////////////
// from motor
const uint8_t STBYB = 5;  // Motor B standby  // white
const uint8_t ENCB = 2;   // Motor B encoder  // yellow // MUST BE 2 OR 3
                                              // blue --> 5V
                                              // green --> ground

// from motor controller
const uint8_t PWMB = 10;  // Motor B PWM control
const uint8_t BIN1 = 12;  // Motor B PWM control
const uint8_t BIN2 = 7;  // Motor B PWM control


//////////////////////////////////////////

const char BWD = 'r';
const char FWD = 'f';
const char STOP = 's';


// encoders
long countIntA = 0;
long countIntB = 0;

// PWM
const uint8_t ANALOG_WRITE_BITS = 8;
const uint8_t MAX_PWM = pow(2, ANALOG_WRITE_BITS) - 1;
const uint8_t MIN_PWM = 0;

// speeding up
int i = 0;
long prevTime;
long nowTime;


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


//////////////////////////////////////////


void initMotors() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(STBYA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBYB, OUTPUT);
  
  standbyMotors(false);
}

void initEncoders() {
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), isr_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB), isr_B, CHANGE);
}


void setup() {
  initMotors();
  initEncoders();
  Serial.begin(115200);
  while(!Serial){
   // wait for serial to start
  }
  prevTime = millis();
  setADir(FWD);
  setBDir(FWD);

//  int k = 0;
//
//  for (int j = 0; j < 100; j++) {
//    moveA(max(0, min(255, k)));
//    moveB(max(0, min(255, k)));
//    delay(1000);
//  }
  

  

}

void loop() {
  nowTime = millis();

//  Serial.print(i);
//  Serial.print(' ');
//  Serial.print(countIntA);
//  Serial.print(' ');
//  Serial.print(countIntB);
//  
  // put your main code here, to run repeatedly:

  moveA(max(0, min(255, i)));
  moveB(max(0, min(255, i)));

  if (nowTime - prevTime >= 5000) {
    Serial.print(i);
    Serial.print(',');
    Serial.print(countIntA);
    Serial.print(',');
    Serial.print(nowTime - prevTime);
    Serial.print('\n');
    prevTime = nowTime;
    i += 5;
    countIntA = 0;
    
  }
  
}

void isr_A() {
  countIntA++;
}

void isr_B() {
  countIntB++;
}
