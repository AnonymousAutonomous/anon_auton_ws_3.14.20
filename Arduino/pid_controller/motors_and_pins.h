#ifndef Pins_h
#define Pins_h

/***********************************************************
 * CONSTANTS                                               *
 ***********************************************************/
#define FWD 'f'
#define BWD 'r'
#define STOP 's'

#define REVERSE_A true
#define REVERSE_B true

//define motor 1 related pins (right)
#define AIN1_tmp 8
#define AIN2_tmp 9
#define PWMA 11
#define RIGHT_MOTOR PWMA

const int AIN1 = REVERSE_A ? AIN2_tmp : AIN1_tmp;
const int AIN2 = REVERSE_A ? AIN1_tmp : AIN2_tmp;


#define ENCA 3 // yellow, INTERRUPT
#define STBYA 4 // white

//define motor 2 related pins (left)
#define BIN1_tmp 7
#define BIN2_tmp 12
#define PWMB 10
#define LEFT_MOTOR PWMB

const int BIN1 = REVERSE_B ? BIN2_tmp : BIN1_tmp;
const int BIN2 = REVERSE_B ? BIN1_tmp : BIN2_tmp;

                          
#define ENCB 2 // yellow, INTERRUPT
#define STBYB 5 // white

#define ENCODER_CONVERSION 309.523809524

// PWM
#define ANALOG_WRITE_BITS 8
#define MAX_PWM pow(2, ANALOG_WRITE_BITS)-1
#define MIN_PWM 0    // Let motors stop

#define readA bitRead(PIND,ENCA)
#define readB bitRead(PIND,ENCB)


/***********************************************************
 * HELPER FUNCTIONS                                        *
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
};

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
};

void moveA(uint16_t pwm){
  analogWrite(PWMA, pwm);
};

void moveB(uint16_t pwm){
  analogWrite(PWMB, pwm);
};

void standbyMotors(bool standby){
  if (standby == true){
    digitalWrite(STBYA, LOW);
    digitalWrite(STBYB, LOW);
  }
  else{
    digitalWrite(STBYA, HIGH);
    digitalWrite(STBYB, HIGH);
  }
};


void initMotors(){
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBYA, OUTPUT);
  pinMode(STBYB, OUTPUT);
//   analogWriteResolution(ANALOG_WRITE_BITS);
  standbyMotors(false);
};



#endif
