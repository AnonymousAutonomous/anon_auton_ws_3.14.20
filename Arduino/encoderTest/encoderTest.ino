                           //encoder motor
const int dirPin1 = 5;     //purple
const int speedPin1 = 6;   //grey

const int motorA = 3;
const int motorB = 4;

volatile long count = 0;

void setup() {
  pinMode(motorA, INPUT);
  pinMode(motorB, INPUT);

  attachInterrupt(digitalPinToInterrupt(3), EncoderEvent, CHANGE);
  
  pinMode(dirPin1, OUTPUT);
  pinMode(speedPin1, OUTPUT);
}

void loop() {
  digitalWrite(dirPin1, HIGH);
  analogWrite(speedPin1, 0);
}

void EncoderEvent() {
   if (digitalRead(motorA) == HIGH) {
     if (digitalRead(motorB) == LOW) {
       ++count; 
     } else {
       --count;
     }
   } else {
     if (digitalRead(motorB) == LOW) {
       --count;
     } else {
       ++count;
     }
   }
 }
