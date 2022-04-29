#ifndef PID_Serial_h
#define PID_Serial_h

#include <Arduino.h>

//Constants used in some of the functions below
#define MAX_INPUT 50
#define BAUD_RATE 57600

// helper **************************************************************************

float strToFloat(String str) {
  char buffer[10];
   str.toCharArray(buffer, 10);
   return atof(buffer);
};

// init **************************************************************************

void initPIDSerial() {
   // Connect to Arduino serial
   Serial.begin(BAUD_RATE);
   while(!Serial){
    // wait for serial to start
  }
};

// output **************************************************************************

void printForPID(String str) {
  Serial.print(str);
};

void printPIDHeader() {
    Serial.print("Kp"); Serial.print(","); Serial.print("Ki"); Serial.print(","); Serial.print("Kd"); Serial.print(","); Serial.print("setpoint"); Serial.print(","); Serial.print("feedfwd"); Serial.print(","); Serial.print("adjust"); Serial.print("\n");
};

void printPID(double Kp, double Ki, double Kd, double setpoint, int feedfwd, int adjust) {
    Serial.print(Kp); Serial.print(","); Serial.print(Ki); Serial.print(","); Serial.print(Kd); Serial.print(","); Serial.print(setpoint); Serial.print(","); Serial.print(feedfwd); Serial.print(","); Serial.print(adjust); Serial.print("\n");
};


void printPIDUpdateHeader() {
   Serial.print("input"); Serial.print(","); Serial.print("output"); Serial.print(","); Serial.print("adjust"); Serial.print("\n"); 
};

void printPIDUpdate(double input, double output, int adjust) {
     Serial.print(input); Serial.print(","); Serial.print(output); Serial.print(","); Serial.print(adjust); Serial.print("\n"); 
};

// input **************************************************************************




                    
#endif
