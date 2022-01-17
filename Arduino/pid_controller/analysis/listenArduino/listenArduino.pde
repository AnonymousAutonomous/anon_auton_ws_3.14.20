import processing.serial.*;
Serial mySerial;
PrintWriter output;

int savedTime;
int totalTime = 60000;   // 1 min
int startTime;

boolean sentCommands = false;
   
 float p = 1.0;
 float i = 1.0;
 float d = 1.0;
 float s = 3.0;

void setup() {
   int arduino_index = 0;
   for(int i = 0; i < Serial.list().length; i++)  {
     if (Serial.list()[i].contains("usbmodem")) {
       arduino_index = i;
       break;
     }
   }
   
   printArray(Serial.list());
   print(arduino_index);
   mySerial = new Serial( this, Serial.list()[arduino_index], 115200 );
   mySerial.clear();


   output = createWriter( "data/" + p + "_" + i + "_" + d + "_" + s + ".csv" );
   
   savedTime = millis();
}
void draw() {
    if (mySerial.available() > 0 ) {
        if (!sentCommands) {
           mySerial.write("p " + str(p) + '\n');
           mySerial.write("i " + str(i) + '\n');
           mySerial.write("d " + str(d) + '\n');
           mySerial.write("s " + str(s) + '\n');
           startTime = millis();
           sentCommands = true;
        }
       String value = mySerial.readStringUntil('\n');
       if ( value != null ) {
            output.println( str(millis() - startTime) + "," + value );
       }  
         
         
         
    }
    
    int passedTime = millis() - savedTime;
    if (passedTime > totalTime) {
      print("DONE");
      output.flush();  // Writes the remaining data to the file
      output.close();  // Finishes the file
      exit();  // Stops the program
    }
}

void keyPressed() {
    output.flush();  // Writes the remaining data to the file
    output.close();  // Finishes the file
    exit();  // Stops the program
}
