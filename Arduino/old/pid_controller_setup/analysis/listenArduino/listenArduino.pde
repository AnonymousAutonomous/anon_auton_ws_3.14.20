import processing.serial.*;
Serial mySerial;
PrintWriter output;

int savedTime;
int totalTime = 30000;   // 30 seconds
int startTime;

boolean sentCommands = false;
   
 float p = 10.0;
 float i = 25.0;
 float d = 0.0;
 String s = "f3.0f3.0";

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
   mySerial = new Serial( this, Serial.list()[arduino_index], 57600 );
   mySerial.clear();


   output = createWriter( "data/csvs/" + p + "_" + i + "_" + d + "_" + s + ".csv" );
   
   savedTime = millis();
}
void draw() {
    if (mySerial.available() > 0 ) {
        if (!sentCommands) {
           mySerial.write("p " + str(p) + '\n');
           mySerial.write("i " + str(i) + '\n');
           mySerial.write("d " + str(d) + '\n');
           mySerial.write("s " + s + '\n');
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
