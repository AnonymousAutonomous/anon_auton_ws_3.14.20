int x, y, z;
int Xrest, Yrest, Zrest;
int dull;

int vx, vy, vz;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  delay(1000);
  Xrest = (analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0)) / 10;
  Yrest = (analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1)) / 10;
  Zrest = (analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2)) / 10;

  dull = 10;

  vx = 0;
  vy = 0;
  vz = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  x = ((analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0) + analogRead(0)) / 10 - Xrest) / dull;
  y = ((analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1) + analogRead(1)) / 10 - Yrest) / dull;
  z = ((analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2) + analogRead(2)) / 10 - Zrest) / dull;

  /*
  Serial.print("accelerations are x, y, z: ");
  Serial.print(x, DEC);
  Serial.print(" ");
  Serial.print(y, DEC);
  Serial.print(" ");
  Serial.print(z, DEC);
  Serial.println(" ");
  */

  vx += x;
  vy += y;
  vz += z;

  
  Serial.print("velocities are x, y, z: ");
  Serial.print(vx, DEC);
  Serial.print(" ");
  Serial.print(vy, DEC);
  Serial.print(" ");
  Serial.print(vz, DEC);
  Serial.println(" ");
  

  delay(100);
}
