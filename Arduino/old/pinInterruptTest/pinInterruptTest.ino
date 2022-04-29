void pciSetup(byte pin) {
  *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));
  PCIFR |= bit(digitalPinToPCICRbit(pin));
  PCICR |= bit(digitalPinToPCICRbit(pin));
}

ISR(PCINT0_vect) {
  digitalWrite(LED_BUILTIN, HIGH - digitalRead(LED_BUILTIN));
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(8, HIGH);
  pciSetup(8);
}

void loop() {
  // put your main code here, to run repeatedly:

}
