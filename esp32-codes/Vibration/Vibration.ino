// the setup routine runs once when you press reset:
void setup() {
  pinMode(2, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  digitalWrite(2, HIGH);
  delay(500);
  digitalWrite(2, LOW);
  delay(1000);
 
}
