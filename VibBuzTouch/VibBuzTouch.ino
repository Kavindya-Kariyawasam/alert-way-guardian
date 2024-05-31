// Merged code combining Samuel's and MohammedDamirchi's code (modified for ESP32)

#define buzzerPin 13   // Define buzzerPin for ESP32
#define ctsPin 2       // Define capacitive touch sensor pin for ESP32
#define ledPin 12      // Define LED pin for ESP32
#define vibrationPin 5 // Define pin for vibration motor for ESP32

void setup() {
  pinMode(buzzerPin, OUTPUT);   // Set buzzerPin as output
  pinMode(ledPin, OUTPUT);      // Set LED pin as output
  pinMode(ctsPin, INPUT);       // Set capacitive touch sensor pin as input
  pinMode(vibrationPin, OUTPUT);// Set vibration motor pin as output
  Serial.begin(115200);           // Initialize serial communication
}

void loop() {
  int ctsValue = digitalRead(ctsPin); // Read capacitive touch sensor value
  
  if (ctsValue == HIGH) {
    digitalWrite(ledPin, HIGH); // Turn on LED
    Serial.println("TOUCHED");
    tone(buzzerPin,1535,500); // Beep when touched
    vibrate(500); // Vibrate when touched
  } else {
    digitalWrite(ledPin, LOW); // Turn off LED
    Serial.println("Not touched");
  }
  
  delay(500);
}

// void beep(unsigned char delayms) { // Function to produce beep
//   ledcAttachPin(buzzerPin, 0);     // Attach buzzerPin to channel 0 of LEDC
//   ledcSetup(0, 2000, 8);            // Setup PWM with frequency of 2000Hz and 8-bit resolution
//   ledcWriteTone(0, 2000);           // Set tone frequency to 2000Hz
//   delay(delayms);                   // Delay for specified time
//   ledcWriteTone(0, 0);              // Turn off buzzer
//   delay(delayms);                   // Delay for specified time
// }

void vibrate(unsigned long delayms) { // Function to vibrate
  digitalWrite(vibrationPin, HIGH); // Turn on vibration motor
  delay(delayms);                   // Vibrate for specified time
  digitalWrite(vibrationPin, LOW);  // Turn off vibration motor
}