#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

#define buzzerPin 13   // Define buzzerPin for ESP32
#define ctsPin1 2       // Define capacitive touch sensor 1 pin for ESP32
#define ctsPin2 4       // Define capacitive touch sensor 2 pin for ESP32
#define ledPin1 32      // Define LED pin 1 for ESP32
#define ledPin2 33      // Define LED pin 2 for ESP32
#define vibrationPin 5 // Define pin for vibration motor for ESP32

// Define pins for hardware serial
#define GPS_RX 14  // Connect to TX of GPS Module
#define GPS_TX 12  // Connect to RX of GPS Module

// Initialize the hardware serial port for GPS module
HardwareSerial serialGPS(1); // Use UART1

// The TinyGPSPlus object
TinyGPSPlus gps;

void setup() {
  pinMode(buzzerPin, OUTPUT);   // Set buzzerPin as output
  pinMode(ledPin1, OUTPUT);      // Set LED pin 1 as output
  pinMode(ledPin2, OUTPUT);      // Set LED pin 2 as output
  pinMode(ctsPin1, INPUT);       // Set capacitive touch sensor 1 pin as input
  pinMode(ctsPin2, INPUT);       // Set capacitive touch sensor 2 pin as input
  pinMode(vibrationPin, OUTPUT);// Set vibration motor pin as output

  // Serial.begin(115200);           // Initialize serial communication

  // Start the hardware serial for debugging
  Serial.begin(9600);

  // Configure the hardware serial port for GPS module
  serialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  Serial.println("GPS Module Test");
}

void loop() {
  int ctsValue1 = digitalRead(ctsPin1); // Read capacitive touch sensor value
  int ctsValue2 = digitalRead(ctsPin2); // Read capacitive touch sensor value

  // Read data from GPS module and encode it
  if (serialGPS.available() > 0) {
    char c = serialGPS.read();
    Serial.write(c);  // Print raw data for debugging
    gps.encode(c);
  }

  // Display GPS data if available
  displayInfo();

  // Check if no GPS data is detected - Commented this code because it's going to halt the code below this check if GPS signal is not found
  // if (millis() > 5000 && gps.charsProcessed() < 10) {
  //   Serial.println(F("No GPS detected: check wiring."));
  //   while (true);
  // }
  
  if (ctsValue1 == HIGH) {
    digitalWrite(ledPin1, HIGH); // Turn on LED 1
    Serial.println("SENSOR 1 TOUCHED");
    tone(buzzerPin,1535,500); // Beep when touched
    vibrate(500); // Vibrate when touched
  } else {
    digitalWrite(ledPin1, LOW); // Turn off LED 1
    // Serial.println("Not touched");
  }

  if (ctsValue2 == HIGH) {
    digitalWrite(ledPin2, HIGH); // Turn on LED 2
    Serial.println("SENSOR 2 TOUCHED");
    tone(buzzerPin,1300,500); // Beep when touched
    vibrate(3000); // Vibrate when touched
  } else {
    digitalWrite(ledPin2, LOW); // Turn off LED 2
    // Serial.println("Not touched");
  }
  
  // delay(2000);
}

void vibrate(unsigned long delayms) { // Function to vibrate
  digitalWrite(vibrationPin, HIGH); // Turn on vibration motor
  delay(delayms);                   // Vibrate for specified time
  digitalWrite(vibrationPin, LOW);  // Turn off vibration motor
}

void displayInfo() {
  // Only print GPS data if new data is available
  if (gps.location.isUpdated()) {
    Serial.print(F("Location: "));
    if (gps.location.isValid()) {
      Serial.print("Lat: ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(F(", Lng: "));
      Serial.print(gps.location.lng(), 6);
      Serial.println();
    } else {
      Serial.println(F("INVALID"));
    }
  }

  if (gps.date.isUpdated()) {
    Serial.print(F("Date: "));
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
    Serial.println();
  }

  if (gps.time.isUpdated()) {
    Serial.print(F("Time: "));
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
    Serial.println();
  }
}