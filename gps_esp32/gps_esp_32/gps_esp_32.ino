#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

// Define pins for hardware serial
#define GPS_RX 14  // Connect to TX of GPS Module
#define GPS_TX 12  // Connect to RX of GPS Module

// Initialize the hardware serial port for GPS module
HardwareSerial serialGPS(1); // Use UART1

// The TinyGPSPlus object
TinyGPSPlus gps;

void setup() {
  // Start the hardware serial for debugging
  Serial.begin(9600);

  // Configure the hardware serial port for GPS module
  serialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  Serial.println("GPS Module Test");
}

void loop() {
  // Read data from GPS module and encode it
  while (serialGPS.available() > 0) {
    char c = serialGPS.read();
    Serial.write(c);  // Print raw data for debugging
    gps.encode(c);
  }

  // Display GPS data if available
  displayInfo();

  // Check if no GPS data is detected
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }
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
