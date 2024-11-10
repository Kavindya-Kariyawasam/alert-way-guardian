#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>  // Include WiFiClientSecure for HTTPS
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BH1750.h>  // Include the BH1750 library for the light sensor

Adafruit_MPU6050 mpu; // MPU6050 object
TinyGPSPlus gps; // GPS object
HardwareSerial neo6m(1); // Use hardware serial port 1 on ESP32
BH1750 lightMeter;  // Create an instance of the BH1750 sensor

// WiFi credentials
const char* ssid = "<SSID>";
const char* password = "<PASSWORD>";
// int runOnce = 1;

// Touch sensor and actuator pins
#define touchSensorPin1 2   // GPIO2 (D2) for first TTP 223 touch sensor
#define touchSensorPin2 4   // GPIO4 (D4) for second TTP 223 touch sensor
#define vibrationPin 5      // GPIO5 for the vibration module
#define buzzerPin 26        // GPIO25 for the buzzer
#define ledPin 27           // GPIO13 for the LED controlled by light sensor
//CAM
#define cameraPin 13  // GPIO13 for camera

// GPS module pins
#define gpsRxPin 16         // GPIO16 for Tx pin of GPS module
#define gpsTxPin 17         // GPIO17 for Rx pin of GPS module

int globMode;
bool State = false;
bool gyroTriggered = false;
bool sensor1State = false;
bool sensor2State = false;
// bool vibrationOn = false;
bool buzzerOn = false;
bool emergencyActive = false;  // To track if emergency is active
unsigned long vibrationStartTime = 0;
const unsigned long vibrationDuration = 10000; // 10 seconds

// Light sensor threshold
const float lightThreshold = 28.0; // Light intensity threshold in lux

// Long press configuration
const unsigned long touchDuration = 3000; // 3 seconds
unsigned long touchStartTime1 = 0;
unsigned long touchStartTime2 = 0;

void setup() {
  pinMode(touchSensorPin1, INPUT);  // Set touch sensor 1 pin as input
  pinMode(touchSensorPin2, INPUT);  // Set touch sensor 2 pin as input
  pinMode(vibrationPin, OUTPUT);    // Set vibration module pin as output
  pinMode(buzzerPin, OUTPUT);       // Set buzzer pin as output
  pinMode(ledPin, OUTPUT);          // Set LED pin as output
  digitalWrite(vibrationPin, LOW);  // Ensure vibration module is off initially
  digitalWrite(buzzerPin, LOW);     // Ensure buzzer is off initially
  digitalWrite(ledPin, LOW);        // Ensure LED is off initially

  pinMode(cameraPin, OUTPUT);  // Set camera pin as output

  Serial.begin(115200);             // Initialize serial communication

  WiFi.begin(ssid, password); // Connect to WiFi with ESP32-specific parameters
  neo6m.begin(9600, SERIAL_8N1, gpsRxPin, gpsTxPin); // Start serial communication with GPS module

  while (!Serial) delay(10);        // Wait for serial monitor to open
  // Serial.println("Initializing MPU6050...");
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  Serial.println("Initializing GPS module...");
  neo6m.begin(9600, SERIAL_8N1, gpsRxPin, gpsTxPin);

  // Serial.println("Initializing light sensor...");
  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("Error initializing BH1750");
    while (1);
  }
  
  Serial.println("Setup completed.");
}

static void smartdelay_gps(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (neo6m.available()) {
      char c = neo6m.read();
      Serial.write(c);  // Print raw data for debugging
      gps.encode(c);
    }
  } while (millis() - start < ms); 
}

String generateMapLink(float lat, float lon) {
  String latDirection = (lat >= 0) ? "N" : "S";  // Determine if latitude is North or South
  String lonDirection = (lon >= 0) ? "E" : "W";  // Determine if longitude is East or West

  // Convert latitude and longitude to absolute values for formatting
  float absLat = abs(lat);
  float absLon = abs(lon);

  // Construct the formatted URL string
  String mapLink = "https://www.google.com/maps/place/";
  mapLink += String(absLat, 6);  // Append latitude with 6 decimal places
  mapLink += latDirection;       // Append North (N) or South (S)
  mapLink += String(absLon, 6);  // Append longitude with 6 decimal places
  mapLink += lonDirection;       // Append East (E) or West (W)
  mapLink += "/";

  return mapLink;
}

void sendMsg(int emg_mode, float lat, float lon) {
    String mapLink = generateMapLink(lat, lon);
    String msgString;
    if (emg_mode == 0) {
      msgString = "Emergency\n\n" + mapLink;
    } else {
      msgString = "Health emergency\n\n" + mapLink;  
    }
    Serial.println(msgString); 
}

void updateLocation_sendMSG(int mode) {
  smartdelay_gps(1000); // Collect GPS data

  if (WiFi.status() == WL_CONNECTED) {
    if (gps.location.isUpdated()) {
      Serial.print(F("Location: "));
      if (gps.location.isValid()) {
        Serial.print("Lat: ");
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(", Lng: "));
        Serial.print(gps.location.lng(), 6);
        Serial.println();

        sendMsg(mode, gps.location.lat(), gps.location.lng());

        Serial.println(F("Location update sent."));
      } else {
        Serial.println(F("INVALID"));
      }
    }
  }
}

void activateEmergency(int mode) {
  digitalWrite(vibrationPin, HIGH);  // Turn on vibration module
  // vibrationOn = true;                // Set vibration state to on
  vibrationStartTime = millis();     // Record the start time

  digitalWrite(buzzerPin, HIGH);  // Turn on buzzer
  tone(buzzerPin, 1535, 500);      // Sound the buzzer with a frequency of 200 Hz
  buzzerOn = true; 

  emergencyActive = true;   // Mark emergency as active

  
  digitalWrite(cameraPin, HIGH);    //Camera

  gyroTriggered = false;

  // Initial GPS and message sending
  updateLocation_sendMSG(mode);
}

void deactivateEmergency() {
  digitalWrite(vibrationPin, LOW);   // Turn off vibration module
  // vibrationOn = false;               // Reset vibration state

  digitalWrite(buzzerPin, LOW);      // Turn off buzzer
  noTone(buzzerPin);                 // Stop the tone (optional)
  buzzerOn = false;                  // Reset buzzer state

  digitalWrite(ledPin, LOW);         // Turn off LED when emergency is deactivated
  digitalWrite(cameraPin, LOW);   // Turn off camera
  
  emergencyActive = false;           // Mark emergency as inactive
}

void controlLEDByLightSensor() {
  float lux = lightMeter.readLightLevel(); // Read the light intensity in lux
  
  Serial.print("Light Intensity: ");
  Serial.print(lux);
  Serial.println(" lux");

  if (lux < lightThreshold) {
    digitalWrite(ledPin, HIGH); // Turn on LED
    // Serial.println("It's dark, LED ON");
  } else {
    digitalWrite(ledPin, LOW); // Turn off LED
    // Serial.println("It's bright, LED OFF");
  }
}

void loop() {
  // Read touch sensors
  sensor1State = digitalRead(touchSensorPin1);
  sensor2State = digitalRead(touchSensorPin2);

  // Read gyroscope data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  Serial.printf("Accel X: %.2f, Y: %.2f, Z: %.2f\n", a.acceleration.x, a.acceleration.y, a.acceleration.z);
  Serial.printf("Gyro X: %.2f, Y: %.2f, Z: %.2f\n", g.gyro.x, g.gyro.y, g.gyro.z);
  // Serial.printf("Temp: %.2f C\n", temp.temperature);
  
  // Check for sudden movement
  if (abs(a.acceleration.x) > 15 || abs(a.acceleration.y) > 15 || abs(a.acceleration.z) > 15 ||
      abs(g.gyro.x) > 120 || abs(g.gyro.y) > 120 || abs(g.gyro.z) > 120) {

    if (!gyroTriggered) {
      Serial.println("Sudden movement detected - Vibration activated.");

      digitalWrite(vibrationPin, HIGH);  // Turn on vibration module
      // vibrationOn = true;                // Set vibration state to on
      gyroTriggered = true;
      vibrationStartTime = millis();     // Record the start time
    }
  }

  // Check if vibration has been on for more than the threshold
  if (gyroTriggered && ((millis() - vibrationStartTime) >= vibrationDuration)) {
    Serial.println("Vibration duration exceeded - Emergency activated");
    globMode = 1;
    activateEmergency(globMode);
  }

  // Check for long press activation on touch sensor 1 or 2
  if (sensor1State || sensor2State) {
    //for touch sensor 1
    if (touchStartTime1 == 0) { // If not already timing
      touchStartTime1 = millis(); // Start timing
    } else if (millis() - touchStartTime1 >= touchDuration) {
      // if (!vibrationOn) {                                                           //WHY?
      //   // Serial.println("Touch sensor 1 long press - Emergency activated");
      //   activateEmergency(0);
      // }
      globMode = 0;
      activateEmergency(globMode);
    }
    //for touch sensor 2
    if (touchStartTime2 == 0) { // If not already timing
      touchStartTime2 = millis(); // Start timing
    } else if (millis() - touchStartTime2 >= touchDuration) {
      // if (!vibrationOn) {
      //   // Serial.println("Touch sensor 2 long press - Emergency activated");
      //   activateEmergency(0);
      // }
      globMode = 0;
      activateEmergency(globMode);
    }
  } else {
    touchStartTime1 = 0; // Reset the timer if touch is released
    touchStartTime2 = 0; // Reset the timer if touch is released
  }

  // Deactivate emergency if both touch sensors are touched simultaneously
  if (sensor1State && sensor2State) {
    Serial.println("Both touch sensors activated - Emergency deactivated");
    deactivateEmergency();
  }

  // If emergency is active, check lux reading and turn the flash on
  if (emergencyActive) {
    controlLEDByLightSensor();
    // GPS and message sending
    updateLocation_sendMSG(globMode);
  }

  delay(1000); // Delay for stability
}