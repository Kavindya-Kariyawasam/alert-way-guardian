#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Arduino.h>
#include <ArduinoJson.h>

TinyGPSPlus gps;
HardwareSerial neo8m(1);

// WiFi credentials
const char* ssid = "<SSID>";
const char* password = "<PASSWOED>";

// Google Geolocation API key
const char* apiKey = "<API_KEY>";

String msgString;
String macAddress;
int signalStrength;

#define gpsRxPin 16
#define gpsTxPin 17

unsigned long gpsStartTime = 0;
const unsigned long gpsTimeout = 360000;  //6 minutes
bool gpsSearching = false;

int globMode;

unsigned long msgTime = 0 - 120000;
// const unsigned long msgInterval = 120000; // 2 minutes
const unsigned long msgInterval = 8000;  // 30 secs

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  neo8m.begin(9600, SERIAL_8N1, gpsRxPin, gpsTxPin);

  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  macAddress = WiFi.BSSIDstr();
  signalStrength = WiFi.RSSI();
}

void loop() {
  globMode = 1;  //Hardcoded emergency mode to health emergency for testing.
  if (gpsSearching && (millis() - gpsStartTime < gpsTimeout)) {
    smartdelay_gps(1000);
    if (WiFi.status() == WL_CONNECTED) {
      if (gps.location.isUpdated() && gps.location.isValid()) {
        Serial.print(F("Location: "));
        Serial.print("Lat: ");
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(", Lng: "));
        Serial.print(gps.location.lng(), 6);
        Serial.println();

        sendMsg1(globMode, gps.location.lat(), gps.location.lng());
        gpsSearching = false;
        msgTime = millis();

        Serial.println(F("Location update sent."));
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
  } else if (gpsSearching && (millis() - gpsStartTime >= gpsTimeout)) {
    gpsSearching = false;
    msgTime = millis();
    sendGeolocationRequest(globMode, macAddress, signalStrength);
  }

  // After 1 msg is sent, for two minutes it waits.
  if (millis() - msgTime >= msgInterval) {
    gpsSearching = true;
    gpsStartTime = millis();
    Serial.println("Starting GPS search...");
  }

  delay(1000);
}

void sendGeolocationRequest(int mode, String macAddress, int signalStrength) {
  HTTPClient https;
  WiFiClientSecure wifiClient;
  wifiClient.setInsecure();

  String url = "https://www.googleapis.com/geolocation/v1/geolocate?key=" + String(apiKey);

  StaticJsonDocument<200> jsonPayload;
  jsonPayload["wifiAccessPoints"][0]["macAddress"] = macAddress;
  jsonPayload["wifiAccessPoints"][0]["signalStrength"] = signalStrength;

  String jsonString;
  serializeJson(jsonPayload, jsonString);

  https.begin(wifiClient, url);
  https.addHeader("Content-Type", "application/json");
  int httpResponseCode = https.POST(jsonString);

  if (httpResponseCode > 0) {
    if (httpResponseCode == HTTP_CODE_OK || httpResponseCode == HTTP_CODE_CREATED) {
      String payload = https.getString();
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, payload);

      if (error) {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        return;
      }

      float latitude = doc["location"]["lat"];
      float longitude = doc["location"]["lng"];
      float accuracy = doc["accuracy"];

      Serial.print("Latitude: ");
      Serial.println(latitude, 7);
      Serial.print("Longitude: ");
      Serial.println(longitude, 7);
      Serial.print("Accuracy (meters): ");
      Serial.println(accuracy);

      sendMsg2(mode, latitude, longitude, accuracy);
    } else {
      Serial.print("HTTPS error code: ");
      Serial.println(httpResponseCode);
    }
  } else {
    Serial.print("HTTPS request failed: ");
    Serial.println(httpResponseCode);
  }

  https.end();
}

// void sendMsg1(int emg_mode, float lat, float lon) {
//     HTTPClient https;
//     WiFiClientSecure wifiClient;
//     wifiClient.setInsecure();

//     https.begin(wifiClient, "https://app.notify.lk/api/v1/send");
//     https.addHeader("Content-Type", "application/x-www-form-urlencoded");

//     String mapLink = generateMapLink(lat, lon);
//     String receiverNumber = "<PHONE>";
//     String httpRequestData = "user_id=<ID>&api_key=<KEY>&sender_id=NotifyDEMO&to=contactNumber&message=MSG String";

//     String msgString = (emg_mode == 0) ? "Emergency\n\n" : "Health emergency\n\n";
//     msgString += mapLink;
//     Serial.println(msgString);

//     httpRequestData.replace("MSG String", msgString);
//     httpRequestData.replace("contactNumber", receiverNumber);
//     int httpCode = https.POST(httpRequestData);
//     Serial.println(httpCode);

//     if (httpCode > 0) {
//       String payload = https.getString();
//       Serial.println(payload);
//     } else {
//       Serial.println("Error on HTTP request");
//     }

//     https.end();
// }

// void sendMsg1(int emg_mode, float lat, float lon) {
//     HTTPClient http;
//     WiFiClient wifiClient;

//     http.begin(wifiClient, "https://app.notify.lk/api/v1/send");
//     http.addHeader("Content-Type", "application/x-www-form-urlencoded");

//     String mapLink = generateMapLink(lat, lon);
//     String receiverNumber = "<PHONE>";
//     String httpRequestData = "user_id=<ID>&api_key=<KEY>&sender_id=NotifyDEMO&to=contactNumber&message=MSG String";
//     String msgString = (emg_mode == 0) ? "Emergency\n\n" : "Health emergency\n\n";
//     msgString += mapLink;
//     Serial.println(msgString);
//     httpRequestData.replace("MSG String", msgString);
//     httpRequestData.replace("contactNumber", receiverNumber);
//     int httpCode = http.POST(httpRequestData);
//     Serial.println(httpCode);

//     if (httpCode > 0) {
//       String payload = http.getString();
//       Serial.println(payload);
//     } else {
//       Serial.println("Error on HTTP request");
//     }

//     http.end();
// }

void sendMsg1(int emg_mode, float lat, float lon) {
  // Simulate HTTP client and request preparation
  Serial.println("Preparing to send message...");

  String mapLink = generateMapLink(lat, lon);
  String receiverNumber = "<PHONE>";
  String httpRequestData = "user_id=<ID>&api_key=<KEY>&sender_id=NotifyDEMO&to=contactNumber&message=MSG String";
  String msgString = (emg_mode == 0) ? "Emergency\n\n" : "Health emergency\n\n";
  msgString += mapLink;

  Serial.println("Message content:");
  Serial.println(msgString);

  httpRequestData.replace("MSG String", msgString);
  httpRequestData.replace("contactNumber", receiverNumber);

  Serial.println("Simulated HTTP POST data:");
  Serial.println(httpRequestData);

  // Simulate sending the request
  Serial.println("Simulating HTTP POST request to https://app.notify.lk/api/v1/send");

  // Simulate a successful HTTP response
  int simulatedHttpCode = 200;
  Serial.println("Simulated HTTP Response Code: " + String(simulatedHttpCode));

  if (simulatedHttpCode > 0) {
    Serial.println("Message sent successfully (simulated)");
    String simulatedPayload = "{\"status\":\"success\",\"message_id\":\"12345\"}";
    Serial.println("Simulated API Response: " + simulatedPayload);
  } else {
    Serial.println("Error on HTTP request (simulated)");
  }

  Serial.println("Message sending process completed (simulated)");
}

// void sendMsg2(int emg_mode, float lat, float lon, float accuracy) {
//     HTTPClient http;
//     WiFiClient wifiClient;

//     http.begin(wifiClient, "https://app.notify.lk/api/v1/send");
//     http.addHeader("Content-Type", "application/x-www-form-urlencoded");

//     String mapLink = generateMapLink(lat, lon);
//     String receiverNumber = "<PHONE>";
//     String httpRequestData = "user_id=<ID>&api_key=<KEY>&sender_id=NotifyDEMO&to=contactNumber&message=MSG String";
//     String msgString = (emg_mode == 0) ? "Emergency\n\n" : "Health emergency\n\n";
//     msgString += mapLink;
//     msgString += "\n\nAccuracy in meters: " + String(accuracy, 6);
//     Serial.println(msgString);
//     httpRequestData.replace("MSG String", msgString);
//     httpRequestData.replace("contactNumber", receiverNumber);
//     int httpCode = http.POST(httpRequestData);
//     Serial.println(httpCode);

//     if (httpCode > 0) {
//       String payload = http.getString();
//       Serial.println(payload);
//     } else {
//       Serial.println("Error on HTTP request");
//     }

//     http.end();
// }

void sendMsg2(int emg_mode, float lat, float lon, float accuracy) {
  // Simulate HTTP client and request preparation
  Serial.println("Preparing to send message...");

  String mapLink = generateMapLink(lat, lon);
  String receiverNumber = "<PHONE>";
  String httpRequestData = "user_id=<ID>&api_key=<KEY>&sender_id=NotifyDEMO&to=contactNumber&message=MSG String";
  String msgString = (emg_mode == 0) ? "Emergency\n\n" : "Health emergency\n\n";
  msgString += mapLink;
  msgString += "\n\nAccuracy in meters: " + String(accuracy, 6);

  Serial.println("Message content:");
  Serial.println(msgString);

  httpRequestData.replace("MSG String", msgString);
  httpRequestData.replace("contactNumber", receiverNumber);

  Serial.println("Simulated HTTP POST data:");
  Serial.println(httpRequestData);

  // Simulate sending the request
  Serial.println("Simulating HTTP POST request to https://app.notify.lk/api/v1/send");

  // Simulate a successful HTTP response
  int simulatedHttpCode = 200;
  Serial.println("Simulated HTTP Response Code: " + String(simulatedHttpCode));

  if (simulatedHttpCode > 0) {
    Serial.println("Message sent successfully (simulated)");
    String simulatedPayload = "{\"status\":\"success\",\"message_id\":\"12345\"}";
    Serial.println("Simulated API Response: " + simulatedPayload);
  } else {
    Serial.println("Error on HTTP request (simulated)");
  }

  Serial.println("Message sending process completed (simulated)");
}

String generateMapLink(float lat, float lon) {
  String latDirection = (lat >= 0) ? "N" : "S";
  String lonDirection = (lon >= 0) ? "E" : "W";
  float absLat = abs(lat);
  float absLon = abs(lon);
  String mapLink = "https://www.google.com/maps/place/";
  mapLink += String(absLat, 6) + latDirection + String(absLon, 6) + lonDirection + "/";
  return mapLink;
}

void smartdelay_gps(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (neo8m.available()) {
      char c = neo8m.read();
      Serial.write(c);
      gps.encode(c);
    }
  } while (millis() - start < ms);
}