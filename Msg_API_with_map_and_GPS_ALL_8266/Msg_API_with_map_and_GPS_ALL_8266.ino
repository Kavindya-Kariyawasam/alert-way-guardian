#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h> 

#define gpsRxPin 5      //Rx pin of gps D1
#define gpsTxPin 4      //Tx pin of gps D2
SoftwareSerial neo6m(gpsTxPin, gpsRxPin);

TinyGPSPlus gps;

const char* ssid = "Galaxy S21 FE 5G 64df";
const char* password = "kdma5906";
int runOnce = 1;

// void sendMsg() {
//     HTTPClient http;
//     WiFiClientSecure wifiClient;
//     wifiClient.setInsecure();

//     http.begin(wifiClient, "https://app.notify.lk/api/v1/send");
//     http.addHeader("Content-Type", "application/x-www-form-urlencoded");

//     // replace user_id & api_key vakues with your new values (Current values are for Robin Hood)
//     String httpRequestData = "user_id=26989&api_key=uZGg7vlV6lzvyW6BcBwH&sender_id=NotifyDEMO&to=contactNumber&message=MSG String";
//     String msgString = "Emergency";
//     String receiverNumber = "94724043919";
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

static void smartdelay_gps(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
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
    // mapLink += "%C2%B0";           // Append degree symbol
    mapLink += latDirection;       // Append North (N) or South (S)
    // mapLink += "+";
    mapLink += String(absLon, 6);  // Append longitude with 6 decimal places
    // mapLink += "%C2%B0";           // Append degree symbol
    mapLink += lonDirection;       // Append East (E) or West (W)
    mapLink += "/";

    return mapLink;
}

void sendMsg(int emg_mode, float lat, float lon) {
    HTTPClient http;
    WiFiClientSecure wifiClient;
    wifiClient.setInsecure();

    http.begin(wifiClient, "https://app.notify.lk/api/v1/send");
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    // replace user_id & api_key vakues with your new values (Current values are for Robin Hood)
    String mapLink = generateMapLink(lat, lon);
    String receiverNumber = "94724043919";
    String httpRequestData = "user_id=27348&api_key=j7WAzi6LIQ6eywOsbTbl&sender_id=NotifyDEMO&to=contactNumber&message=MSG String";
    String msgString;
    if (emg_mode == 0) {
      msgString = "Emergency\n\n" + mapLink;
    } else {
      msgString = "Health emergency\n\n" + mapLink;  
    }
    Serial.println(msgString); 
    httpRequestData.replace("MSG String", msgString);
    httpRequestData.replace("contactNumber", receiverNumber);
    int httpCode = http.POST(httpRequestData);
    Serial.println(httpCode);

    if (httpCode > 0) {
      String payload = http.getString();
      Serial.println(payload);
    } else {
      Serial.println("Error on HTTP request");
    }

    http.end();
}

void getLocation_sendMSG(int mode) {
  if (runOnce == 1) {
    smartdelay_gps(1000);
    
    if (WiFi.status() == WL_CONNECTED) {

      if (gps.location.isUpdated()) {
        Serial.print(F("Location: "));
        if (gps.location.isValid()) {
          Serial.print("Lat: ");
          Serial.print(gps.location.lat(), 6);
          Serial.print(F(", Lng: "));
          Serial.print(gps.location.lng(), 6);
          Serial.println();

          // sendMsg(0, 6.892940, 79.904337);     // remove this hardcoded values
          sendMsg(mode, gps.location.lat(), gps.location.lng());
          runOnce=0;

          Serial.println(F("Succesfully Completed"));

        } else {
          Serial.println(F("INVALID"));
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  neo6m.begin(9600);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
}

void loop() {
  getLocation_sendMSG(1);
}















