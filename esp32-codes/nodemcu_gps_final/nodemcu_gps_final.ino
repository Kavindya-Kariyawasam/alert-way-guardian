#include <SoftwareSerial.h>
#include <TinyGPS++.h> 
#include <ESP8266WiFi.h>

#define gpsRxPin 5      //Rx pin of gps D1
#define gpsTxPin 4      //Tx pin of gps D2
SoftwareSerial neo6m(gpsTxPin, gpsRxPin);

TinyGPSPlus gps;

void setup()
{
  Serial.begin(115200);
  Serial.println();
  neo6m.begin(9600);

  Serial.print("Setup done ");  //Debugging
}

void loop()
{
  smartdelay_gps(1000);
  
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
}

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
