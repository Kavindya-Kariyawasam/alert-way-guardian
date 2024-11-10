#include <Wire.h>
#include <BH1750.h>

BH1750 lightMeter;

void setup() {
  Serial.begin(9600);
  Wire.begin(); // Join I2C bus
  
  if (lightMeter.begin()) {
    Serial.println(F("BH1750 Advanced begin"));
  } else {
    Serial.println(F("Error initializing BH1750"));
  }
}

void loop() {
  float lux = lightMeter.readLightLevel();
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");

  delay(1000); // Wait for a second before next reading
}
