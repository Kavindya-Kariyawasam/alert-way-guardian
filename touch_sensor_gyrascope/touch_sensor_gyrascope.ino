#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
#define BUZZER_PIN 4 // Define the buzzer pin
#define TOUCH_PIN 5  // Define the touch sensor pin

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TOUCH_PIN, INPUT_PULLUP); // Assuming the touch sensor is active LOW
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  // Calculate acceleration magnitude
  float acceleration = sqrt(a.acceleration.x * a.acceleration.x +
                            a.acceleration.y * a.acceleration.y +
                            a.acceleration.z * a.acceleration.z);

  // Check if acceleration exceeds 15 m/s^2
  if (acceleration > 15.0) {
    // If yes, activate the buzzer
    tone(BUZZER_PIN, 2000, 10000);
    Serial.println("High acceleration detected! Activating buzzer.");

    // Continue buzzing until touch sensor is touched
    while (digitalRead(TOUCH_PIN) == HIGH) {
      // Wait for touch sensor to be touched
    }
    // Turn off the buzzer once touch sensor is touched
    noTone(BUZZER_PIN);
    Serial.println("Touch sensor touched. Buzzer turned off.");
  }

  Serial.println("");
  delay(500);
}
