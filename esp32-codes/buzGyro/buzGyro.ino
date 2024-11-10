#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define BUZZER_PIN 2

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);

  // Try to initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Set accelerometer range
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate acceleration magnitude
  float acceleration = sqrt(a.acceleration.x * a.acceleration.x +
                            a.acceleration.y * a.acceleration.y +
                            a.acceleration.z * a.acceleration.z);

  // Check for sudden acceleration
  if (acceleration > 15.0) { // Adjust threshold as needed
    activateBuzzer();
  }

  // Print data
  Serial.print("Acceleration: ");
  Serial.print(acceleration);
  Serial.println(" m/s^2");

  delay(100);
}

void activateBuzzer() {
  tone(BUZZER_PIN, 1000, 500); // Play a tone for 500ms
  delay(500); // Add a brief pause
}
