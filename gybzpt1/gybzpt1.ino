// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
#define BUZZER_PIN 4 // Define the buzzer pin
#define LED_PIN 13   // Define the LED pin

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  
  // Set buzzer pin and LED pin as output
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("");
  delay(100);
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
    // If yes, sound the buzzer and light the LED
    tone(BUZZER_PIN, 2000); // Sound the buzzer at 2000 Hz
    digitalWrite(LED_PIN, HIGH); // Turn on the LED
    Serial.println("High acceleration detected! Activating buzzer and LED.");
  } else {
    // Otherwise, turn off the buzzer and LED
    noTone(BUZZER_PIN); // Stop the buzzer
    digitalWrite(LED_PIN, LOW); // Turn off the LED
    Serial.println("Normal acceleration.");
  }

  Serial.println("");
  delay(1500);
}
