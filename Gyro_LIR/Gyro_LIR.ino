// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <BH1750.h>

BH1750 lightMeter;
Adafruit_MPU6050 mpu;

void setup(void) {
  //noInterrupts(); // Disable interrupts
  
  Serial.begin(9600);
  Wire.begin(); // Join I2C bus
  
  if (lightMeter.begin()) {
    Serial.println(F("BH1750 Advanced begin"));
  } else {
    Serial.println(F("Error initializing BH1750"));
  }
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

  // Set buzzer pin as output
  // pinMode(BUZZER_PIN, OUTPUT);
  // pinMode(ledPin, OUTPUT); 
  
  //interrupts(); // Re-enable interrupts
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
    // tone(BUZZER_PIN, 2000, 1000);
    // digitalWrite(ledPin, HIGH);
    Serial.println("High acceleration detected! Activating buzzer.");
  } else {
    // Otherwise, turn off the buzzer
    //noTone(BUZZER_PIN);
    Serial.println("Normal acceleration.");
  }

  Serial.println("");

  float lux = lightMeter.readLightLevel();
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
  delay(500);
}
