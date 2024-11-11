#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BH1750.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <driver/i2s.h>

#include <WebSocketsClient.h>
#include <CircularBuffer.hpp>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_task_wdt.h"

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial neo8m(1);
BH1750 lightMeter;

// WiFi credentials
const char* ssid = "Galaxy A13CB91";
const char* password = "12345678";

// const char* ssid = "Galaxy S21 FE 5G 64df";
// const char* password = "kdma5906";

// WebSocket server details
const char *wsServer = "192.168.26.64";
const uint16_t wsPort = 8080;

// Google Geolocation API key
const char* apiKey = "AIzaSyBBrZDws6wFp8BDCnlnelpKL9y5rTRctZc";

String macAddress;
int signalStrength;

// Pin definitions
#define touchSensorPin1 2
#define touchSensorPin2 4
#define vibrationPin 5
#define buzzerPin 26
#define ledPin 19
#define cameraPin 13
#define gpsRxPin 16
#define gpsTxPin 17

// LEDC configuration for buzzer
#define LEDC_CHANNEL_0     0
#define LEDC_TIMER_13_BIT  13
#define LEDC_BASE_FREQ     5000

// Audio configuration
#define SAMPLE_RATE       44100
#define BITS_PER_SAMPLE   I2S_BITS_PER_SAMPLE_16BIT
#define I2S_PORT          I2S_NUM_0
#define DMA_BUF_COUNT     4
#define DMA_BUF_LEN       512

// INMP441 Pin assignment
#define I2S_SD            33
#define I2S_WS            25
#define I2S_SCK           32

// Audio buffer configuration
#define AUDIO_BUFFER_SIZE 10
#define AUDIO_CHUNK_SIZE  512

struct AudioChunk {
    uint8_t data[AUDIO_CHUNK_SIZE];
    size_t size;
};

// Global objects
WebSocketsClient webSocket;
CircularBuffer<AudioChunk, AUDIO_BUFFER_SIZE> audioBuffer;
SemaphoreHandle_t audioBufferSemaphore;

// Recording state
volatile bool isRecording = false;

// Buzzer beep pattern
const int beepDuration = 300;
const int beepInterval = 1000;
unsigned long lastBeepTime = 0;
bool beepState = false;

unsigned long emergencyStartTime = 0;
const unsigned long buzzerDuration = 20000; // 20 seconds in milliseconds

int globMode;
bool State = false;
bool gyroTriggered = false;
bool sensor1State = false;
bool sensor2State = false;
bool buzzerOn = false;
bool emergencyActive = false;

unsigned long vibrationStartTime = 0;
const unsigned long vibrationDuration = 10000;

const float lightThreshold = 20.0;

const unsigned long touchDuration = 3000;
unsigned long touchStartTime1 = 0;
unsigned long touchStartTime2 = 0;

unsigned long msgTime = 0 - 40000;
const unsigned long msgInterval = 40000; // 40 seconds

unsigned long gpsStartTime = 0;
const unsigned long gpsTimeout = 360000; //6 minutes
bool gpsSearching = false;

void setupWiFi();
void setupI2S();
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length);
void i2sReaderTask(void *parameter);
void audioSenderTask(void *parameter);
void automaticRecordingTask(void *parameter);
void gpsTask(void *parameter);
void sensorMonitorTask(void *parameter);
void webSocketLoopTask(void *parameter);
void resetI2S();

void setup() {
    pinMode(touchSensorPin1, INPUT);
    pinMode(touchSensorPin2, INPUT);
    pinMode(vibrationPin, OUTPUT);
    pinMode(buzzerPin, OUTPUT);
    pinMode(ledPin, OUTPUT);
    pinMode(cameraPin, OUTPUT);
    
    digitalWrite(vibrationPin, LOW);
    digitalWrite(buzzerPin, LOW);
    digitalWrite(ledPin, LOW);
    esp_task_wdt_init(20, true);

    Serial.begin(115200);

    ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
    ledcAttachPin(buzzerPin, LEDC_CHANNEL_0);

    WiFi.begin(ssid, password);
    neo8m.begin(9600, SERIAL_8N1, gpsRxPin, gpsTxPin);

    while (!Serial) delay(10);
    
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

    macAddress = WiFi.BSSIDstr();
    signalStrength = WiFi.RSSI();

    if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
        Serial.println("Error initializing BH1750");
        while (1);
    }
    
    setupI2S();
    
    webSocket.begin(wsServer, wsPort, "/");
    webSocket.onEvent(webSocketEvent);  
    webSocket.setReconnectInterval(5000);
    
    audioBufferSemaphore = xSemaphoreCreateMutex();
    if (audioBufferSemaphore == NULL) {
        Serial.println("Error creating semaphore");
    }
    
    // Audio tasks on Core 0
    xTaskCreatePinnedToCore(i2sReaderTask, "I2S Reader", 4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(audioSenderTask, "Audio Sender", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(automaticRecordingTask, "Auto Recording", 4096, NULL, 2, NULL, 0);

    // Other tasks on Core 1
    // xTaskCreatePinnedToCore(gpsTask, "GPS Task", 8192, NULL, 2, NULL, 1);
    // xTaskCreatePinnedToCore(gpsTask, "GPS Task", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(webSocketLoopTask, "WebSocketLoop", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(sensorMonitorTask, "Sensor Monitor", 4096, NULL, 3, NULL, 1);

    Serial.println("Setup completed.");
}

void loop() {
    // delay(1000);
    if (emergencyActive) {
      if (!gpsSearching) {
        if ((millis() - msgTime) >= msgInterval) {
          gpsSearching = true;
          gpsStartTime = millis();
        }
      }
    }

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

  vTaskDelay(pdMS_TO_TICKS(1000)); // Delay to prevent watchdog timer issues
}

static void smartdelay_gps(unsigned long ms) {
    unsigned long start = millis();
    do {
        while (neo8m.available()) {
            char c = neo8m.read();
            Serial.write(c);
            gps.encode(c);
        }
    } while (millis() - start < ms);
}

void sensorMonitorTask(void *parameter) {
    esp_task_wdt_add(NULL);
    while (true) {
        esp_task_wdt_reset();

        sensor1State = digitalRead(touchSensorPin1);
        sensor2State = digitalRead(touchSensorPin2);

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
  
        Serial.printf("Accel X: %.2f, Y: %.2f, Z: %.2f\n", a.acceleration.x, a.acceleration.y, a.acceleration.z);
        Serial.printf("Gyro X: %.2f, Y: %.2f, Z: %.2f\n", g.gyro.x, g.gyro.y, g.gyro.z);
        
        // smartdelay_gps(1000);
        
        if (abs(a.acceleration.x) > 15 || abs(a.acceleration.y) > 15 || abs(a.acceleration.z) > 15 ||
            abs(g.gyro.x) > 120 || abs(g.gyro.y) > 120 || abs(g.gyro.z) > 120) {
            if (!gyroTriggered) {
                Serial.println("Sudden movement detected - Vibration activated.");
                digitalWrite(vibrationPin, HIGH);
                gyroTriggered = true;
                vibrationStartTime = millis();
            }
        }

        if (gyroTriggered && ((millis() - vibrationStartTime) >= vibrationDuration)) {
            Serial.println("Vibration duration exceeded - Emergency activated");
            globMode = 1;
            activateEmergency(globMode);
        }

        if (sensor1State || sensor2State) {
            if (touchStartTime1 == 0) {
                touchStartTime1 = millis();
            } else if (millis() - touchStartTime1 >= touchDuration) {
                globMode = 0;
                activateEmergency(globMode);
            }
            if (touchStartTime2 == 0) {
                touchStartTime2 = millis();
            } else if (millis() - touchStartTime2 >= touchDuration) {
                globMode = 0;
                activateEmergency(globMode);
            }
        } else {
            touchStartTime1 = 0;
            touchStartTime2 = 0;
        }

        if (sensor1State && sensor2State) {
            Serial.println("Both touch sensors activated - Emergency deactivated");
            deactivateEmergency();
        }

        if (buzzerOn) {
            unsigned long currentTime = millis();
            if (currentTime - emergencyStartTime < buzzerDuration) {
        if (beepState && currentTime - lastBeepTime >= beepDuration) {
            ledcWriteTone(LEDC_CHANNEL_0, 0);
            beepState = false;
            lastBeepTime = currentTime;
        } else if (!beepState && currentTime - lastBeepTime >= beepInterval - beepDuration) {
            ledcWriteTone(LEDC_CHANNEL_0, 200);
            beepState = true;
            lastBeepTime = currentTime;
        }
    } else {
        // Turn off the buzzer after 20 seconds
        ledcWriteTone(LEDC_CHANNEL_0, 0);
        buzzerOn = false;
    }
        }

        if (emergencyActive) {
            controlLEDByLightSensor();
            
            if (!gpsSearching) {
              if ((millis() - msgTime) >= msgInterval) {
                gpsSearching = true;
                gpsStartTime = millis();
              }
            }
        }

        // delay(500);
        vTaskDelay(pdMS_TO_TICKS(100)); // Run every 100ms
    }
}

void setupI2S() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = BITS_PER_SAMPLE,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = DMA_BUF_COUNT,
        .dma_buf_len = DMA_BUF_LEN,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD
    };

    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_PORT, &pin_config);
}

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.println("WebSocket disconnected");
            break;
        case WStype_CONNECTED:
            Serial.println("WebSocket connected");
            break;
        default:
            break;
    }
}

void webSocketLoopTask(void *parameter) {
    esp_task_wdt_add(NULL);
    for(;;) {
        esp_task_wdt_reset();
        if(WiFi.status() == WL_CONNECTED) {
            webSocket.loop();
        } else {
            Serial.println("WiFi disconnected. Attempting to reconnect...");
            WiFi.begin(ssid, password);
            int attempts = 0;
            while (WiFi.status() != WL_CONNECTED && attempts < 20) {
                delay(500);
                Serial.print(".");
                attempts++;
            }
            if(WiFi.status() == WL_CONNECTED) {
                Serial.println("\nWiFi reconnected");
                webSocket.begin(wsServer, wsPort, "/");
            } else {
                Serial.println("\nFailed to reconnect WiFi");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void i2sReaderTask(void *parameter) {
    esp_task_wdt_add(NULL);
    static AudioChunk chunk;
    size_t bytesRead = 0;

    while (true) {
        esp_task_wdt_reset();
        if (isRecording) {
            i2s_read(I2S_PORT, chunk.data, AUDIO_CHUNK_SIZE, &bytesRead, portMAX_DELAY);
            if (bytesRead > 0) {
                chunk.size = bytesRead;
                if (xSemaphoreTake(audioBufferSemaphore, portMAX_DELAY) == pdTRUE) {
                    if (!audioBuffer.push(chunk)) {
                        audioBuffer.shift();
                        if (!audioBuffer.push(chunk)) {
                            Serial.println("Error: Cannot push to audio buffer");
                        }
                    }
                    xSemaphoreGive(audioBufferSemaphore);
                }
            }
        }
        vTaskDelay(1);
    }
}

void audioSenderTask(void *parameter) {
    const int chunksToSend = 4;
    while (true) {
        if (webSocket.isConnected() && isRecording) {
            if (xSemaphoreTake(audioBufferSemaphore, portMAX_DELAY) == pdTRUE) {
                for (int i = 0; i < chunksToSend && !audioBuffer.isEmpty(); i++) {
                    AudioChunk chunk = audioBuffer.shift();
                    webSocket.sendBIN(chunk.data, chunk.size);
                }
                xSemaphoreGive(audioBufferSemaphore);
            }
        }
        vTaskDelay(1);
    }
}

void automaticRecordingTask(void *parameter) {
    esp_task_wdt_add(NULL);
    const TickType_t xDelay = pdMS_TO_TICKS(100);
    
    while (true) {
        esp_task_wdt_reset();
        if(emergencyActive){
            isRecording = true;
            Serial.println("Recording started");
            webSocket.sendTXT("start_recording");

            for(int i = 0; i < 100; i++) {
                vTaskDelay(xDelay);
                esp_task_wdt_reset(); // Reset watchdog timer in the loop
            }

            isRecording = false;
            Serial.println("Recording stopped");
            webSocket.sendTXT("stop_recording");

            if (xSemaphoreTake(audioBufferSemaphore, portMAX_DELAY) == pdTRUE) {
                while (!audioBuffer.isEmpty()) {
                    audioBuffer.shift();
                    esp_task_wdt_reset(); // Reset watchdog timer while clearing buffer
                }
                xSemaphoreGive(audioBufferSemaphore);
            }

            resetI2S();
        }
        vTaskDelay(xDelay);
    }
}

void resetI2S() {
    i2s_stop(I2S_PORT);
    i2s_start(I2S_PORT);
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

void sendMsg1(int emg_mode, float lat, float lon) {
    HTTPClient https;
    WiFiClientSecure wifiClient;
    wifiClient.setInsecure();

    https.begin(wifiClient, "https://app.notify.lk/api/v1/send");
    https.addHeader("Content-Type", "application/x-www-form-urlencoded");

    String mapLink = generateMapLink(lat, lon);
    String receiverNumber = "94724043976";
    String httpRequestData = "user_id=27578&api_key=Db5y3HnbH1cxDFrjvuNO&sender_id=NotifyDEMO&to=contactNumber&message=MSG String";

    String msgString = (emg_mode == 0) ? "Emergency\n\n" : "Health emergency\n\n";
    msgString += mapLink;
    Serial.println(msgString); 
    
    httpRequestData.replace("MSG String", msgString);
    httpRequestData.replace("contactNumber", receiverNumber);
    int httpCode = https.POST(httpRequestData);
    Serial.println(httpCode);

    if (httpCode > 0) {
      String payload = https.getString();
      Serial.println(payload);
    } else {
      Serial.println("Error on HTTP request");
    }

    https.end();
}

void sendMsg2(int emg_mode, float lat, float lon, float accuracy) {
    HTTPClient https;
    WiFiClientSecure wifiClient;
    wifiClient.setInsecure();

    https.begin(wifiClient, "https://app.notify.lk/api/v1/send");
    https.addHeader("Content-Type", "application/x-www-form-urlencoded");

    String mapLink = generateMapLink(lat, lon);
    String receiverNumber = "94724043976";
    String httpRequestData = "user_id=27578&api_key=Db5y3HnbH1cxDFrjvuNO&sender_id=NotifyDEMO&to=contactNumber&message=MSG String";

    String msgString = (emg_mode == 0) ? "Emergency\n\n" : "Health emergency\n\n";
    msgString += mapLink;
    msgString += "\n\nAccuracy in meters: " + String(accuracy, 6);
    Serial.println(msgString); 
    
    httpRequestData.replace("MSG String", msgString);
    httpRequestData.replace("contactNumber", receiverNumber);
    int httpCode = https.POST(httpRequestData);
    Serial.println(httpCode);

    if (httpCode > 0) {
      String payload = https.getString();
      Serial.println(payload);
    } else {
      Serial.println("Error on HTTP request");
    }

    https.end();
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

void activateEmergency(int mode) {
    emergencyActive = true;
    // digitalWrite(vibrationPin, HIGH);
    digitalWrite(vibrationPin, LOW);
    vibrationStartTime = millis();

    buzzerOn = true; 
    lastBeepTime = millis();
    beepState = true;
    emergencyStartTime = millis();

    digitalWrite(cameraPin, HIGH);
    gyroTriggered = false;
}

void deactivateEmergency() {
    // digitalWrite(vibrationPin, LOW);
    ledcWriteTone(LEDC_CHANNEL_0, 0);
    buzzerOn = false;
    beepState = false;
    digitalWrite(ledPin, LOW);
    digitalWrite(cameraPin, LOW);
    webSocket.sendTXT("emergency_deactivated");
    emergencyActive = false;
    gpsSearching = false;
}

void controlLEDByLightSensor() {
    float lux = lightMeter.readLightLevel();
    Serial.print("Light Intensity: ");
    Serial.print(lux);
    Serial.println(" lux");
    digitalWrite(ledPin, lux < lightThreshold ? HIGH : LOW);
}
