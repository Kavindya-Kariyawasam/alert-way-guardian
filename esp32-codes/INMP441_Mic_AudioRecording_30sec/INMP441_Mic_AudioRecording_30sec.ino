#include <Arduino.h>
#include <driver/i2s.h>
#include <WiFi.h>
#include <ArduinoWebsockets.h>

using namespace websockets;

// WiFi credentials
const char *ssid = "Mano";
const char *password = "12345678";

// WebSocket server details
const char *wsServer = "192.168.242.232";
const uint16_t wsPort = 8080;

// Audio configuration
#define SAMPLE_RATE       44100
#define BITS_PER_SAMPLE   I2S_BITS_PER_SAMPLE_16BIT
#define I2S_PORT          I2S_NUM_0
#define DMA_BUF_COUNT     8
#define DMA_BUF_LEN       512  // Reduced buffer size

// INMP441 Pin assignment
#define I2S_SD            33
#define I2S_WS            25
#define I2S_SCK           32

// Global objects
WebsocketsClient webSocket;
SemaphoreHandle_t audioSemaphore;

// Recording state
volatile bool isRecording = false;

// Audio buffer
uint8_t audioBuffer[DMA_BUF_LEN * 2];

// Debugging variables
unsigned long totalBytesSent = 0;
unsigned long lastPrintTime = 0;

// Function prototypes
void setupWiFi();
void setupI2S();
void onWebSocketEvent(WebsocketsEvent event, String data);
void i2sReaderTask(void *parameter);
void audioSenderTask(void *parameter);
void automaticRecordingTask(void *parameter);

void setup() {
    Serial.begin(115200);
    
    audioSemaphore = xSemaphoreCreateBinary();
    
    setupWiFi();
    setupI2S();
    
    webSocket.onEvent(onWebSocketEvent);
    webSocket.connect(wsServer, wsPort, "/");
    
    xTaskCreatePinnedToCore(i2sReaderTask, "I2S Reader", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(audioSenderTask, "Audio Sender", 8192, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(automaticRecordingTask, "Auto Recording", 2048, NULL, 2, NULL, 0);
}

void loop() {
    if (webSocket.available()) {
        webSocket.poll();
    }
    
    // Print debug info every 5 seconds
    if (millis() - lastPrintTime > 5000) {
        Serial.printf("Total bytes sent: %lu\n", totalBytesSent);
        lastPrintTime = millis();
    }
    
    delay(1);  // Small delay to prevent watchdog triggering
}

// ... (rest of the functions remain the same)
void setupWiFi() {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
}

void setupI2S() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
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

void onWebSocketEvent(WebsocketsEvent event, String data) {
    if (event == WebsocketsEvent::ConnectionOpened) {
        Serial.println("WebSocket connected");
    } else if (event == WebsocketsEvent::ConnectionClosed) {
        Serial.println("WebSocket disconnected");
    }
}

void i2sReaderTask(void *parameter) {
    size_t bytesRead = 0;
    while (true) {
        if (isRecording) {
            i2s_read(I2S_PORT, audioBuffer, DMA_BUF_LEN * 2, &bytesRead, portMAX_DELAY);
            if (bytesRead > 0) {
                xSemaphoreGive(audioSemaphore);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void audioSenderTask(void *parameter) {
    while (true) {
        if (xSemaphoreTake(audioSemaphore, portMAX_DELAY) == pdTRUE) {
            if (webSocket.available() && isRecording) {
                if (webSocket.sendBinary((const char*)audioBuffer, DMA_BUF_LEN * 2)) {
                    totalBytesSent += DMA_BUF_LEN * 2;
                } else {
                    Serial.println("Failed to send audio data");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));  // Short delay to prevent watchdog triggering
    }
}

void automaticRecordingTask(void *parameter) {
    vTaskDelay(pdMS_TO_TICKS(5000));  // Wait for 5 seconds after booting

    while (true) {
        isRecording = true;
        totalBytesSent = 0;  // Reset counter
        Serial.println("Recording started");
        webSocket.send("start_recording");

        vTaskDelay(pdMS_TO_TICKS(30000));  // Record for 30 seconds

        isRecording = false;
        Serial.println("Recording stopped");
        webSocket.send("stop_recording");
        Serial.printf("Total bytes sent in this session: %lu\n", totalBytesSent);

        vTaskDelay(pdMS_TO_TICKS(5000));  // Wait for 5 seconds before next recording
    }
}