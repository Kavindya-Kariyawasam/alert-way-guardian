# AlertWay Guardian - Women and Child Safety Device

In response to rising safety concerns for women and children, especially in urban areas and during solo travel, our project aimed to create a cost-effective, cutting-edge microcontroller-driven device designed to enhance personal safety and reduce risk.

The device includes a variety of sensors to monitor surroundings and, in emergencies, automatically alerts a designated contact. A user-friendly panic button enables real-time data sharing with an emergency contact while triggering a loud alarm to deter potential threats. Additionally, it captures video and audio and uploads it to Azure Cloud in real time in an emergency.

[View Project Report on Google Drive(suggested)](https://drive.google.com/file/d/15ZoKAmLVrlMV1tzM6Qf1nSS_WbsrMQF1/view?usp=sharing)
[View Project Report on GitHub](report/project-report-alert-way-guardian.pdf)


### Key Components and Features:

- ESP32 WROOM 32U: Main development board
- Touch Sensors: Emergency activation/deactivation
- GPS Module: Provides immediate location data
- Accelerometer: Detects sudden movements or falls
- Microphone: Captures audio in real-time
- ESP32 Camera Module: Captures images for video generation
- Light-Intensity Sensor: Provide data to control LEDs
- Vibration Motor: Provides signals to user on activation
- Buzzer Module: Sounds in emergency situations

### Kudos to our team:

- [Shehan Lokuwella (Team Leader)](https://github.com/Shehan013)
- Dineth Gamage
- [Lasini Pallewatte](https://github.com/lasiniip)
- [Manodi Gamage](https://github.com/manodi-gamage)
- [Kavindya Kariyawasam](https://github.com/Kavindya-Kariyawasam)


#### How to Setup the Environment Variables for the Microphone WebSocket Server

1. Create a `.env` file in the root directory of the project:

```bash
touch .env
```

2. Add the following configurations to your `.env` file:

```plaintext
# Azure Storage Configuration
AZURE_STORAGE_CONNECTION_STRING=your_azure_storage_connection_string
AZURE_CONTAINER_NAME=your_container_name

# Server Configuration
SERVER_HOST=0.0.0.0
SERVER_PORT=8080
```

##### Environment Variables Reference

| Variable                          | Description                             | Required | Default   |
| --------------------------------- | --------------------------------------- | -------- | --------- |
| `AZURE_STORAGE_CONNECTION_STRING` | Azure Storage account connection string | Yes      | None      |
| `AZURE_CONTAINER_NAME`            | Name of the Azure Storage container     | Yes      | None      |
| `SERVER_HOST`                     | Host address for the WebSocket server   | No       | '0.0.0.0' |
| `SERVER_PORT`                     | Port number for the WebSocket server    | No       | 8080      |  

---  

## GPS Emergency Location Tracking System

## Overview
This project implements an emergency location tracking system using an ESP32 microcontroller and NEO-8M GPS module. The system is designed to send location-based emergency alerts through SMS, with a fallback to IP-based geolocation when GPS signals are unavailable.

## Features
- Real-time GPS location tracking
- Automatic fallback to Google Geolocation API when GPS signal is weak/unavailable
- Emergency message dispatch with location links
- Configurable timeout and messaging intervals
- Support for different emergency modes (general emergency/health emergency)
- Google Maps integration for easy location sharing

## Hardware Components
- ESP32 Development Board
- NEO-8M GPS Module
- Required connecting wires

### About NEO-8M GPS Module
The NEO-8M is a high-performance GPS module that offers significant improvements over its predecessor, the NEO-6M. Key specifications:
- Operating voltage: 2.7V to 3.6V
- Navigation update rate: up to 10Hz
- Position accuracy: 2.0m CEP (Circular Error Probable)
- Time to First Fix:
  - Cold start: 26s
  - Hot start: <1s
  - Aided start: 2s
- Sensitivity:
  - Tracking & Navigation: -167 dBm
  - Reacquisition: -160 dBm
  - Cold start: -148 dBm
- Operating temperature: -40°C to 85°C
- Improved multipath detection and suppression
- Superior jam detection and mitigation capabilities

## Software Dependencies
```cpp
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Arduino.h>
#include <ArduinoJson.h>
```

## Working Principle
1. **Primary GPS Location Detection**:
   - System continuously monitors GPS signals through the NEO-8M module
   - Location updates are processed every 8 seconds (configurable)
   - Valid coordinates trigger immediate emergency message dispatch

2. **Fallback Mechanism**:
   - If GPS fails to get a fix within 6 minutes (configurable)
   - System switches to Google Geolocation API
   - Uses WiFi BSSID and signal strength for location approximation

3. **Message Dispatch**:
   - Generates Google Maps link with coordinates
   - Formats emergency message based on emergency type
   - Sends SMS through notify.lk API (currently in simulation mode) [Code](esp32-codes/Gps_MsgAPI_GeoLoc_CompleteFunctionality)
   - Includes accuracy metrics when using IP-based location

## Configuration
### Required Setup
```cpp
const char* ssid = "<SSID>";              // WiFi SSID
const char* password = "<PASSWORD>";       // WiFi password
const char* apiKey = "<API_KEY>";         // Google Geolocation API key
```

### Pin Configuration
```cpp
#define gpsRxPin 16                       // GPS module RX pin
#define gpsTxPin 17                       // GPS module TX pin
```

### Timing Parameters
```cpp
const unsigned long gpsTimeout = 360000;   // GPS timeout (6 minutes)
const unsigned long msgInterval = 8000;    // Message interval (8 seconds)
```

## Installation & Setup
1. Install required Arduino libraries
2. Connect NEO-6M GPS module to ESP32:
   - GPS VCC → 3.3V
   - GPS GND → GND
   - GPS TX → GPIO16 (RX2)
   - GPS RX → GPIO17 (TX2)
3. Configure WiFi credentials
4. Add Google Geolocation API key
5. Configure notify.lk API credentials (when using actual SMS service)

## Location Format
The system generates Google Maps links in the format:
```
https://www.google.com/maps/place/[latitude][N/S][longitude][E/W]/
```

## Error Handling
- GPS timeout management
- WiFi connection monitoring
- HTTP request error handling
- JSON parsing error detection

## Future Improvements
- Battery monitoring capability
- Multiple emergency contact support
- Custom emergency message templates
- Power optimization features

## Notes
- Current implementation uses simulated message sending for testing
- Original SMS implementation through notify.lk is preserved in comments
- System designed for emergency use - consider reliability and power backup
- GPS accuracy may vary based on environmental conditions

## Technical Considerations
- NEO-8M requires clear sky view for optimal performance
- Initial GPS fix may take longer in cold start conditions
- WiFi-based geolocation accuracy depends on database coverage
- Consider local privacy laws when implementing location tracking

## Credits
### GPS Module Integration & Location Services
  - ** Developer**: [Kavindya Kariyawasam](https://github.com/Kavindya-Kariyawasam) 
  - *Contributions*:
    - Complete GPS module integration
    - Location tracking system implementation
    - Google Geolocation API fallback system
    - Emergency messaging system
    - Documentation and testing
