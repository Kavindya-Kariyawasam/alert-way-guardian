# AlertWay Guardian - Women and Child Safety Device

In response to rising safety concerns for women and children, especially in urban areas and during solo travel, our project aimed to create a cost-effective, cutting-edge microcontroller-driven device designed to enhance personal safety and reduce risk.

The device includes a variety of sensors to monitor surroundings and, in emergencies, automatically alerts a designated contact. A user-friendly panic button enables real-time data sharing with an emergency contact while triggering a loud alarm to deter potential threats. Additionally, it captures video and audio and uploads it to Azure Cloud in real time in an emergency.

[View Project Report](report/project-report-alert-way-guardian.pdf)


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
