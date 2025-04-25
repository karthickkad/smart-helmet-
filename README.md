# IoT-Based Smart Helmet System 

This is an IoT-based Smart Helmet System designed for rider safety. It includes sensors for accident detection, alcohol detection, real-time GPS tracking, and communication with a bike unit. The system uses **ESP32** microcontrollers to enable efficient communication and functionality.

## Features
- **Accident Detection**: The helmet detects vibration from accidents using a vibration sensor and triggers an alert to the bike unit.
- **Alcohol Detection**: The MQ-3 sensor detects alcohol levels in the rider's breath to ensure safe riding.
- **GPS Tracking**: The helmet unit tracks the rider's location using a GPS module, and in case of an accident, the coordinates are sent as part of the SMS alert.
- **SMS Alerts**: In case of an accident, the helmet unit sends an SMS alert with the rider's location and emergency information via a GSM module.
- **Communication**: The helmet unit communicates with the bike unit via **ESP-NOW** to transmit data about accident detection, alcohol levels, and helmet status.
- **Geolocation & Emergency Response**: Provides the location via GPS in case of an accident and sends SMS notifications to predefined contacts.

## Components
### Helmet Unit (ESP32)
- **GPS Module** (e.g., NEO-6M, u-blox): Tracks rider's location in real-time.
- **GSM Module** (e.g., SIM800L): Sends SMS alerts to emergency contacts.
- **Vibration Sensor**: Detects vibration from accidents.
- **MQ-3 Gas Sensor**: Detects alcohol levels.
- **IR Sensor**: Detects if the helmet is being worn.
- **ESP32**: Main microcontroller for handling sensor data and communication.

### Bike Unit (ESP32 or LilyGo T-Display)
- **SMS Alert System**: Sends SMS alerts to emergency contacts when an accident is detected.
- **LED Control**: Activates visual alerts (LEDs) and motor when an accident occurs.
- **ESP32**: Main microcontroller for communication with the helmet unit.

## Getting Started

### Prerequisites
- **Hardware**:
  - 2 × ESP32 or compatible boards
  - GPS Module (e.g., NEO-6M)
  - GSM Module (e.g., SIM800L)
  - MQ-3 Gas Sensor
  - Vibration Sensor
  - IR Sensor
  - ESP32 (optional for the bike unit)
  - Jumper wires and other electronic components

- **Software**:
  - Arduino IDE or PlatformIO for ESP32 development
  - Fast2SMS API for sending SMS alerts
  - ESP32 Board Support installed in the Arduino IDE

### Installation
1. **Clone the repository**:
   ```bash
   git clone https://github.com/username/smart-helmet-iot.git
   cd smart-helmet-iot


