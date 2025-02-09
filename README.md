# ESP8266 Water Flow Monitoring System

This project is an implementation of a water flow monitoring system using an ESP8266 microcontroller. It calculates the water flow rate and total volume based on pulses received from a flow sensor. The system supports multiple sensors, can reset filter data, and serve data via a web interface.

## Features

- **Flow Measurement**: Measures water flow and total volume.
- **Multiple Sensor Support**: Supports different flow sensors (YF-G1, FS300A, FS400).
- **Web Interface**: Provides a web interface for monitoring and resetting data.
- **MQTT Support**: Publishes data to an MQTT server.
- **Persistent Storage**: Stores data in EEPROM and configuration in LittleFS.

## Components

- ESP8266 Microcontroller
- Flow Sensors (YF-G1, FS300A, FS400)
- EEPROM for data storage
- LittleFS for configuration storage
- NTP client for time synchronization

## Hardware Setup

1. Connect the flow sensor to the ESP8266:
   - Sensor VCC to ESP8266 3.3V
   - Sensor GND to ESP8266 GND
   - Sensor OUT to ESP8266 D2 (or any other GPIO pin, adjust the code accordingly)

## Software Setup

### Prerequisites

- [PlatformIO](https://platformio.org/)
- [ArduinoJson Library](https://arduinojson.org/)
- [ESPAsyncWebServer Library](https://github.com/me-no-dev/ESPAsyncWebServer)
- [ESP8266WiFi Library](https://github.com/esp8266/Arduino)
- [LittleFS Library](https://github.com/earlephilhower/LittleFS)

### Installation

1. Clone this repository.
2. Ensure all dependencies are installed.
3. Place the `config.json`, `index.html`, and `style.css` files in the `data` directory.

### Configuration

Create a `config.json` file in the `data` directory with the following content:

```json
{
  "sensors": [
    {
      "name": "YF-G1",
      "calibrationFactor": 64.8,
      "correctionFactor": 1.08
    },
    {
      "name": "FS300A",
      "calibrationFactor": 73.0,
      "correctionFactor": 1.0
    },
    {
      "name": "FS400",
      "calibrationFactor": 80.0,
      "correctionFactor": 1.0
    }
  ]
}
```

## Upload Filesystem
Upload the LittleFS filesystem to the ESP8266:
```bash
pio run --target uploadfs
```

## Upload Firmware
```bash
pio run --target upload
```
## Web Interface
Access the web interface by navigating to the IP address of the ESP8266 in a web browser. The web interface displays filter status and allows resetting filter data.

## MQTT
The system publishes data to an MQTT server. Configure the MQTT server in the config.h file.

## Code Overview

main.cpp
The main code file where the core functionality is implemented:

Setup Function: Initializes the system, sets up Wi-Fi, and loads configuration.
Loop Function: Handles the main logic, including flow calculation and data publishing.
ISR Function: pulseCounter counts pulses from the flow sensor.
Helper Functions: Various functions for handling EEPROM, configuration, and data calculations.
config.json: Configuration file for sensor calibration and correction factors.
index.html: Web interface for displaying data and resetting filters.
style.css: Stylesheet for the web interface.

## Contributing

Feel free to open issues or submit pull requests for improvements and bug fixes.

## License

This project is licensed under the MIT License.

## Acknowledgements

PlatformIO
ArduinoJson
ESPAsyncWebServer
LittleFS
YF-G1 https://gb.lightmalls.com/yf-g1-plastic-water-flow-dn25-hall-sensor-flowmeter-counter-black?gad_source=1&gbraid=0AAAAADt5-bJjC6fGgppMviggf0I_WqVJh&gclid=EAIaIQobChMIsbegnIGqhwMVRZRQBh2zzgyrEAQYBCABEgK9i_D_BwE