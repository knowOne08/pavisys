# PAVI Flight Computer System

A comprehensive LoRa-based flight data logging and control system for parachute testing and experimental aviation. The system consists of two ESP32 modules: a ground station (RX) and flight computer (TX) that communicate via LoRa radio for real-time monitoring and control.

## 🚀 Project Overview

PAVI is a dual-module flight system designed for parachute drop testing and experimental flight data collection. The system provides real-time telemetry, flight data logging, remote pyrotechnic control, and web-based data recovery capabilities.

### Key Features

- **Dual-Module Architecture**: Ground Station (RX) and Flight Computer (TX)
- **LoRa Communication**: Long-range, reliable radio communication (433MHz)
- **Multi-Sensor Data Logging**: Barometer, IMU, Load Cell integration
- **Real-Time Flight Control**: Remote pyrotechnic channel control
- **Web Interface**: WiFi-based file download and system management
- **Robust Data Storage**: LittleFS filesystem with comprehensive logging
- **Interactive Menu System**: User-friendly operation interface
- **Configurable Data Rates**: 10Hz to 40Hz sensor sampling

## 📁 Project Structure

```
pavi/
├── README.md                           # This file
├── RX_Pavi/
│   └── RX_Pavi/
│       └── RX_Pavi.ino                # Ground Station firmware
├── TX_Pavi/
│   └── TX_Pavi.ino                    # Flight Computer firmware
├── test/
│   └── test/
│       └── test.ino                   # Hardware testing utilities
└── Documentation/                      # Additional documentation files
    ├── CRITICAL_LORA_QUEUE_FIX.md
    ├── ESP32_C3_CRASH_FIXES.md
    ├── INTUITIVE_COMMAND_SYSTEM.md
    ├── RX_TX_SYNC_FIXES.md
    ├── TEST_WORKFLOW.md
    ├── TX_COMPILATION_FIXES.md
    ├── WIFI_SOFTAP_IMPLEMENTATION.md
    └── WEB_INTERFACE_GUIDE.md
```

## 🛠 Hardware Requirements

### Ground Station (RX) Components
- **ESP32 Development Board** (any variant)
- **LoRa Module** (SX1276/SX1278 compatible, 433MHz)
- **OLED Display** (SSD1306, 128x64)
- **Power Supply** (USB or battery pack)

### Flight Computer (TX) Components
- **ESP32 Development Board** (recommended: ESP32-WROOM-32)
- **LoRa Module** (SX1276/SX1278 compatible, 433MHz)
- **MS5611 Barometric Pressure Sensor**
- **MPU6050 IMU** (Accelerometer + Gyroscope)
- **HX711 Load Cell Amplifier + Load Cell**
- **4x MOSFET Circuits** for pyrotechnic channels
- **MicroSD Card** (optional, LittleFS used by default)
- **Li-Po Battery** (recommended: 2S 7.4V, 2200mAh+)

## 🔌 Pin Connections

### Ground Station (RX) Pinout
```
ESP32 Pin    Component           Notes
---------    ---------           -----
GPIO5        LoRa NSS           SPI Chip Select
GPIO17       LoRa RST           Reset Pin
GPIO13       LoRa DIO0          Interrupt Pin
GPIO18       LoRa SCK           SPI Clock
GPIO19       LoRa MISO          SPI Data In
GPIO23       LoRa MOSI          SPI Data Out
GPIO21       OLED SDA           I2C Data
GPIO22       OLED SCL           I2C Clock
3.3V         VCC (LoRa/OLED)    Power Supply
GND          GND                Ground
```

### Flight Computer (TX) Pinout
```
ESP32 Pin    Component           Notes
---------    ---------           -----
GPIO5        LoRa NSS           SPI Chip Select
GPIO17       LoRa RST           Reset Pin
GPIO13       LoRa DIO0          Interrupt Pin
GPIO18       LoRa SCK           SPI Clock
GPIO19       LoRa MISO          SPI Data In
GPIO23       LoRa MOSI          SPI Data Out
GPIO21       I2C SDA            Sensors (MS5611, MPU6050)
GPIO22       I2C SCL            Sensors (MS5611, MPU6050)
GPIO4        HX711 DOUT         Load Cell Data
GPIO16       HX711 SCK          Load Cell Clock
GPIO33       Pyro Channel 1     MOSFET Control
GPIO32       Pyro Channel 2     MOSFET Control
GPIO25       Pyro Channel 3     MOSFET Control
GPIO26       Pyro Channel 4     MOSFET Control
3.3V         VCC (Sensors)      Power Supply
GND          GND                Ground
```

## 📚 Software Dependencies

### Arduino Libraries Required
```
// Core ESP32 libraries (included)
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <SPI.h>
#include "FS.h"
#include "LittleFS.h"

// Third-party libraries (install via Library Manager)
#include <LoRa.h>               // LoRa communication
#include <MS5611.h>             // Barometric pressure sensor
#include <MPU6050.h>            // IMU (accelerometer/gyroscope)
#include <HX711.h>              // Load cell amplifier
#include <Adafruit_SSD1306.h>   // OLED display (RX only)
#include <Adafruit_GFX.h>       // Graphics library (RX only)
```

### Installation Steps
1. Install Arduino IDE (1.8.x or 2.x)
2. Add ESP32 board support:
   - File → Preferences → Additional Board Manager URLs
   - Add: `https://dl.espressif.com/dl/package_esp32_index.json`
3. Install ESP32 boards via Board Manager
4. Install required libraries via Library Manager

## ⚙️ Configuration

### System Configuration Options

#### TX (Flight Computer) Configuration

**Easy Flight Computer Selection:**
```cpp
// Change this single macro to switch between two FC builds (TX_Pavi.ino)
#define FC_NO 1  // 1 = FC1 (20Hz, "PaviFlightData"), 2 = FC2 (40Hz, "PaviFlightData-2")
```

This automatically configures:
- **FC1**: 20Hz data rate + "PaviFlightData" WiFi SSID
- **FC2**: 40Hz data rate + "PaviFlightData-2" WiFi SSID

**Other Configuration Options:**
```cpp
// Sensor enable/disable
bool ENABLE_BAROMETER = true;
bool ENABLE_ACCELEROMETER = true;
bool ENABLE_GYROSCOPE = true;
bool ENABLE_TEMPERATURE = false;
bool ENABLE_LOAD_CELL = true;

// Test modes
#define SERIAL_TEST_MODE false  // Set true for testing without hardware
#define PYRO_TEST_MODE false    // Set true for LED testing instead of pyro

// WiFi SoftAP settings (auto-configured by FC_NO)
#define WIFI_PASSWORD ""        // Open network by default
```

#### RX (Ground Station) Configuration
```cpp
// Display settings (RX_Pavi.ino)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// LoRa configuration (both modules)
Frequency: 433MHz
Spreading Factor: SF7
Bandwidth: 125kHz
Coding Rate: 4/5
TX Power: 20dBm
```

## 🚁 System Operation

### Pre-Flight Setup
1. **Power On Both Modules**
   - Ground Station (RX): Connect via USB or battery
   - Flight Computer (TX): Install in test vehicle with battery

2. **Verify LoRa Connection**
   - RX: Send `PING` command
   - TX: Should respond with `PONG`

3. **Configure Flight Parameters**
   - Use RX menu system or send individual commands
   - Required: Filename, Weight, Wind Speed, Initial Height

### Flight Operation Sequence

#### Method 1: Interactive Menu System (Recommended)
```
RX Ground Station:
1. Type 'MENU' in serial console
2. Select "2. Flight Configuration"
3. Choose "1. Set All Parameters (Quick Setup)"
4. Enter: filename, weight, wind speed, height
5. Select "3. Flight Operations" → "1. Enter FLIGHT Mode"
6. Confirm configuration and enter FLIGHT mode
7. Send flight commands as needed
```

#### Method 2: Direct LoRa Commands
```
RX Commands → TX Responses:
CONFIG_START     → Configuration mode activated
FILENAME:test1   → Filename set
WEIGHT:2.5      → Weight set to 2.5 kg
WIND:5.2        → Wind speed set to 5.2 m/s  
HEIGHT:100      → Height set to 100 m
CONFIG_READY    → Configuration complete, data file created
FLIGHT_START    → Recording started, sensor origins reset
PYRO1          → Pyro channel 1 fired
FLIGHT_STOP    → Recording stopped, file saved
```

### Data Recovery
1. **Via WiFi Interface** (Recommended)
   - Send `WIFI_START` command to TX
   - Connect device to "PaviFlightData" network
   - Navigate to `http://192.168.4.1`
   - Download flight data files

2. **Via Serial Interface**
   ```
   FILES           # List all stored files
   DOWNLOAD test1.txt  # Download specific file
   DELETE old.txt  # Delete unwanted files
   SPACE          # Check storage usage
   ```

## 📊 Data Format

### Flight Data File Structure
```
# Header (commented lines with #)
- Test configuration (filename, weight, wind, height)
- System information (start time, data rate, chip ID)
- Column definitions

# CSV Data Columns:
Time_ms,Event_Type,Altitude_m,Vertical_Velocity_ms,Load_Cell_kg,
Accel_X_ms2,Accel_Y_ms2,Accel_Z_ms2,Gyro_X_rads,Gyro_Y_rads,Gyro_Z_rads,Notes

# Sample Data:
0,OFFSET_EVENT,0.00,0.00,0.15,0.12,-0.05,9.81,0.001,-0.002,0.000,Sensor zero points reset (before flight start)
1250,DATA,2.34,1.20,0.18,0.15,-0.08,10.2,0.012,-0.008,0.003,
2500,DATA,5.67,2.45,0.22,0.28,-0.12,9.95,0.018,-0.015,0.008,
5000,PYRO1,12.45,4.32,0.35,2.45,-0.89,8.12,0.125,-0.078,0.045,Pyro channel 1 fired
```

### Data Units and Precision
- **Time**: Milliseconds from recording start
- **Altitude**: Meters (2 decimal places)
- **Velocity**: m/s (2 decimal places)  
- **Weight**: Kilograms (2 decimal places)
- **Acceleration**: m/s² (2 decimal places)
- **Angular Velocity**: rad/s (3 decimal places)

## 🔧 Troubleshooting

### Common Issues

#### LoRa Communication Problems
```
Symptoms: No response to PING, commands not received
Solutions:
- Check antenna connections
- Verify frequency (433MHz)
- Check power supply voltage (3.3V)
- Ensure both modules use same LoRa settings
- Check serial monitor for error messages
```

#### Sensor Reading Issues
```
MS5611 Barometer:
- Check I2C wiring (SDA=21, SCL=22)
- Verify 3.3V power supply
- Look for "MS5611 not detected!" message

MPU6050 IMU:
- Check I2C address (0x68 default)
- Verify power and ground connections
- Check for I2C bus conflicts

HX711 Load Cell:
- Verify DOUT and SCK connections
- Check load cell wiring (red=+, black=-, white/green=signal)
- Ensure proper calibration factor
```

#### File System Problems
```
Symptoms: "LittleFS initialization failed"
Solutions:
- Format filesystem using FORMAT command
- Check available flash memory
- Verify ESP32 partition table
- Use different ESP32 board if persistent
```

### Debug Commands
```
# System diagnostics
STATUS          # Full system status
MENU → 4 → 5   # Sensor diagnostics via menu
SERIALTOGGLE   # Enable detailed sensor debug output
SPACE          # Check filesystem usage

# Hardware testing  
MENU → 4 → 1   # Test load cell readings
MENU → 4 → 2   # Zero load cell (tare)
MENU → 4 → 3   # Test pyro channels (LED mode)
```

## 🔒 Safety Considerations

### Pyrotechnic Safety
- **NEVER** connect actual pyrotechnics during testing
- Use LEDs or multimeter for circuit validation
- Set `PYRO_TEST_MODE = true` for safe testing
- Implement proper isolation and safety switches
- Follow local regulations for pyrotechnic devices

### Flight Safety  
- Always maintain visual contact with test vehicle
- Use appropriate recovery systems (parachutes)
- Follow local aviation regulations
- Conduct ground tests before flight
- Have emergency procedures in place

### Electrical Safety
- Use appropriate fuses and protection circuits
- Verify battery voltage and current ratings
- Implement proper grounding
- Use quality connectors and wiring

## 📈 Performance Specifications

### Data Logging Performance
- **Maximum Sample Rate**: 40Hz (25ms interval)
- **Load Cell Rate**: 5-8Hz (smart sampling)
- **Storage Capacity**: ~1.5MB (8-10 minutes @ 40Hz)
- **File Format**: Human-readable CSV with headers
- **Precision**: 2-3 decimal places (optimized)

### LoRa Communication
- **Frequency**: 433MHz ISM band
- **Range**: ~2km line-of-sight (theoretical)
- **Data Rate**: Variable (command-response)
- **Reliability**: CRC error checking enabled
- **Latency**: <100ms typical command response

### Power Consumption
- **TX Active**: ~200-300mA (all sensors + LoRa)
- **RX Active**: ~150-200mA (LoRa + OLED)
- **Battery Life**: 3-6 hours (2200mAh Li-Po)

## 🎯 Advanced Features

### Menu System Features
- **Interactive Navigation**: Hierarchical menu structure
- **Mode-Aware Operation**: Automatic mode switching
- **Parameter Validation**: Real-time input checking
- **Quick Setup**: Single-flow configuration entry
- **Safety Confirmations**: Critical operation warnings

### Web Interface Features
- **File Management**: Download, delete, list files
- **System Monitoring**: Real-time status display
- **Storage Analytics**: Space usage and file statistics
- **Remote Access**: WiFi SoftAP for field operations

### Data Analysis Features
- **Timestamp Correlation**: Millisecond precision logging
- **Event Marking**: Pyro firing, offset resets logged
- **Multi-Sensor Fusion**: Combined velocity calculations
- **Filter Processing**: Moving average + exponential smoothing

## 🚧 Future Enhancements

### Planned Features
- [ ] GPS integration for position logging
- [ ] Real-time telemetry streaming
- [ ] Mobile app interface
- [ ] Data visualization dashboard
- [ ] Multi-vehicle support
- [ ] Encrypted communications
- [ ] Weather station integration

### Hardware Expansions
- [ ] External SD card support
- [ ] Additional sensor inputs
- [ ] Video recording integration
- [ ] Backup communication methods
- [ ] Solar charging capability

## 📄 License

This project is released under the MIT License. See LICENSE file for details.

## 🤝 Contributing

Contributions are welcome! Please read CONTRIBUTING.md for guidelines on:
- Code style and standards
- Testing procedures
- Documentation requirements
- Pull request process

## 📞 Support

For questions, issues, or contributions:
- Create an issue on GitHub
- Check existing documentation files
- Review troubleshooting section
- Join project discussions

## ⚠️ Disclaimer

This system is designed for experimental and educational purposes. Users are responsible for:
- Compliance with local regulations
- Safety procedures and protocols  
- Proper testing and validation
- Risk assessment and mitigation

The developers assume no liability for damages or injuries resulting from use of this system.

---

**Project Status**: Active Development  
**Last Updated**: December 2024  
**Version**: 2.0.0  
**Compatibility**: Arduino IDE 1.8.x, 2.x | ESP32 Core 2.0.x+
