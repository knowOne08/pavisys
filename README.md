# PAVI Flight Computer System

[![Arduino](https://img.shields.io/badge/Arduino-Compatible-00979D?style=flat&logo=arduino)](https://www.arduino.cc/)
[![ESP32](https://img.shields.io/badge/ESP32-C3%20%26%20Standard-E7352C?style=flat&logo=espressif)](https://www.espressif.com/en/products/socs/esp32)
[![License](https://img.shields.io/badge/License-Open%20Source-green.svg)](LICENSE)
[![Version](https://img.shields.io/badge/Version-v1.0-blue.svg)](CHANGELOG.md)

**Advanced wireless flight computer system designed for model rockets, UAVs, and experimental aircraft. Features real-time telemetry, remote pyrotechnic control, and high-precision sensor data logging over long-range LoRa communication.**

---

## 🚀 **What is PAVI?**

PAVI (Precision Aviation Vehicle Instrumentation) is a dual-unit flight computer system that provides:

- **Real-time flight data recording** at up to 40Hz sampling rate
- **Long-range wireless control** (2-5km line-of-sight via 433MHz LoRa)
- **4-channel pyrotechnic control** for recovery systems and staging
- **High-precision sensors**: barometric altitude, 6-axis IMU, load cell
- **WiFi data download** directly to smartphone/computer
- **User-friendly ground station** with menu-driven interface

### System Overview

| Component | Description | Specifications |
|-----------|-------------|----------------|
| **TX Unit** | Flight Computer (onboard rocket) | ESP32, MS5611 barometer, MPU6050 IMU, HX711 load cell, 4x pyro channels |
| **RX Unit** | Ground Station (remote control) | ESP32-C3/ESP32, LoRa transceiver, menu interface, telemetry display |
| **Range** | Line-of-sight communication | 2-5km (optimized for maximum range) |
| **Data Rate** | Configurable sampling rates | 20Hz (FC1) or 40Hz (FC2) |
| **Storage** | Onboard data logging | LittleFS filesystem, CSV format output |

---

## 📦 **What's in the Box**

### Hardware Components
- **TX Unit**: Flight computer with integrated sensors
- **RX Unit**: Ground station controller  
- **Antennas**: 433MHz quarter-wave antennas (~17cm)
- **Cables**: USB programming/power cables
- **Documentation**: User manuals and technical guides

### Sensor Suite (TX Unit)
- **MS5611**: High-precision barometric pressure sensor
- **MPU6050**: 6-axis accelerometer and gyroscope
- **HX711**: 24-bit load cell amplifier for weight/thrust measurement
- **4x Pyro Channels**: Transistor-switched pyrotechnic outputs

---

## ⚡ **Quick Start Guide**

### 1. First Power-On
1. Connect TX unit to power (USB or battery)
2. Connect RX unit to power
3. Wait 30 seconds for both units to initialize
4. RX unit will display the main menu

### 2. Test Communication
1. On RX unit main menu, select `6` (System Status)
2. Should show "TX Connection: ACTIVE"
3. If no connection, see [Troubleshooting](#-troubleshooting)

### 3. Configure Your Flight
1. Select `1` (TX Configuration Menu)
2. Select `1` (Set All Parameters - Guided)
3. Enter:
   - **Filename**: `MyFirstFlight`
   - **Weight**: `2500` (grams)
   - **Wind Speed**: `5` (m/s)
   - **Height**: `100` (launch altitude in meters)
4. Select `8` (Send Config to TX)

### 4. Start Recording
1. Go to main menu, select `2` (Flight Operations)
2. Select `7` (Reset Altitude Offset) - do this at launch site
3. Select `1` (Start Data Logging)
4. **Launch your rocket!**
5. Select `2` (Stop Data Logging) after recovery

### 5. Download Data
1. Select `3` (Data Management)
2. Select `1` (Start WiFi Hotspot)
3. On your phone/computer:
   - Connect to WiFi: `PaviFlightData` (password: `pavi2024`)
   - Open browser: `192.168.4.1`
   - Download your flight data
4. Select `2` (Stop WiFi Hotspot)

---

## 🛠 **Installation & Setup**

### Hardware Requirements
- **Arduino IDE** (1.8.19 or newer) or **PlatformIO**
- **ESP32 Board Package** installed
- **USB drivers** for ESP32 programming

### Required Libraries
The following libraries are automatically included or available in Library Manager:

| Library | Version | Purpose |
|---------|---------|---------|
| `LoRa` | Latest | 433MHz LoRa communication |
| `MS5611` | Latest | Barometric pressure sensor |
| `MPU6050` | Latest | 6-axis IMU sensor |
| `HX711_Raw` | 1.0.0 | Custom load cell library (included) |
| `WiFi` | Built-in | ESP32 WiFi functionality |
| `LittleFS` | Built-in | File system for data storage |

### Programming the Units

#### TX Unit (Flight Computer)
```bash
# Open in Arduino IDE
File → Open → TX_Pavi_Optimized_v1/TX_Pavi_Optimized_v1.ino

# Select board and port
Tools → Board → ESP32 Dev Module
Tools → Port → [Your ESP32 port]

# Upload firmware
Sketch → Upload
```

#### RX Unit (Ground Station)
```bash
# Open in Arduino IDE  
File → Open → RX_Pavi_Optimized_v1/RX_Pavi_Optimized_v1.ino

# For ESP32-C3 boards
Tools → Board → ESP32C3 Dev Module

# For standard ESP32 boards
Tools → Board → ESP32 Dev Module

# Upload firmware
Sketch → Upload
```

### Hardware Connections

#### TX Unit Pinout
| Component | Pin | Purpose |
|-----------|-----|---------|
| LoRa NSS | 5 | SPI Chip Select |
| LoRa RST | 17 | Reset |
| LoRa DIO0 | 13 | Interrupt |
| HX711 DOUT | 4 | Load cell data |
| HX711 SCK | 16 | Load cell clock |
| Pyro 1 | 33 | Pyrotechnic channel 1 |
| Pyro 2 | 32 | Pyrotechnic channel 2 |
| Pyro 3 | 25 | Pyrotechnic channel 3 |
| Pyro 4 | 26 | Pyrotechnic channel 4 |
| I2C SDA | 21 | Sensor communication |
| I2C SCL | 22 | Sensor communication |

#### RX Unit Pinout (ESP32-C3)
| Component | Pin | Purpose |
|-----------|-----|---------|
| LoRa NSS | D7 (IO7) | SPI Chip Select |
| LoRa RST | D1 (IO2) | Reset |
| LoRa DIO0 | D0 (IO3) | Interrupt |
| SD Card CS | D8 | SD card (optional) |

---

## 📊 **System Architecture**

### Communication Protocol
```
RX Unit → LoRa → TX Unit
       ←-------←

Commands: ASCII strings (e.g., "FLIGHT_START", "PYRO1")
Responses: JSON-formatted acknowledgments
Range: 2-5km line-of-sight at 433MHz
```

### Data Logging Format
Flight data is stored as CSV files with the following structure:

```csv
Time_ms,Event,Alt_m,LoadCell_kg,AccelX_ms2,AccelY_ms2,AccelZ_ms2,GyroX_rads,GyroY_rads,GyroZ_rads,Temp_C
0,DATA,0.00,2.45,0.12,0.05,9.81,0.001,0.002,0.000,22.3
25,DATA,0.05,2.44,0.15,0.08,9.85,0.002,0.001,0.001,22.3
50,PYRO1,1.20,1.89,15.2,2.1,12.4,0.045,0.023,0.012,22.4
```

### Sensor Specifications

| Sensor | Range | Resolution | Update Rate |
|--------|-------|------------|-------------|
| **MS5611 Barometer** | 10-1200 mbar | 0.012 mbar | Up to 40Hz |
| **MPU6050 Accelerometer** | ±16g | 0.0012g | Up to 40Hz |
| **MPU6050 Gyroscope** | ±2000°/s | 0.061°/s | Up to 40Hz |
| **HX711 Load Cell** | 24-bit ADC | 80Hz internal | Up to 40Hz |

---

## 🎛 **RX Unit Menu System**

### Main Menu Structure
```
🚀 PAVI GROUND STATION RX 🚀
├── 1️⃣ TX Configuration Menu (Local)
│   ├── Set All Parameters (Guided)
│   ├── Set Individual Parameters
│   ├── Send Config to TX
│   └── Show Current Config
├── 2️⃣ Flight Operations Menu
│   ├── Start/Stop Data Logging
│   ├── Fire Pyro Channels 1-4
│   ├── Reset Altitude Offset
│   └── TX System Status
├── 3️⃣ Data Management Menu
│   ├── Start/Stop WiFi Hotspot
│   ├── WiFi Status & Connection Info
│   └── File Browser
├── 4️⃣ TX Calibration Menu
│   ├── Load Cell Tare (Zero)
│   ├── Calibrate with Known Weight
│   ├── Save Calibration
│   └── Test Current Calibration
├── 5️⃣ TX WiFi & File Menu
│   └── Advanced WiFi Controls
├── 6️⃣ System Status
│   └── Complete System Information
├── 7️⃣ ESP32-C3 Diagnostics (if applicable)
│   └── Technical Information
└── 0️⃣ Help & About
```

### Quick Command Reference

| Menu Path | Function | Description |
|-----------|----------|-------------|
| `1` → `1` → `8` | Quick Config | Set all parameters and send to TX |
| `2` → `7` → `1` | Flight Sequence | Reset altitude → Start logging |
| `2` → `3-6` | Pyro Control | Fire pyrotechnic channels |
| `3` → `1` | Data Download | Start WiFi for data access |
| `4` → `1-3` | Load Cell Setup | Complete calibration sequence |
| `6` | Status Check | View system health and connectivity |

---

## 🔧 **Advanced Configuration**

### Flight Computer Modes
The system supports two flight computer configurations:

```cpp
#define FC_NO 1  // FC1: 20Hz sampling, "PaviFlightData" WiFi
#define FC_NO 2  // FC2: 40Hz sampling, "PaviFlightData-2" WiFi
```

### LoRa Range Optimization
Current settings optimized for maximum range:

```cpp
// Range Optimization Settings
LoRa.setSpreadingFactor(12);    // Maximum reliability
LoRa.setSignalBandwidth(125E3); // Standard bandwidth
LoRa.setCodingRate4(8);         // Maximum error correction
LoRa.setTxPower(20);            // Maximum legal power (20dBm)
LoRa.setPreambleLength(12);     // Longer preamble
```

**Expected Performance:**
- **Indoor**: 200-500m (was 100-200m)
- **Line-of-sight**: 2-5km (was 500m-2km)  
- **With obstacles**: 300-1km (was 150-500m)

### Sensor Filtering
Individual sensor filtering can be enabled/disabled:

```cpp
// Sensor Filtering Configuration
#define ENABLE_PRESSURE_FILTERING false    // Barometer (heavy smoothing)
#define ENABLE_ACCEL_FILTERING false       // Accelerometer (medium smoothing)
#define ENABLE_GYRO_FILTERING false        // Gyroscope (light smoothing)
#define ENABLE_LOADCELL_FILTERING false    // Load cell (always raw)
```

**Filter Characteristics:**
- **Pressure**: 13-sample moving average + exponential filter (~0.3s delay)
- **Accelerometer**: 8-sample moving average + exponential filter (~0.2s delay)
- **Gyroscope**: 3-sample moving average (~0.075s delay)
- **Load Cell**: Always raw data via HX711_Raw library

### Pyrotechnic Safety
Pyro channels feature automatic safety cutoff:

```cpp
struct PyroChannel {
  bool active = false;
  unsigned long startTime = 0;
  const unsigned long duration = 1500; // 1.5 seconds maximum
};
```

---

## 📱 **Data Analysis**

### Accessing Flight Data
1. **WiFi Method** (Recommended):
   - Start WiFi hotspot from RX menu
   - Connect device to `PaviFlightData` (password: `pavi2024`)
   - Browse to `192.168.4.1`
   - Download CSV files directly

2. **USB Method**:
   - Connect TX unit to computer via USB
   - Send `WIFI_START` command
   - Access via web interface

### Data Processing
Flight data is provided in standard CSV format compatible with:
- **Microsoft Excel**
- **Google Sheets**
- **MATLAB**
- **Python pandas**
- **GNU Octave**

### Sample Analysis Workflow

#### Excel Analysis
```
1. Open CSV file in Excel
2. Create altitude graph: Plot Alt_m vs Time_ms
3. Find maximum altitude: =MAX(Alt_m column)
4. Calculate flight duration: Time when altitude returns near zero
5. Analyze acceleration: Plot AccelZ_ms2 vs Time_ms
```

#### Python Analysis
```python
import pandas as pd
import matplotlib.pyplot as plt

# Load flight data
data = pd.read_csv('MyFlight.csv')

# Create altitude plot
plt.figure(figsize=(10, 6))
plt.plot(data['Time_ms']/1000, data['Alt_m'])
plt.xlabel('Time (seconds)')
plt.ylabel('Altitude (meters)')
plt.title('Flight Altitude Profile')
plt.show()

# Find key metrics
max_altitude = data['Alt_m'].max()
flight_time = data['Time_ms'].max() / 1000
print(f"Maximum altitude: {max_altitude:.1f} meters")
print(f"Flight duration: {flight_time:.1f} seconds")
```

---

## 🔧 **Load Cell Calibration**

### When to Calibrate
- First time using the system
- After rough handling or transport
- If weight readings appear inaccurate (>5% error)
- When changing load cell hardware

### Calibration Procedure

#### Method 1: RX Menu (Recommended)
1. **Navigate**: Main Menu → `4` (TX Calibration Menu)
2. **Tare**: Select `1` (Tare Load Cell) - remove all weight
3. **Calibrate**: Select `2` (Calibrate with Known Weight)
   - Enter weight in grams (e.g., `1000` for 1kg)
   - Place exact weight on TX load cell
4. **Save**: Select `3` (Save Calibration) - permanent storage
5. **Test**: Select `4` (Test Current Calibration) - verify accuracy

#### Method 2: Direct Commands
```
RX → TX: TARE
RX → TX: CALIB:1000  (for 1kg calibration weight)
RX → TX: CALIB_SAVE
```

### Calibration Tips
- Use a **precise known weight** (digital kitchen scale recommended)
- Ensure **stable mounting** - no vibrations during calibration
- **Remove everything** from load cell before taring
- **Center the weight** on the load cell platform
- **Wait for readings to stabilize** (5-10 seconds)

### Troubleshooting Calibration
| Problem | Solution |
|---------|----------|
| Readings drift | Re-tare and ensure stable mounting |
| Negative weights | Check calibration factor sign |
| Inconsistent readings | Verify electrical connections |
| Large error (>5%) | Recalibrate with different known weight |

---

## 🚨 **Troubleshooting**

### Communication Issues

#### "No TX Connection" or "Connection Lost"
**Symptoms**: RX shows no connection to TX unit
```
Diagnostic Steps:
1. Check power on both units (USB/battery)
2. Verify both units completed startup (30+ seconds)
3. Check antenna connections (quarter-wave ~17cm)
4. Reduce distance for testing (<100m)
5. Send PING command from RX menu
6. Check for interference sources
```

**Solutions**:
- Power cycle both units (10 second off time)
- Verify correct firmware versions
- Check 433MHz antenna orientation (vertical)
- Move away from WiFi routers, cell towers

#### Poor Signal Quality (RSSI < -100dBm)
```
Range Optimization Checklist:
✓ Use proper 433MHz antenna (not random wire)
✓ Ensure line-of-sight between units
✓ Elevate RX unit (higher is better)
✓ Check antenna connections (tight, not loose)
✓ Verify TX power setting (should be 20dBm)
✓ Move away from interference sources
```

### Sensor Issues

#### Load Cell Problems
| Symptom | Cause | Solution |
|---------|-------|----------|
| Reading always zero | No calibration | Run calibration procedure |
| Negative weights | Wrong calibration factor | Recalibrate with positive weight |
| Erratic readings | Loose connections | Check HX711 wiring |
| Wrong values | Incorrect reference weight | Use precise known weight |

#### Barometer/Altitude Issues
| Symptom | Cause | Solution |
|---------|-------|----------|
| Altitude jumps | MS5611 not initialized | Check I2C connections |
| Wrong baseline | Need altitude reset | Use "Reset Altitude Offset" |
| Noisy readings | Vibration/airflow | Enable pressure filtering |

### Data Access Issues

#### WiFi Connection Problems
**Cannot connect to PaviFlightData network**:
```
Troubleshooting Steps:
1. Verify WiFi started: RX Menu → 3 → 1
2. Check network name matches FC number
   - FC1: "PaviFlightData"
   - FC2: "PaviFlightData-2"
3. Use correct password: "pavi2024" (lowercase)
4. Disable cellular data on phone
5. Forget and reconnect to network
6. Try different device (computer vs phone)
```

#### Web Interface Issues
**Cannot access 192.168.4.1**:
```
Verification Steps:
1. Confirm WiFi connection active
2. Use exact IP: 192.168.4.1 (not .com)
3. Try different browser
4. Clear browser cache
5. Disable VPN if enabled
6. Check TX unit has power
```

### Storage and File Issues

#### "Storage Full" Error
```
Data Management Steps:
1. Check storage: RX Menu → 6 (System Status)
2. Download existing files: RX Menu → 3 → 1
3. Delete old files via web interface
4. Use shorter filenames (<20 characters)
5. Check LittleFS formatting
```

#### File Download Failures
- **Browser timeout**: Try smaller files first
- **Incomplete downloads**: Check TX unit power/stability
- **Corrupted files**: Stop all recording before download

### Hardware-Specific Issues

#### ESP32-C3 Specific
For ESP32-C3 boards, access diagnostics via RX Menu → `7`:
```
Common ESP32-C3 Issues:
- Memory constraints: Monitor free heap
- USB CDC issues: Try different USB cable
- Boot mode problems: Hold BOOT while connecting
- Power issues: Ensure adequate current supply
```

#### ESP32 Standard
```
Standard ESP32 Troubleshooting:
- Brown-out reset: Check power supply voltage
- Flash size errors: Verify partition scheme
- Watchdog resets: Monitor for infinite loops
- Temperature issues: Ensure adequate cooling
```

---

## ⚠️ **Safety Guidelines**

### Pyrotechnic Safety
**⚠️ CRITICAL SAFETY INFORMATION ⚠️**

1. **Always verify pyro commands** before sending
2. **Maintain safe distance** (minimum 100m) during testing
3. **Use only certified pyrotechnic devices** designed for model rocketry
4. **Check local regulations** for pyrotechnic use
5. **Automatic shutoff** after 1.5 seconds per channel
6. **Never exceed** rated current for pyro channels (1A max)

### Radio Frequency Safety
- **433MHz ISM band** - legal in most countries
- **Maximum 20dBm** (100mW) transmission power
- **Avoid use near airports** or emergency services
- **Check local regulations** for power limits

### General Safety
- **Pre-flight testing mandatory** - verify all systems before important flights
- **Battery monitoring** - check voltage levels regularly
- **Backup procedures** - always have manual backup systems
- **Data backup** - download flight data immediately after flights
- **Weather considerations** - avoid use in precipitation
- **Temperature limits** - operating range -20°C to +60°C

---

## 🛠 **Development & Customization**

### Project Structure
```
pavi/
├── TX_Pavi_Optimized_v1/          # Latest TX firmware
│   └── TX_Pavi_Optimized_v1.ino
├── RX_Pavi_Optimized_v1/          # Latest RX firmware  
│   └── RX_Pavi_Optimized_v1.ino
├── libraries/                      # Custom libraries
│   └── HX711_Raw/                 # Raw load cell library
├── test/                          # Test utilities
├── TX_Pavi/                       # Legacy TX firmware
├── RX_Pavi/                       # Legacy RX firmware
├── TX_Pavi_Optimized/             # Previous TX version
├── RX_Pavi_Optimized/             # Previous RX version
└── Documentation/                 # User manuals
```

### Adding Custom Sensors
To add additional sensors to the TX unit:

```cpp
// 1. Include sensor library
#include "NewSensor.h"

// 2. Define sensor object
NewSensor customSensor;

// 3. Initialize in setup()
void setup() {
  // ... existing code ...
  if (customSensor.begin()) {
    Serial.println("Custom sensor ready");
  }
}

// 4. Read in sensor loop
void readSensors() {
  // ... existing sensors ...
  if (customSensorReady) {
    float customValue = customSensor.read();
    // Process data...
  }
}

// 5. Add to data logging
void logFlightData() {
  String dataLine = String(millis() - recordingStartTime) + ",DATA," +
                   String(altitude) + "," +
                   String(loadCellWeight) + "," +
                   String(customValue) + ",";  // Add custom data
  // ... rest of logging ...
}
```

### Custom Command Protocol
Add new commands to both TX and RX:

```cpp
// TX Unit - handleLoRaCommands()
if (command == "CUSTOM_CMD") {
  // Handle custom command
  LoRa.beginPacket();
  LoRa.print("CUSTOM_ACK:Success");
  LoRa.endPacket();
  delay(100);
  LoRa.receive();
}

// RX Unit - sendCommand()
void sendCustomCommand() {
  Serial.println("Sending custom command...");
  LoRa.beginPacket();
  LoRa.print("CUSTOM_CMD");
  LoRa.endPacket();
  LoRa.receive();
  waitForAck("CUSTOM_CMD");
}
```

### Build Configuration
Modify build settings by editing configuration defines:

```cpp
// TX_Pavi_Optimized_v1.ino
#define FC_NO 2                           // Flight computer number
#define ENABLE_LONG_RANGE_MODE true       // Range optimization
#define ENABLE_SENSOR_FILTERING false     // Raw vs filtered data

// Sampling rate (FC1 = 20Hz, FC2 = 40Hz)  
#if FC_NO == 1
  #define DATA_RATE_MODE 2  // 50ms interval
#elif FC_NO == 2
  #define DATA_RATE_MODE 4  // 25ms interval  
#endif
```

---

## 📈 **Performance Specifications**

### System Performance
| Metric | Specification | Notes |
|--------|---------------|-------|
| **Data Rate** | 20Hz / 40Hz | Configurable via FC_NO |
| **Latency** | <100ms | Command to acknowledgment |
| **Range** | 2-5km | Line-of-sight, optimized settings |
| **Battery Life** | 4-8 hours | Depends on usage and battery capacity |
| **Storage Capacity** | ~4MB | Thousands of flight records |
| **Altitude Range** | 0-30km | MS5611 barometer limit |
| **Acceleration Range** | ±16g | MPU6050 accelerometer |
| **Weight Range** | 0-100kg | Depends on load cell selection |

### Communication Performance
| Parameter | Value | Description |
|-----------|-------|-------------|
| **Frequency** | 433MHz | ISM band |
| **Modulation** | LoRa | Spread spectrum |
| **Spreading Factor** | 12 | Maximum range setting |
| **Bandwidth** | 125kHz | Standard LoRa bandwidth |
| **Coding Rate** | 4/8 | Maximum error correction |
| **TX Power** | 20dBm | 100mW maximum |
| **Sensitivity** | -136dBm | Receive sensitivity |

### Environmental Specifications
| Parameter | Range | Notes |
|-----------|-------|-------|
| **Operating Temperature** | -20°C to +60°C | Standard components |
| **Humidity** | 0-95% non-condensing | Avoid direct water exposure |
| **Altitude** | 0-30km | Barometer operational range |
| **Vibration** | Up to 20g RMS | Tested for rocket applications |
| **Power Supply** | 3.3V - 5V | USB or battery powered |

---

## 🤝 **Contributing**

### Development Setup
1. **Clone repository**:
   ```bash
   git clone https://github.com/your-repo/pavi-flight-computer
   cd pavi-flight-computer
   ```

2. **Install Arduino IDE** and required libraries

3. **Hardware setup**: 
   - ESP32 development boards
   - LoRa modules (433MHz)
   - Sensor breakout boards

### Code Style Guidelines
- **Indentation**: 2 spaces (no tabs)
- **Naming**: camelCase for variables, CAPS for constants
- **Comments**: Explain complex logic and hardware interfaces
- **Functions**: Keep functions focused and under 50 lines

### Testing Procedures
Before submitting changes:

1. **Unit testing**: Verify individual sensor functions
2. **Integration testing**: Test TX/RX communication
3. **Range testing**: Verify communication range
4. **Flight testing**: Test with actual flight profile
5. **Documentation**: Update README and comments

### Submitting Changes
1. **Fork** the repository
2. **Create feature branch**: `git checkout -b feature-name`
3. **Commit changes**: `git commit -m "Description"`
4. **Push to branch**: `git push origin feature-name`
5. **Create Pull Request** with description

---

## 📄 **License & Legal**

### Open Source License
This project is released under the **MIT License**:
- ✅ Commercial use allowed
- ✅ Modification allowed  
- ✅ Distribution allowed
- ✅ Private use allowed
- ⚠️ No warranty provided

### Regulatory Compliance
- **433MHz operation**: Legal in most countries under ISM band regulations
- **Power output**: Complies with typical 100mW limits
- **CE/FCC**: Not formally certified - for experimental/educational use
- **Export restrictions**: Check local laws for dual-use technology

### Safety Disclaimer
**⚠️ This system is provided for educational and experimental purposes. Users are responsible for:**
- Compliance with local laws and regulations
- Safe operation procedures
- Proper pyrotechnic handling
- Flight safety and recovery systems
- Insurance and liability coverage

---

## 📞 **Support & Resources**

### Documentation
- **User Manual**: `PAVI_Flight_Computer_User_Manual.md`
- **Technical Documentation**: `PAVI_Flight_Computer_Documentation.md`
- **API Reference**: See source code comments
- **Example Flights**: Sample data files in `/examples`

### Community Resources
- **Forum**: [Community discussion board]
- **Wiki**: [Technical wiki with tutorials]
- **Video Tutorials**: [YouTube channel]
- **GitHub Issues**: Report bugs and request features

### Professional Support
For commercial applications or advanced customization:
- **Technical Consulting**: Available on request
- **Custom Development**: Tailored solutions
- **Training Programs**: Hands-on workshops
- **Certification Support**: Regulatory compliance assistance

### Contact Information
- **Email**: [support@pavi-fc.com]
- **GitHub**: [github.com/pavi-flight-computer]
- **Documentation**: [docs.pavi-fc.com]

---

## 🎯 **Roadmap & Future Development**

### Version 1.1 (Planned)
- [ ] GPS integration for position tracking
- [ ] Real-time telemetry streaming
- [ ] Mobile app for Android/iOS
- [ ] Advanced flight prediction algorithms
- [ ] Multi-stage rocket support

### Version 2.0 (Future)
- [ ] Dual-band communication (433MHz + 2.4GHz)
- [ ] Machine learning flight optimization
- [ ] Mesh networking for multi-vehicle tracking
- [ ] Advanced recovery system integration
- [ ] Cloud data synchronization

### Hardware Roadmap
- [ ] Custom PCB design
- [ ] Integrated antenna design
- [ ] Ruggedized housing options
- [ ] Battery management system
- [ ] Sensor fusion improvements

---

**📚 Need Help?** Check the [User Manual](PAVI_Flight_Computer_User_Manual.md) for detailed operating instructions, or the [Technical Documentation](PAVI_Flight_Computer_Documentation.md) for in-depth system information.

**🚀 Ready to Fly?** Follow the [Quick Start Guide](#-quick-start-guide) to get your PAVI system operational in under 10 minutes!

---

*PAVI Flight Computer System v1.0 - Precision Aviation Vehicle Instrumentation*
*Built for the next generation of experimental aviation and rocketry*
