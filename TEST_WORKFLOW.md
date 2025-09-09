# LoRa Command/Response System Test Workflow

## System Overview
- **TX (Flight Computer)**: Receives commands, executes actions, logs data
- **RX (Ground Station)**: Sends commands to flight computer
- **Communication**: One-way LoRa (RX sends commands to TX)

## Hardware Setup
1. Connect RA-02 LoRa modules to both TX and RX
2. Connect sensors to TX (MS5611 barometer, MPU6050 IMU, HX711 load cell)
3. Connect pyro channels to TX (GPIOs 33, 32, 25, 26)
4. Power both units and open serial monitors

## Test Sequence

### 1. Basic Connectivity Test
**RX Commands:**
```
HELP        (Show available commands)
PING        (Test LoRa connection)
STATUS      (Get TX system status)
```
**Expected TX Response:**
- Should print "PING RECEIVED! TX is working!" for PING
- Should show system status including sensors for STATUS

### 2. Sensor Reset Test
**RX Commands:**
```
RESET       (Reset sensor origins)
```
**Expected TX Response:**
- Should reset barometer reference altitude
- Should reset velocity calculations

### 3. Flight Configuration Test
**RX Commands:**
```
START                    (Begin flight configuration)
FILENAME:test_flight     (Set filename)
WEIGHT:2.5              (Set weight to 2.5 kg)
WIND:3.0                (Set wind speed to 3.0 m/s)
HEIGHT:50               (Set initial height to 50 m)
CONFIRM                 (Confirm and start recording)
```
**Expected TX Response:**
- Should enter FLIGHT_CONFIGURING state after START
- Should accept each parameter and confirm values
- Should start data recording after CONFIRM
- Should create CSV file with header information

### 4. Data Recording Test
**After CONFIRM:**
- TX should begin logging sensor data every 100ms
- Data should include: time, altitude, velocity, load cell, accelerometer, gyroscope
- Should log to both serial and LittleFS file

### 5. Pyro Channel Test (CAREFUL!)
**RX Commands:**
```
PYRO1       (Fire pyro channel 1 - requires confirmation)
```
**Expected TX Response:**
- Should fire 100ms pulse on GPIO33
- Should log pyro event with timestamp

### 6. Stop Recording Test
**RX Commands:**
```
STOP        (Stop data recording)
```
**Expected TX Response:**
- Should stop data logging
- Should close data file
- Should show recording statistics

### 7. File Management Test
**TX Serial Commands:**
```
FILES                   (List stored files)
DOWNLOAD test_flight.csv (View file contents)
SPACE                   (Show filesystem usage)
```

## Expected File Format
```csv
# PARACHUTE TEST FLIGHT DATA
# Filename: test_flight.csv
# Total Weight: 2.5 kg
# Wind Speed: 3.0 m/s
# Initial Height: 50 m
# Start Time: 12345 ms
# Data Rate Mode: 1
# Columns: Time(ms),Height(m),VerticalVel(m/s),LoadCell(kg),AccelX(m/s²),AccelY(m/s²),AccelZ(m/s²),GyroX(rad/s),GyroY(rad/s),GyroZ(rad/s)
0,0.00,0.00,2.45,0.12,0.05,-0.98,0.02,-0.01,0.00
100,0.05,0.50,2.43,0.15,0.03,-0.95,0.03,-0.02,0.01
...
```

## Troubleshooting

### If LoRa not working:
1. Check wiring connections
2. Verify both units have same frequency, SF, BW settings
3. Check power supply stability

### If sensors not reading:
1. Check I2C connections (SDA/SCL)
2. Verify sensor power supply
3. Check for proper sensor initialization

### If filesystem errors:
1. LittleFS will auto-format on first use
2. Data will still log to serial if filesystem fails
3. Use FORMAT command to reset filesystem

## Success Criteria
- ✅ PING command works (LoRa communication established)
- ✅ START command enters configuration mode
- ✅ All configuration parameters accepted
- ✅ CONFIRM starts data recording with proper file header
- ✅ Sensor data logs continuously at ~10Hz
- ✅ STOP command ends recording and saves file
- ✅ RESET command resets sensor origins
- ✅ Pyro channels can be fired (test mode)
- ✅ Files can be listed and downloaded

## Notes
- System is designed for robust operation in field conditions
- No continuous telemetry - only command/response for reliability
- All critical data is logged locally on TX for data integrity
- Pyro channels should only be tested with LEDs/resistors, not actual pyrotechnics
