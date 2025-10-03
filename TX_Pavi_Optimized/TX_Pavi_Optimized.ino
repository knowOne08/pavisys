#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <MS5611.h>  
#include <MPU6050.h>
#include "HX711_Raw.h"
#include "FS.h"
#include "LittleFS.h"
#include <WiFi.h>
#include <WebServer.h>

// === CONFIGURATION ===
#define FC_NO 2  // 1 = FC1 (20Hz), 2 = FC2 (40Hz)

#if FC_NO == 1
  #define DATA_RATE_MODE 2
  #define WIFI_SSID "PaviFlightData"
#elif FC_NO == 2  
  #define DATA_RATE_MODE 4
  #define WIFI_SSID "PaviFlightData-2"
#endif

// === SENSOR FILTERING CONFIGURATION ===
// Individual sensor filtering controls
#define ENABLE_PRESSURE_FILTERING false    // Barometer filtering (heavy smoothing)
#define ENABLE_ACCEL_FILTERING false       // Accelerometer filtering (medium smoothing)
#define ENABLE_GYRO_FILTERING false        // Gyroscope filtering (light smoothing)
#define ENABLE_LOADCELL_FILTERING false    // Load cell filtering (always raw in HX711_Raw)

// Master enable - set to true to enable filtering, then customize individual sensors above
#define ENABLE_SENSOR_FILTERING (ENABLE_PRESSURE_FILTERING || ENABLE_ACCEL_FILTERING || ENABLE_GYRO_FILTERING || ENABLE_LOADCELL_FILTERING)

/*
 * FILTERING CONFIGURATION GUIDE:
 * 
 * PRESSURE_FILTERING: Heavy moving average + exponential filter (13-sample buffer)
 *   - Best for: Altitude calculations, smooth barometric readings
 *   - Impact: Reduces noise but adds ~0.3s delay
 * 
 * ACCEL_FILTERING: Medium moving average + exponential filter (8-sample buffer)  
 *   - Best for: Orientation calculations, reducing vibration noise
 *   - Impact: Smoother acceleration data, ~0.2s delay
 * 
 * GYRO_FILTERING: Light moving average (3-sample buffer)
 *   - Best for: Angular velocity calculations, reducing sensor jitter
 *   - Impact: Minimal smoothing, ~0.075s delay
 * 
 * LOADCELL_FILTERING: Currently not implemented (HX711_Raw always provides raw data)
 *   - Load cell uses hardware filtering and averaging in HX711_Raw library
 * 
 * RECOMMENDED SETTINGS:
 *   For Flight Analysis: All false (raw data)
 *   For Real-time Display: Pressure=true, Accel=true, Gyro=false
 *   For Orientation Control: Pressure=true, Accel=true, Gyro=true
 */

// === PINS ===
#define NSS   5
#define RST   17
#define DIO0  13
#define PYRO_1  33
#define PYRO_2  32
#define PYRO_3  25
#define PYRO_4  26
#define HX711_DOUT  4
#define HX711_SCK   16

// === SENSOR CONFIG ===
bool ENABLE_BAROMETER = true;
bool ENABLE_ACCELEROMETER = true;
bool ENABLE_GYROSCOPE = true;
bool ENABLE_LOAD_CELL = true;

// === FLIGHT STATE ===
enum FlightState { FLIGHT_IDLE, FLIGHT_CONFIGURING, FLIGHT_CONFIGURED, FLIGHT_RECORDING, FLIGHT_STOPPED };
FlightState flightState = FLIGHT_IDLE;

struct FlightConfig {
  String filename;
  float totalWeight;
  float windSpeed;
  float initialHeight;
  unsigned long startTime;
} flightConfig;

// === HARDWARE OBJECTS ===
MS5611 baro;
MPU6050 mpu;
HX711_Raw loadCell;
WebServer server(80);

// === SENSOR DATA ===
float pressure, temperature, altitude, referenceAltitude = 0.0;
float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
float loadCellWeight, loadCellCalibrationFactor = -7050.0, loadCellOffset = 0.0;
long loadCellRaw;
int16_t ax, ay, az, gx, gy, gz;

// === SYSTEM STATE ===
bool filesystemReady = false, baroReady = false, mpuReady = false, loadCellReady = false;
bool wifiEnabled = false;
String commandBuffer = "", dataBuffer = "";
File dataFile;
unsigned long sampleNumber = 0, recordingStartTime = 0;
int pyroPins[] = {PYRO_1, PYRO_2, PYRO_3, PYRO_4};

// === FUNCTION DECLARATIONS ===
void startWiFi();
void stopWiFi(); 
void setupWebServer();
void handleRoot();
void handleFileDownload();
void handleFileListAPI();
void handleSystemInfoAPI();
void handleFileDeleteAPI();
void handleNotFound();
void handleStatusAPI();
String generateMainPage();
String getFileType(String filename);
int getLittleFSFileCount();

// Simple command processing - no menus needed on TX

// === FILTERING (CONDITIONAL) ===
#if ENABLE_SENSOR_FILTERING
  #if ENABLE_PRESSURE_FILTERING
    float pressureReadings[13];
    int pressureIndex = 0;
  #endif
  #if ENABLE_ACCEL_FILTERING  
    float accelReadings[8];
    int accelIndex = 0;
  #endif
  #if ENABLE_GYRO_FILTERING
    float gyroReadings[3];
    int gyroIndex = 0;
  #endif
#endif
float filterPressure(float rawPressure);
float filterAccel(float rawAccel, int axis);
float filterGyro(float rawGyro, int axis);

// === TIMING ===
const unsigned long sendInterval = (DATA_RATE_MODE == 1) ? 100 : (DATA_RATE_MODE == 2) ? 50 : (DATA_RATE_MODE == 3) ? 33 : 25;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== PAVI TX Flight Computer ===");
  Serial.print("Mode: FC"); Serial.print(FC_NO);
  Serial.print(", Rate: "); Serial.print(1000/sendInterval); Serial.println("Hz");
  Serial.println("Sensor Filtering Status:");
  Serial.println("  Pressure: " + String(ENABLE_PRESSURE_FILTERING ? "FILTERED" : "RAW"));
  Serial.println("  Accel: " + String(ENABLE_ACCEL_FILTERING ? "FILTERED" : "RAW"));
  Serial.println("  Gyro: " + String(ENABLE_GYRO_FILTERING ? "FILTERED" : "RAW"));
  Serial.println("  LoadCell: RAW (HX711_Raw)");
  
  // Initialize pyro channels
  for (int i = 0; i < 4; i++) {
    pinMode(pyroPins[i], OUTPUT);
    digitalWrite(pyroPins[i], LOW);
  }
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize LoRa
  LoRa.setPins(NSS, RST, DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa failed!");
    return;
  }
  // Match RX settings exactly for reliable communication
  LoRa.setSpreadingFactor(12);    // Maximum reliability (matches RX)
  LoRa.setSignalBandwidth(125E3); // Standard bandwidth (matches RX)
  LoRa.setCodingRate4(8);         // Maximum error correction (matches RX)
  LoRa.enableCrc();               // Enable CRC (matches RX)
  LoRa.setSyncWord(0x12);         // Custom sync word (matches RX)
  LoRa.receive();
  Serial.println("LoRa ready - Settings synchronized with RX (Conservative 5dBm)");
  
  // Initialize sensors
  initSensors();
  
  // Initialize filesystem
  if (LittleFS.begin()) {
    filesystemReady = true;
    Serial.println("LittleFS ready");
  }
  
  Serial.println("=== System Ready ===");
  Serial.println();
  Serial.println("💡 Available Commands:");
  Serial.println("   • Send LoRa commands for automated operation");
  Serial.println("   • Type 'CALIB' for interactive load cell calibration");
  Serial.println("   • Type 'STATUS' for system status");
  Serial.println();
}

void initSensors() {
  // MS5611 Barometer
  if (ENABLE_BAROMETER && baro.begin()) {
    baroReady = true;
    delay(1000);
    setReferenceAltitude();
    Serial.println("MS5611 ready");
  }
  
  // MPU6050 IMU
  if ((ENABLE_ACCELEROMETER || ENABLE_GYROSCOPE) && mpu.testConnection()) {
    mpu.initialize();
    mpuReady = true;
    Serial.println("MPU6050 ready");
  }
  
  // HX711 Load Cell
  if (ENABLE_LOAD_CELL) {
    loadCell.begin(HX711_DOUT, HX711_SCK);
    if (loadCell.is_ready()) {
      loadCell.set_scale(loadCellCalibrationFactor);
      loadCell.tare();
      loadCellReady = true;
      loadCalibrationFromFile();
      Serial.println("HX711_Raw ready");
    }
  }
}

void setReferenceAltitude() {
  float totalPressure = 0;
  int validReadings = 0;
  
  for (int i = 0; i < 10; i++) {
    baro.read();
    float p = baro.getPressure();
    if (p > 500 && p < 1200) {
      totalPressure += p;
      validReadings++;
    }
    delay(100);
  }
  
  if (validReadings > 0) {
    float avgPressure = totalPressure / validReadings;
    referenceAltitude = 44330.0 * (1.0 - pow(avgPressure / 1013.25, 0.1903));
  }
}

void loop() {
  handleSerialCommands();
  handleLoRaCommands();
  
  static unsigned long lastSensorRead = 0;
  static unsigned long lastStatusPrint = 0;
  
  if (millis() - lastSensorRead >= sendInterval) {
    lastSensorRead = millis();
    readSensors();
    
    if (flightState == FLIGHT_RECORDING) {
      logFlightData();
    }
  }
  
  // Print status every 10 seconds to show TX is alive and listening
  if (millis() - lastStatusPrint >= 10000) {
    lastStatusPrint = millis();
    Serial.print("TX Status: "); 
    switch(flightState) {
      case FLIGHT_IDLE: Serial.print("IDLE"); break;
      case FLIGHT_CONFIGURING: Serial.print("CONFIGURING"); break;
      case FLIGHT_CONFIGURED: Serial.print("CONFIGURED"); break;
      case FLIGHT_RECORDING: Serial.print("RECORDING"); break;
      case FLIGHT_STOPPED: Serial.print("STOPPED"); break;
    }
    Serial.print(" | Uptime: "); Serial.print(millis()/1000); 
    Serial.print("s | "); Serial.println("Send commands via LoRa or Serial");
  }
  
  if (wifiEnabled) server.handleClient();
}

void readSensors() {
  // Read barometer
  if (baroReady && ENABLE_BAROMETER) {
    baro.read();
    float rawPressure = baro.getPressure();
    if (rawPressure > 500 && rawPressure < 1200) {
      pressure = ENABLE_PRESSURE_FILTERING ? filterPressure(rawPressure) : rawPressure;
      altitude = 44330.0 * (1.0 - pow(pressure / 1013.25, 0.1903)) - referenceAltitude;
      temperature = baro.getTemperature();
    }
  }
  
  // Read IMU
  if (mpuReady && (ENABLE_ACCELEROMETER || ENABLE_GYROSCOPE)) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (ENABLE_ACCELEROMETER) {
      float rawAccelX = (ax / 4096.0) * 9.80665;
      float rawAccelY = (ay / 4096.0) * 9.80665;
      float rawAccelZ = (az / 4096.0) * 9.80665;
      
      accelX = ENABLE_ACCEL_FILTERING ? filterAccel(rawAccelX, 0) : rawAccelX;
      accelY = ENABLE_ACCEL_FILTERING ? filterAccel(rawAccelY, 1) : rawAccelY;
      accelZ = ENABLE_ACCEL_FILTERING ? filterAccel(rawAccelZ, 2) : rawAccelZ;
    }
    
    if (ENABLE_GYROSCOPE) {
      float rawGyroX = (gx / 32.8) * (M_PI / 180.0);
      float rawGyroY = (gy / 32.8) * (M_PI / 180.0);
      float rawGyroZ = (gz / 32.8) * (M_PI / 180.0);
      
      gyroX = ENABLE_GYRO_FILTERING ? filterGyro(rawGyroX, 0) : rawGyroX;
      gyroY = ENABLE_GYRO_FILTERING ? filterGyro(rawGyroY, 1) : rawGyroY;
      gyroZ = ENABLE_GYRO_FILTERING ? filterGyro(rawGyroZ, 2) : rawGyroZ;
    }
  }
  
  // Read load cell (smart sampling)
  static int loadCellCycle = 0;
  const int interval = (DATA_RATE_MODE <= 2) ? 3 : 5;
  
  if (loadCellReady && ENABLE_LOAD_CELL && ++loadCellCycle >= interval) {
    loadCellCycle = 0;
    if (loadCell.is_ready()) {
      loadCellRaw = loadCell.read();
      loadCellWeight = loadCell.get_units(1);
    }
  }
}

#if ENABLE_SENSOR_FILTERING
#if ENABLE_PRESSURE_FILTERING
float filterPressure(float newReading) {
  static float total = 0, avg = 1013.25;
  static bool init = false;
  
  if (!init) {
    for (int i = 0; i < 13; i++) pressureReadings[i] = 1013.25;
    total = 13 * 1013.25;
    init = true;
  }
  
  total = total - pressureReadings[pressureIndex] + newReading;
  pressureReadings[pressureIndex] = newReading;
  pressureIndex = (pressureIndex + 1) % 13;
  
  float movingAvg = total / 13;
  avg = 0.9 * avg + 0.1 * movingAvg;
  return avg;
}
#endif

#if ENABLE_ACCEL_FILTERING
float filterAccel(float newReading, int axis) {
  static float totals[3] = {0}, avgs[3] = {0};
  static bool init = false;
  
  if (!init) {
    for (int i = 0; i < 8; i++) accelReadings[i] = 0;
    init = true;
  }
  
  totals[axis] = totals[axis] - accelReadings[accelIndex + axis*8] + newReading;
  accelReadings[accelIndex + axis*8] = newReading;
  
  float movingAvg = totals[axis] / 8;
  avgs[axis] = 0.7 * avgs[axis] + 0.3 * movingAvg;
  
  if (axis == 2) accelIndex = (accelIndex + 1) % 8;  // Update index only once
  return avgs[axis];
}
#endif

#if ENABLE_GYRO_FILTERING
float filterGyro(float newReading, int axis) {
  static float totals[3] = {0};
  
  totals[axis] = totals[axis] - gyroReadings[gyroIndex + axis*3] + newReading;
  gyroReadings[gyroIndex + axis*3] = newReading;
  
  if (axis == 2) gyroIndex = (gyroIndex + 1) % 3;
  return totals[axis] / 3;
}
#endif
#endif

// Fallback functions for when filtering is disabled
#if !ENABLE_PRESSURE_FILTERING
float filterPressure(float newReading) { return newReading; }
#endif
#if !ENABLE_ACCEL_FILTERING  
float filterAccel(float newReading, int axis) { return newReading; }
#endif
#if !ENABLE_GYRO_FILTERING
float filterGyro(float newReading, int axis) { return newReading; }
#endif

void handleSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (commandBuffer.length() > 0) {
        processCommand(commandBuffer);
        commandBuffer = "";
      }
    } else {
      commandBuffer += c;
    }
  }
}

void handleLoRaCommands() {
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    Serial.print("LoRa packet received ("); Serial.print(packetSize); Serial.print(" bytes, RSSI: "); Serial.print(LoRa.packetRssi()); Serial.println(")");
    
    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();
    }
    received.trim();
    
    Serial.print("Raw data: '"); Serial.print(received); Serial.println("'");
    processCommand(received);
    LoRa.receive();
  }
}

void processCommand(String cmd) {
  cmd.toUpperCase();
  cmd.trim();
  
  Serial.println("RX Command: " + cmd); // Debug: show received command
  
  // Handle combined CONFIG packet from RX: "CONFIG:filename,weight,wind,height"
  if (cmd.startsWith("CONFIG:")) {
    flightState = FLIGHT_CONFIGURING;
    Serial.println("Combined CONFIG packet received");
    
    String configData = cmd.substring(7); // Remove "CONFIG:"
    
    // Parse comma-separated values: filename,weight,wind,height
    int comma1 = configData.indexOf(',');
    int comma2 = configData.indexOf(',', comma1 + 1);
    int comma3 = configData.indexOf(',', comma2 + 1);
    
    if (comma1 > 0 && comma2 > comma1 && comma3 > comma2) {
      // Extract values
      flightConfig.filename = configData.substring(0, comma1) + ".csv";
      flightConfig.totalWeight = configData.substring(comma1 + 1, comma2).toFloat();
      flightConfig.windSpeed = configData.substring(comma2 + 1, comma3).toFloat();
      flightConfig.initialHeight = configData.substring(comma3 + 1).toFloat();
      
      // Display parsed configuration
      Serial.println("Parsed CONFIG:");
      Serial.println("  Filename: " + flightConfig.filename);
      Serial.println("  Weight: " + String(flightConfig.totalWeight, 2) + " kg");
      Serial.println("  Wind: " + String(flightConfig.windSpeed, 2) + " m/s");
      Serial.println("  Height: " + String(flightConfig.initialHeight, 2) + " m");
      
      // Validate and set state
      if (validateConfig()) {
        flightState = FLIGHT_CONFIGURED;
        createDataFile();
        Serial.println("Configuration complete - ready to start");
      } else {
        Serial.println("Configuration validation failed");
        flightState = FLIGHT_IDLE;
      }
    } else {
      Serial.println("Error: Invalid CONFIG packet format");
      flightState = FLIGHT_IDLE;
    }
  }
  
  // Legacy individual configuration commands (for backward compatibility)
  else if (cmd == "CONFIG_START") {
    flightState = FLIGHT_CONFIGURING;
    Serial.println("Configuration mode started");
  }
  else if (cmd.startsWith("FILENAME:")) {
    flightConfig.filename = cmd.substring(9) + ".csv";
    Serial.println("Filename set: " + flightConfig.filename);
  }
  else if (cmd.startsWith("WEIGHT:")) {
    flightConfig.totalWeight = cmd.substring(7).toFloat();
    Serial.println("Weight set: " + String(flightConfig.totalWeight, 2) + " kg");
  }
  else if (cmd.startsWith("WIND:")) {
    flightConfig.windSpeed = cmd.substring(5).toFloat();
    Serial.println("Wind speed set: " + String(flightConfig.windSpeed, 2) + " m/s");
  }
  else if (cmd.startsWith("HEIGHT:")) {
    flightConfig.initialHeight = cmd.substring(7).toFloat();
    Serial.println("Height set: " + String(flightConfig.initialHeight, 2) + " m");
  }
  else if (cmd == "CONFIG_READY") {
    if (validateConfig()) {
      flightState = FLIGHT_CONFIGURED;
      createDataFile();
      Serial.println("Configuration complete - ready to start");
    }
  }
  
  // Flight commands
  else if (cmd == "FLIGHT_START" || cmd == "START_LOGGING") {
    if (flightState == FLIGHT_CONFIGURED) {
      startRecording();
      Serial.println("Flight data logging started successfully");
    } else {
      Serial.println("Cannot start flight - not configured properly (State: " + String(flightState) + ")");
    }
  }
  else if (cmd == "FLIGHT_STOP" || cmd == "STOP_LOGGING") {
    if (flightState == FLIGHT_RECORDING) {
      stopRecording();
      Serial.println("Flight data logging stopped successfully");
    } else {
      Serial.println("Flight logging was not active (State: " + String(flightState) + ")");
    }
  }
  
  // Pyro commands
  else if (cmd.startsWith("PYRO") && cmd.length() == 5) {
    int channel = cmd.charAt(4) - '1';
    if (channel >= 0 && channel <= 3) {
      firePyroChannel(channel);
    }
  }
  
  // Calibration commands
  else if (cmd == "CALIB") {
    startInteractiveCalibration();
  }
  else if (cmd == "CALIB_START") {
    Serial.println("Load cell calibration started");
    calibrateZero(); // Start with zeroing
  }
  else if (cmd == "CALIB_ZERO" || cmd == "CAL_TARE") {
    calibrateZero();
  }
  else if (cmd.startsWith("CALIB_WEIGHT:") || cmd.startsWith("CAL_WEIGHT:")) {
    int colonPos = cmd.indexOf(':');
    if (colonPos > 0) {
      float weight = cmd.substring(colonPos + 1).toFloat();
      calibrateWeight(weight);
    }
  }
  else if (cmd == "CALIB_SAVE" || cmd == "CAL_SAVE") {
    saveCalibration();
  }
  else if (cmd == "CALIB_TEST" || cmd == "CAL_STATUS") {
    testCalibration();
  }
  
  // Sensor commands
  else if (cmd == "SENSOR_RESET") {
    Serial.println("Resetting sensor zero points...");
    if (baroReady) {
      setReferenceAltitude();
      Serial.println("Barometer reference altitude reset");
    }
    if (loadCellReady) {
      loadCell.tare();
      Serial.println("Load cell tared");
    }
    Serial.println("Sensor reset complete");
  }
  
  // File system commands
  else if (cmd == "FILE_LIST" || cmd == "LIST_FILES") {
    Serial.println("Listing files in storage:");
    if (filesystemReady) {
      File root = LittleFS.open("/");
      File file = root.openNextFile();
      int fileCount = 0;
      while (file) {
        Serial.println("  - " + String(file.name()) + " (" + String(file.size()) + " bytes)");
        file = root.openNextFile();
        fileCount++;
      }
      if (fileCount == 0) {
        Serial.println("  No files found");
      } else {
        Serial.println("Total files: " + String(fileCount));
      }
    } else {
      Serial.println("  Filesystem not available");
    }
  }
  
  // System commands
  else if (cmd == "PING") {
    Serial.println("PONG received from RX - connection active");
  }
  else if (cmd == "STATUS") {
    printStatus();
  }
  else if (cmd == "WIFI_START" || cmd == "START_WIFI") {
    startWiFi();
    Serial.println("WiFi access point started for data download");
  }
  else if (cmd == "WIFI_STOP" || cmd == "STOP_WIFI") {
    stopWiFi();
  }
  
  // Unknown command
  else {
    Serial.println("Unknown command: " + cmd);
  }
}

bool validateConfig() {
  return !flightConfig.filename.isEmpty() && 
         flightConfig.totalWeight > 0 && 
         flightConfig.windSpeed >= 0 && 
         flightConfig.initialHeight > 0;
}

void createDataFile() {
  if (!filesystemReady) return;
  
  dataFile = LittleFS.open("/" + flightConfig.filename, "w");
  if (dataFile) {
    writeDataHeader();
    Serial.println("Data file created: " + flightConfig.filename);
  }
}

void writeDataHeader() {
  // Enhanced metadata header with system configuration
  dataFile.println("# ========================================");
  dataFile.println("# PAVI Flight Computer Data Log");
  dataFile.println("# ========================================");
  dataFile.println("# Flight Configuration:");
  dataFile.println("# Filename: " + flightConfig.filename);
  dataFile.println("# Total Weight: " + String(flightConfig.totalWeight, 2) + " kg");
  dataFile.println("# Wind Speed: " + String(flightConfig.windSpeed, 2) + " m/s");
  dataFile.println("# Launch Height: " + String(flightConfig.initialHeight, 2) + " m");
  dataFile.println("#");
  dataFile.println("# System Configuration:");
  dataFile.println("# Flight Computer: FC" + String(FC_NO));
  dataFile.println("# Data Rate Mode: " + String(DATA_RATE_MODE) + " (" + String(1000/sendInterval) + " Hz)");
  dataFile.println("# Sample Interval: " + String(sendInterval) + " ms");
  dataFile.println("#");
  dataFile.println("# Sensor Filtering Status:");
  dataFile.println("# Barometer (Pressure): " + String(ENABLE_PRESSURE_FILTERING ? "FILTERED (13-sample MA + EMA)" : "RAW"));
  dataFile.println("# Accelerometer: " + String(ENABLE_ACCEL_FILTERING ? "FILTERED (8-sample MA + EMA)" : "RAW"));
  dataFile.println("# Gyroscope: " + String(ENABLE_GYRO_FILTERING ? "FILTERED (3-sample MA)" : "RAW"));
  dataFile.println("# Load Cell: RAW (HX711_Raw library)");
  dataFile.println("#");
  dataFile.println("# Enabled Sensors:");
  dataFile.println("# Barometer: " + String(ENABLE_BAROMETER ? "YES" : "NO"));
  dataFile.println("# Accelerometer: " + String(ENABLE_ACCELEROMETER ? "YES" : "NO"));
  dataFile.println("# Gyroscope: " + String(ENABLE_GYROSCOPE ? "YES" : "NO"));
  dataFile.println("# Load Cell: " + String(ENABLE_LOAD_CELL ? "YES" : "NO"));
  dataFile.println("#");
  dataFile.println("# Hardware Information:");
  dataFile.println("# Chip: " + String(ESP.getChipModel()) + " Rev" + String(ESP.getChipRevision()));
  dataFile.println("# CPU Frequency: " + String(ESP.getCpuFreqMHz()) + " MHz");
  dataFile.println("# Free Heap: " + String(ESP.getFreeHeap()) + " bytes");
  dataFile.println("# Recording Started: " + String(millis()) + " ms (system uptime)");
  dataFile.println("#");
  dataFile.println("# Calibration:");
  dataFile.println("# Load Cell Factor: " + String(loadCellCalibrationFactor, 6));
  dataFile.println("# Load Cell Offset: " + String(loadCellOffset));
  dataFile.println("#");
  dataFile.println("# Data Format:");
  dataFile.println("# Time_ms: Milliseconds since recording started");
  dataFile.println("# Event: DATA=sensor sample, PYRO1-4=pyro fired, OFFSET=sensor reset");
  dataFile.println("# Alt_m: Altitude in meters (relative to launch point)");
  dataFile.println("# LoadCell_kg: Weight/force reading in kilograms"); 
  dataFile.println("# AccelX/Y/Z_ms2: Linear acceleration in m/s²");
  dataFile.println("# GyroX/Y/Z_rads: Angular velocity in rad/s");
  dataFile.println("# ========================================");
  dataFile.println();
  dataFile.println("Time_ms,Event,Alt_m,LoadCell_kg,AccelX_ms2,AccelY_ms2,AccelZ_ms2,GyroX_rads,GyroY_rads,GyroZ_rads");
}

void startRecording() {
  flightState = FLIGHT_RECORDING;
  recordingStartTime = millis();
  sampleNumber = 0;
  resetSensorOrigins();
  Serial.println("Recording started");
}

void stopRecording() {
  flightState = FLIGHT_STOPPED;
  if (dataFile) dataFile.close();
  Serial.println("Recording stopped - " + String(sampleNumber) + " samples");
}

void logFlightData() {
  if (!filesystemReady || !dataFile) return;
  
  unsigned long timestamp = millis() - recordingStartTime;
  
  String line = String(timestamp) + ",DATA," + 
                String(altitude, 2) + "," +
                String(loadCellWeight, 2) + "," +
                String(accelX, 2) + "," + String(accelY, 2) + "," + String(accelZ, 2) + "," +
                String(gyroX, 3) + "," + String(gyroY, 3) + "," + String(gyroZ, 3);
  
  dataFile.println(line);
  dataFile.flush();
  sampleNumber++;
}

void resetSensorOrigins() {
  if (baroReady) {
    setReferenceAltitude();
    logEvent("OFFSET", "Sensor origins reset");
  }
}

void firePyroChannel(int channel) {
  digitalWrite(pyroPins[channel], HIGH);
  delay(100);
  digitalWrite(pyroPins[channel], LOW);
  logEvent("PYRO" + String(channel + 1), "Pyro channel fired");
  Serial.println("Pyro " + String(channel + 1) + " fired");
}

void logEvent(String eventType, String note) {
  if (!filesystemReady || !dataFile) return;
  
  unsigned long timestamp = (flightState == FLIGHT_RECORDING) ? 
                           millis() - recordingStartTime : millis();
  
  String line = String(timestamp) + "," + eventType + "," +
                String(altitude, 2) + "," + String(loadCellWeight, 2) + "," +
                String(accelX, 2) + "," + String(accelY, 2) + "," + String(accelZ, 2) + "," +
                String(gyroX, 3) + "," + String(gyroY, 3) + "," + String(gyroZ, 3) + "," + note;
  
  dataFile.println(line);
  dataFile.flush();
}

// === CALIBRATION FUNCTIONS ===
void calibrateZero() {
  if (!loadCellReady) return;
  
  Serial.println("Calibrating zero... remove all weight");
  delay(3000);
  
  long total = 0;
  int count = 0;
  for (int i = 0; i < 10; i++) {
    if (loadCell.is_ready()) {
      total += loadCell.read();
      count++;
    }
    delay(200);
  }
  
  if (count > 5) {
    loadCellOffset = total / count;
    loadCell.set_offset(loadCellOffset);
    Serial.println("Zero calibrated: " + String(loadCellOffset));
  }
}

void calibrateWeight(float weight) {
  if (!loadCellReady || weight <= 0) return;
  
  Serial.println("Place " + String(weight, 2) + " kg weight");
  delay(5000);
  
  long total = 0;
  int count = 0;
  for (int i = 0; i < 10; i++) {
    if (loadCell.is_ready()) {
      total += loadCell.get_value();
      count++;
    }
    delay(200);
  }
  
  if (count > 5) {
    loadCellCalibrationFactor = (total / count) / weight;
    loadCell.set_scale(loadCellCalibrationFactor);
    Serial.println("Weight calibrated: factor = " + String(loadCellCalibrationFactor, 2));
  }
}

void saveCalibration() {
  if (!filesystemReady) return;
  
  File calFile = LittleFS.open("/loadcell_cal.txt", "w");
  if (calFile) {
    calFile.println("calibration_factor=" + String(loadCellCalibrationFactor, 6));
    calFile.println("zero_offset=" + String(loadCellOffset));
    calFile.close();
    Serial.println("Calibration saved");
  }
}

void loadCalibrationFromFile() {
  if (!filesystemReady) return;
  
  File calFile = LittleFS.open("/loadcell_cal.txt", "r");
  if (calFile) {
    while (calFile.available()) {
      String line = calFile.readStringUntil('\n');
      if (line.startsWith("calibration_factor=")) {
        loadCellCalibrationFactor = line.substring(19).toFloat();
      } else if (line.startsWith("zero_offset=")) {
        loadCellOffset = line.substring(12).toFloat();
      }
    }
    calFile.close();
    
    if (loadCellReady) {
      loadCell.set_scale(loadCellCalibrationFactor);
      loadCell.set_offset(loadCellOffset);
    }
    Serial.println("Calibration loaded");
  }
}

void testCalibration() {
  if (!loadCellReady) return;
  
  Serial.println("Load cell test:");
  for (int i = 0; i < 5; i++) {
    if (loadCell.is_ready()) {
      float weight = loadCell.get_units(1);
      Serial.println("  " + String(i + 1) + ". " + String(weight, 3) + " kg");
    }
    delay(500);
  }
}

// === INTERACTIVE CALIBRATION SUBROUTINE ===
void startInteractiveCalibration() {
  if (!loadCellReady) {
    Serial.println("❌ Load cell not ready! Check connections.");
    return;
  }
  
  Serial.println("\n╔═══════════════════════════════════════╗");
  Serial.println("║    ⚖️  INTERACTIVE LOAD CELL CALIB ⚖️  ║");
  Serial.println("║          Simple Pipeline Guide        ║");
  Serial.println("╚═══════════════════════════════════════╝");
  Serial.println("🔧 This will walk you through calibration");
  Serial.println("⏱️  Total time: ~2 minutes");
  Serial.println();
  
  // Step 1: Zero Calibration
  Serial.println("📍 STEP 1/4: ZERO CALIBRATION");
  Serial.println("🚫 Remove ALL weight from the load cell");
  Serial.println("📏 Make sure the platform is empty");
  Serial.println("⏰ You have 10 seconds to prepare...");
  
  for (int i = 10; i > 0; i--) {
    Serial.print(String(i) + "... ");
    delay(1000);
  }
  Serial.println("🔄 GO!");
  
  // Perform zero calibration
  Serial.println("🔧 Calibrating zero point...");
  long total = 0;
  int count = 0;
  for (int i = 0; i < 20; i++) {  // More samples for better accuracy
    if (loadCell.is_ready()) {
      total += loadCell.read();
      count++;
      Serial.print(".");
    }
    delay(100);
  }
  Serial.println();
  
  if (count < 10) {
    Serial.println("❌ FAILED: Not enough readings. Check load cell connection!");
    return;
  }
  
  loadCellOffset = total / count;
  loadCell.set_offset(loadCellOffset);
  Serial.println("✅ Zero calibrated! Offset: " + String(loadCellOffset));
  Serial.println();
  delay(1000);
  
  // Step 2: Get Calibration Weight
  Serial.println("📍 STEP 2/4: CALIBRATION WEIGHT SETUP");
  Serial.println("⚖️  You need a known weight for calibration");
  Serial.println("💡 Common weights: 100g, 500g, 1kg, 2kg");
  Serial.println("📝 Enter the weight you will use (in grams):");
  Serial.println("   Example: 500 (for 500 grams)");
  Serial.print("⚖️  Weight (g): ");
  
  // Wait for user input
  String weightInput = "";
  while (weightInput.length() == 0) {
    if (Serial.available()) {
      weightInput = Serial.readStringUntil('\n');
      weightInput.trim();
    }
    delay(100);
  }
  
  float calibWeight = weightInput.toFloat();
  if (calibWeight <= 0) {
    Serial.println("❌ Invalid weight! Calibration aborted.");
    return;
  }
  
  Serial.println("✅ Using calibration weight: " + String(calibWeight) + "g");
  Serial.println();
  delay(1000);
  
  // Step 3: Weight Calibration
  Serial.println("📍 STEP 3/4: WEIGHT CALIBRATION");
  Serial.println("📦 Place EXACTLY " + String(calibWeight) + "g on the load cell");
  Serial.println("⚠️  Make sure it's centered and stable");
  Serial.println("⏰ You have 15 seconds to place the weight...");
  
  for (int i = 15; i > 0; i--) {
    Serial.print(String(i) + "... ");
    delay(1000);
  }
  Serial.println("🔄 GO!");
  
  // Perform weight calibration
  Serial.println("🔧 Reading calibration weight...");
  total = 0;
  count = 0;
  for (int i = 0; i < 20; i++) {  // More samples for better accuracy
    if (loadCell.is_ready()) {
      total += loadCell.get_value();  // Get value without scale
      count++;
      Serial.print(".");
    }
    delay(100);
  }
  Serial.println();
  
  if (count < 10) {
    Serial.println("❌ FAILED: Not enough readings. Check load cell!");
    return;
  }
  
  // Convert grams to kg for calculation
  float calibWeightKg = calibWeight / 1000.0;
  loadCellCalibrationFactor = (total / count) / calibWeightKg;
  loadCell.set_scale(loadCellCalibrationFactor);
  
  Serial.println("✅ Weight calibrated! Factor: " + String(loadCellCalibrationFactor, 2));
  Serial.println();
  delay(1000);
  
  // Step 4: Test and Save
  Serial.println("📍 STEP 4/4: TEST & SAVE");
  Serial.println("🧪 Testing calibration...");
  
  // Test current reading
  if (loadCell.is_ready()) {
    float testWeight = loadCell.get_units(3) * 1000; // Convert to grams
    Serial.println("📊 Current reading: " + String(testWeight, 1) + "g");
    Serial.println("🎯 Expected: " + String(calibWeight) + "g");
    
    float error = abs(testWeight - calibWeight);
    float errorPercent = (error / calibWeight) * 100;
    
    if (errorPercent < 5) {
      Serial.println("✅ EXCELLENT! Error: " + String(errorPercent, 1) + "%");
    } else if (errorPercent < 10) {
      Serial.println("⚠️  ACCEPTABLE: Error: " + String(errorPercent, 1) + "%");
    } else {
      Serial.println("❌ HIGH ERROR: " + String(errorPercent, 1) + "% - Consider recalibrating");
    }
  }
  
  // Save calibration
  if (filesystemReady) {
    File calFile = LittleFS.open("/loadcell_cal.txt", "w");
    if (calFile) {
      calFile.println("calibration_factor=" + String(loadCellCalibrationFactor, 6));
      calFile.println("zero_offset=" + String(loadCellOffset));
      calFile.close();
      Serial.println("💾 Calibration saved to file!");
    } else {
      Serial.println("⚠️  Could not save calibration file");
    }
  } else {
    Serial.println("⚠️  File system not ready - calibration not saved");
  }
  
  Serial.println();
  Serial.println("🎉 CALIBRATION COMPLETE!");
  Serial.println("📊 Remove weight to test zero reading");
  Serial.println("⚖️  Current calibration factor: " + String(loadCellCalibrationFactor, 2));
  Serial.println("📍 Zero offset: " + String(loadCellOffset));
  Serial.println("💡 Use 'CAL_STATUS' command to test anytime");
  Serial.println("╔═══════════════════════════════════════╗");
  Serial.println("║            🏁 ALL DONE! 🏁            ║");
  Serial.println("╚═══════════════════════════════════════╝");
}

void printStatus() {
  Serial.println("\n=== SYSTEM STATUS ===");
  Serial.println("Flight State: " + String(flightState));
  Serial.println("Barometer: " + String(baroReady ? "OK" : "FAIL"));
  Serial.println("IMU: " + String(mpuReady ? "OK" : "FAIL"));
  Serial.println("Load Cell: " + String(loadCellReady ? "OK" : "FAIL"));
  Serial.println("Filesystem: " + String(filesystemReady ? "OK" : "FAIL"));
  Serial.println("WiFi: " + String(wifiEnabled ? "ACTIVE" : "OFF"));
  
  // Enhanced filtering status display
  Serial.println("Sensor Filtering Status:");
  Serial.println("  Pressure: " + String(ENABLE_PRESSURE_FILTERING ? "FILTERED (13-sample MA+EMA)" : "RAW"));
  Serial.println("  Accelerometer: " + String(ENABLE_ACCEL_FILTERING ? "FILTERED (8-sample MA+EMA)" : "RAW"));
  Serial.println("  Gyroscope: " + String(ENABLE_GYRO_FILTERING ? "FILTERED (3-sample MA)" : "RAW"));
  Serial.println("  Load Cell: RAW (HX711_Raw)");
  
  // Data rate information
  Serial.println("Data Rate: " + String(1000/sendInterval) + " Hz (Mode " + String(DATA_RATE_MODE) + ")");
  
  // Memory information
  Serial.println("Free Heap: " + String(ESP.getFreeHeap()) + " bytes");
  Serial.println("Min Free Heap: " + String(ESP.getMinFreeHeap()) + " bytes");
  Serial.println("Heap Size: " + String(ESP.getHeapSize()) + " bytes");
  
  if (loadCellReady) {
    Serial.println("Calibration Factor: " + String(loadCellCalibrationFactor, 2));
  }
  
  if (flightState == FLIGHT_RECORDING) {
    Serial.println("Recording: " + String(sampleNumber) + " samples");
    unsigned long recordingTime = (millis() - recordingStartTime) / 1000;
    Serial.println("Recording Time: " + String(recordingTime) + " seconds");
  }
  
  if (wifiEnabled) {
    Serial.println("WiFi IP: 192.168.4.1");
    Serial.println("Connected Clients: " + String(WiFi.softAPgetStationNum()));
  }
}

// === WIFI AND WEB SERVER IMPLEMENTATIONS ===
void startWiFi() {
  if (wifiEnabled) {
    Serial.println("WiFi already active");
    return;
  }
  
  Serial.println("Starting WiFi access point...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, "pavi2024");
  
  delay(1000);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.println("WiFi AP started");
  Serial.println("SSID: " + String(WIFI_SSID));
  Serial.println("Password: pavi2024");
  Serial.println("IP: " + IP.toString());
  
  setupWebServer();
  server.begin();
  wifiEnabled = true;
  
  Serial.println("Web server started - access files at http://" + IP.toString());
}

void stopWiFi() {
  if (!wifiEnabled) {
    Serial.println("WiFi already inactive");
    return;
  }
  
  Serial.println("Stopping WiFi access point...");
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiEnabled = false;
  Serial.println("WiFi stopped");
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/download", handleFileDownload);
  server.on("/api/files", handleFileListAPI);
  server.on("/api/system", handleSystemInfoAPI);
  server.on("/api/status", handleStatusAPI);
  server.on("/api/delete", HTTP_POST, handleFileDeleteAPI);
  server.onNotFound(handleNotFound);
}

void handleRoot() {
  server.send(200, "text/html", generateMainPage());
}

void handleFileDownload() {
  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "Missing file parameter");
    return;
  }
  
  String filename = server.arg("file");
  String filepath = "/" + filename;
  
  if (!LittleFS.exists(filepath)) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  
  File file = LittleFS.open(filepath, "r");
  if (!file) {
    server.send(500, "text/plain", "Cannot open file");
    return;
  }
  
  String contentType = getFileType(filename);
  server.streamFile(file, contentType);
  file.close();
}

void handleFileListAPI() {
  String json = "{\"files\":[";
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  bool first = true;
  
  while (file) {
    if (!file.isDirectory()) {
      if (!first) json += ",";
      json += "{\"name\":\"" + String(file.name()) + "\",\"size\":" + String(file.size()) + "}";
      first = false;
    }
    file = root.openNextFile();
  }
  
  json += "],\"count\":" + String(getLittleFSFileCount()) + "}";
  server.send(200, "application/json", json);
}

void handleSystemInfoAPI() {
  String json = "{";
  json += "\"flightState\":" + String(flightState) + ",";
  json += "\"wifiEnabled\":" + String(wifiEnabled ? "true" : "false") + ",";
  json += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
  json += "\"uptime\":" + String(millis()/1000) + ",";
  json += "\"sensors\":{";
  json += "\"barometer\":" + String(baroReady ? "true" : "false") + ",";
  json += "\"imu\":" + String(mpuReady ? "true" : "false") + ",";
  json += "\"loadCell\":" + String(loadCellReady ? "true" : "false");
  json += "}}";
  server.send(200, "application/json", json);
}

void handleStatusAPI() {
  String json = "{";
  json += "\"status\":\"" + String(flightState == FLIGHT_RECORDING ? "RECORDING" : "IDLE") + "\",";
  json += "\"samples\":" + String(sampleNumber) + ",";
  json += "\"clients\":" + String(WiFi.softAPgetStationNum());
  json += "}";
  server.send(200, "application/json", json);
}

void handleFileDeleteAPI() {
  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "Missing file parameter");
    return;
  }
  
  String filename = server.arg("file");
  String filepath = "/" + filename;
  
  if (LittleFS.remove(filepath)) {
    server.send(200, "text/plain", "File deleted successfully");
  } else {
    server.send(500, "text/plain", "Failed to delete file");
  }
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

String generateMainPage() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<title>PAVI Flight Data</title>";
  html += "<meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body{font-family:Arial,sans-serif;margin:20px;background-color:#f5f5f5;}";
  html += ".container{max-width:800px;margin:0 auto;background:white;padding:20px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1);}";
  html += ".header{text-align:center;color:#2c3e50;border-bottom:3px solid #3498db;padding-bottom:15px;margin-bottom:20px;}";
  html += ".file-list{margin:20px 0;}";
  html += ".file-item{display:flex;justify-content:space-between;align-items:center;padding:10px;border:1px solid #ddd;margin:5px 0;border-radius:5px;background-color:#f9f9f9;}";
  html += ".btn{padding:8px 15px;margin:5px;border:none;border-radius:5px;cursor:pointer;text-decoration:none;display:inline-block;}";
  html += ".btn-download{background-color:#27ae60;color:white;}";
  html += ".btn-delete{background-color:#e74c3c;color:white;}";
  html += ".status{background-color:#ecf0f1;padding:15px;border-radius:5px;margin:20px 0;}";
  html += ".btn:hover{opacity:0.8;}";
  html += "</style>";
  html += "<script>";
  html += "function deleteFile(filename){";
  html += "if(confirm('Delete '+filename+'?')){";
  html += "var xhr=new XMLHttpRequest();";
  html += "xhr.open('POST','/api/delete',true);";
  html += "xhr.setRequestHeader('Content-type','application/x-www-form-urlencoded');";
  html += "xhr.onload=function(){alert(xhr.responseText);location.reload();};";
  html += "xhr.send('file='+filename);}}";
  html += "</script></head><body>";
  
  html += "<div class='container'>";
  html += "<div class='header'>";
  html += "<h1>🚀 PAVI Flight Computer</h1>";
  html += "<h2>📡 " + String(WIFI_SSID) + "</h2>";
  html += "</div>";
  
  html += "<div class='status'>";
  html += "<h3>📊 System Status</h3>";
  html += "<p><strong>Flight State:</strong> " + String(flightState == FLIGHT_RECORDING ? "🔴 RECORDING" : "💤 IDLE") + "</p>";
  html += "<p><strong>Connected Clients:</strong> " + String(WiFi.softAPgetStationNum()) + "</p>";
  html += "<p><strong>Uptime:</strong> " + String(millis()/60000) + " minutes</p>";
  html += "<p><strong>Free Memory:</strong> " + String(ESP.getFreeHeap()/1024) + " KB</p>";
  html += "</div>";
  
  html += "<div class='file-list'>";
  html += "<h3>📁 Available Files</h3>";
  
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  int fileCount = 0;
  
  while (file) {
    if (!file.isDirectory()) {
      String fileName = String(file.name());
      html += "<div class='file-item'>";
      html += "<span><strong>" + fileName + "</strong> (" + String(file.size()/1024) + " KB)</span>";
      html += "<div>";
      html += "<a href='/download?file=" + fileName + "' class='btn btn-download'>📥 Download</a>";
      html += "<button onclick='deleteFile(\"" + fileName + "\")' class='btn btn-delete'>🗑️ Delete</button>";
      html += "</div></div>";
      fileCount++;
    }
    file = root.openNextFile();
  }
  
  if (fileCount == 0) {
    html += "<p>📂 No files available</p>";
  }
  
  html += "</div>";
  
  html += "<div style='text-align:center;margin-top:30px;color:#7f8c8d;'>";
  html += "<p>🔄 Refresh this page to see new files</p>";
  html += "<p>💡 Download CSV files and open in Excel or similar</p>";
  html += "</div>";
  
  html += "</div></body></html>";
  return html;
}

String getFileType(String filename) {
  if (filename.endsWith(".csv")) return "text/csv";
  if (filename.endsWith(".txt")) return "text/plain";
  if (filename.endsWith(".json")) return "application/json";
  return "application/octet-stream";
}

int getLittleFSFileCount() {
  int count = 0;
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) count++;
    file = root.openNextFile();
  }
  return count;
}


