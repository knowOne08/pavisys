#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <MS5611.h>  
#include <MPU6050.h>
#include "HX711_Raw.h"  // Custom raw HX711 implementation (no internal filtering)
#include "FS.h"
#include "LittleFS.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>


// === FLIGHT COMPUTER CONFIGURATION ===
// Change this single macro to switch between two FC builds
#define FC_NO 2  // 1 = FC1 (20Hz, "PaviFlightData"), 2 = FC2 (40Hz, "PaviFlightData-2")

// Auto-configure based on FC number
#if FC_NO == 1
  #define DATA_RATE_MODE 2        // 20Hz for FC1
  #define WIFI_SSID "PaviFlightData"
#elif FC_NO == 2  
  #define DATA_RATE_MODE 4        // 40Hz for FC2
  #define WIFI_SSID "PaviFlightData-2"
#else
  #error "Invalid FC_NO! Please set FC_NO to 1 or 2"
#endif

/*
 * TO SWITCH BETWEEN FLIGHT COMPUTERS:
 * 1. Change FC_NO to 1 or 2
 * 2. That's it! The system automatically configures:
 *    - FC1: 20Hz data rate + "PaviFlightData" WiFi SSID
 *    - FC2: 40Hz data rate + "PaviFlightData-2" WiFi SSID
 */

// === SENSOR FILTERING CONFIGURATION ===
// Set to false for RAW mode (no filtering), true for SMOOTH mode (with filtering)
#define ENABLE_SENSOR_FILTERING false  // Change this to toggle ALL sensor filtering

// Individual sensor filtering overrides (only used if ENABLE_SENSOR_FILTERING is true)
#define ENABLE_PRESSURE_FILTERING true   // Barometer smoothing (heavy filtering)
#define ENABLE_ACCEL_FILTERING true      // Accelerometer smoothing (medium filtering) 
#define ENABLE_GYRO_FILTERING true       // Gyroscope smoothing (light filtering)
#define ENABLE_LOADCELL_FILTERING false  // Load cell is always raw (HX711_Raw library)

/*
 * FILTERING MODES:
 * 
 * RAW MODE (ENABLE_SENSOR_FILTERING = false):
 * - All sensors provide direct, unfiltered readings
 * - Maximum responsiveness and sensitivity
 * - Best for flight data analysis and research
 * - May be noisier but shows true sensor behavior
 * 
 * SMOOTH MODE (ENABLE_SENSOR_FILTERING = true):
 * - Sensors use moving average and exponential filtering
 * - Smoother data for display and basic flight control
 * - Better for real-time monitoring during flight
 * - Some delay but cleaner readings
 * 
 * MIXED MODE:
 * - Set ENABLE_SENSOR_FILTERING = true
 * - Then adjust individual sensor filtering flags above
 * - Customize which sensors are filtered
 */

// LoRa pin mapping
#define NSS   5
#define RST   17
#define DIO0  13

// Pyro channel MOSFET pins
#define PYRO_1  33  // GPIO33
#define PYRO_2  32  // GPIO32
#define PYRO_3  25  // GPIO25
#define PYRO_4  26  // GPIO26

// HX711 Load Cell pins
#define HX711_DOUT  4   // GPIO4 - Data pin
#define HX711_SCK   16   // GPIO16 - Clock pin

// LittleFS configuration
#define FORMAT_LITTLEFS_IF_FAILED true

// Test mode - set to true to test serial only, false for full operation
#define SERIAL_TEST_MODE false

// Test mode for pyro channels - set to true to test LEDs
#define PYRO_TEST_MODE false

// LORA TRANSMISSION CONTROL - TX is now pure command responder
#define LORA_TRANSMISSION_MODE 0   // TX never sends unsolicited telemetry

// TX is always in receive mode, only responds to commands
bool commandReceived = false;
String lastCommand = "";
unsigned long lastCommandTime = 0;

// Sensor-specific rate limits (Hz)
#define MAX_BARO_RATE 100     // MS5611 theoretical max  
#define MAX_IMU_RATE 1000     // MPU6050 max rate
#define MAX_HX711_RATE_HIGH_GAIN 10   // HX711 high precision
#define MAX_HX711_RATE_LOW_GAIN 80    // HX711 fast mode

// Sensor enable flags - set to false to disable specific sensors
bool ENABLE_BAROMETER = true;
bool ENABLE_ACCELEROMETER = true;
bool ENABLE_GYROSCOPE = true;
bool ENABLE_TEMPERATURE = false;  
bool ENABLE_LOAD_CELL = true;   // Enable for parachute testing

// === SERIAL DATA TOGGLE ===
bool showSerialData = false; // Set to false to suppress serial data output

// === FLIGHT DATA LOGGING SYSTEM ===
// Flight test configuration
struct FlightConfig {
  String filename;        // Data file name
  float totalWeight;      // Total weight in kg
  float windSpeed;        // Wind speed in m/s
  float initialHeight;    // Initial height in m
  unsigned long startTime; // Flight start timestamp
};

// Flight state management
enum FlightState {
  FLIGHT_IDLE,           // Waiting for configuration
  FLIGHT_CONFIGURING,    // Collecting configuration parameters
  FLIGHT_CONFIGURED,     // Configuration received, ready to start
  FLIGHT_RECORDING,      // Active data recording
  FLIGHT_STOPPED         // Recording stopped
};

// System state variables
FlightState flightState = FLIGHT_IDLE;
FlightConfig flightConfig;
bool filesystemReady = false;
File dataFile;
unsigned long lastLogTime = 0;
unsigned long recordingStartTime = 0;
unsigned long sampleNumber = 0;

// Data buffering for efficient writes
String dataBuffer = "";
int bufferLineCount = 0;
const int BUFFER_SIZE = 20;  // Lines to buffer before writing

// Calculated data
float verticalVelocity = 0.0;
float lastAltitude = 0.0;
unsigned long lastVelocityTime = 0;

// Remote command system
bool waitingForCommand = false;
String commandBuffer = "";

// === MENU SYSTEM ===
bool menuMode = false;
int currentMenu = 0;
int currentSubMenu = 0;

// Menu constants
#define MENU_MAIN 0
#define MENU_FLIGHT 1
#define MENU_FILES 2
#define MENU_WIFI 3
#define MENU_CALIBRATION 4

// === WIFI SOFTAP CONFIGURATION ===
// SSID is auto-configured based on FC_NO above

#define WIFI_PASSWORD ""
#define WIFI_CHANNEL 6
#define MAX_CONNECTIONS 4

// WiFi and Web Server objects
WebServer server(80);
bool wifiEnabled = false;
IPAddress apIP(192, 168, 4, 1);
IPAddress netMsk(255, 255, 255, 0);

// Data logging control
bool enableDataLogging = true;

// Moving Average filter parameters  
#define PRESSURE_FILTER_SIZE 13   // Much larger filter for very smooth altitude
#define ACCEL_FILTER_SIZE 8       // Medium filter for acceleration
#define GYRO_FILTER_SIZE 3        // Smaller filter for gyro (more responsive)

// Filter buffers for different sensors
float pressureReadings[PRESSURE_FILTER_SIZE];
float accelXReadings[ACCEL_FILTER_SIZE];
float accelYReadings[ACCEL_FILTER_SIZE]; 
float accelZReadings[ACCEL_FILTER_SIZE];
float gyroXReadings[GYRO_FILTER_SIZE];
float gyroYReadings[GYRO_FILTER_SIZE];
float gyroZReadings[GYRO_FILTER_SIZE];

int pressureIndex = 0;
int accelIndex = 0;
int gyroIndex = 0;

// Sensor objects
MS5611 baro;
MPU6050 mpu;
HX711_Raw loadCell;  // Using raw implementation (no internal filtering)
bool baroReady = false;
bool mpuReady = false;
bool loadCellReady = false;

// Sensor data variables
float pressure, temperature, altitude;
int16_t ax, ay, az, gx, gy, gz;
float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;

// Altitude calculation parameters
#define SEA_LEVEL_PRESSURE 1013.25  // Standard sea level pressure in hPa
float referenceAltitude = 0.0;      // Reference altitude for relative height (set on startup)
long loadCellRaw;
float loadCellWeight;
float loadCellCalibrationFactor = -7050.0;  // Calibration factor (adjust for your load cell)
float loadCellOffset = 0.0;  // Tare offset

// Pyro channel variables
int pyroPins[] = {PYRO_1, PYRO_2, PYRO_3, PYRO_4};
int pyroTestCycles = 0;
bool pyroTestCompleted = false;

unsigned long lastSend = 0;
// Calculate send interval based on data rate mode
const unsigned long sendInterval = (DATA_RATE_MODE == 1) ? 100 :   // 10Hz
                                   (DATA_RATE_MODE == 2) ? 50 :    // 20Hz  
                                   (DATA_RATE_MODE == 3) ? 33 :    // 30Hz (~33ms)
                                   (DATA_RATE_MODE == 4) ? 25 : 50; // 40Hz (25ms)

unsigned long sensorReadCount = 0;

// LoRa transmission timing
unsigned long lastLoraTransmission = 0;
const unsigned long loraTransmissionInterval = (LORA_TRANSMISSION_MODE == 0) ? 0 :     // OFF
                                               (LORA_TRANSMISSION_MODE == 1) ? 200 :   // 5Hz (200ms)
                                               (LORA_TRANSMISSION_MODE == 2) ? 100 : 0; // 10Hz (100ms)

/* 
=== SIMPLIFIED COMMAND/RESPONSE ARCHITECTURE ===
TX (Flight Computer): Pure command responder
- Never sends unsolicited telemetry
- Always listens for LoRa commands
- Only sends data when requested by RX
- Continues sensor reading for local logging

RX (Ground Station): Pure command sender  
- Sends commands immediately when entered
- Waits for responses with timeout
- No continuous telemetry reception

DATA RATE (sensor reading frequency for local logging):
Mode 1 - 10Hz: 100ms interval, Load cell every 2nd cycle (5Hz)
Mode 2 - 20Hz: 50ms interval,  Load cell every 3rd cycle (6.7Hz) 
Mode 3 - 30Hz: 33ms interval,  Load cell every 4th cycle (7.5Hz)
Mode 4 - 40Hz: 25ms interval,  Load cell every 5th cycle (8Hz)
*/

// === FUNCTION DECLARATIONS ===
// WiFi and Web Server functions
void startWiFiSoftAP();
void stopWiFiSoftAP();
void setupWebServer();
void handleRoot();
void handleFileDownload();
void handleFileListAPI();
void handleSystemInfoAPI();
void handleFileDeleteAPI();
void handleNotFound();
void handleWiFiClients();

// Calibration functions
void startLoadCellCalibration();
void stopLoadCellCalibration();
bool isLoadCellCalibrating();
void calibrateLoadCellZero();
void calibrateLoadCellWeight(float weight);
void saveLoadCellCalibration();
void loadCalibrationFromFile();
void testLoadCellCalibration();

// Flight Data functions
void createDataFile();
void writeDataHeader();
void logFlightData();
void logOffsetEvent();
void logPyroEvent(int channel);
void firePyroChannel(int channel);

// File Management functions
void listFiles();
void downloadFile(String filename);
void deleteFile(String filename);
void showFilesystemSpace();
void formatFilesystem();

// Web interface functions
String generateMainPage();
String getFileType(String filename);
int getLittleFSFileCount();

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("=== TX: LoRa Command Responder ===");
  Serial.println("Pure command responder - no unsolicited telemetry");
  Serial.println("Sensors: MS5611 + MPU6050 + HX711_Raw + 4x Pyro Channels");
  Serial.print("Barometer: "); Serial.println(ENABLE_BAROMETER ? "ON" : "OFF");
  Serial.print("Accelerometer: "); Serial.println(ENABLE_ACCELEROMETER ? "ON" : "OFF");
  Serial.print("Gyroscope: "); Serial.println(ENABLE_GYROSCOPE ? "ON" : "OFF");
  Serial.print("Temperature: "); Serial.println(ENABLE_TEMPERATURE ? "ON" : "OFF");
  Serial.print("Load Cell: "); Serial.println(ENABLE_LOAD_CELL ? "ON (RAW MODE)" : "OFF");
  Serial.print("Pyro Test Mode: "); Serial.println(PYRO_TEST_MODE ? "ON" : "OFF");
  
  // Display filtering status
  Serial.println("");
  Serial.println("=== SENSOR FILTERING STATUS ===");
  if (ENABLE_SENSOR_FILTERING) {
    Serial.println("📊 FILTERING MODE: SMOOTH (filtered data)");
    Serial.print("  • Pressure filtering: "); Serial.println(ENABLE_PRESSURE_FILTERING ? "ON" : "OFF");
    Serial.print("  • Accelerometer filtering: "); Serial.println(ENABLE_ACCEL_FILTERING ? "ON" : "OFF");
    Serial.print("  • Gyroscope filtering: "); Serial.println(ENABLE_GYRO_FILTERING ? "ON" : "OFF");
    Serial.print("  • Load cell filtering: ALWAYS OFF (HX711_Raw)");
  } else {
    Serial.println("🎯 FILTERING MODE: RAW (unfiltered data)");
    Serial.println("  • All sensors provide direct, unfiltered readings");
    Serial.println("  • Maximum responsiveness and sensitivity");
    Serial.println("  • Best for flight data analysis and research");
  }
  Serial.println("💡 Change ENABLE_SENSOR_FILTERING to toggle modes");
  Serial.println("=======================================");
  Serial.println("");
  
  if (SERIAL_TEST_MODE) {
    Serial.println("RUNNING IN SERIAL TEST MODE");
    Serial.println("Hardware initialization SKIPPED for testing");
    Serial.println("Change SERIAL_TEST_MODE to false for full operation");
    Serial.println("============================");
    return;
  }

  // Initialize Pyro Channels
  Serial.println("Initializing Pyro Channels...");
  for (int i = 0; i < 4; i++) {
    pinMode(pyroPins[i], OUTPUT);
    digitalWrite(pyroPins[i], LOW);  // Ensure all channels start OFF
    Serial.print("Pyro Channel "); Serial.print(i+1); 
    Serial.print(" (GPIO"); Serial.print(pyroPins[i]); Serial.println(") - Ready");
  }
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize LoRa
  Serial.println("Initializing LoRa...");
  LoRa.setPins(NSS, RST, DIO0);
  Serial.print(">>> Initializing LoRa on 433MHz...");
  if (!LoRa.begin(433E6)) {
    Serial.println(" FAILED!");
    Serial.println(">>> ERROR: LoRa initialization failed - check wiring");
    return; // Don't continue if LoRa fails
  } else {
    Serial.println(" SUCCESS!");
    Serial.println(">>> LoRa module ready");
    
    // Configure LoRa settings for better compatibility
    LoRa.setSpreadingFactor(7);     // SF7 (faster data rate)
    LoRa.setSignalBandwidth(125E3); // 125 kHz bandwidth
    LoRa.setCodingRate4(5);         // 4/5 coding rate
    LoRa.setPreambleLength(8);      // 8 symbol preamble
    LoRa.setSyncWord(0x12);         // Private sync word
    LoRa.setTxPower(20);            // Max power for range
    LoRa.crc();                     // Enable CRC
    
    Serial.println("LoRa configured: SF7, BW125, CR4/5, Pwr20dBm");
    
    // TX is pure command responder - always in receive mode
    LoRa.receive();
    Serial.println("*** TX READY FOR LORA TEST ***");
    Serial.println("*** Waiting for LoRa packets from RX ***");
    Serial.println("*** Send PING from RX to test ***");
    
    // Test LoRa module responsiveness
    Serial.print(">>> LoRa module test - Frequency: ");
    // Serial.print(LoRa.());
    Serial.println(" Hz");
    Serial.print(">>> Spreading Factor: ");
    // Serial.println(LoRa.getSpreadingFactor());
    Serial.print(">>> Signal Bandwidth: ");
    // Serial.println(LoRa.getSignalBandwidth());
    Serial.println(">>> If you see this, LoRa module is responsive");
  }

  // Initialize MS5611 Barometer
  if (ENABLE_BAROMETER || ENABLE_TEMPERATURE) {
    Serial.println("Initializing MS5611...");
    if (!baro.begin()) {
      Serial.println("MS5611 not detected!");
      baroReady = false;
    } else {
      Serial.println("MS5611 ready");
      baroReady = true;
      
      // Wait a bit for sensor to stabilize
      delay(1000);
      
      // Set reference altitude (current location as zero)
      Serial.println("Setting reference altitude...");
      float refPressure = 0;
      int validReadings = 0;
      
      for (int i = 0; i < 20; i++) {  // Try more readings for better average
        baro.read();  // Read from sensor
        float currentPressure = baro.getPressure();
        
        if (currentPressure > 500 && currentPressure < 1200) {  // Valid range check
          refPressure += currentPressure;
          validReadings++;
          Serial.print("Valid pressure reading ");
          Serial.print(validReadings);
          Serial.print(": ");
          Serial.print(currentPressure);
          Serial.println(" hPa");
        } else {
          Serial.print("Invalid pressure reading: ");
          Serial.print(currentPressure);
          Serial.println(" hPa");
        }
        delay(100);
      }
      if (validReadings > 0) {
        refPressure /= validReadings;  // Average of valid readings
        Serial.print("Average reference pressure: ");
        Serial.print(refPressure);
        Serial.println(" hPa");
        
        // Calculate reference altitude with validation
        if (refPressure > 500 && refPressure < 1200) {  // Valid pressure range
          float pressureRatio = refPressure / SEA_LEVEL_PRESSURE;
          referenceAltitude = 44330.0 * (1.0 - pow(pressureRatio, 0.1903));
          Serial.print("Calculated reference altitude: ");
          Serial.print(referenceAltitude);
          Serial.println(" m");
        } else {
          referenceAltitude = 0.0;  // Fallback for invalid pressure
          Serial.println("Warning: Invalid reference pressure reading");
        }
      } else {
        referenceAltitude = 0.0;
        Serial.println("Warning: No valid pressure readings obtained");
      }
      Serial.print("Reference altitude set to: ");
      Serial.print(referenceAltitude);
      Serial.println(" m (relative height will be 0 at startup)");
    }
  }

  // Initialize MPU6050 IMU
  if (ENABLE_ACCELEROMETER || ENABLE_GYROSCOPE) {
    Serial.println("Initializing MPU6050...");
    mpu.initialize();
    if (!mpu.testConnection()) {
      Serial.println("MPU6050 not detected!");
      mpuReady = false;
    } else {
      Serial.println("MPU6050 ready");
      // Configure MPU6050 for optimal performance
      mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);  // ±8g
      mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000); // ±1000°/s
      mpu.setDLPFMode(MPU6050_DLPF_BW_42); // Low pass filter
      mpuReady = true;
    }
  }

  // Initialize HX711 Load Cell (RAW - No Internal Filtering)
  if (ENABLE_LOAD_CELL) {
    Serial.println("Initializing HX711 Load Cell (RAW MODE - No Internal Filtering)...");
    loadCell.begin(HX711_DOUT, HX711_SCK);
    
    if (loadCell.is_ready()) {
      Serial.println("HX711 ready (RAW mode for maximum responsiveness)");
      
      // Set initial calibration factor (default)
      loadCell.set_scale(loadCellCalibrationFactor);
      
      // Tare the scale (set current reading as zero)
      Serial.println("Taring load cell... (remove any weight)");
      delay(2000);  // Give time to remove weight
      loadCell.tare();
      loadCellOffset = loadCell.get_offset();
      
      Serial.print("Load cell tared. Offset: ");
      Serial.println(loadCellOffset);
      Serial.println("📈 RAW MODE: No internal smoothing - you get direct HX711 readings!");
      loadCellReady = true;
      
      // Try to load saved calibration
      Serial.println("🔍 Checking for saved calibration...");
      loadCalibrationFromFile();
    } else {
      Serial.println("HX711 not detected!");
      loadCellReady = false;
    }
  }

  // Initialize filter buffers
  for (int i = 0; i < PRESSURE_FILTER_SIZE; i++) {
    pressureReadings[i] = 1013.25;  // Initialize with standard pressure
  }
  for (int i = 0; i < ACCEL_FILTER_SIZE; i++) {
    accelXReadings[i] = 0;
    accelYReadings[i] = 0; 
    accelZReadings[i] = 0;
  }
  for (int i = 0; i < GYRO_FILTER_SIZE; i++) {
    gyroXReadings[i] = 0;
    gyroYReadings[i] = 0;
    gyroZReadings[i] = 0;
  }
  
  // Initialize LittleFS for data logging
  Serial.println("Initializing LittleFS...");
  if (LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) {
    Serial.println("LittleFS filesystem ready");
    filesystemReady = true;
    
    // Print filesystem info
    size_t totalBytes = LittleFS.totalBytes();
    size_t usedBytes = LittleFS.usedBytes();
    Serial.print("Total space: "); Serial.print(totalBytes / 1024); Serial.println(" KB");
    Serial.print("Used space: "); Serial.print(usedBytes / 1024); Serial.println(" KB");
    Serial.print("Free space: "); Serial.print((totalBytes - usedBytes) / 1024); Serial.println(" KB");
  } else {
    Serial.println("LittleFS initialization failed - logging to serial only");
    filesystemReady = false;
  }
  
  Serial.println("=== System Ready ===");
  
  // Print flight computer status
  printSystemStatus();
  
  Serial.print("Sensor Data Rate: "); 
  switch(DATA_RATE_MODE) {
    case 1:
      Serial.println("*** 10 Hz MODE *** (100ms interval)");
      Serial.println("Load cell: Read every 2nd cycle (5 Hz effective)");
      break;
    case 2:
      Serial.println("*** 20 Hz MODE *** (50ms interval)"); 
      Serial.println("Load cell: Read every 3rd cycle (6.7 Hz effective)");
      break;
    case 3:
      Serial.println("*** 30 Hz MODE *** (33ms interval)"); 
      Serial.println("Load cell: Read every 4th cycle (7.5 Hz effective)");
      break;
    case 4:
      Serial.println("*** 40 Hz MODE *** (25ms interval)"); 
      Serial.println("Load cell: Read every 5th cycle (8 Hz effective)");
      break;
    default:
      Serial.println("*** 20 Hz MODE *** (default)");
  }
  Serial.println("Fast sensors (Baro, IMU): Full rate");
  Serial.print("Sensor read interval: ");
  Serial.print(sendInterval);
  Serial.println(" ms");
  
  // Show comprehensive flash information
  Serial.println("\n=== FLASH MEMORY BREAKDOWN ===");
  Serial.print("Total ESP32 Flash: ~");
  Serial.print(ESP.getFlashChipSize() / (1024 * 1024));
  Serial.println(" MB (hardware)");
  
  Serial.println("Flash Partitions:");
  Serial.println("  • Bootloader: ~32 KB");
  Serial.println("  • Partition Table: ~4 KB"); 
  Serial.println("  • nvs (settings): ~24 KB");
  Serial.println("  • otadata (OTA): ~8 KB");
  Serial.println("  • app0 (firmware): ~1.3 MB");
  Serial.println("  • app1 (OTA backup): ~1.3 MB");
  Serial.print("  • spiffs/LittleFS (data): ~");
  Serial.print(LittleFS.totalBytes() / 1024);
  Serial.println(" KB ← Available for flight data");
  
  Serial.print("LittleFS Available: ");
  Serial.print(LittleFS.totalBytes() / 1024);
  Serial.print(" KB (");
  Serial.print(LittleFS.totalBytes() / (1024.0 * 1024.0), 2);
  Serial.println(" MB)");
  
  Serial.println("💡 To get more data storage:");
  Serial.println("   • Use custom partition table");
  Serial.println("   • Remove OTA partition (app1)"); 
  Serial.println("   • Could increase to ~2.7 MB data storage");
  
  // Storage and recording time estimates
  Serial.println("\n=== RECORDING TIME ESTIMATES ===");
  Serial.print("Data precision: OPTIMIZED (2-3 decimal places)\n");
  Serial.print("Estimated bytes per sample: ~70 bytes\n");
  Serial.print("At 40Hz: ~2.8 KB/second\n");
  Serial.print("Estimated recording time: ~8-10 minutes\n");
  Serial.println("Use 'SPACE' command to monitor storage usage");
  Serial.println("=====================================");
  
  Serial.println("================================");
  
  Serial.println("LoRa Mode: *** COMMAND RESPONDER ONLY ***");
  Serial.println("- TX never sends unsolicited telemetry");
  Serial.println("- TX only responds to RX commands");
  Serial.println("- RX sends commands and waits for responses");
  
  // Run pyro channel test if enabled
  if (PYRO_TEST_MODE) {
    Serial.println();
    Serial.println("=== PYRO CHANNEL TEST STARTING ===");
    Serial.println("Running moving light pattern 3 times...");
    runPyroTest();
  }
}

void loop() {
  // Process incoming serial commands
  processSerialCommands();
  
  // TX is now a pure command responder - always listen for LoRa commands
  handleLoRaCommands();
  
  // Read sensors periodically for local logging (no LoRa transmission)
  static unsigned long lastSensorRead = 0;
  static unsigned long lastTimingDebug = 0;
  static unsigned long timingSum = 0;
  static int timingCount = 0;
  
  if (millis() - lastSensorRead >= sendInterval) {
    unsigned long actualInterval = millis() - lastSensorRead;
    lastSensorRead = millis();
    sensorReadCount++;
    
    // Track timing statistics
    timingSum += actualInterval;
    timingCount++;
    
    // Print timing debug every 5 seconds
    if (millis() - lastTimingDebug > 5000) {
      lastTimingDebug = millis();
      float avgInterval = (float)timingSum / timingCount;
      float avgRate = 1000.0 / avgInterval;
      Serial.print("🔧 Timing Debug: Target="); Serial.print(sendInterval);
      Serial.print("ms, Actual Avg="); Serial.print(avgInterval, 1);
      Serial.print("ms, Rate="); Serial.print(avgRate, 1); Serial.println("Hz");
      timingSum = 0;
      timingCount = 0;
    }
    
    // Read sensors with smart load cell timing
    readSensorsWithSmartLoadCell();
    
    // Calculate additional flight parameters
    calculateVerticalVelocity();
    
    // Log flight data immediately after sensor read if recording
    if (flightState == FLIGHT_RECORDING && enableDataLogging) {
      logFlightData();
    }
  }
  
  // Handle WiFi web server clients
  handleWiFiClients();
}

// Removed unused readSensors() function - using readSensorsWithSmartLoadCell() instead

// Removed unused readSensorsOptimized() function - using readSensorsWithSmartLoadCell() instead

// Helper functions for conditional filtering
float filterPressure(float newReading) {
  // Check if pressure filtering is enabled
  if (!ENABLE_SENSOR_FILTERING || !ENABLE_PRESSURE_FILTERING) {
    return newReading;  // Return raw reading without any filtering
  }
  
  static float total = 0;
  static bool initialized = false;
  static float exponentialAverage = 0;
  static int readingCount = 0;
  
  // Initialize total with the sum of initial buffer values on first call
  if (!initialized) {
    total = 0;
    for (int i = 0; i < PRESSURE_FILTER_SIZE; i++) {
      total += pressureReadings[i];  // Sum up the 1013.25 values
    }
    readingCount = PRESSURE_FILTER_SIZE;  // We start with a full buffer
    exponentialAverage = total / PRESSURE_FILTER_SIZE;  // Initialize with average
    initialized = true;
  }
  
  // Simple moving average
  total = total - pressureReadings[pressureIndex];
  pressureReadings[pressureIndex] = newReading;
  total = total + pressureReadings[pressureIndex];
  pressureIndex = (pressureIndex + 1) % PRESSURE_FILTER_SIZE;
  
  // Calculate moving average 
  float movingAverage = total / PRESSURE_FILTER_SIZE;
  
  // Add exponential smoothing for extra stability
  // Alpha = 0.1 for very smooth filtering (90% previous, 10% new)
  exponentialAverage = 0.9 * exponentialAverage + 0.1 * movingAverage;
  
  return exponentialAverage;
}

float filterAccelX(float newReading) {
  // Check if acceleration filtering is enabled
  if (!ENABLE_SENSOR_FILTERING || !ENABLE_ACCEL_FILTERING) {
    return newReading;  // Return raw reading without any filtering
  }
  
  static float total = 0;
  static float exponentialAverage = 0;
  static bool initialized = false;
  
  total = total - accelXReadings[accelIndex];
  accelXReadings[accelIndex] = newReading;
  total = total + accelXReadings[accelIndex];
  
  float movingAverage = total / ACCEL_FILTER_SIZE;
  
  // Light exponential smoothing for acceleration
  if (!initialized) {
    exponentialAverage = movingAverage;
    initialized = true;
  } else {
    exponentialAverage = 0.7 * exponentialAverage + 0.3 * movingAverage;
  }
  
  return exponentialAverage;
}

float filterAccelY(float newReading) {
  // Check if acceleration filtering is enabled
  if (!ENABLE_SENSOR_FILTERING || !ENABLE_ACCEL_FILTERING) {
    return newReading;  // Return raw reading without any filtering
  }
  
  static float total = 0;
  static float exponentialAverage = 0;
  static bool initialized = false;
  
  total = total - accelYReadings[accelIndex];
  accelYReadings[accelIndex] = newReading;
  total = total + accelYReadings[accelIndex];
  
  float movingAverage = total / ACCEL_FILTER_SIZE;
  
  if (!initialized) {
    exponentialAverage = movingAverage;
    initialized = true;
  } else {
    exponentialAverage = 0.7 * exponentialAverage + 0.3 * movingAverage;
  }
  
  return exponentialAverage;
}

float filterAccelZ(float newReading) {
  // Check if acceleration filtering is enabled
  if (!ENABLE_SENSOR_FILTERING || !ENABLE_ACCEL_FILTERING) {
    accelIndex = (accelIndex + 1) % ACCEL_FILTER_SIZE; // Still update index for consistency
    return newReading;  // Return raw reading without any filtering
  }
  
  static float total = 0;
  static float exponentialAverage = 0;
  static bool initialized = false;
  
  total = total - accelZReadings[accelIndex];
  accelZReadings[accelIndex] = newReading;
  total = total + accelZReadings[accelIndex];
  accelIndex = (accelIndex + 1) % ACCEL_FILTER_SIZE; // Update index after all accel
  
  float movingAverage = total / ACCEL_FILTER_SIZE;
  
  if (!initialized) {
    exponentialAverage = movingAverage;
    initialized = true;
  } else {
    exponentialAverage = 0.7 * exponentialAverage + 0.3 * movingAverage;
  }
  
  return exponentialAverage;
}

float filterGyroX(float newReading) {
  // Check if gyroscope filtering is enabled
  if (!ENABLE_SENSOR_FILTERING || !ENABLE_GYRO_FILTERING) {
    return newReading;  // Return raw reading without any filtering
  }
  
  static float total = 0;
  
  total = total - gyroXReadings[gyroIndex];
  gyroXReadings[gyroIndex] = newReading;
  total = total + gyroXReadings[gyroIndex];
  
  return total / GYRO_FILTER_SIZE;
}

float filterGyroY(float newReading) {
  // Check if gyroscope filtering is enabled
  if (!ENABLE_SENSOR_FILTERING || !ENABLE_GYRO_FILTERING) {
    return newReading;  // Return raw reading without any filtering
  }
  
  static float total = 0;
  
  total = total - gyroYReadings[gyroIndex];
  gyroYReadings[gyroIndex] = newReading;
  total = total + gyroYReadings[gyroIndex];
  
  return total / GYRO_FILTER_SIZE;
}

float filterGyroZ(float newReading) {
  // Check if gyroscope filtering is enabled
  if (!ENABLE_SENSOR_FILTERING || !ENABLE_GYRO_FILTERING) {
    gyroIndex = (gyroIndex + 1) % GYRO_FILTER_SIZE; // Still update index for consistency
    return newReading;  // Return raw reading without any filtering
  }
  
  static float total = 0;
  
  total = total - gyroZReadings[gyroIndex];
  gyroZReadings[gyroIndex] = newReading;
  total = total + gyroZReadings[gyroIndex];
  gyroIndex = (gyroIndex + 1) % GYRO_FILTER_SIZE; // Update index after all gyro
  
  return total / GYRO_FILTER_SIZE;
}

String createDataPacket() {
  String packet = "";
  bool first = true;
  
  // Compact telemetry packet format: TYPE:VALUE|TYPE:VALUE|...
  // H=Height, V=VerticalVel, W=Weight, A=Accel(X,Y,Z), G=Gyro(X,Y,Z), T=Temperature
  
  if (ENABLE_BAROMETER && baroReady) {
    if (!first) packet += "|";
    packet += "H:" + String(altitude, 1);  // Height in meters (1 decimal place)
    first = false;
    
    // Add vertical velocity
    if (!first) packet += "|";
    packet += "V:" + String(verticalVelocity, 2);  // Vertical velocity in m/s
    first = false;
  }
  
  if (ENABLE_LOAD_CELL && loadCellReady) {
    if (!first) packet += "|";
    packet += "W:" + String(loadCellWeight, 2);  // Weight in kg (2 decimal places)
    first = false;
  }
  
  if (ENABLE_ACCELEROMETER && mpuReady) {
    if (!first) packet += "|";
    packet += "A:" + String(accelX, 2) + "," + String(accelY, 2) + "," + String(accelZ, 2);  // m/s²
    first = false;
  }
  
  if (ENABLE_GYROSCOPE && mpuReady) {
    if (!first) packet += "|";
    packet += "G:" + String(gyroX, 3) + "," + String(gyroY, 3) + "," + String(gyroZ, 3);  // rad/s
    first = false;
  }
  
  if (ENABLE_TEMPERATURE && baroReady) {
    if (!first) packet += "|";
    packet += "T:" + String(temperature, 1);
    first = false;
  }
  
  // Add flight state indicator
  if (!first) packet += "|";
  packet += "S:" + String(flightState);
  
  return packet;
}

void runPyroTest() {
  Serial.println("Turning ON all pyro channels for 5 seconds...");
  Serial.println("*** Use multimeter to test each GPIO pin now ***");
  
  // Turn ON all channels
  for (int i = 0; i < 4; i++) {
    digitalWrite(pyroPins[i], HIGH);
    Serial.print("Pyro "); Serial.print(i+1); 
    Serial.print(" (GPIO"); Serial.print(pyroPins[i]); Serial.println(") = HIGH");
  }
  
  Serial.println();
  Serial.println("All channels ON - Measure with multimeter:");
  Serial.println("GPIO33 should read ~3.3V");
  Serial.println("GPIO32 should read ~3.3V"); 
  Serial.println("GPIO25 should read ~3.3V");
  Serial.println("GPIO26 should read ~3.3V");
  
  // Count down 5 seconds
  for (int countdown = 5; countdown > 0; countdown--) {
    Serial.print("Countdown: "); Serial.println(countdown);
    delay(1000);
  }
  
  // Turn OFF all channels
  Serial.println();
  Serial.println("Turning OFF all pyro channels...");
  for (int i = 0; i < 4; i++) {
    digitalWrite(pyroPins[i], LOW);
    Serial.print("Pyro "); Serial.print(i+1); 
    Serial.print(" (GPIO"); Serial.print(pyroPins[i]); Serial.println(") = LOW");
  }
  
  Serial.println();
  Serial.println("All channels OFF - Should read ~0V on multimeter");
  Serial.println("=== PYRO CHANNEL TEST COMPLETED ===");
  Serial.println();
  pyroTestCompleted = true;
}

// Removed performance test function - keeping it simple!

// All complex analysis functions removed - clean and simple!

void readSensorsWithSmartLoadCell() {
  // Smart load cell reading - only read every N cycles to avoid blocking
  static int loadCellCycle = 0;
  const int LOAD_CELL_INTERVAL = (DATA_RATE_MODE == 1) ? 2 :   // 10Hz: every 2nd cycle
                                 (DATA_RATE_MODE == 2) ? 3 :   // 20Hz: every 3rd cycle  
                                 (DATA_RATE_MODE == 3) ? 4 :   // 30Hz: every 4th cycle
                                 (DATA_RATE_MODE == 4) ? 5 : 3; // 40Hz: every 5th cycle
  
  // Always read fast sensors (MS5611 and MPU6050)
  
    // Read MS5611 (Barometer/Temperature) - this is actually fast enough
      if (baroReady && (ENABLE_BAROMETER || ENABLE_TEMPERATURE)) {
        baro.read();  // Read from sensor
        if (ENABLE_BAROMETER) {
          float rawPressure = baro.getPressure();
          
          // Check if we got a valid pressure reading
          if (rawPressure < 500 || rawPressure > 1200) {
            static int errorCount = 0;
            if (++errorCount <= 5) {  // Only print first 5 errors to avoid spam
              Serial.print("Invalid raw pressure: ");
              Serial.print(rawPressure);
              Serial.println(" hPa");
            }
            return;  // Skip this reading if pressure is invalid
          }
          // Apply smooth filtering to pressure
          pressure = filterPressure(rawPressure);
          
          // Debug: Print pressure values for troubleshooting
          static int debugCounter = 0;
          if (++debugCounter >= 20 && showSerialData) {  // Print every 20 readings for faster debugging
            Serial.print("Debug - Raw P: ");
            Serial.print(rawPressure, 2);
            Serial.print(" hPa, Filtered P: ");
            Serial.print(pressure, 2);
            Serial.print(" hPa, Ref Alt: ");
            Serial.print(referenceAltitude, 2);
            Serial.print(" m, Calc Alt: ");
            Serial.print(altitude, 2);
            Serial.println(" m");
            debugCounter = 0;
          }
          
          // Calculate altitude from filtered pressure using barometric formula
          // altitude = 44330 * (1 - (pressure/sea_level_pressure)^0.1903)
          if (pressure > 500 && pressure < 1200) {  // More realistic pressure range for normal altitudes
            float pressureRatio = pressure / SEA_LEVEL_PRESSURE;
            float rawAltitude = 44330.0 * (1.0 - pow(pressureRatio, 0.1903));
            altitude = rawAltitude - referenceAltitude;
            
            // Debug altitude calculation
            static int altDebugCounter = 0;
            if (++altDebugCounter >= 50 && showSerialData) {  // Debug every 50 readings
              Serial.print("Alt Debug - Ratio: ");
              Serial.print(pressureRatio, 4);
              Serial.print(", Raw Alt: ");
              Serial.print(rawAltitude, 2);
              Serial.print(", Final Alt: ");
              Serial.println(altitude, 2);
              altDebugCounter = 0;
            }
          } else {
            altitude = 0.0;  // Fallback for invalid pressure
            if (showSerialData) {
              Serial.print("Pressure out of range: ");
              Serial.println(pressure);
            }
          }
        }
        if (ENABLE_TEMPERATURE) {
          temperature = baro.getTemperature();
        }
      }
  
  
    // Read MPU6050 (IMU) - always fast
    if (mpuReady && (ENABLE_ACCELEROMETER || ENABLE_GYROSCOPE)) {
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      
      if (ENABLE_ACCELEROMETER) {
        // Convert to m/s² and apply smooth filtering
        // LSB sensitivity: 4096 LSB/g for ±8g range
        float rawAccelX = (ax / 4096.0) * 9.80665;
        float rawAccelY = (ay / 4096.0) * 9.80665;
        float rawAccelZ = (az / 4096.0) * 9.80665;
        
        accelX = filterAccelX(rawAccelX);  // m/s²
        accelY = filterAccelY(rawAccelY);  // m/s²
        accelZ = filterAccelZ(rawAccelZ);  // m/s²
      }
      
      if (ENABLE_GYROSCOPE) {
        // Convert to rad/s and apply light filtering (more responsive)
        // LSB sensitivity: 32.8 LSB/(°/s) for ±1000°/s range  
        // Convert degrees to radians: 1° = π/180 rad
        float rawGyroX = (gx / 32.8) * (M_PI / 180.0);
        float rawGyroY = (gy / 32.8) * (M_PI / 180.0);
        float rawGyroZ = (gz / 32.8) * (M_PI / 180.0);
        
        gyroX = filterGyroX(rawGyroX);  // rad/s
        gyroY = filterGyroY(rawGyroY);  // rad/s
        gyroZ = filterGyroZ(rawGyroZ);  // rad/s
      }
    }
  
  // Smart HX711 reading - only on specific cycles to prevent blocking
    if (loadCellReady && ENABLE_LOAD_CELL) {
      loadCellCycle++;
      if (loadCellCycle >= LOAD_CELL_INTERVAL) {
        loadCellCycle = 0;  // Reset cycle
        
        // Only read if HX711 is actually ready (non-blocking check)
        if (loadCell.is_ready()) {
          // RAW MODE: Single reading, no averaging for maximum responsiveness
          loadCellRaw = loadCell.read();
          loadCellWeight = loadCell.get_units(1);  // Single reading, no averaging
        }
        // If not ready, we just skip this cycle and use the previous value
      }
    }
}
  
  // === MENU SYSTEM FUNCTIONS ===
void showMainMenu() {
  Serial.println("\n╔══════════════════════════════════════╗");
  Serial.println("║        PAVI FLIGHT COMPUTER          ║");
  Serial.println("║            MAIN MENU                 ║");
  Serial.println("╠══════════════════════════════════════╣");
  Serial.println("║  1. Flight Operations                ║");
  Serial.println("║  2. File Management                  ║");
  Serial.println("║  3. WiFi Controls                    ║");
  Serial.println("║  4. Calibration & Testing            ║");
  Serial.println("║  5. System Status                    ║");
  Serial.println("║  6. Toggle Serial Data Output        ║");
  Serial.println("║  0. Exit Menu                        ║");
  Serial.println("╚══════════════════════════════════════╝");
  Serial.print("📋 Select option (0-6): ");
}

void showFlightMenu() {
  Serial.println("\n╔══════════════════════════════════════╗");
  Serial.println("║          FLIGHT OPERATIONS           ║");
  Serial.println("╠══════════════════════════════════════╣");
  Serial.println("║  1. Configure Flight Parameters      ║");
  Serial.println("║  2. Start Recording                  ║");
  Serial.println("║  3. Stop Recording                   ║");
  Serial.println("║  4. Reset Sensor Origins (OFFSET)   ║");
  Serial.println("║  5. Fire Pyro Channel 1             ║");
  Serial.println("║  6. Fire Pyro Channel 2             ║");
  Serial.println("║  7. Fire Pyro Channel 3             ║");
  Serial.println("║  8. Fire Pyro Channel 4             ║");
  Serial.println("║  9. Back to Main Menu                ║");
  Serial.println("╚══════════════════════════════════════╝");
  Serial.print("🚀 Select flight option (1-9): ");
}

void showFileMenu() {
  Serial.println("\n╔══════════════════════════════════════╗");
  Serial.println("║          FILE MANAGEMENT             ║");
  Serial.println("╠══════════════════════════════════════╣");
  Serial.println("║  1. List All Files                  ║");
  Serial.println("║  2. Show Storage Space               ║");
  Serial.println("║  3. Download File                    ║");
  Serial.println("║  4. Delete File                      ║");
  Serial.println("║  5. Format Filesystem (WARNING!)     ║");
  Serial.println("║  9. Back to Main Menu                ║");
  Serial.println("╚══════════════════════════════════════╝");
  Serial.print("📁 Select file option (1-5, 9): ");
}

void showWiFiMenu() {
  Serial.println("\n╔══════════════════════════════════════╗");
  Serial.println("║            WIFI CONTROLS             ║");
  Serial.println("╠══════════════════════════════════════╣");
  if (wifiEnabled) {
    Serial.println("║  Status: 📶 WiFi SoftAP ACTIVE       ║");
    Serial.println("║  1. Stop WiFi SoftAP                 ║");
  } else {
    Serial.println("║  Status: 📵 WiFi SoftAP INACTIVE     ║");
    Serial.println("║  1. Start WiFi SoftAP                ║");
  }
  Serial.println("║  2. Show WiFi Information            ║");
  Serial.println("║  9. Back to Main Menu                ║");
  Serial.println("╚══════════════════════════════════════╝");
  Serial.print("📶 Select WiFi option (1-2, 9): ");
}

void showCalibrationMenu() {
  Serial.println("\n╔══════════════════════════════════════╗");
  Serial.println("║       CALIBRATION & TESTING          ║");
  Serial.println("╠══════════════════════════════════════╣");
  Serial.println("║  1. Test Load Cell Reading           ║");
  Serial.println("║  2. Zero Load Cell (Tare)            ║");
  Serial.println("║  3. Full Load Cell Calibration       ║");
  Serial.println("║  4. Save Calibration to File         ║");
  Serial.println("║  5. Test All Pyro Channels           ║");
  Serial.println("║  6. Test LoRa Communication          ║");
  Serial.println("║  7. Run Sensor Diagnostics           ║");
  Serial.println("║  0. Back to Main Menu                 ║");
  Serial.println("╚══════════════════════════════════════╝");
  Serial.print("⚖️ Select calibration option (0-7): ");
}

void processMenuInput(String input) {
  input.trim();
  int choice = input.toInt();
  
  if (currentMenu == MENU_MAIN) {
    switch (choice) {
      case 1:
        currentMenu = MENU_FLIGHT;
        showFlightMenu();
        return;
      case 2:
        currentMenu = MENU_FILES;
        showFileMenu();
        return;
      case 3:
        currentMenu = MENU_WIFI;
        showWiFiMenu();
        return;
      case 4:
        currentMenu = MENU_CALIBRATION;
        showCalibrationMenu();
        return;
      case 5:
        Serial.println("\n🔍 Displaying system status...");
        printSystemStatus();
        showMainMenu();
        return;
      case 6:
        showSerialData = !showSerialData;
        Serial.print("📊 Serial data output ");
        Serial.println(showSerialData ? "ENABLED" : "DISABLED");
        showMainMenu();
        return;
      case 0:
        Serial.println("👋 Exiting menu mode...");
        Serial.println("💡 Type 'MENU' to return to menu system");
        menuMode = false;
        return;
      default:
        Serial.println("❌ Invalid option! Please select 0-6");
        showMainMenu();
        return;
    }
  }
  
  // Flight Operations Menu
  if (currentMenu == MENU_FLIGHT) {
    switch (choice) {
      case 1:
        Serial.println("🔧 Starting flight configuration...");
        configureFlightParameters();
        break;
      case 2:
        Serial.println("🚀 Starting data recording...");
        startDataRecording();
        break;
      case 3:
        Serial.println("⏹️ Stopping data recording...");
        stopDataRecording();
        break;
      case 4:
        Serial.println("🎯 Resetting sensor origins...");
        resetSensorOrigins();
        break;
      case 5:
        Serial.println("💥 Firing Pyro Channel 1...");
        firePyroChannel(0);
        break;
      case 6:
        Serial.println("💥 Firing Pyro Channel 2...");
        firePyroChannel(1);
        break;
      case 7:
        Serial.println("💥 Firing Pyro Channel 3...");
        firePyroChannel(2);
        break;
      case 8:
        Serial.println("💥 Firing Pyro Channel 4...");
        firePyroChannel(3);
        break;
      case 9:
        currentMenu = MENU_MAIN;
        showMainMenu();
        return;
      default:
        Serial.println("❌ Invalid option! Please select 1-9");
        break;
    }
    delay(2000);  // Give time to read the result
    showFlightMenu();
  }
  
  // File Management Menu
  else if (currentMenu == MENU_FILES) {
    switch (choice) {
      case 1:
        Serial.println("📁 Listing all files...");
        listFiles();
        break;
      case 2:
        Serial.println("💾 Checking storage space...");
        showFilesystemSpace();
        break;
      case 3:
        Serial.print("📥 Enter filename to download: ");
        // This will be handled by the next input
        currentSubMenu = 3;
        return;
      case 4:
        Serial.print("🗑️ Enter filename to delete: ");
        currentSubMenu = 4;
        return;
      case 5:
        Serial.println("⚠️ Starting filesystem format...");
        formatFilesystem();
        break;
      case 9:
        currentMenu = MENU_MAIN;
        showMainMenu();
        return;
      default:
        Serial.println("❌ Invalid option! Please select 1-5 or 9");
        break;
    }
    if (choice != 3 && choice != 4) {  // Don't show menu if waiting for filename
      delay(2000);
      showFileMenu();
    }
  }
  
  // WiFi Menu
  else if (currentMenu == MENU_WIFI) {
    switch (choice) {
      case 1:
        if (wifiEnabled) {
          Serial.println("📵 Stopping WiFi SoftAP...");
          stopWiFiSoftAP();
        } else {
          Serial.println("📶 Starting WiFi SoftAP...");
          startWiFiSoftAP();
        }
        break;
      case 2:
        Serial.println("📊 WiFi Information:");
        if (wifiEnabled) {
          Serial.println("   Status: ACTIVE");
          Serial.print("   IP: "); Serial.println(WiFi.softAPIP());
          Serial.print("   Clients: "); Serial.println(WiFi.softAPgetStationNum());
        } else {
          Serial.println("   Status: INACTIVE");
        }
        break;
      case 9:
        currentMenu = MENU_MAIN;
        showMainMenu();
        return;
      default:
        Serial.println("❌ Invalid option! Please select 1-2 or 9");
        break;
    }
    delay(2000);
    showWiFiMenu();
  }
  
  // Calibration Menu
  else if (currentMenu == MENU_CALIBRATION) {
    switch (choice) {
      case 1:
        Serial.println("⚖️ Testing load cell...");
        testLoadCellCalibration();
        break;
      case 2:
        Serial.println("🎯 Zeroing load cell...");
        calibrateLoadCellZero();
        break;
      case 3:
        Serial.println("⚖️ Starting full load cell calibration...");
        Serial.println("📋 Step-by-step calibration process:");
        Serial.println("   1. First zero the load cell (if not done already)");
        Serial.println("   2. Then provide a known weight for calibration");
        Serial.println("");
        Serial.print("🎯 Enter known weight in kg (e.g. 1.5): ");
        currentSubMenu = 3;  // Set submenu mode for weight input
        return;
      case 4:
        Serial.println("� Saving calibration to file...");
        saveLoadCellCalibration();
        break;
      case 5:
        Serial.println("�💡 Testing pyro channels...");
        if (PYRO_TEST_MODE) {
          runPyroTest();
        } else {
          Serial.println("⚠️ Pyro test mode is disabled. Set PYRO_TEST_MODE to true to enable.");
        }
        break;
      case 6:
        Serial.println("📡 Testing LoRa communication...");
        Serial.println("Send 'PING' command from RX to test...");
        break;
      case 7:
        Serial.println("🔍 Running sensor diagnostics...");
        runSensorDiagnostics();
        break;
      case 0:
        currentMenu = MENU_MAIN;
        showMainMenu();
        return;
      default:
        Serial.println("❌ Invalid option! Please select 0-7");
        break;
    }
    delay(2000);
    showCalibrationMenu();
  }
}

void sendLoRaMenu() {
  if (SERIAL_TEST_MODE) {
    Serial.println("LoRa menu not available in test mode");
    return;
  }
  
  Serial.println("📡 Sending LoRa remote menu to ground station...");
  
  String menu = "MENU:\n";
  menu += "1-CONFIG,2-START,3-STOP,4-OFFSET\n";
  menu += "5-PYRO1,6-PYRO2,7-PYRO3,8-PYRO4\n";
  menu += "9-STATUS,0-WIFI\n";
  menu += "Send number+ENTER";
  
  LoRa.beginPacket();
  LoRa.print(menu);
  LoRa.endPacket();
  
  // Return to receive mode
  LoRa.receive();
  
  Serial.println("✅ LoRa menu sent to ground station");
}

void runSensorDiagnostics() {
  Serial.println("\n=== SENSOR DIAGNOSTICS ===");
  Serial.print("Barometer MS5611: "); Serial.println(baroReady ? "✅ OK" : "❌ FAIL");
  Serial.print("IMU MPU6050: "); Serial.println(mpuReady ? "✅ OK" : "❌ FAIL");
  Serial.print("Load Cell HX711_Raw: "); Serial.println(loadCellReady ? "✅ OK (RAW MODE)" : "❌ FAIL");
  Serial.print("Filesystem: "); Serial.println(filesystemReady ? "✅ OK" : "❌ FAIL");
  
  // Show filtering status
  Serial.println("\n--- Filtering Status ---");
  if (ENABLE_SENSOR_FILTERING) {
    Serial.println("📊 Mode: SMOOTH (filtered data)");
    Serial.print("  • Pressure: "); Serial.println(ENABLE_PRESSURE_FILTERING ? "FILTERED" : "RAW");
    Serial.print("  • Accelerometer: "); Serial.println(ENABLE_ACCEL_FILTERING ? "FILTERED" : "RAW");
    Serial.print("  • Gyroscope: "); Serial.println(ENABLE_GYRO_FILTERING ? "FILTERED" : "RAW");
  } else {
    Serial.println("🎯 Mode: RAW (all sensors unfiltered)");
  }
  Serial.println("  • Load Cell: ALWAYS RAW (HX711_Raw library)");
  
  if (baroReady) {
    Serial.println("\n--- Current Readings ---");
    Serial.print("Current pressure: "); Serial.print(pressure, 2); Serial.println(" hPa");
    Serial.print("Current altitude: "); Serial.print(altitude, 2); Serial.println(" m");
  }
  
  if (loadCellReady && loadCell.is_ready()) {
    Serial.print("Load cell reading (raw): "); Serial.print(loadCellWeight, 3); Serial.println(" kg");
    Serial.print("Load cell raw value: "); Serial.println(loadCellRaw);
    Serial.println("📈 Using RAW HX711 - no internal filtering for maximum responsiveness");
  }
  
  Serial.println("========================");
}

// === FLIGHT DATA FUNCTIONS ===
void createDataFile() {
    if (!filesystemReady) {
      Serial.println("WARNING: LittleFS not ready, file creation skipped");
      return;
    }
    
    if (flightConfig.filename.isEmpty()) {
      Serial.println("ERROR: Cannot create file - no filename configured");
      return;
    }
    
    // Close any existing file first
    if (dataFile) {
      dataFile.close();
      Serial.println("Closed previous data file");
    }
    
    // Create the data file
    String filepath = "/" + flightConfig.filename;
    dataFile = LittleFS.open(filepath, "w");
    if (dataFile) {
      Serial.print("📁 Created data file: "); Serial.println(filepath);
      
      // Write the header immediately
      writeDataHeader();
      
      Serial.println("✅ Data file ready for events and recording");
    } else {
      Serial.println("❌ ERROR: Failed to create data file");
      filesystemReady = false;  // Fall back to serial logging
    }
  }

  void closeDataFile() {
    if (filesystemReady && dataFile) {
      // Flush any remaining buffered data before closing
      if (dataBuffer.length() > 0) {
        dataFile.print(dataBuffer);
        dataBuffer = "";
        bufferLineCount = 0;
        Serial.println("Final data buffer flushed before close");
      }
      
      dataFile.close();
      Serial.print("📁 Data file closed: /"); Serial.println(flightConfig.filename);
    }
  }

  void calculateVerticalVelocity() {
    unsigned long currentTime = millis();
    
    if (lastVelocityTime > 0) {
      float deltaTime = (currentTime - lastVelocityTime) / 1000.0;  // Convert to seconds
      float deltaAltitude = altitude - lastAltitude;
      
      if (deltaTime > 0) {
        // Calculate velocity from barometer
        float baroVelocity = deltaAltitude / deltaTime;
        
        // Combine with accelerometer for better accuracy (complementary filter)
        // Use Z-axis acceleration (subtract gravity)
        float accelVelocity = (accelZ - 9.80665) * deltaTime;
        
        // Weighted combination: 70% baro, 30% accel for smoothness
        verticalVelocity = 0.7 * baroVelocity + 0.3 * (verticalVelocity + accelVelocity);
      }
    }
    
    lastAltitude = altitude;
    lastVelocityTime = currentTime;
  }

  void printSystemStatus() {
    Serial.println("\n=== PARACHUTE TEST FLIGHT COMPUTER ===");
    Serial.print("Flight State: ");
    switch(flightState) {
      case FLIGHT_IDLE: Serial.println("IDLE - Waiting for configuration"); break;
      case FLIGHT_CONFIGURING: Serial.println("CONFIGURING - Enter flight parameters"); break;
      case FLIGHT_CONFIGURED: Serial.println("CONFIGURED - Ready to start"); break;
      case FLIGHT_RECORDING: Serial.println("RECORDING - Flight in progress"); break;
      case FLIGHT_STOPPED: Serial.println("STOPPED - Flight completed"); break;
    }
    
    Serial.print("LittleFS: "); Serial.println(filesystemReady ? "Ready" : "Not ready");
    Serial.print("Barometer: "); Serial.println(baroReady ? "Ready" : "Not ready");
    Serial.print("IMU: "); Serial.println(mpuReady ? "Ready" : "Not ready");  
    Serial.print("Load Cell: "); Serial.println(loadCellReady ? "Ready" : "Not ready");
    
    // Load Cell Calibration Status
    if (loadCellReady) {
      Serial.println("\n--- Load Cell Calibration ---");
      Serial.print("Calibration Factor: "); Serial.println(loadCellCalibrationFactor, 6);
      Serial.print("Zero Offset: "); Serial.println(loadCellOffset);
      
      // Check if calibration file exists
      if (filesystemReady && LittleFS.exists("/loadcell_cal.txt")) {
        Serial.println("Saved Calibration: ✅ Available");
      } else {
        Serial.println("Saved Calibration: ❌ Not found");
      }
      
      // Show calibration mode if active
      if (isLoadCellCalibrating()) {
        Serial.println("Calibration Mode: 🔧 ACTIVE");
      } else {
        Serial.println("Calibration Mode: 💤 Inactive");
      }
    }
    
    if (flightState >= FLIGHT_CONFIGURED) {
      Serial.println("\n--- Flight Configuration ---");
      Serial.print("Filename: "); Serial.println(flightConfig.filename);
      Serial.print("Total Weight: "); Serial.print(flightConfig.totalWeight); Serial.println(" kg");
      Serial.print("Wind Speed: "); Serial.print(flightConfig.windSpeed); Serial.println(" m/s");
      Serial.print("Initial Height: "); Serial.print(flightConfig.initialHeight); Serial.println(" m");
    }
    
    if (flightState == FLIGHT_RECORDING) {
      Serial.print("Recording Time: "); 
      Serial.print((millis() - recordingStartTime) / 1000.0); 
      Serial.println(" seconds");
      Serial.print("Samples Logged: "); Serial.println(sampleNumber);
    }
    
    Serial.println("\n--- TX: COMMAND RESPONDER MODE ---");
    Serial.println("TX only responds to LoRa commands from RX");
    Serial.println("No unsolicited telemetry transmission");
    Serial.println("");
    Serial.println("Available LoRa Commands (sent from RX):");
    Serial.println("PING - Test LoRa communication");
    Serial.println("STATUS - Get flight computer status");
    Serial.println("START - Start data recording");
    Serial.println("STOP - Stop data recording");
    Serial.println("PYRO1, PYRO2, PYRO3, PYRO4 - Fire pyro channels");
    Serial.println("");
    Serial.println("Load Cell Calibration Commands:");
    Serial.println("CALIB_START - Start calibration process");
    Serial.println("CALIB_ZERO - Zero the load cell (tare)");
    Serial.println("CALIB_WEIGHT:1.5 - Calibrate with known weight (kg)");
    Serial.println("CALIB_SAVE - Save calibration to file");
    Serial.println("CALIB_TEST - Test current calibration");
    Serial.println("");
    Serial.println("💡 EASY ACCESS - Interactive Menu System:");
    Serial.println("MENU or M - Open interactive menu system");
    Serial.println("");
    Serial.println("Legacy Serial Commands (still available):");
    Serial.println("CONFIG - Configure flight parameters");
    Serial.println("SERIALTOGGLE - Toggle serial data output");
    Serial.println("FILES - List all stored files");
    Serial.println("DOWNLOAD <filename> - Download file content");
    Serial.println("DELETE <filename> - Delete a file");
    Serial.println("SPACE - Show filesystem usage");
    Serial.println("FORMAT - Format filesystem (WARNING: deletes all data!)");
    Serial.println("======================================\n");
    
  }

  void processSerialCommands() {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (commandBuffer.length() > 0) {
          
          // Handle menu system
          if (menuMode) {
            // Handle file operations that need filename input
            if (currentMenu == MENU_FILES && (currentSubMenu == 3 || currentSubMenu == 4)) {
              if (currentSubMenu == 3) {
                Serial.println("📥 Downloading file: " + commandBuffer);
                downloadFile(commandBuffer);
              } else if (currentSubMenu == 4) {
                Serial.println("🗑️ Deleting file: " + commandBuffer);
                deleteFile(commandBuffer);
              }
              currentSubMenu = 0;
              delay(2000);
              showFileMenu();
            }
            // Handle calibration weight input
            else if (currentMenu == MENU_CALIBRATION && currentSubMenu == 3) {
              float weight = commandBuffer.toFloat();
              if (weight > 0) {
                Serial.print("⚖️ Starting weight calibration with "); 
                Serial.print(weight, 2); Serial.println(" kg");
                calibrateLoadCellWeight(weight);
              } else {
                Serial.println("❌ Invalid weight entered");
              }
              currentSubMenu = 0;
              delay(3000);  // Give more time to read calibration results
              showCalibrationMenu();
            }
            else {
              processMenuInput(commandBuffer);
            }
          }
          // Check for menu activation
          else if (commandBuffer.equalsIgnoreCase("MENU") || commandBuffer.equalsIgnoreCase("M")) {
            Serial.println("\n🎛️ Entering interactive menu system...");
            menuMode = true;
            currentMenu = MENU_MAIN;
            showMainMenu();
          }
          // Legacy command support
          else {
            // Check for serial toggle command
            if (commandBuffer.equalsIgnoreCase("SERIALTOGGLE")) {
              showSerialData = !showSerialData;
              Serial.print("Serial data output ");
              Serial.println(showSerialData ? "ENABLED" : "DISABLED");
            } else {
              processCommand(commandBuffer);
            }
          }
          commandBuffer = "";
        }
      } else {
        commandBuffer += c;
      }
    }
  }

  void handleLoRaCommands() {
    if (SERIAL_TEST_MODE) return; // Skip LoRa in test mode
    
    int packetSize = LoRa.parsePacket();
    if (packetSize > 0) {
      Serial.println("*****************************");
      Serial.println("*** LORA PACKET RECEIVED! ***");
      Serial.print("*** Packet size: ");
      Serial.print(packetSize);
      Serial.println(" bytes ***");
      
      String received = "";
      while (LoRa.available()) {
        received += (char)LoRa.read();
      }
      
      Serial.print("*** Message: '");
      Serial.print(received);
      Serial.println("' ***");
      Serial.print("*** RSSI: ");
      Serial.print(LoRa.packetRssi());
      Serial.println(" dBm ***");
      Serial.println("*****************************");
      
      // Process the command
      received.trim();  // Trim the string in place
      processCommand(received);
      
      // Stay in receive mode
      LoRa.receive();
    }
  }

  void processCommand(String cmd) {
    cmd.toUpperCase();
    cmd.trim();
    
    Serial.print(">>> Processing LoRa command: "); Serial.println(cmd);
    Serial.print(">>> Current flight state: "); 
    
    switch(flightState) {
      case FLIGHT_IDLE: Serial.println("IDLE"); break;
      case FLIGHT_CONFIGURING: Serial.println("CONFIGURING"); break;
      case FLIGHT_CONFIGURED: Serial.println("CONFIGURED"); break;
      case FLIGHT_RECORDING: Serial.println("RECORDING"); break;
      case FLIGHT_STOPPED: Serial.println("STOPPED"); break;
      default: Serial.println("UNKNOWN"); break;
    }
    
    // === LORA MENU SYSTEM ===
    if (cmd == "MENU" || cmd == "M") {
      Serial.println("📡 LoRa Menu System activated from ground station");
      sendLoRaMenu();
      return;
    }
    
    // === NEW INTUITIVE COMMAND PROTOCOL ===
    
    // Configuration Commands
    if (cmd == "CONFIG_START") {
      Serial.println("🔧 Starting configuration mode...");
      flightState = FLIGHT_CONFIGURING;
      // Reset config to defaults
      flightConfig.filename = "";
      flightConfig.totalWeight = 0.0;
      flightConfig.windSpeed = 0.0;
      flightConfig.initialHeight = 0.0;
      Serial.println("✅ Ready for configuration parameters");
    }
    else if (cmd == "CONFIG_READY") {
      // Enhanced validation with detailed status
      bool hasFilename = !flightConfig.filename.isEmpty();
      bool hasWeight = flightConfig.totalWeight > 0;
      bool hasWind = flightConfig.windSpeed >= 0;
      bool hasHeight = flightConfig.initialHeight > 0;
      bool hasValidConfig = hasFilename && hasWeight && hasWind && hasHeight;
      
      Serial.println("📋 === CONFIG_READY RECEIVED ===");
      Serial.println("Configuration status check:");
      Serial.println("  • Filename: " + (hasFilename ? "✅ '" + flightConfig.filename + "'" : "❌ MISSING"));
      Serial.println("  • Weight: " + (hasWeight ? "✅ " + String(flightConfig.totalWeight, 2) + " kg" : "❌ MISSING"));
      Serial.println("  • Wind: " + (hasWind ? "✅ " + String(flightConfig.windSpeed, 2) + " m/s" : "❌ MISSING"));
      Serial.println("  • Height: " + (hasHeight ? "✅ " + String(flightConfig.initialHeight, 2) + " m" : "❌ MISSING"));
      
      if (flightState == FLIGHT_CONFIGURING || (hasValidConfig && flightState != FLIGHT_RECORDING)) {
        if (hasValidConfig) {
          Serial.println("🚀 Configuration complete! Ready for flight operations.");
          flightState = FLIGHT_CONFIGURED;
          
          // Show final config
          Serial.println("=== FLIGHT CONFIGURATION ACCEPTED ===");
          Serial.print("Filename: "); Serial.println(flightConfig.filename);
          Serial.print("Weight: "); Serial.print(flightConfig.totalWeight); Serial.println(" kg");
          Serial.print("Wind Speed: "); Serial.print(flightConfig.windSpeed); Serial.println(" m/s");
          Serial.print("Height: "); Serial.print(flightConfig.initialHeight); Serial.println(" m");
          Serial.println("=========================================");
          
          // NEW: Create the data file immediately after configuration
          Serial.println("📁 Creating data file for upcoming flight...");
          createDataFile();
        } else {
          Serial.println("❌ Configuration incomplete - missing parameters above");
          Serial.println("💡 Some LoRa packets may have been lost. Try resending configuration.");
        }
        
        if (flightState != FLIGHT_CONFIGURING) {
          Serial.println("ℹ️ Note: Configuration accepted even though not explicitly in config mode");
        }
      } else {
        Serial.print("❌ ERROR: Cannot complete configuration. State: ");
        Serial.print(flightState);
        Serial.print(", Valid config: ");
        Serial.println(hasValidConfig ? "Yes" : "No");
        Serial.println("� If parameters are missing, it's likely due to LoRa packet loss.");
      }
    }
    else if (cmd.startsWith("FILENAME:")) {
      if (flightState == FLIGHT_CONFIGURING || flightState == FLIGHT_IDLE) {
        String filename = cmd.substring(9);
        filename.trim();
        if (filename.length() > 0) {
          flightConfig.filename = filename + ".txt";  // Use .txt format
          Serial.print("✅ Filename set to: "); Serial.println(flightConfig.filename);
          // Ensure we stay in configuring mode
          if (flightState == FLIGHT_IDLE) {
            flightState = FLIGHT_CONFIGURING;
            Serial.println("🔧 Entered configuration mode");
          }
        } else {
          Serial.println("❌ Invalid filename");
        }
      } else {
        Serial.println("❌ Cannot set filename - not in configuration mode");
      }
    }
    else if (cmd.startsWith("WEIGHT:")) {
      if (flightState == FLIGHT_CONFIGURING || flightState == FLIGHT_IDLE) {
        float weight = cmd.substring(7).toFloat();
        if (weight > 0) {
          flightConfig.totalWeight = weight;
          Serial.print("✅ Weight set to: "); Serial.print(weight, 2); Serial.println(" kg");
          // Ensure we stay in configuring mode
          if (flightState == FLIGHT_IDLE) {
            flightState = FLIGHT_CONFIGURING;
            Serial.println("🔧 Entered configuration mode");
          }
        } else {
          Serial.println("❌ Invalid weight value");
        }
      } else {
        Serial.println("❌ Cannot set weight - not in configuration mode");
      }
    }
    else if (cmd.startsWith("WIND:")) {
      if (flightState == FLIGHT_CONFIGURING || flightState == FLIGHT_IDLE) {
        float wind = cmd.substring(5).toFloat();
        if (wind >= 0) {
          flightConfig.windSpeed = wind;
          Serial.print("✅ Wind speed set to: "); Serial.print(wind, 2); Serial.println(" m/s");
          // Ensure we stay in configuring mode
          if (flightState == FLIGHT_IDLE) {
            flightState = FLIGHT_CONFIGURING;
            Serial.println("🔧 Entered configuration mode");
          }
        } else {
          Serial.println("❌ Invalid wind speed");
        }
      } else {
        Serial.println("❌ Cannot set wind speed - not in configuration mode");
      }
    }
    else if (cmd.startsWith("HEIGHT:")) {
      if (flightState == FLIGHT_CONFIGURING || flightState == FLIGHT_IDLE) {
        float height = cmd.substring(7).toFloat();
        flightConfig.initialHeight = height;
        Serial.print("✅ Height set to: "); Serial.print(height, 2); Serial.println(" m");
        // Ensure we stay in configuring mode
        if (flightState == FLIGHT_IDLE) {
          flightState = FLIGHT_CONFIGURING;
          Serial.println("🔧 Entered configuration mode");
        }
      } else {
        Serial.println("❌ Cannot set height - not in configuration mode");
      }
    }
    
    // Flight Control Commands
    else if (cmd == "FLIGHT_START") {
      Serial.println("🚀 Starting flight data logging...");
      startDataRecording();
    }
    else if (cmd == "FLIGHT_STOP") {
      Serial.println("🛑 Stopping flight data logging...");
      stopDataRecording();
    }
    else if (cmd == "SENSOR_RESET") {
      Serial.println("🎯 Resetting sensor zero points...");
      resetSensorOrigins();
    }
    
    // Data Recovery Commands
    else if (cmd == "WIFI_START") {
      startWiFiSoftAP();
    }
    else if (cmd == "WIFI_STOP") {
      stopWiFiSoftAP();
    }
    else if (cmd == "FILE_LIST") {
      Serial.println("📁 Listing available files...");
      listFiles();
    }
    
    // Load Cell Calibration Commands
    else if (cmd == "CALIB_START") {
      Serial.println("⚖️ Starting load cell calibration...");
      startLoadCellCalibration();
    }
    else if (cmd == "CALIB_ZERO") {
      Serial.println("⚖️ Setting load cell zero point...");
      calibrateLoadCellZero();
    }
    else if (cmd.startsWith("CALIB_WEIGHT:")) {
      float weight = cmd.substring(13).toFloat();
      Serial.print("⚖️ Setting calibration weight: "); Serial.print(weight, 2); Serial.println(" kg");
      calibrateLoadCellWeight(weight);
    }
    else if (cmd == "CALIB_SAVE") {
      Serial.println("⚖️ Saving calibration constants...");
      saveLoadCellCalibration();
    }
    else if (cmd == "CALIB_TEST") {
      Serial.println("⚖️ Testing load cell calibration...");
      testLoadCellCalibration();
    }
    
    // Legacy and Direct Commands
    else if (cmd == "PING") {
      Serial.println("🏓 PONG - LoRa connection OK!");
    }
    else if (cmd == "STATUS") {
      printSystemStatus();
    }
    else if (cmd.startsWith("PYRO")) {
      int channel = cmd.charAt(4) - '1';  // Convert PYRO1->0, PYRO2->1, etc.
      if (channel >= 0 && channel <= 3) {
        Serial.print("💥 Firing PYRO channel "); Serial.println(channel + 1);
        firePyroChannel(channel);
        logPyroEvent(channel + 1);  // Log pyro firing to data file
      } else {
        Serial.println("❌ Invalid pyro channel");
      }
    }
    
    // File Management (legacy support)
    else if (cmd == "FILES") {
      listFiles();
    }
    else if (cmd.startsWith("DOWNLOAD ")) {
      String filename = cmd.substring(9);
      filename.trim();
      downloadFile(filename);
    }
    else if (cmd.startsWith("DELETE ")) {
      String filename = cmd.substring(7);
      filename.trim();
      deleteFile(filename);
    }
    else if (cmd == "SPACE") {
      showFilesystemSpace();
    }
    else if (cmd == "FORMAT") {
      formatFilesystem();
    }
    
    // Unknown command
    else {
      Serial.print(">>> ERROR: Unknown LoRa command: "); Serial.println(cmd);
      Serial.println(">>> Use PING to test connection or check RX command syntax");
    }
  }

  void configureFlightParameters() {
    Serial.println("\n=== FLIGHT CONFIGURATION ===");
    
    // Get filename
    Serial.print("Enter filename (without extension): ");
    flightConfig.filename = getSerialInput();
    if (flightConfig.filename.length() == 0) {
      flightConfig.filename = "flight_" + String(millis());
    }
    flightConfig.filename += ".csv";
    
    // Get total weight
    Serial.print("Enter total weight (kg): ");
    String weightStr = getSerialInput();
    flightConfig.totalWeight = weightStr.toFloat();
    if (flightConfig.totalWeight <= 0) {
      flightConfig.totalWeight = 1.0;  // Default
      Serial.println("Using default weight: 1.0 kg");
    }
    
    // Get wind speed
    Serial.print("Enter wind speed (m/s): ");
    String windStr = getSerialInput();
    flightConfig.windSpeed = windStr.toFloat();
    
    // Get initial height
    Serial.print("Enter initial height (m): ");
    String heightStr = getSerialInput();
    flightConfig.initialHeight = heightStr.toFloat();
    
    flightConfig.startTime = millis();
    flightState = FLIGHT_CONFIGURED;
    
    Serial.println("\n--- Configuration Complete ---");
    Serial.print("Filename: "); Serial.println(flightConfig.filename);
    Serial.print("Weight: "); Serial.print(flightConfig.totalWeight); Serial.println(" kg");
    Serial.print("Wind Speed: "); Serial.print(flightConfig.windSpeed); Serial.println(" m/s");  
    Serial.print("Initial Height: "); Serial.print(flightConfig.initialHeight); Serial.println(" m");
    Serial.println("Ready to start recording! Send START command.");
    
    // Create and prepare the data file immediately
    createDataFile();
  }

  String getSerialInput() {
    String input = "";
    unsigned long timeout = millis() + 30000;  // 30 second timeout
    
    while (millis() < timeout) {
      if (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
          break;
        } else {
          input += c;
          Serial.print(c);  // Echo character
        }
      }
      delay(10);
    }
    Serial.println();  // New line after input
    return input;
  }

  void startDataRecording() {
    if (flightState != FLIGHT_CONFIGURED) {
      Serial.println("ERROR: Must configure flight parameters first (CONFIG command)");
      return;
    }
    
    if (!filesystemReady) {
      Serial.println("WARNING: LittleFS not ready, logging to serial only");
    }
    
    // Check if data file is already prepared (it should be from CONFIG_READY)
    if (filesystemReady && !dataFile) {
      Serial.println("WARNING: Data file not prepared, creating now...");
      createDataFile();
    }
    
    // Reset sensors to current position as origin (this will now log OFFSET properly)
    resetSensorOrigins();
    
    recordingStartTime = millis();
    sampleNumber = 0;
    flightState = FLIGHT_RECORDING;
    
    Serial.println("=== DATA RECORDING STARTED ===");
    Serial.print("📁 Using data file: /"); Serial.println(flightConfig.filename);
    Serial.println("Send STOP command to end recording");
    Serial.println("Send PYRO1-PYRO4 commands to fire pyro channels");
  }

  void stopDataRecording() {
    if (flightState != FLIGHT_RECORDING) {
      Serial.println("ERROR: Not currently recording");
      return;
    }
    
    flightState = FLIGHT_STOPPED;
    
    // Flush any remaining buffered data and close file
    if (filesystemReady && dataFile && dataBuffer.length() > 0) {
      dataFile.print(dataBuffer);
      dataFile.flush();
      dataBuffer = "";
      bufferLineCount = 0;
      Serial.println("Final data buffer flushed");
    }
    
    // Close data file
    if (filesystemReady && dataFile) {
      dataFile.close();
      Serial.println("Data file closed successfully");
    }
    
    unsigned long recordingTime = (millis() - recordingStartTime) / 1000;
    
    Serial.println("\n=== DATA RECORDING STOPPED ===");
    Serial.print("Recording duration: "); Serial.print(recordingTime); Serial.println(" seconds");
    Serial.print("Total samples: "); Serial.println(sampleNumber);
    Serial.print("Average sample rate: "); 
    Serial.print(sampleNumber / (recordingTime > 0 ? recordingTime : 1)); 
    Serial.println(" Hz");
    
  if (filesystemReady) {
    Serial.print("Data saved to: /"); Serial.println(flightConfig.filename);
    
    // Show file size
    String filepath = "/" + flightConfig.filename;
    File checkFile = LittleFS.open(filepath, "r");
    if (checkFile) {
      Serial.print("File size: "); Serial.print(checkFile.size()); Serial.println(" bytes");
      checkFile.close();
    }
  } else {
    Serial.println("Data was logged to serial output only");
  }
  }

  void resetSensorOrigins() {
    Serial.println("🎯 Resetting sensor zero points...");
    
    // Reset barometer to current altitude as zero
    if (ENABLE_BAROMETER && baroReady) {
      Serial.println("   📊 Resetting barometer zero point...");
      
      // Calculate current absolute altitude
      float currentPressure = baro.getPressure();
      if (currentPressure > 500 && currentPressure < 1200) {
        float pressureRatio = currentPressure / SEA_LEVEL_PRESSURE;
        float currentAbsoluteAltitude = 44330.0 * (1.0 - pow(pressureRatio, 0.1903));
        
        // Set this as the new reference point - future altitude readings will be relative to this
        referenceAltitude = currentAbsoluteAltitude;
        
        Serial.print("   New reference altitude set to: ");
        Serial.print(referenceAltitude, 2);
        Serial.println(" m");
        Serial.println("   ✅ Barometer reset complete - altitude zeroed");
      } else {
        Serial.println("   ❌ Invalid pressure reading, cannot reset barometer");
      }
    }
    
    // Reset IMU if needed (future expansion)
    if (ENABLE_ACCELEROMETER || ENABLE_GYROSCOPE) {
      Serial.println("   📊 IMU reset (future feature)");
    }
    
    // Log the offset event to flight data
    logOffsetEvent();
    
    Serial.println("✅ All sensor origins reset and logged");
  }

  void writeDataHeader() {
    // Create comprehensive header for .txt file
    String header = "";
    header += "# ========================================\n";
    header += "# PARACHUTE TEST FLIGHT DATA\n";
    header += "# ========================================\n";
    header += "# Test Configuration:\n";
    header += "#   Filename: " + flightConfig.filename + "\n";
    header += "#   Total Weight: " + String(flightConfig.totalWeight, 2) + " kg\n";
    header += "#   Wind Speed: " + String(flightConfig.windSpeed, 2) + " m/s\n";
    header += "#   Initial Height: " + String(flightConfig.initialHeight, 2) + " m\n";
    header += "#\n";
    header += "# System Information:\n";
    header += "#   Start Time: " + String(flightConfig.startTime) + " ms (boot time)\n";
    header += "#   Data Rate Mode: " + String(DATA_RATE_MODE) + "\n";
    header += "#   Chip ID: " + String(ESP.getEfuseMac()) + "\n";
    header += "#   Firmware: TX Flight Computer v1.0\n";
    header += "#\n";
    header += "# Data Format:\n";
    header += "#   All times in milliseconds relative to recording start\n";
    header += "#   All measurements in SI units (m, kg, m/s, m/s², rad/s)\n";
    header += "#   Events (PYRO, OFFSET) logged with timestamps\n";
    header += "#\n";
    header += "# Column Headers:\n";
    header += "#   Time_ms,Event_Type,Altitude_m,Vertical_Velocity_ms,Load_Cell_kg,Accel_X_ms2,Accel_Y_ms2,Accel_Z_ms2,Gyro_X_rads,Gyro_Y_rads,Gyro_Z_rads,Notes\n";
    header += "# ========================================\n";
    header += "Time_ms,Event_Type,Altitude_m,Vertical_Velocity_ms,Load_Cell_kg,Accel_X_ms2,Accel_Y_ms2,Accel_Z_ms2,Gyro_X_rads,Gyro_Y_rads,Gyro_Z_rads,Notes\n";
    
    // Write header to serial
    Serial.println("📝 Writing flight data header:");
    Serial.print(header);
    
    // Write to LittleFS file
    if (filesystemReady && dataFile) {
      dataFile.print(header);
      dataFile.flush();  // Ensure header is written immediately
      Serial.println("✅ Header written to " + flightConfig.filename);
    }
  }

  void logFlightData() {
    if (flightState != FLIGHT_RECORDING) {
      return;
    }
    
    unsigned long currentTime = millis();
    unsigned long relativeTime = currentTime - recordingStartTime;
    
    // Create efficient CSV data line with consistent formatting for better readability
    String dataLine = "";
    
    // Timestamp (right-aligned for readability)
    dataLine += String(relativeTime);
    dataLine += ",DATA,";
    
    // Altitude (2 decimal places - sufficient for flight data)
    dataLine += String(altitude, 2);
    dataLine += ",";
    
    // Velocity (2 decimal places - sufficient precision)
    dataLine += String(verticalVelocity, 2);
    dataLine += ",";
    
    // Load cell (2 decimal places - sufficient for weight)
    dataLine += String(loadCellWeight, 2);
    dataLine += ",";
    
       
    // Accelerometer data (2 decimal places - reduced from 4)
    dataLine += String(accelX, 2) + ",";
    dataLine += String(accelY, 2) + ",";
    dataLine += String(accelZ, 2) + ",";
    
    // Gyroscope data (3 decimal places - reduced from 5) 
    dataLine += String(gyroX, 3) + ",";
    dataLine += String(gyroY, 3) + ",";
    dataLine += String(gyroZ, 3) + ",";
    
    // Notes field - use dash for data lines to keep file clean
    dataLine += "-\n";
    
    // Write to serial (reduced frequency for performance)
    if (sampleNumber % 50 == 0) {  // Print every 50th sample to serial for monitoring
      Serial.print("📊 Sample "); Serial.print(sampleNumber); 
      Serial.print(": Alt="); Serial.print(altitude, 1); 
      Serial.print("m, Vel="); Serial.print(verticalVelocity, 1); 
      Serial.print("m/s, Load="); Serial.print(loadCellWeight, 2); Serial.println("kg");
    }
    
    // Efficient buffered writing to file
    if (filesystemReady && dataFile) {
      dataBuffer += dataLine;
      bufferLineCount++;
      
      // Write buffer when full or every 50 samples for better timing consistency
      if (bufferLineCount >= BUFFER_SIZE || sampleNumber % 50 == 0) {
        dataFile.print(dataBuffer);
        dataFile.flush();
        dataBuffer = "";
        bufferLineCount = 0;
      }
    }
      
    sampleNumber++;
  }

  void firePyroChannel(int channel) {
    if (channel < 0 || channel > 3) {
      Serial.println("ERROR: Invalid pyro channel");
      return;
    }
    
    Serial.print("FIRING PYRO CHANNEL "); Serial.print(channel + 1);
    Serial.println(" - **DANGER: HIGH VOLTAGE**");
    
    // Fire the pyro channel
    digitalWrite(pyroPins[channel], HIGH);
    delay(100);  // 100ms pulse
    digitalWrite(pyroPins[channel], LOW);
    
    Serial.print("Pyro channel "); Serial.print(channel + 1); Serial.println(" fired successfully");
    
    // Log pyro event if recording
    if (flightState == FLIGHT_RECORDING) {
      logPyroEvent(channel + 1);  // Use proper logging function
    }
  }

// === BUFFER MANAGEMENT ===
void flushDataBuffer() {
  if (filesystemReady && dataFile && dataBuffer.length() > 0) {
    dataFile.print(dataBuffer);
    dataFile.flush();
    dataBuffer = "";
    bufferLineCount = 0;
    Serial.println("🔄 Data buffer flushed for event logging");
  }
}

// === FILE MANAGEMENT FUNCTIONS ===

void listFiles() {
  if (!filesystemReady) {
    Serial.println("ERROR: LittleFS not available");
    return;
  }
  
  Serial.println("\n=== STORED FILES ===");
  File root = LittleFS.open("/");
  if (!root) {
    Serial.println("ERROR: Failed to open root directory");
    return;
  }
  
  if (!root.isDirectory()) {
    Serial.println("ERROR: Root is not a directory");
    return;
  }
  
  int fileCount = 0;
  size_t totalSize = 0;
  
  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) {
      fileCount++;
      size_t fileSize = file.size();
      totalSize += fileSize;
      
      Serial.print("  ");
      Serial.print(file.name());
      Serial.print(" - ");
      Serial.print(fileSize);
      Serial.print(" bytes (");
      Serial.print(fileSize / 1024.0, 1);
      Serial.println(" KB)");
    }
    file = root.openNextFile();
  }
  
  Serial.println("==================");
  Serial.print("Total files: "); Serial.println(fileCount);
  Serial.print("Total size: "); Serial.print(totalSize); Serial.print(" bytes (");
  Serial.print(totalSize / 1024.0, 1); Serial.println(" KB)");
  
  if (fileCount == 0) {
    Serial.println("No files stored");
  } else {
    Serial.println("Use DOWNLOAD <filename> to retrieve file data");
  }
  Serial.println();
}

void downloadFile(String filename) {
  if (!filesystemReady) {
    Serial.println("ERROR: LittleFS not available");
    return;
  }
  
  if (filename.length() == 0) {
    Serial.println("ERROR: Please specify filename. Usage: DOWNLOAD <filename>");
    return;
  }
  
  // Add leading slash if not present
  String filepath = filename.startsWith("/") ? filename : "/" + filename;
  
  File file = LittleFS.open(filepath, "r");
  if (!file) {
    Serial.print("ERROR: File not found: "); Serial.println(filepath);
    Serial.println("Use FILES command to see available files");
    return;
  }
  
  Serial.println("\n=== FILE DOWNLOAD START ===");
  Serial.print("Filename: "); Serial.println(file.name());
  Serial.print("Size: "); Serial.print(file.size()); Serial.println(" bytes");
  Serial.println("--- FILE CONTENT ---");
  
  // Stream file content
  while (file.available()) {
    Serial.write(file.read());
  }
  
  file.close();
  Serial.println("\n--- END OF FILE ---");
  Serial.println("=== FILE DOWNLOAD COMPLETE ===\n");
}

void deleteFile(String filename) {
  if (!filesystemReady) {
    Serial.println("ERROR: LittleFS not available");
    return;
  }
  
  if (filename.length() == 0) {
    Serial.println("ERROR: Please specify filename. Usage: DELETE <filename>");
    return;
  }
  
  // Prevent deletion during recording
  if (flightState == FLIGHT_RECORDING) {
    Serial.println("ERROR: Cannot delete files during recording");
    return;
  }
  
  // Add leading slash if not present
  String filepath = filename.startsWith("/") ? filename : "/" + filename;
  
  // Check if file exists
  if (!LittleFS.exists(filepath)) {
    Serial.print("ERROR: File not found: "); Serial.println(filepath);
    return;
  }
  
  // Get file size before deletion
  File file = LittleFS.open(filepath, "r");
  size_t fileSize = 0;
  if (file) {
    fileSize = file.size();
    file.close();
  }
  
  // Delete the file
  if (LittleFS.remove(filepath)) {
    Serial.print("SUCCESS: Deleted file "); Serial.println(filepath);
    Serial.print("Freed "); Serial.print(fileSize); Serial.print(" bytes (");
    Serial.print(fileSize / 1024.0, 1); Serial.println(" KB)");
  } else {
    Serial.print("ERROR: Failed to delete file "); Serial.println(filepath);
  }
}

void showFilesystemSpace() {
  if (!filesystemReady) {
    Serial.println("ERROR: LittleFS not available");
    return;
  }
  
  size_t totalBytes = LittleFS.totalBytes();
  size_t usedBytes = LittleFS.usedBytes();
  size_t freeBytes = totalBytes - usedBytes;
  
  Serial.println("\n=== FILESYSTEM SPACE ===");
  Serial.print("Total space: "); 
  Serial.print(totalBytes); Serial.print(" bytes (");
  Serial.print(totalBytes / 1024.0, 1); Serial.println(" KB)");
  
  Serial.print("Used space: "); 
  Serial.print(usedBytes); Serial.print(" bytes (");
  Serial.print(usedBytes / 1024.0, 1); Serial.println(" KB)");
  
  Serial.print("Free space: "); 
  Serial.print(freeBytes); Serial.print(" bytes (");
  Serial.print(freeBytes / 1024.0, 1); Serial.println(" KB)");
  
  float usedPercent = (usedBytes * 100.0) / totalBytes;
  Serial.print("Usage: "); Serial.print(usedPercent, 1); Serial.println("%");
  
  // Improved storage warnings with recording time estimates
  if (usedPercent > 90) {
    Serial.println("⚠️  WARNING: Filesystem is nearly full!");
    Serial.print("Estimated remaining recording time: ~");
    Serial.print((freeBytes / 1024) / 2.8, 1); // ~2.8KB/second at 40Hz with reduced precision
    Serial.println(" minutes");
  } else if (usedPercent > 75) {
    Serial.println("📊 NOTICE: Filesystem is getting full");
    Serial.print("Estimated remaining recording time: ~");
    Serial.print((freeBytes / 1024) / 2.8, 1);
    Serial.println(" minutes");
  } else {
    Serial.print("📈 Estimated total recording time: ~");
    Serial.print((totalBytes / 1024) / 2.8, 1);
    Serial.println(" minutes");
  }
  
  Serial.println("💡 Data precision optimized: ~40% storage reduction achieved");
  Serial.println("========================\n");
}

void formatFilesystem() {
  if (flightState == FLIGHT_RECORDING) {
    Serial.println("ERROR: Cannot format during recording");
    return;
  }
  
  Serial.println("WARNING: This will delete ALL stored flight data!");
  Serial.print("Type 'YES' to confirm format: ");
  
  String confirmation = getSerialInput();
  if (confirmation != "YES") {
    Serial.println("Format cancelled");
    return;
  }
  
  Serial.println("Formatting LittleFS...");
  
  if (LittleFS.format()) {
    Serial.println("SUCCESS: Filesystem formatted");
    filesystemReady = true;
    showFilesystemSpace();
  } else {
    Serial.println("ERROR: Format failed");
    filesystemReady = false;
  }
}

// Update pyro event logging
void logPyroEvent(int channel) {
  // Allow logging if file is prepared (CONFIGURED state) or actively recording
  if ((flightState == FLIGHT_CONFIGURED || flightState == FLIGHT_RECORDING) && filesystemReady && dataFile) {
    // CRITICAL: Flush any buffered data first to maintain chronological order
    if (flightState == FLIGHT_RECORDING) {
      flushDataBuffer();
    }
    
    // Calculate timestamp - use 0 if recording hasn't started yet
    unsigned long timestamp = (flightState == FLIGHT_RECORDING) ? (millis() - recordingStartTime) : 0;
    
    // Log as critical event with current sensor readings (consistent format)
    String eventLine = String(timestamp) + ",PYRO" + String(channel) + ",";
    eventLine += String(altitude, 2) + ",";
    eventLine += String(verticalVelocity, 2) + ",";
    eventLine += String(loadCellWeight, 2) + ",";
    eventLine += String(accelX, 2) + ",";
    eventLine += String(accelY, 2) + ",";
    eventLine += String(accelZ, 2) + ",";
    eventLine += String(gyroX, 3) + ",";
    eventLine += String(gyroY, 3) + ",";
    eventLine += String(gyroZ, 3) + ",";
    eventLine += "Pyro channel " + String(channel) + " fired";
    if (flightState == FLIGHT_CONFIGURED) {
      eventLine += " (before flight start)";
    }
    eventLine += "\n";
    
    // Critical events write immediately
    dataFile.print(eventLine);
    dataFile.flush();
    
    Serial.print("📝 Logged PYRO"); Serial.print(channel); Serial.println(" event to flight data");
  } else {
    Serial.println("⚠️ Cannot log PYRO event - file not ready or invalid state");
  }
}

void logOffsetEvent() {
  // Allow logging if file is prepared (CONFIGURED state) or actively recording
  if ((flightState == FLIGHT_CONFIGURED || flightState == FLIGHT_RECORDING) && filesystemReady && dataFile) {
    // CRITICAL: Flush any buffered data first to maintain chronological order
    if (flightState == FLIGHT_RECORDING) {
      flushDataBuffer();
    }
    
    // Calculate timestamp - use 0 if recording hasn't started yet
    unsigned long timestamp = (flightState == FLIGHT_RECORDING) ? (millis() - recordingStartTime) : 0;
    
    // Log sensor reset event with consistent format
    String eventLine = String(timestamp) + ",OFFSET_EVENT,";
    eventLine += String(altitude, 2) + ",";
    eventLine += String(verticalVelocity, 2) + ",";
    eventLine += String(loadCellWeight, 2) + ",";
    eventLine += String(accelX, 2) + ",";
    eventLine += String(accelY, 2) + ",";
    eventLine += String(accelZ, 2) + ",";
    eventLine += String(gyroX, 3) + ",";
    eventLine += String(gyroY, 3) + ",";
    eventLine += String(gyroZ, 3) + ",";
    eventLine += "Sensor zero points reset";
    if (flightState == FLIGHT_CONFIGURED) {
      eventLine += " (before flight start)";
    }
    eventLine += "\n";
    
    // Critical events write immediately
    dataFile.print(eventLine);
    dataFile.flush();
    
    Serial.println("📝 Logged OFFSET_EVENT to flight data");
  } else {
    Serial.println("⚠️ Cannot log OFFSET event - file not ready or invalid state");
    Serial.print("   Flight State: "); Serial.println(flightState);
    Serial.print("   Filesystem Ready: "); Serial.println(filesystemReady);
    Serial.print("   Data File Open: "); Serial.println(dataFile ? "Yes" : "No");
  }
}

// === WIFI SOFTAP FUNCTIONS ===

void startWiFiSoftAP() {
  if (wifiEnabled) {
    Serial.println("📶 WiFi already running");
    return;
  }
  
  Serial.println("📶 Starting WiFi SoftAP...");
  Serial.println("   SSID: " + String(WIFI_SSID));
  Serial.println("   Password: " + String(WIFI_PASSWORD));
  
  // Configure SoftAP
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, netMsk);
  
  bool success = WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL, false, MAX_CONNECTIONS);
  
  if (success) {
    IPAddress IP = WiFi.softAPIP();
    Serial.print("   ✅ SoftAP IP address: ");
    Serial.println(IP);
    
    // Start mDNS
    if (MDNS.begin("paviflightdata")) {
      Serial.println("   ✅ mDNS responder started");
    }
    
    // Setup web server routes
    setupWebServer();
    
    // Start web server
    server.begin();
    Serial.println("   ✅ Web server started on port 80");
    Serial.println("");
    Serial.println("🌐 Access flight data at:");
    Serial.println("   • http://" + IP.toString());
    Serial.println("   • http://paviflightdata.local");
    Serial.println("📱 Connect to WiFi: " + String(WIFI_SSID));
    
    wifiEnabled = true;
  } else {
    Serial.println("   ❌ Failed to start SoftAP");
  }
}

void stopWiFiSoftAP() {
  if (!wifiEnabled) {
    Serial.println("📶 WiFi not running");
    return;
  }
  
  Serial.println("📶 Stopping WiFi SoftAP...");
  
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  
  wifiEnabled = false;
  Serial.println("   ✅ WiFi SoftAP stopped");
}

void setupWebServer() {
  // Main page - file browser
  server.on("/", HTTP_GET, handleRoot);
  
  // File download endpoint
  server.on("/download", HTTP_GET, handleFileDownload);
  
  // File list API endpoint
  server.on("/api/files", HTTP_GET, handleFileListAPI);
  
  // System info API endpoint
  server.on("/api/info", HTTP_GET, handleSystemInfoAPI);
  
  // File delete API endpoint
  server.on("/api/delete", HTTP_DELETE, handleFileDeleteAPI);
  
  // 404 handler
  server.onNotFound(handleNotFound);
  
  Serial.println("   📡 Web server routes configured");
}

void handleRoot() {
  String html = generateMainPage();
  server.send(200, "text/html", html);
}

void handleFileDownload() {
  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "Missing file parameter");
    return;
  }
  
  String filename = server.arg("file");
  String filepath = "/" + filename;
  
  if (!LittleFS.exists(filepath)) {
    server.send(404, "text/plain", "File not found: " + filename);
    return;
  }
  
  File file = LittleFS.open(filepath, "r");
  if (!file) {
    server.send(500, "text/plain", "Failed to open file: " + filename);
    return;
  }
  
  // Set headers for file download
  server.sendHeader("Content-Disposition", "attachment; filename=" + filename);
  server.sendHeader("Content-Type", "text/plain");
  
  // Stream file content
  server.streamFile(file, "text/plain");
  file.close();
  
  Serial.println("📥 Downloaded: " + filename + " to client " + server.client().remoteIP().toString());
}

void handleFileListAPI() {
  String json = "{\"files\":[";
  
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  bool first = true;
  
  while (file) {
    if (!file.isDirectory()) {
      if (!first) json += ",";
      json += "{";
      json += "\"name\":\"" + String(file.name()) + "\",";
      json += "\"size\":" + String(file.size()) + ",";
      json += "\"type\":\"" + getFileType(file.name()) + "\"";
      json += "}";
      first = false;
    }
    file = root.openNextFile();
  }
  
  json += "],\"totalFiles\":" + String(getLittleFSFileCount()) + "}";
  
  server.send(200, "application/json", json);
}

void handleSystemInfoAPI() {
  String json = "{";
  json += "\"chipModel\":\"" + String(ESP.getChipModel()) + "\",";
  json += "\"chipRevision\":" + String(ESP.getChipRevision()) + ",";
  json += "\"cpuFreq\":" + String(ESP.getCpuFreqMHz()) + ",";
  json += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
  json += "\"totalHeap\":" + String(ESP.getHeapSize()) + ",";
  json += "\"flashSize\":" + String(ESP.getFlashChipSize()) + ",";
  json += "\"flashUsed\":" + String(LittleFS.usedBytes()) + ",";
  json += "\"flashTotal\":" + String(LittleFS.totalBytes()) + ",";
  json += "\"uptime\":" + String(millis()) + ",";
  json += "\"flightState\":" + String(flightState) + ",";
  json += "\"wifiClients\":" + String(WiFi.softAPgetStationNum());
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
  
  // Security check - only allow .txt files to be deleted
  // Can delete all files if desired, comment out the next lines
  // if (!filename.endsWith(".txt")) {
  //   server.send(403, "text/plain", "Only .txt files can be deleted");
  //   return;
  // }
  
  if (!LittleFS.exists(filepath)) {
    server.send(404, "text/plain", "File not found: " + filename);
    return;
  }
  
  if (LittleFS.remove(filepath)) {
    server.send(200, "text/plain", "File deleted successfully: " + filename);
    Serial.println("🗑️ File deleted via web interface: " + filename);
  } else {
    server.send(500, "text/plain", "Failed to delete file: " + filename);
    Serial.println("❌ Failed to delete file via web interface: " + filename);
  }
}

void handleNotFound() {
  server.send(404, "text/plain", "Page not found");
}

String generateMainPage() {
String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>🚁 Pavi Flight Data Server</title>
    <style>
        body { 
            font-family: Arial, sans-serif; 
            margin: 20px; 
            background-color: #f5f5f5; 
        }
        .container { 
            max-width: 800px; 
            margin: 0 auto; 
            background: white; 
            padding: 20px; 
            border-radius: 10px; 
            box-shadow: 0 2px 10px rgba(0,0,0,0.1); 
        }
        .header { 
            text-align: center; 
            color: #2c3e50; 
            border-bottom: 2px solid #3498db; 
            padding-bottom: 10px; 
            margin-bottom: 20px; 
        }
        .info-box { 
            background: #ecf0f1; 
            padding: 15px; 
            border-radius: 5px; 
            margin: 15px 0; 
        }
        .file-list { 
            margin: 20px 0; 
        }
        .file-item { 
            display: flex; 
            justify-content: space-between; 
            align-items: center; 
            padding: 10px; 
            border: 1px solid #ddd; 
            margin: 5px 0; 
            border-radius: 5px; 
            background: #fff; 
        }
        .file-name { 
            font-weight: bold; 
            color: #2c3e50; 
        }
        .file-size { 
            color: #7f8c8d; 
            font-size: 0.9em; 
        }
        .download-btn { 
            background: #3498db; 
            color: white; 
            border: none; 
            padding: 8px 15px; 
            border-radius: 3px; 
            cursor: pointer; 
            text-decoration: none; 
            display: inline-block; 
        }
        .download-btn:hover { 
            background: #2980b9; 
        }
        .delete-btn { 
            background: #e74c3c; 
            color: white; 
            border: none; 
            padding: 8px 15px; 
            border-radius: 3px; 
            cursor: pointer; 
            text-decoration: none; 
            display: inline-block; 
            margin-left: 10px; 
            font-size: 0.9em; 
        }
        .delete-btn:hover { 
            background: #c0392b; 
        }
        .refresh-btn { 
            background: #27ae60; 
            color: white; 
            border: none; 
            padding: 10px 20px; 
            border-radius: 5px; 
            cursor: pointer; 
            margin: 10px 0; 
        }
        .refresh-btn:hover { 
            background: #219a52; 
        }
        .status { 
            display: inline-block; 
            padding: 5px 10px; 
            border-radius: 15px; 
            font-size: 0.8em; 
            font-weight: bold; 
        }
        .status.online { 
            background: #2ecc71; 
            color: white; 
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <div class="status online">📶 SoftAP Online</div>
        </div>
        
        <div class="info-box">
            <h3>📊 System Information</h3>
            <p><strong>Device:</strong> ESP32 Flight Computer</p>
            <p><strong>Uptime:</strong> <span id="uptime">Loading...</span></p>
            <p><strong>RAM Memory:</strong> <span id="memory">Loading...</span></p>
            <p><strong>Flash Storage:</strong> <span id="storage">Loading...</span></p>
            <p><strong>Connected Clients:</strong> <span id="clients">Loading...</span></p>
        </div>
        
        <div class="file-list">
            <h3>📁 Available Flight Data Files</h3>
            <button class="refresh-btn" onclick="refreshFiles()"> Refresh File List </button>
            <div id="files">
                <p>📡 Loading files...</p>
            </div>
        </div>
    </div>

    <script>
        function formatBytes(bytes) {
            if (bytes === 0) return '0 B';
            const k = 1024;
            const sizes = ['B', 'KB', 'MB', 'GB'];
            const i = Math.floor(Math.log(bytes) / Math.log(k));
            return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];
        }
        
        function formatUptime(ms) {
            const seconds = Math.floor(ms / 1000);
            const hours = Math.floor(seconds / 3600);
            const minutes = Math.floor((seconds % 3600) / 60);
            const secs = seconds % 60;
            return hours + 'h ' + minutes + 'm ' + secs + 's';
        }
        
        async function refreshFiles() {
            try {
                const response = await fetch('/api/files');
                const data = await response.json();
                
                let html = '';
                if (data.files.length === 0) {
                    html = '<p>📂 No files found. Start a flight recording to generate data files!</p>';
                } else {
                    data.files.forEach(file => {
                        const icon = file.type === 'txt' ? '📄' : '📁';
                        html += `
                            <div class="file-item">
                                <div>
                                    <span class="file-name">${icon} ${file.name}</span><br>
                                    <span class="file-size">${formatBytes(file.size)}</span>
                                </div>
                                <div>
                                    <a href="/download?file=${file.name}" class="download-btn"> Download</a>
                                    <button class="delete-btn" onclick="deleteFile('${file.name}')"> Delete</button>
                                </div>
                            </div>
                        `;
                    });
                }
                document.getElementById('files').innerHTML = html;
            } catch (error) {
                document.getElementById('files').innerHTML = '<p style="color: red;">❌ Error loading files</p>';
            }
        }
        
        async function updateSystemInfo() {
            try {
                const response = await fetch('/api/info');
                const data = await response.json();
                
                document.getElementById('uptime').textContent = formatUptime(data.uptime);
                document.getElementById('memory').textContent = formatBytes(data.freeHeap) + ' free / ' + formatBytes(data.totalHeap) + ' total';
                document.getElementById('storage').textContent = formatBytes(data.flashTotal - data.flashUsed) + ' free / ' + formatBytes(data.flashTotal) + ' total (' + formatBytes(data.flashUsed) + ' used)';
                document.getElementById('clients').textContent = data.wifiClients;
            } catch (error) {
                console.error('Error updating system info:', error);
            }
        }

        async function deleteFile(filename) {
            if (!confirm(`Are you sure you want to delete "${filename}"?\\n\\nThis action cannot be undone!`)) {
                return;
            }
            
            try {
                const response = await fetch(`/api/delete?file=${encodeURIComponent(filename)}`, {
                    method: 'DELETE'
                });
                
                if (response.ok) {
                    alert(`✅ File "${filename}" deleted successfully`);
                    refreshFiles(); // Refresh the file list
                } else {
                    const error = await response.text();
                    alert(`❌ Failed to delete file: ${error}`);
                }
            } catch (error) {
                alert(`❌ Error deleting file: ${error.message}`);
            }
        }
        
        // Initial load
        refreshFiles();
        updateSystemInfo();
        
        // Auto-refresh every 5 seconds
        setInterval(updateSystemInfo, 5000);
    </script>
</body>
</html>
)rawliteral";
  
  return html;
}

String getFileType(String filename) {
  if (filename.endsWith(".txt")) return "txt";
  if (filename.endsWith(".csv")) return "csv";
  if (filename.endsWith(".log")) return "log";
  return "unknown";
}

int getLittleFSFileCount() {
  int count = 0;
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  
  while (file) {
    if (!file.isDirectory()) {
      count++;
    }
    file = root.openNextFile();
  }
  
  return count;
}

void handleWiFiClients() {
  if (wifiEnabled) {
    server.handleClient();
  }
}

// === LOAD CELL CALIBRATION FUNCTIONS ===
// Calibration state variables
static bool calibrationMode = false;
static bool zeroCalibrated = false;
static bool weightCalibrated = false;
static float calibrationKnownWeight = 0.0;
static long rawZeroReading = 0;
static long rawWeightReading = 0;

void startLoadCellCalibration() {
  Serial.println("\n⚖️ === LOAD CELL CALIBRATION STARTED ===");
  Serial.println("📋 Calibration Process:");
  Serial.println("   1. First, calibrate ZERO (no weight)");
  Serial.println("   2. Then, calibrate with known weight");
  Serial.println("   3. System will calculate calibration factor");
  Serial.println("");
  Serial.println("💡 Use menu system or commands:");
  Serial.println("   • MENU → 4 → 2 (Zero calibration)");
  Serial.println("   • CAL_WEIGHT:<weight> (Weight calibration)");
  Serial.println("   • CAL_SAVE (Save calibration)");
  
  calibrationMode = true;
  zeroCalibrated = false;
  weightCalibrated = false;
  calibrationKnownWeight = 0.0;
  
  Serial.println("✅ Calibration mode activated");
}

void stopLoadCellCalibration() {
  Serial.println("\n⚖️ === LOAD CELL CALIBRATION STOPPED ===");
  if (zeroCalibrated && weightCalibrated) {
    Serial.println("✅ Calibration completed successfully");
    Serial.print("📊 Final calibration factor: ");
    Serial.println(loadCellCalibrationFactor, 2);
  } else {
    Serial.println("⚠️ Calibration incomplete:");
    Serial.print("   Zero calibrated: "); Serial.println(zeroCalibrated ? "✅" : "❌");
    Serial.print("   Weight calibrated: "); Serial.println(weightCalibrated ? "✅" : "❌");
  }
  
  calibrationMode = false;
  Serial.println("==========================================");
}

bool isLoadCellCalibrating() {
  return calibrationMode;
}

void calibrateLoadCellZero() {
  Serial.println("\n🎯 === ZERO POINT CALIBRATION ===");
  
  if (!loadCellReady) {
    Serial.println("❌ ERROR: Load cell not ready");
    return;
  }
  
  Serial.println("📋 Instructions:");
  Serial.println("   1. Remove ALL weight from load cell");
  Serial.println("   2. Ensure load cell is not touching anything");
  Serial.println("   3. Wait for stable readings...");
  Serial.println("");
  
  // Wait for stable readings
  Serial.println("⏳ Stabilizing... (5 seconds)");
  delay(5000);
  
  if (!loadCell.is_ready()) {
    Serial.println("❌ ERROR: Load cell not responding");
    return;
  }
  
  // Take multiple readings for accuracy
  Serial.println("📊 Taking zero readings...");
  long totalReading = 0;
  int validReadings = 0;
  
  for (int i = 0; i < 20; i++) {
    if (loadCell.is_ready()) {
      long reading = loadCell.read();
      if (reading != 0) {  // Skip failed readings
        totalReading += reading;
        validReadings++;
        Serial.print("  Reading "); Serial.print(i + 1); 
        Serial.print(": "); Serial.println(reading);
      }
      delay(100);
    }
  }
  
  if (validReadings < 10) {
    Serial.println("❌ ERROR: Not enough valid readings");
    return;
  }
  
  // Calculate average zero point
  rawZeroReading = totalReading / validReadings;
  loadCell.set_offset(rawZeroReading);
  loadCellOffset = rawZeroReading;
  
  zeroCalibrated = true;
  
  Serial.println("✅ ZERO CALIBRATION COMPLETE");
  Serial.print("📊 Average zero reading: "); Serial.println(rawZeroReading);
  Serial.print("📊 Valid samples: "); Serial.print(validReadings); Serial.println("/20");
  Serial.println("🎯 Load cell zeroed successfully!");
  
  // Test the zero
  delay(1000);
  if (loadCell.is_ready()) {
    float testWeight = loadCell.get_units(5);
    Serial.print("🧪 Zero test reading: "); 
    Serial.print(testWeight, 4); Serial.println(" kg");
    
    if (abs(testWeight) < 0.01) {
      Serial.println("✅ Zero calibration verified");
    } else {
      Serial.println("⚠️ Zero reading not perfect, but acceptable");
    }
  }
  
  Serial.println("================================");
}

void calibrateLoadCellWeight(float weight) {
  Serial.println("\n⚖️ === WEIGHT CALIBRATION ===");
  
  if (!loadCellReady) {
    Serial.println("❌ ERROR: Load cell not ready");
    return;
  }
  
  if (!zeroCalibrated) {
    Serial.println("❌ ERROR: Must calibrate zero point first!");
    Serial.println("💡 Use: MENU → 4 → 2 (Zero Load Cell)");
    return;
  }
  
  if (weight <= 0) {
    Serial.println("❌ ERROR: Weight must be positive");
    return;
  }
  
  calibrationKnownWeight = weight;
  
  Serial.print("📋 Instructions for "); Serial.print(weight, 2); Serial.println(" kg calibration:");
  Serial.println("   1. Place EXACTLY the known weight on load cell");
  Serial.println("   2. Ensure weight is centered and stable");
  Serial.println("   3. Wait for stable readings...");
  Serial.println("");
  
  // Wait for weight to be placed and stabilized
  Serial.println("⏳ Stabilizing... (10 seconds)");
  Serial.println("   (Time to place weight and let it settle)");
  delay(10000);
  
  if (!loadCell.is_ready()) {
    Serial.println("❌ ERROR: Load cell not responding");
    return;
  }
  
  // Take multiple readings for accuracy
  Serial.println("📊 Taking weight readings...");
  long totalReading = 0;
  int validReadings = 0;
  
  for (int i = 0; i < 20; i++) {
    if (loadCell.is_ready()) {
      long reading = loadCell.get_value(1);  // Raw reading minus offset
      if (reading != 0) {  // Skip failed readings
        totalReading += reading;
        validReadings++;
        Serial.print("  Reading "); Serial.print(i + 1); 
        Serial.print(": "); Serial.println(reading);
      }
      delay(200);
    }
  }
  
  if (validReadings < 10) {
    Serial.println("❌ ERROR: Not enough valid readings");
    return;
  }
  
  // Calculate average weight reading and calibration factor
  rawWeightReading = totalReading / validReadings;
  
  if (rawWeightReading == 0) {
    Serial.println("❌ ERROR: No weight detected");
    Serial.println("💡 Check that weight is properly placed");
    return;
  }
  
  // Calculate calibration factor: raw_reading / actual_weight
  float newCalibrationFactor = (float)rawWeightReading / weight;
  
  // Update calibration
  loadCellCalibrationFactor = newCalibrationFactor;
  loadCell.set_scale(loadCellCalibrationFactor);
  
  weightCalibrated = true;
  
  Serial.println("✅ WEIGHT CALIBRATION COMPLETE");
  Serial.print("📊 Known weight: "); Serial.print(weight, 2); Serial.println(" kg");
  Serial.print("📊 Average raw reading: "); Serial.println(rawWeightReading);
  Serial.print("📊 Calculated factor: "); Serial.println(newCalibrationFactor, 2);
  Serial.print("📊 Valid samples: "); Serial.print(validReadings); Serial.println("/20");
  
  // Test the calibration immediately
  delay(1000);
  if (loadCell.is_ready()) {
    float testWeight = loadCell.get_units(5);
    Serial.print("🧪 Calibration test: "); 
    Serial.print(testWeight, 3); Serial.println(" kg");
    
    float error = abs(testWeight - weight);
    float errorPercent = (error / weight) * 100;
    
    Serial.print("🎯 Error: ±"); Serial.print(error, 3); 
    Serial.print(" kg ("); Serial.print(errorPercent, 1); Serial.println("%)");
    
    if (errorPercent < 5.0) {
      Serial.println("✅ Calibration accuracy: EXCELLENT");
    } else if (errorPercent < 10.0) {
      Serial.println("✅ Calibration accuracy: GOOD");
    } else {
      Serial.println("⚠️ Calibration accuracy: FAIR (consider recalibrating)");
    }
  }
  
  Serial.println("================================");
}

void saveLoadCellCalibration() {
  Serial.println("\n💾 === SAVING CALIBRATION ===");
  
  if (!filesystemReady) {
    Serial.println("❌ ERROR: Filesystem not ready");
    return;
  }
  
  if (!zeroCalibrated || !weightCalibrated) {
    Serial.println("❌ ERROR: Incomplete calibration");
    Serial.print("   Zero: "); Serial.println(zeroCalibrated ? "✅" : "❌");
    Serial.print("   Weight: "); Serial.println(weightCalibrated ? "✅" : "❌");
    return;
  }
  
  // Create calibration file
  File calFile = LittleFS.open("/loadcell_cal.txt", "w");
  if (!calFile) {
    Serial.println("❌ ERROR: Cannot create calibration file");
    return;
  }
  
  // Write calibration data
  calFile.println("# Load Cell Calibration Data");
  calFile.println("# Generated by PAVI Flight Computer");
  calFile.print("# Timestamp: "); calFile.println(millis());
  calFile.println("");
  
  calFile.print("calibration_factor="); calFile.println(loadCellCalibrationFactor, 6);
  calFile.print("zero_offset="); calFile.println(loadCellOffset);
  calFile.print("zero_raw_reading="); calFile.println(rawZeroReading);
  calFile.print("weight_raw_reading="); calFile.println(rawWeightReading);
  calFile.print("known_weight_kg="); calFile.println(calibrationKnownWeight, 3);
  
  calFile.println("");
  calFile.println("# Usage:");
  calFile.println("# loadCell.set_scale(calibration_factor);");
  calFile.println("# loadCell.set_offset(zero_offset);");
  
  calFile.close();
  
  Serial.println("✅ Calibration saved successfully!");
  Serial.println("📄 File: /loadcell_cal.txt");
  Serial.print("📊 Calibration factor: "); Serial.println(loadCellCalibrationFactor, 6);
  Serial.print("📊 Zero offset: "); Serial.println(loadCellOffset);
  
  // Also save to EEPROM-like preferences for auto-loading
  Serial.println("💾 Calibration will be used for all future measurements");
  Serial.println("================================");
}

void loadCalibrationFromFile() {
  if (!filesystemReady) return;
  
  File calFile = LittleFS.open("/loadcell_cal.txt", "r");
  if (!calFile) {
    Serial.println("📄 No saved calibration found");
    return;
  }
  
  Serial.println("📄 Loading saved calibration...");
  
  while (calFile.available()) {
    String line = calFile.readStringUntil('\n');
    line.trim();
    
    if (line.startsWith("calibration_factor=")) {
      String value = line.substring(19);
      loadCellCalibrationFactor = value.toFloat();
    } else if (line.startsWith("zero_offset=")) {
      String value = line.substring(12);
      loadCellOffset = (long)value.toFloat();
    }
  }
  
  calFile.close();
  
  // Apply loaded calibration
  if (loadCellReady && loadCellCalibrationFactor != 0) {
    loadCell.set_scale(loadCellCalibrationFactor);
    loadCell.set_offset(loadCellOffset);
    
    Serial.println("✅ Calibration loaded and applied");
    Serial.print("📊 Factor: "); Serial.println(loadCellCalibrationFactor, 6);
    Serial.print("📊 Offset: "); Serial.println(loadCellOffset);
  }
}

void testLoadCellCalibration() {
  Serial.println("\n🧪 === LOAD CELL TEST ===");
  
  if (!loadCellReady) {
    Serial.println("❌ Load cell not ready");
    return;
  }
  
  if (!loadCell.is_ready()) {
    Serial.println("❌ Load cell not responding");
    return;
  }
  
  Serial.println("📊 Current load cell status:");
  Serial.print("   Calibration factor: "); Serial.println(loadCellCalibrationFactor, 6);
  Serial.print("   Zero offset: "); Serial.println(loadCellOffset);
  Serial.println("");
  
  // Take multiple test readings
  Serial.println("📊 Taking 10 test readings...");
  float totalWeight = 0;
  int validReadings = 0;
  
  for (int i = 0; i < 10; i++) {
    if (loadCell.is_ready()) {
      long rawReading = loadCell.read();
      float weight = loadCell.get_units(1);
      
      Serial.print("  "); Serial.print(i + 1); 
      Serial.print(". Raw: "); Serial.print(rawReading);
      Serial.print(", Weight: "); Serial.print(weight, 3); Serial.println(" kg");
      
      if (rawReading != 0) {
        totalWeight += weight;
        validReadings++;
      }
      delay(500);
    }
  }
  
  if (validReadings > 0) {
    float avgWeight = totalWeight / validReadings;
    Serial.println("");
    Serial.print("📊 Average reading: "); Serial.print(avgWeight, 3); Serial.println(" kg");
    Serial.print("📊 Valid samples: "); Serial.print(validReadings); Serial.println("/10");
    
    if (abs(avgWeight) < 0.01) {
      Serial.println("✅ Load cell appears to be properly zeroed");
    } else if (abs(avgWeight) < 0.1) {
      Serial.println("⚠️ Small offset detected - consider re-zeroing");
    } else {
      Serial.println("❌ Significant offset - calibration may be needed");
    }
  } else {
    Serial.println("❌ No valid readings obtained");
  }
  
  Serial.println("========================");
}

// === END OF FILE ===
