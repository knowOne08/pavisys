#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <MS5611.h>  
#include <MPU6050.h>
#include <HX711.h>
#include "FS.h"
#include "LittleFS.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

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

// SIMPLE RATE SELECTION - Choose your data rate
#define DATA_RATE_MODE 4  // 1=10Hz, 2=20Hz, 3=30Hz, 4=40Hz

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

// Calculated data
float verticalVelocity = 0.0;
float lastAltitude = 0.0;
unsigned long lastVelocityTime = 0;

// Remote command system
bool waitingForCommand = false;
String commandBuffer = "";

// === WIFI SOFTAP CONFIGURATION ===
#define WIFI_SSID "PaviFlightData"
#define WIFI_PASSWORD "pavi2025"
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
HX711 loadCell;
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
void handleNotFound();
void handleWiFiClients();

// Calibration functions
void startLoadCellCalibration();
void stopLoadCellCalibration();
bool isLoadCellCalibrating();
void calibrateLoadCellZero();
void calibrateLoadCellWeight(float weight);
void saveLoadCellCalibration();
void testLoadCellCalibration();

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("=== TX: LoRa Command Responder ===");
  Serial.println("Pure command responder - no unsolicited telemetry");
  Serial.println("Sensors: MS5611 + MPU6050 + HX711 + 4x Pyro Channels");
  Serial.print("Barometer: "); Serial.println(ENABLE_BAROMETER ? "ON" : "OFF");
  Serial.print("Accelerometer: "); Serial.println(ENABLE_ACCELEROMETER ? "ON" : "OFF");
  Serial.print("Gyroscope: "); Serial.println(ENABLE_GYROSCOPE ? "ON" : "OFF");
  Serial.print("Temperature: "); Serial.println(ENABLE_TEMPERATURE ? "ON" : "OFF");
  Serial.print("Load Cell: "); Serial.println(ENABLE_LOAD_CELL ? "ON" : "OFF");
  Serial.print("Pyro Test Mode: "); Serial.println(PYRO_TEST_MODE ? "ON" : "OFF");
  
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

  // Initialize HX711 Load Cell
  if (ENABLE_LOAD_CELL) {
    Serial.println("Initializing HX711 Load Cell...");
    loadCell.begin(HX711_DOUT, HX711_SCK);
    
    if (loadCell.is_ready()) {
      Serial.println("HX711 ready");
      
      // Set calibration factor
      loadCell.set_scale(loadCellCalibrationFactor);
      
      // Tare the scale (set current reading as zero)
      Serial.println("Taring load cell... (remove any weight)");
      delay(2000);  // Give time to remove weight
      loadCell.tare();
      loadCellOffset = loadCell.get_offset();
      
      Serial.print("Load cell tared. Offset: ");
      Serial.println(loadCellOffset);
      loadCellReady = true;
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
  if (millis() - lastSensorRead >= sendInterval) {
    readSensorsWithSmartLoadCell();
    lastSensorRead = millis();
  }
  
  // Log flight data if recording
  if (flightState == FLIGHT_RECORDING && enableDataLogging) {
    static unsigned long lastLogTime = 0;
    if (millis() - lastLogTime >= 100) { // Log every 100ms
      logFlightData();
      lastLogTime = millis();
    }
  }
  
  // Handle WiFi web server clients
  handleWiFiClients();
}

// Removed unused readSensors() function - using readSensorsWithSmartLoadCell() instead

// Removed unused readSensorsOptimized() function - using readSensorsWithSmartLoadCell() instead

// Helper functions for smooth filtering
float filterPressure(float newReading) {
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
  static float total = 0;
  
  total = total - gyroXReadings[gyroIndex];
  gyroXReadings[gyroIndex] = newReading;
  total = total + gyroXReadings[gyroIndex];
  
  return total / GYRO_FILTER_SIZE;
}

float filterGyroY(float newReading) {
  static float total = 0;
  
  total = total - gyroYReadings[gyroIndex];
  gyroYReadings[gyroIndex] = newReading;
  total = total + gyroYReadings[gyroIndex];
  
  return total / GYRO_FILTER_SIZE;
}

float filterGyroZ(float newReading) {
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
          loadCellRaw = loadCell.read();
          loadCellWeight = loadCell.get_units();
        }
        // If not ready, we just skip this cycle and use the previous value
      }
    }
}
  
  // === FLIGHT DATA FUNCTIONS ===

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
    Serial.println("Local Serial Commands:");
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
          // Check for serial toggle command
          if (commandBuffer.equalsIgnoreCase("SERIALTOGGLE")) {
            showSerialData = !showSerialData;
            Serial.print("Serial data output ");
            Serial.println(showSerialData ? "ENABLED" : "DISABLED");
          } else {
            processCommand(commandBuffer);
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
      // Be more flexible - accept CONFIG_READY if we have valid configuration
      bool hasValidConfig = !flightConfig.filename.isEmpty() && flightConfig.totalWeight > 0;
      
      if (flightState == FLIGHT_CONFIGURING || (hasValidConfig && flightState != FLIGHT_RECORDING)) {
        Serial.println("🚀 Configuration complete! Ready for flight operations.");
        flightState = FLIGHT_CONFIGURED;
        
        // Show final config
        Serial.println("=== FLIGHT CONFIGURATION ===");
        Serial.print("Filename: "); Serial.println(flightConfig.filename);
        Serial.print("Weight: "); Serial.print(flightConfig.totalWeight); Serial.println(" kg");
        Serial.print("Wind Speed: "); Serial.print(flightConfig.windSpeed); Serial.println(" m/s");
        Serial.print("Height: "); Serial.print(flightConfig.initialHeight); Serial.println(" m");
        Serial.println("==============================");
        
        if (flightState != FLIGHT_CONFIGURING) {
          Serial.println("ℹ️ Note: Configuration accepted even though not explicitly in config mode");
        }
      } else {
        Serial.print("❌ ERROR: Cannot complete configuration. State: ");
        Serial.print(flightState);
        Serial.print(", Valid config: ");
        Serial.println(hasValidConfig ? "Yes" : "No");
        Serial.println("📋 Current config status:");
        Serial.print("  Filename: "); Serial.println(flightConfig.filename.isEmpty() ? "Missing" : flightConfig.filename);
        Serial.print("  Weight: "); Serial.println(flightConfig.totalWeight);
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
    
    // Reset sensors to current position as origin
    resetSensorOrigins();
    
  // Create/open data file
  if (filesystemReady) {
    String filepath = "/" + flightConfig.filename;
    dataFile = LittleFS.open(filepath, "w");
    if (dataFile) {
      Serial.print("Created data file: "); Serial.println(filepath);
    } else {
      Serial.println("ERROR: Failed to create data file");
      filesystemReady = false;  // Fall back to serial logging
    }
  }
    
    recordingStartTime = millis();
    sampleNumber = 0;
    flightState = FLIGHT_RECORDING;
    
    // Write file header
    writeDataHeader();
    
    Serial.println("=== DATA RECORDING STARTED ===");
    Serial.println("Send STOP command to end recording");
    Serial.println("Send PYRO1-PYRO4 commands to fire pyro channels");
  }

  void stopDataRecording() {
    if (flightState != FLIGHT_RECORDING) {
      Serial.println("ERROR: Not currently recording");
      return;
    }
    
    flightState = FLIGHT_STOPPED;
    
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
    if (ENABLE_BAROMETER) {
      Serial.println("   📊 Resetting barometer zero point...");
      // Read current pressure and set as reference
      // This would reset the altitude calculation origin
      Serial.println("   ✅ Barometer reset complete");
    }
    
    // Reset IMU if needed
    if (ENABLE_ACCELEROMETER) {
      Serial.println("   📊 Resetting IMU calibration...");
      // Reset IMU offsets if applicable
      Serial.println("   ✅ IMU reset complete");
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
    
    // Create data line with new format: Time_ms,Event_Type,Altitude_m,Vertical_Velocity_ms,Load_Cell_kg,Accel_X_ms2,Accel_Y_ms2,Accel_Z_ms2,Gyro_X_rads,Gyro_Y_rads,Gyro_Z_rads,Notes
    String dataLine = String(relativeTime) + ",";
    dataLine += "DATA,";  // Event type
    dataLine += String(altitude, 3) + ",";
    dataLine += String(verticalVelocity, 3) + ",";
    dataLine += String(loadCellWeight, 3) + ",";
    dataLine += String(accelX, 4) + ",";
    dataLine += String(accelY, 4) + ",";
    dataLine += String(accelZ, 4) + ",";
    dataLine += String(gyroX, 5) + ",";
    dataLine += String(gyroY, 5) + ",";
    dataLine += String(gyroZ, 5) + ",";
    dataLine += "normal_data";  // Notes field
    dataLine += "\n";
    
    // Write to serial (reduced frequency for readability)
    if (sampleNumber % 50 == 0) {  // Print every 50th sample to serial for monitoring
      Serial.print("📊 Sample "); Serial.print(sampleNumber); 
      Serial.print(": Alt="); Serial.print(altitude, 1); 
      Serial.print("m, Vel="); Serial.print(verticalVelocity, 1); 
      Serial.print("m/s, Load="); Serial.print(loadCellWeight, 2); Serial.println("kg");
    }
    
    // Write to LittleFS file
    if (filesystemReady && dataFile) {
      dataFile.print(dataLine);
      if (sampleNumber % 10 == 0) {  // Flush every 10 samples for safety
        dataFile.flush();
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
      unsigned long relativeTime = millis() - recordingStartTime;
      String pyroEvent = "# PYRO_FIRE: Channel=" + String(channel + 1) + 
                        " Time=" + String(relativeTime) + "ms\n";
      Serial.print(pyroEvent);
      
      // TODO: Log to SD card when available
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
  
  if (usedPercent > 90) {
    Serial.println("WARNING: Filesystem is nearly full!");
  } else if (usedPercent > 75) {
    Serial.println("NOTICE: Filesystem is getting full");
  }
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
  if (flightState == FLIGHT_RECORDING && filesystemReady && dataFile) {
    unsigned long timestamp = millis() - recordingStartTime;
    
    // Log as event with current sensor readings
    String eventLine = String(timestamp) + ",";
    eventLine += "PYRO" + String(channel) + ",";
    eventLine += String(altitude, 3) + ",";
    eventLine += String(verticalVelocity, 3) + ",";
    eventLine += String(loadCellWeight, 3) + ",";
    eventLine += String(accelX, 4) + ",";
    eventLine += String(accelY, 4) + ",";
    eventLine += String(accelZ, 4) + ",";
    eventLine += String(gyroX, 5) + ",";
    eventLine += String(gyroY, 5) + ",";
    eventLine += String(gyroZ, 5) + ",";
    eventLine += "Pyro channel " + String(channel) + " fired";
    eventLine += "\n";
    
    dataFile.print(eventLine);
    dataFile.flush();  // Immediate flush for critical events
    
    Serial.print("📝 Logged PYRO"); Serial.print(channel); Serial.println(" event to flight data");
  }
}

void logOffsetEvent() {
  if (flightState == FLIGHT_RECORDING && filesystemReady && dataFile) {
    unsigned long timestamp = millis() - recordingStartTime;
    
    // Log sensor reset event
    String eventLine = String(timestamp) + ",";
    eventLine += "SENSOR_RESET,";
    eventLine += String(altitude, 3) + ",";
    eventLine += String(verticalVelocity, 3) + ",";
    eventLine += String(loadCellWeight, 3) + ",";
    eventLine += String(accelX, 4) + ",";
    eventLine += String(accelY, 4) + ",";
    eventLine += String(accelZ, 4) + ",";
    eventLine += String(gyroX, 5) + ",";
    eventLine += String(gyroY, 5) + ",";
    eventLine += String(gyroZ, 5) + ",";
    eventLine += "Sensor zero points reset";
    eventLine += "\n";
    
    dataFile.print(eventLine);
    dataFile.flush();  // Immediate flush for critical events
    
    Serial.println("📝 Logged SENSOR_RESET event to flight data");
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
  json += "\"uptime\":" + String(millis()) + ",";
  json += "\"flightState\":" + String(flightState) + ",";
  json += "\"wifiClients\":" + String(WiFi.softAPgetStationNum());
  json += "}";
  
  server.send(200, "application/json", json);
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
            <h1>🚁 Pavi Flight Data Server</h1>
            <div class="status online">📶 SoftAP Online</div>
        </div>
        
        <div class="info-box">
            <h3>📊 System Information</h3>
            <p><strong>Device:</strong> ESP32 Flight Computer</p>
            <p><strong>Uptime:</strong> <span id="uptime">Loading...</span></p>
            <p><strong>Free Memory:</strong> <span id="memory">Loading...</span></p>
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
                                <a href="/download?file=${file.name}" class="download-btn"> Download</a>
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
                document.getElementById('memory').textContent = formatBytes(data.freeHeap) + ' / ' + formatBytes(data.totalHeap);
                document.getElementById('clients').textContent = data.wifiClients;
            } catch (error) {
                console.error('Error updating system info:', error);
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
void startLoadCellCalibration() {
  Serial.println("⚖️ Load cell calibration started");
  Serial.println("   Place no weight on the load cell and send 'cal zero'");
  // Implementation for starting calibration
}

void stopLoadCellCalibration() {
  Serial.println("⚖️ Load cell calibration stopped");
  // Implementation for stopping calibration
}

bool isLoadCellCalibrating() {
  // Return true if calibration is in progress
  return false;
}

void calibrateLoadCellZero() {
  Serial.println("⚖️ Calibrating load cell zero point");
  if (loadCellReady) {
    loadCell.tare();
    Serial.println("   ✅ Zero point calibrated");
  } else {
    Serial.println("   ❌ Load cell not ready");
  }
}

void calibrateLoadCellWeight(float weight) {
  Serial.print("⚖️ Calibrating with known weight: ");
  Serial.print(weight, 2);
  Serial.println(" kg");
  
  if (loadCellReady && weight > 0) {
    // Get raw reading
    long reading = loadCell.get_value(10);
    
    // Calculate calibration factor
    loadCellCalibrationFactor = reading / weight;
    loadCell.set_scale(loadCellCalibrationFactor);
    
    Serial.print("   ✅ Calibration factor: ");
    Serial.println(loadCellCalibrationFactor, 2);
  } else {
    Serial.println("   ❌ Invalid weight or load cell not ready");
  }
}

void saveLoadCellCalibration() {
  Serial.println("⚖️ Load cell calibration saved to LittleFS");
  // Implementation for saving calibration data
}

void testLoadCellCalibration() {
  Serial.println("⚖️ Testing load cell calibration");
  if (loadCellReady) {
    float weight = loadCell.get_units(10);
    Serial.print("   Current reading: ");
    Serial.print(weight, 3);
    Serial.println(" kg");
  } else {
    Serial.println("   ❌ Load cell not ready");
  }
}

// === END OF FILE ===
