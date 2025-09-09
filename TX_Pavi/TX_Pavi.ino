#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <MS5611.h>  
#include <MPU6050.h>
#include <HX711.h>
#include "FS.h"
#include "LittleFS.h"

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
#define DATA_RATE_MODE 1   // 1=10Hz, 2=20Hz, 3=30Hz, 4=40Hz

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
bool showSerialData = true; // Set to false to suppress serial data output

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
    readSensorsOnly();
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
      processLoRaCommand(received);
      
      // Stay in receive mode
      LoRa.receive();
    }
  }

  void processCommand(String cmd) {
    cmd.toUpperCase();
    cmd.trim();
    
    Serial.print("Command received: "); Serial.println(cmd);
    
    // Handle flight configuration commands when in FLIGHT_CONFIGURING state
    if (flightState == FLIGHT_CONFIGURING) {
      processFlightConfigCommand(cmd);
      return;
    }
    
    if (cmd == "PING") {
      Serial.println("PONG - LoRa command received successfully!");
    }
    else if (cmd == "STATUS") {
      printSystemStatus();
    }
    else if (cmd == "CONFIG") {
      if (flightState == FLIGHT_RECORDING) {
        Serial.println("ERROR: Cannot configure during recording. Stop recording first.");
        return;
      }
      configureFlightParameters();
    }
    else if (cmd == "START") {
      startDataRecording();
    }
    else if (cmd == "STOP") {
      stopDataRecording();
    }
    else if (cmd.startsWith("PYRO")) {
      int channel = cmd.charAt(4) - '1';  // Convert PYRO1->0, PYRO2->1, etc.
      if (channel >= 0 && channel <= 3) {
        firePyroChannel(channel);
      } else {
        Serial.println("ERROR: Invalid pyro channel. Use PYRO1, PYRO2, PYRO3, or PYRO4");
      }
    }
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
      else {
        Serial.println("ERROR: Unknown command. Type STATUS for help.");
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
    // Reset altitude reference
    if (baroReady) {
      Serial.println("Resetting altitude reference...");
      // Take a few readings to establish new reference
      float newRefPressure = 0;
      int validReadings = 0;
      
      for (int i = 0; i < 10; i++) {
        baro.read();
        float currentPressure = baro.getPressure();
        if (currentPressure > 500 && currentPressure < 1200) {
          newRefPressure += currentPressure;
          validReadings++;
        }
        delay(50);
      }
      
      if (validReadings > 0) {
        newRefPressure /= validReadings;
        float pressureRatio = newRefPressure / SEA_LEVEL_PRESSURE;
        referenceAltitude = 44330.0 * (1.0 - pow(pressureRatio, 0.1903));
        Serial.print("New altitude reference: "); Serial.print(referenceAltitude); Serial.println(" m");
      }
    }
    
    // Reset velocity calculation
    lastAltitude = altitude;
    lastVelocityTime = millis();
    verticalVelocity = 0.0;
    
    Serial.println("Sensor origins reset complete");
  }

  void writeDataHeader() {
    String header = "# PARACHUTE TEST FLIGHT DATA\n";
    header += "# Filename: " + flightConfig.filename + "\n";
    header += "# Total Weight: " + String(flightConfig.totalWeight) + " kg\n";
    header += "# Wind Speed: " + String(flightConfig.windSpeed) + " m/s\n";
    header += "# Initial Height: " + String(flightConfig.initialHeight) + " m\n";
    header += "# Start Time: " + String(flightConfig.startTime) + " ms\n";
    header += "# Data Rate Mode: " + String(DATA_RATE_MODE) + "\n";
    header += "# Columns: Time(ms),Height(m),VerticalVel(m/s),LoadCell(kg),AccelX(m/s²),AccelY(m/s²),AccelZ(m/s²),GyroX(rad/s),GyroY(rad/s),GyroZ(rad/s)\n";
    
  // Write header to serial and file
  Serial.print(header);
  
  // Write to LittleFS file
  if (filesystemReady && dataFile) {
    dataFile.print(header);
    dataFile.flush();  // Ensure header is written immediately
  }
  }

  void logFlightData() {
    if (flightState != FLIGHT_RECORDING) {
      return;
    }
    
    unsigned long currentTime = millis();
    unsigned long relativeTime = currentTime - recordingStartTime;
    
    // Create data line
    String dataLine = String(relativeTime) + ",";
    dataLine += String(altitude, 3) + ",";
    dataLine += String(verticalVelocity, 3) + ",";
    dataLine += String(loadCellWeight, 3) + ",";
    dataLine += String(accelX, 4) + ",";
    dataLine += String(accelY, 4) + ",";
    dataLine += String(accelZ, 4) + ",";
    dataLine += String(gyroX, 5) + ",";
    dataLine += String(gyroY, 5) + ",";
    dataLine += String(gyroZ, 5) + "\n";
    
  // Write to serial and file
  if (sampleNumber % 50 == 0) {  // Print every 50th sample to serial for monitoring
    Serial.print("DATA: "); Serial.print(dataLine);
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
  if (flightState == FLIGHT_RECORDING) {
    unsigned long relativeTime = millis() - recordingStartTime;
    String pyroEvent = "# PYRO_FIRE: Channel=" + String(channel + 1) + 
                      " Time=" + String(relativeTime) + "ms\n";
    Serial.print(pyroEvent);
    
    // Log to file
    if (filesystemReady && dataFile) {
      dataFile.print(pyroEvent);
      dataFile.flush();  // Ensure event is immediately written
    }
  }
}

void readSensorsOnly() {
    unsigned long loopStart = micros();
    sensorReadCount++;
    
    // Read all sensors for local logging only
    unsigned long sensorStart = micros();
    readSensorsWithSmartLoadCell();
    unsigned long sensorTime = micros() - sensorStart;
    
    // Calculate vertical velocity
    calculateVerticalVelocity();
    
    // Create data packet for serial output (no LoRa transmission)
    String packet = createDataPacket();
    
    // Print to serial if enabled (for local monitoring)
    if (showSerialData) {
      Serial.print("[DATA] ");
      Serial.println(packet);
    }
  }

  String processCommandAndGetResponse(String command) {
    command.trim();
    command.toUpperCase();
    
    if (command == "PING") {
      return "PONG";
    }
    else if (command == "STATUS") {
      return "STATUS:OK|ALT:" + String(altitude, 1) + "|STATE:" + String(flightState);
    }
    else if (command == "PYRO1") {
      firePyroChannel(0);
      return "PYRO1:FIRED";
    }
    else if (command == "PYRO2") {
      firePyroChannel(1);
      return "PYRO2:FIRED";
    }
    else if (command == "PYRO3") {
      firePyroChannel(2);
      return "PYRO3:FIRED";
    }
    else if (command == "PYRO4") {
      firePyroChannel(3);
      return "PYRO4:FIRED";
    }
    else if (command == "START") {
      if (flightState == FLIGHT_CONFIGURED) {
        startDataRecording();
        return "RECORDING:STARTED";
      } else {
        return "ERROR:NOT_CONFIGURED";
      }
    }
    else if (command == "STOP") {
      if (flightState == FLIGHT_RECORDING) {
        stopDataRecording();
        return "RECORDING:STOPPED";
      } else {
        return "ERROR:NOT_RECORDING";
      }
    }
    else {
      return "ERROR:UNKNOWN_COMMAND";
    }
  }

  void processLoRaCommand(String cmd) {
    cmd.toUpperCase();
    cmd.trim();
    
    Serial.print(">>> Processing LoRa command: ");
    Serial.println(cmd);
    
    // Handle flight configuration commands when in FLIGHT_CONFIGURING state
    if (flightState == FLIGHT_CONFIGURING) {
      Serial.println(">>> Flight configuration mode - processing config command");
      processFlightConfigCommand(cmd);
      return;
    }
    
    if (cmd == "PING") {
      Serial.println(">>> PING RECEIVED! TX is working! <<<");
    }
    else if (cmd == "START") {
      if (flightState == FLIGHT_RECORDING) {
        Serial.println(">>> ERROR: Already recording! Send STOP first.");
      } else {
        Serial.println(">>> START command received - configuring flight test");
        promptForFlightConfiguration();
      }
    }
    else if (cmd == "RESET") {
      Serial.println(">>> RESET command received - resetting sensor origins");
      resetSensorOrigins();
      Serial.println(">>> Sensor origins reset complete");
    }
    else if (cmd == "STOP") {
      if (flightState == FLIGHT_RECORDING) {
        Serial.println(">>> STOP command received - stopping recording");
        stopDataRecording();
      } else {
        Serial.println(">>> ERROR: Not currently recording");
      }
    }
    else if (cmd == "PYRO1") {
      Serial.println(">>> PYRO1 command received");
      firePyroChannel(0);
    }
    else if (cmd == "PYRO2") {
      Serial.println(">>> PYRO2 command received");
      firePyroChannel(1);
    }
    else if (cmd == "PYRO3") {
      Serial.println(">>> PYRO3 command received");
      firePyroChannel(2);
    }
    else if (cmd == "PYRO4") {
      Serial.println(">>> PYRO4 command received");
      firePyroChannel(3);
    }
    else if (cmd == "STATUS") {
      Serial.println(">>> STATUS command received");
      printSystemStatus();
    }
    else {
      Serial.print(">>> ERROR: Unknown LoRa command: ");
      Serial.println(cmd);
    }
  }

  void promptForFlightConfiguration() {
    Serial.println("=== FLIGHT TEST CONFIGURATION ===");
    Serial.println("TX now ready for flight configuration commands via LoRa!");
    Serial.println("Send these commands from RX in any order:");
    Serial.println("1. FILENAME:<name> - e.g., FILENAME:test1");
    Serial.println("2. WEIGHT:<kg> - e.g., WEIGHT:2.5");
    Serial.println("3. WIND:<m/s> - e.g., WIND:3.2");
    Serial.println("4. HEIGHT:<m> - e.g., HEIGHT:100");
    Serial.println("5. CONFIRM - Start recording with entered values");
    Serial.println("6. CANCEL - Cancel configuration");
    Serial.println("=====================================");
    Serial.println(">>> WAITING FOR CONFIGURATION COMMANDS <<<");
    
    flightState = FLIGHT_CONFIGURING;
    
    // Initialize with defaults
    flightConfig.filename = "flight_" + String(millis()) + ".csv";
    flightConfig.totalWeight = 1.0;
    flightConfig.windSpeed = 0.0;
    flightConfig.initialHeight = 0.0;
    
    Serial.println("Current defaults:");
    Serial.print("- Filename: "); Serial.println(flightConfig.filename);
    Serial.print("- Weight: "); Serial.print(flightConfig.totalWeight); Serial.println(" kg");
    Serial.print("- Wind: "); Serial.print(flightConfig.windSpeed); Serial.println(" m/s");
    Serial.print("- Height: "); Serial.print(flightConfig.initialHeight); Serial.println(" m");
    Serial.println("Send commands from RX to override defaults!");
  }

  void processFlightConfigCommand(String cmd) {
    Serial.print(">>> Flight Config Command: "); Serial.println(cmd);
    
    if (cmd.startsWith("FILENAME:")) {
      String filename = cmd.substring(9);
      filename.trim();
      if (filename.length() > 0) {
        flightConfig.filename = filename + ".csv";
        Serial.print("✓ Filename set to: "); Serial.println(flightConfig.filename);
      } else {
        Serial.println("ERROR: Invalid filename");
      }
    }
    else if (cmd.startsWith("WEIGHT:")) {
      String weightStr = cmd.substring(7);
      weightStr.trim();
      float weight = weightStr.toFloat();
      if (weight > 0) {
        flightConfig.totalWeight = weight;
        Serial.print("✓ Weight set to: "); Serial.print(flightConfig.totalWeight); Serial.println(" kg");
      } else {
        Serial.println("ERROR: Invalid weight value");
      }
    }
    else if (cmd.startsWith("WIND:")) {
      String windStr = cmd.substring(5);
      windStr.trim();
      float wind = windStr.toFloat();
      if (wind >= 0) {
        flightConfig.windSpeed = wind;
        Serial.print("✓ Wind speed set to: "); Serial.print(flightConfig.windSpeed); Serial.println(" m/s");
      } else {
        Serial.println("ERROR: Invalid wind speed value");
      }
    }
    else if (cmd.startsWith("HEIGHT:")) {
      String heightStr = cmd.substring(7);
      heightStr.trim();
      float height = heightStr.toFloat();
      flightConfig.initialHeight = height;
      Serial.print("✓ Height set to: "); Serial.print(flightConfig.initialHeight); Serial.println(" m");
    }
    else if (cmd == "CONFIRM") {
      Serial.println(">>> CONFIRM received! Starting flight recording...");
      Serial.println("=== FINAL CONFIGURATION ===");
      Serial.print("Filename: "); Serial.println(flightConfig.filename);
      Serial.print("Weight: "); Serial.print(flightConfig.totalWeight); Serial.println(" kg");
      Serial.print("Wind Speed: "); Serial.print(flightConfig.windSpeed); Serial.println(" m/s");  
      Serial.print("Initial Height: "); Serial.print(flightConfig.initialHeight); Serial.println(" m");
      
      flightConfig.startTime = millis();
      flightState = FLIGHT_CONFIGURED;
      Serial.println("✓ Configuration complete - starting data recording!");
      
      // Auto-start recording after confirmation
      startDataRecording();
    }
    else if (cmd == "CANCEL") {
      Serial.println(">>> Configuration cancelled. Returning to idle state.");
      flightState = FLIGHT_IDLE;
    }
    else {
      Serial.println("ERROR: Unknown config command. Use FILENAME:, WEIGHT:, WIND:, HEIGHT:, CONFIRM, or CANCEL");
    }
    
    // Show current configuration status
    if (flightState == FLIGHT_CONFIGURING) {
      Serial.println("\n--- Current Configuration ---");
      Serial.print("Filename: "); Serial.println(flightConfig.filename);
      Serial.print("Weight: "); Serial.print(flightConfig.totalWeight); Serial.println(" kg");
      Serial.print("Wind: "); Serial.print(flightConfig.windSpeed); Serial.println(" m/s");
      Serial.print("Height: "); Serial.print(flightConfig.initialHeight); Serial.println(" m");
      Serial.println("Send CONFIRM when ready, or more parameter commands to change values");
      Serial.println(">>> WAITING FOR NEXT COMMAND <<<");
    }
  }
