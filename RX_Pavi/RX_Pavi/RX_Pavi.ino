#include <SPI.h>
#include <LoRa.h>

// Pin mapping for ESP32-C3 with LoRa
#define NSS   D7   // IO7
#define RST   D1   // IO2
#define DIO0  D0   // IO3

// === GROUND STATION CONFIGURATION ===
// Test mode - set to true to test serial only, false for full LoRa operation
#define SERIAL_TEST_MODE false

// System state variables
String lastTXResponse = "Waiting for TX response...";
String connectionStatus = "Disconnected";
unsigned long lastCommandTime = 0;
String pendingCommand = "";         // Command to be sent immediately
String commandBuffer = "";

// === INTUITIVE MODE-BASED COMMAND SYSTEM ===
enum GroundStationMode {
  MODE_CONFIG,           // Default: Configure test parameters
  MODE_FLIGHT,          // Flight operations: logging, pyro, stop
  MODE_DATA_RECOVERY,   // Data download from TX via WiFi
  MODE_LOAD_CALIBRATION // Load cell calibration mode
};

GroundStationMode currentMode = MODE_CONFIG;

// Configuration storage
struct TestConfig {
  String filename = "defaultfilename";
  float totalWeight = -1;
  float windSpeed = -1;
  float height = -1;
  bool isComplete = true;
} testConfig;

// Mode state tracking
bool flightModeActive = false;
bool configurationSent = false;
unsigned long lastTelemetryTime = 0;
int telemetryPacketCount = 0;
int commandPacketCount = 0;
bool flightComputerConnected = false;

// Telemetry data storage
struct TelemetryData {
  float altitude = 0.0;
  float verticalVelocity = 0.0;
  float weight = 0.0;
  float accelX = 0.0, accelY = 0.0, accelZ = 0.0;
  float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
  float temperature = 0.0;
  int flightState = 0;
  int rssi = 0;
  unsigned long lastUpdate = 0;
};

TelemetryData telemetry;

// === SERIAL DATA TOGGLE ===
bool showSerialData = true; // Set to false to suppress telemetry/data output

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("=== RX: LoRa Ground Station (Serial Only) ===");
  Serial.println("ESP32-C3 initializing...");
  Serial.print("Free heap: "); Serial.println(ESP.getFreeHeap());
  Serial.println("Serial command interface ready");
  
  if (SERIAL_TEST_MODE) {
    Serial.println("RUNNING IN SERIAL TEST MODE");
    Serial.println("LoRa initialization SKIPPED for testing");
    Serial.println("If you see this, serial communication works!");
    Serial.println("Change SERIAL_TEST_MODE to false to enable LoRa");
    Serial.println("============================");
    printGroundStationHelp();
    return; // Skip LoRa initialization
  }
  
  // Print pin configuration for debugging
  Serial.print("NSS pin: "); Serial.println(NSS);
  Serial.print("RST pin: "); Serial.println(RST);
  Serial.print("DIO0 pin: "); Serial.println(DIO0);

  Serial.println("Setting LoRa pins...");
  LoRa.setPins(NSS, RST, DIO0);

  Serial.println("Initializing LoRa at 433MHz...");
  if (!LoRa.begin(433E6)) {
    Serial.println("ERROR: Starting LoRa failed!");
    Serial.println("Check connections and power supply");
    Serial.println("Continuing without LoRa...");
    return;
  }

  Serial.println("SUCCESS: LoRa transceiver ready!");
  
  // Configure LoRa settings to match TX
  LoRa.setSpreadingFactor(7);     // SF7 (faster data rate)
  LoRa.setSignalBandwidth(125E3); // 125 kHz bandwidth  
  LoRa.setCodingRate4(5);         // 4/5 coding rate
  LoRa.setPreambleLength(8);      // 8 symbol preamble
  LoRa.setSyncWord(0x12);         // Private sync word
  LoRa.setTxPower(20);            // Max power for range
  LoRa.crc();                     // Enable CRC
  
  Serial.println("LoRa configured: SF7, BW125, CR4/5, Pwr20dBm");
  
  Serial.println("🚁 *** RX READY - INTUITIVE MODE-BASED SYSTEM ***");
  Serial.println("💡 Starting in CONFIG mode - set your test parameters");
  Serial.println("📚 Type 'help' for intuitive commands");
  Serial.println("===============================================");
  
  // Initialize configuration with defaults
  testConfig.filename = "";
  testConfig.totalWeight = 0.0;
  testConfig.windSpeed = 0.0;
  testConfig.height = 0.0;
  testConfig.isComplete = false;
  
  // Show current mode and basic help
  printCurrentMode();
  Serial.println("\n💡 Quick start: 'set filename test1', 'set weight 2.5', etc.");
  Serial.println("Type 'help' for complete command list.");
}

void loop() {
  // Process user commands from serial
  processUserCommands();
  
  // RX is now a pure command sender - no continuous telemetry
  if (!SERIAL_TEST_MODE) {
    handleCommandSending();
  }
  
  // System status heartbeat - serial only
  static unsigned long lastHeartbeat = 0;
  static int heartbeatCount = 0;
  
  // General system heartbeat every 30 seconds
  if (millis() - lastHeartbeat > 30000) {
    lastHeartbeat = millis();
    heartbeatCount++;
    
    if (SERIAL_TEST_MODE) {
      Serial.print("Ground station test mode - heartbeat #");
      Serial.println(heartbeatCount);
    }
    
    // System monitoring every 5 minutes
    if (heartbeatCount % 10 == 0) {
      Serial.print("System status - Free heap: ");
      Serial.print(ESP.getFreeHeap());
      Serial.print(" bytes, Uptime: ");
      Serial.print(millis() / 1000);
      Serial.println(" seconds");
    }
  }
}

// === GROUND STATION FUNCTIONS ===

void printGroundStationHelp() {
  Serial.println("\n=== INTUITIVE LORA FLIGHT CONTROL ===");
  printCurrentMode();
  Serial.println("");
  
  if (currentMode == MODE_CONFIG) {
    Serial.println("📋 CONFIG MODE - Test Parameter Setup");
    Serial.println("  set filename <name>    - Set test filename (e.g. 'set filename test1')");
    Serial.println("  set weight <kg>        - Set total weight (e.g. 'set weight 2.5')");
    Serial.println("  set wind <m/s>         - Set wind speed (e.g. 'set wind 3.2')");
    Serial.println("  set height <m>         - Set height (e.g. 'set height 100')");
    Serial.println("  show config            - Display current configuration");
    Serial.println("  test ping              - Test single LoRa command transmission");
    Serial.println("  send config            - Manually send current config to TX");
    Serial.println("  enter flight           - Switch to FLIGHT mode (config must be complete)");
    Serial.println("");
  } 
  else if (currentMode == MODE_FLIGHT) {
    Serial.println("🚀 FLIGHT MODE - Active Operations");
    Serial.println("  start                  - Begin data logging with current config");
    Serial.println("  stop                   - Stop data logging and return to CONFIG");
    Serial.println("  offset                 - Reset sensor zero points");
    Serial.println("  pyro1, pyro2, pyro3, pyro4 - Fire pyro channels (CAUTION!)");
    Serial.println("  back                   - Return to CONFIG mode");
    Serial.println("");
  }
  else if (currentMode == MODE_DATA_RECOVERY) {
    Serial.println("💾 DATA RECOVERY MODE - File Download");
    Serial.println("  start wifi             - Command TX to start WiFi SoftAP");
    Serial.println("  stop wifi              - Command TX to stop WiFi");
    Serial.println("  list files             - Request file list from TX");
    Serial.println("  back                   - Return to CONFIG mode");
    Serial.println("");
  }
  else if (currentMode == MODE_LOAD_CALIBRATION) {
    Serial.println("⚖️ LOAD CELL CALIBRATION MODE");
    Serial.println("  calibrate start        - Begin load cell calibration");
    Serial.println("  calibrate zero         - Set zero point (no load)");
    Serial.println("  calibrate weight <kg>  - Set known weight point");
    Serial.println("  calibrate save         - Save calibration to TX");
    Serial.println("  calibrate test         - Test current calibration");
    Serial.println("  back                   - Return to CONFIG mode");
    Serial.println("");
  }
  
  Serial.println("🔧 SYSTEM COMMANDS (Available in all modes):");
  Serial.println("  mode config            - Switch to CONFIG mode");
  Serial.println("  mode flight            - Switch to FLIGHT mode");  
  Serial.println("  mode data              - Switch to DATA RECOVERY mode");
  Serial.println("  mode calibration       - Switch to LOAD CALIBRATION mode");
  Serial.println("  ping                   - Test LoRa connection");
  Serial.println("  status                 - Show system status");
  Serial.println("  help                   - Show this help");
  Serial.println("======================================");
  Serial.println("💡 TIP: Commands are intuitive - just type what you want to do!");
}

void processUserCommands() {
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
          processGroundStationCommand(commandBuffer);
        }
        commandBuffer = "";
      }
    } else {
      commandBuffer += c;
    }
  }
}

void processGroundStationCommand(String cmd) {
  String originalCmd = cmd;  // Keep original for parsing
  cmd.toLowerCase();
  cmd.trim();
  
  Serial.print("💬 Command: "); Serial.println(originalCmd);
  
  // === GLOBAL COMMANDS (work in any mode) ===
  if (cmd == "help") {
    printGroundStationHelp();
    return;
  }
  else if (cmd == "status") {
    printGroundStationStatus();
    return;
  }
  else if (cmd == "ping") {
    Serial.println("📡 Testing LoRa connection...");
    sendCommandToFlightComputer("PING");
    return;
  }
  else if (cmd == "sysinfo") {
    printSystemInfo();
    return;
  }
  
  // === MODE SWITCHING COMMANDS ===
  else if (cmd == "mode config") {
    switchMode(MODE_CONFIG);
    return;
  }
  else if (cmd == "mode flight") {
    switchMode(MODE_FLIGHT);
    return;
  }
  else if (cmd == "mode data") {
    switchMode(MODE_DATA_RECOVERY);
    return;
  }
  else if (cmd == "mode calibration") {
    switchMode(MODE_LOAD_CALIBRATION);
    return;
  }
  
  // === MODE-SPECIFIC COMMANDS ===
  if (currentMode == MODE_CONFIG) {
    processConfigCommand(cmd, originalCmd);
  }
  else if (currentMode == MODE_FLIGHT) {
    processFlightCommand(cmd, originalCmd);
  }
  else if (currentMode == MODE_DATA_RECOVERY) {
    processDataRecoveryCommand(cmd, originalCmd);
  }
  else if (currentMode == MODE_LOAD_CALIBRATION) {
    processCalibrationCommand(cmd, originalCmd);
  }
}

// === CONFIG MODE COMMANDS ===
void processConfigCommand(String cmd, String originalCmd) {
  if (cmd.startsWith("set filename ")) {
    String filename = originalCmd.substring(13);
    filename.trim();
    if (filename.length() > 0) {
      testConfig.filename = filename;
      Serial.println("✅ Filename set to: " + filename + ".txt");
    } else {
      Serial.println("❌ Please provide a filename: set filename <name>");
    }
  }
  else if (cmd.startsWith("set weight ")) {
    float weight = originalCmd.substring(11).toFloat();
    if (weight > 0) {
      testConfig.totalWeight = weight;
      Serial.println("✅ Weight set to: " + String(weight, 1) + " kg");
    } else {
      Serial.println("❌ Please provide a valid weight: set weight <kg>");
    }
  }
  else if (cmd.startsWith("set wind ")) {
    float wind = originalCmd.substring(9).toFloat();
    if (wind >= 0) {
      testConfig.windSpeed = wind;
      Serial.println("✅ Wind speed set to: " + String(wind, 1) + " m/s");
    } else {
      Serial.println("❌ Please provide a valid wind speed: set wind <m/s>");
    }
  }
  else if (cmd.startsWith("set height ")) {
    float height = originalCmd.substring(11).toFloat();
    if (height > 0) {
      testConfig.height = height;
      Serial.println("✅ Height set to: " + String(height, 1) + " m");
    } else {
      Serial.println("❌ Please provide a valid height: set height <m>");
    }
  }
  else if (cmd == "show config") {
    showConfiguration();
  }
  else if (cmd == "send config") {
    if (isConfigComplete()) {
      Serial.println("📤 Manually sending configuration to TX...");
      sendConfigurationToTX();
      configurationSent = true;
      Serial.println("✅ Configuration resent successfully!");
    } else {
      Serial.println("❌ Configuration incomplete - cannot send to TX");
      Serial.println("💡 Use 'show config' to see what's missing");
    }
  }
  else if (cmd == "test ping") {
    Serial.println("🧪 Testing single LoRa command transmission...");
    sendLoRaCommandNow("PING");
    Serial.println("✅ PING command sent - check TX for response");
  }
  else if (cmd == "enter flight") {
    switchMode(MODE_FLIGHT);
  }
  else {
    Serial.println("❓ Unknown CONFIG command. Type 'help' for available commands.");
  }
}

// === FLIGHT MODE COMMANDS ===
void processFlightCommand(String cmd, String originalCmd) {
  if (cmd == "start") {
    Serial.println("🚀 Starting data logging...");
    sendCommandToFlightComputer("FLIGHT_START");
    flightModeActive = true;
  }
  else if (cmd == "stop") {
    Serial.println("🛑 Stopping data logging...");
    sendCommandToFlightComputer("FLIGHT_STOP");
    flightModeActive = false;
    switchMode(MODE_CONFIG);  // Return to config after stopping
  }
  else if (cmd == "offset") {
    Serial.println("🎯 Resetting sensor zero points...");
    sendCommandToFlightComputer("SENSOR_RESET");
  }
  else if (cmd == "pyro1" || cmd == "pyro2" || cmd == "pyro3" || cmd == "pyro4") {
    cmd.toUpperCase();
    sendPyroCommand(cmd);
  }
  else if (cmd == "back") {
    if (flightModeActive) {
      Serial.println("⚠️ Flight is active! Use 'stop' first, then 'back'");
    } else {
      switchMode(MODE_CONFIG);
    }
  }
  else {
    Serial.println("❓ Unknown FLIGHT command. Type 'help' for available commands.");
  }
}

// === DATA RECOVERY MODE COMMANDS ===
void processDataRecoveryCommand(String cmd, String originalCmd) {
  if (cmd == "start wifi") {
    Serial.println("📶 Starting WiFi on TX for file download...");
    sendCommandToFlightComputer("WIFI_START");
  }
  else if (cmd == "stop wifi") {
    Serial.println("📶 Stopping WiFi on TX...");
    sendCommandToFlightComputer("WIFI_STOP");
  }
  else if (cmd == "list files") {
    Serial.println("📁 Requesting file list from TX...");
    sendCommandToFlightComputer("FILE_LIST");
  }
  else if (cmd == "back") {
    switchMode(MODE_CONFIG);
  }
  else {
    Serial.println("❓ Unknown DATA RECOVERY command. Type 'help' for available commands.");
  }
}

// === LOAD CALIBRATION MODE COMMANDS ===
void processCalibrationCommand(String cmd, String originalCmd) {
  if (cmd == "calibrate start") {
    Serial.println("⚖️ Starting load cell calibration...");
    sendCommandToFlightComputer("CALIB_START");
  }
  else if (cmd == "calibrate zero") {
    Serial.println("⚖️ Setting zero point (ensure no load on cell)...");
    sendCommandToFlightComputer("CALIB_ZERO");
  }
  else if (cmd.startsWith("calibrate weight ")) {
    float weight = originalCmd.substring(17).toFloat();
    if (weight > 0) {
      Serial.println("⚖️ Setting calibration weight: " + String(weight, 2) + " kg");
      sendCommandToFlightComputer("CALIB_WEIGHT:" + String(weight, 2));
    } else {
      Serial.println("❌ Please provide a valid weight: calibrate weight <kg>");
    }
  }
  else if (cmd == "calibrate save") {
    Serial.println("⚖️ Saving calibration constants...");
    sendCommandToFlightComputer("CALIB_SAVE");
  }
  else if (cmd == "calibrate test") {
    Serial.println("⚖️ Testing current calibration...");
    sendCommandToFlightComputer("CALIB_TEST");
  }
  else if (cmd == "back") {
    switchMode(MODE_CONFIG);
  }
  else {
    Serial.println("❓ Unknown CALIBRATION command. Type 'help' for available commands.");
  }
}

void sendCommandToFlightComputer(String command) {
  lastTXResponse = "Command sent: " + command;
  lastCommandTime = millis();
  pendingCommand = command;
  commandPacketCount++;
  
  Serial.println(">>> Command queued for transmission: " + command);
}

void sendPyroCommand(String pyroCmd) {
  Serial.println("*** PYRO COMMAND WARNING ***");
  Serial.print("About to send: "); Serial.println(pyroCmd);
  Serial.println("This will fire a pyro channel on the flight computer!");
  Serial.print("Type 'FIRE' to confirm or anything else to cancel: ");
  
  String confirmation = "";
  unsigned long timeout = millis() + 10000; // 10 second timeout
  
  while (millis() < timeout && confirmation != "FIRE") {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        confirmation.toUpperCase();
        break;
      } else {
        confirmation += c;
        Serial.print(c); // Echo
      }
    }
  }
  Serial.println();
  
  if (confirmation == "FIRE") {
    Serial.println("FIRING PYRO CHANNEL!");
    sendCommandToFlightComputer(pyroCmd);
  } else {
    Serial.println("Pyro command cancelled");
  }
}

void handleCommandSending() {
  // Send commands immediately when queued
  if (pendingCommand.length() > 0) {
    Serial.println("===============================");
    Serial.println("*** SENDING LORA COMMAND ***");
    Serial.print("*** Command: ");
    Serial.print(pendingCommand);
    Serial.println(" ***");
    
    // Send the command
    LoRa.beginPacket();
    LoRa.print(pendingCommand);
    LoRa.endPacket();
    
    Serial.println("*** LoRa packet sent! ***");
    Serial.println("*** Check TX serial output ***");
    Serial.println("===============================");
    
    // Clear pending command
    pendingCommand = "";
    Serial.println(">>> Ready for next command");
  }
}

// === SYSTEM INFORMATION AND DIAGNOSTICS ===

void printSystemInfo() {
  Serial.println("=== ESP32-C3 SYSTEM INFO ===");
  Serial.print("Chip model: "); Serial.println(ESP.getChipModel());
  Serial.print("Chip revision: "); Serial.println(ESP.getChipRevision());
  Serial.print("CPU frequency: "); Serial.print(ESP.getCpuFreqMHz()); Serial.println(" MHz");
  Serial.print("Flash size: "); Serial.print(ESP.getFlashChipSize()); Serial.println(" bytes");
  Serial.print("Free heap: "); Serial.print(ESP.getFreeHeap()); Serial.println(" bytes");
  Serial.print("Largest free block: "); Serial.print(ESP.getMaxAllocHeap()); Serial.println(" bytes");
  Serial.print("Sketch size: "); Serial.print(ESP.getSketchSize()); Serial.println(" bytes");
  Serial.print("Free sketch space: "); Serial.print(ESP.getFreeSketchSpace()); Serial.println(" bytes");
  Serial.print("Uptime: "); Serial.print(millis() / 1000); Serial.println(" seconds");
  
  // Check for any potential issues
  if (ESP.getFreeHeap() < 50000) {
    Serial.println("⚠ WARNING: Low heap memory detected!");
  }
  if (ESP.getMaxAllocHeap() < 20000) {
    Serial.println("⚠ WARNING: Severe heap fragmentation detected!");
  }
  
  Serial.println("===============================");
}

void printGroundStationStatus() {
  Serial.println("\n=== 🚁 GROUND STATION STATUS ===");
  printCurrentMode();
  Serial.print("LoRa Status: "); 
  Serial.println(!SERIAL_TEST_MODE ? "✅ Active" : "🧪 Test Mode");
  
  if (currentMode == MODE_FLIGHT && flightModeActive) {
    Serial.println("Flight Status: 🚀 ACTIVE LOGGING");
  } else if (currentMode == MODE_FLIGHT) {
    Serial.println("Flight Status: 🚀 Ready to start");
  }
  
  Serial.print("Commands sent: "); Serial.println(commandPacketCount);
  Serial.print("Last command: "); Serial.println(lastTXResponse);
  
  // Show config status in summary
  Serial.print("Configuration: ");
  if (isConfigComplete()) {
    Serial.println("✅ Complete (" + testConfig.filename + ", " + 
                   String(testConfig.totalWeight, 1) + "kg, " +
                   String(testConfig.windSpeed, 1) + "m/s, " +
                   String(testConfig.height, 1) + "m)");
  } else {
    Serial.println("❌ Incomplete - use 'show config' for details");
  }
  
  Serial.print("System Health: ");
  if (ESP.getFreeHeap() > 50000) {
    Serial.println("✅ Good (" + String(ESP.getFreeHeap()) + " bytes free)");
  } else {
    Serial.println("⚠️ Low Memory (" + String(ESP.getFreeHeap()) + " bytes free)");
  }
  
  Serial.println("================================\n");
}

// === MODE MANAGEMENT FUNCTIONS ===

void printCurrentMode() {
  Serial.print("Current Mode: ");
  switch (currentMode) {
    case MODE_CONFIG:
      Serial.println("📋 CONFIG");
      break;
    case MODE_FLIGHT:
      Serial.println("🚀 FLIGHT");
      break;
    case MODE_DATA_RECOVERY:
      Serial.println("💾 DATA RECOVERY");
      break;
    case MODE_LOAD_CALIBRATION:
      Serial.println("⚖️ LOAD CALIBRATION");
      break;
  }
}

void switchMode(GroundStationMode newMode) {
  if (newMode == MODE_FLIGHT && !isConfigComplete()) {
    Serial.println("❌ Cannot enter FLIGHT mode - configuration incomplete!");
    Serial.println("Use 'show config' to see what's missing.");
    return;
  }
  
  currentMode = newMode;
  Serial.print("✅ Switched to ");
  printCurrentMode();
  
  // Mode-specific initialization
  if (currentMode == MODE_FLIGHT && !configurationSent) {
    sendConfigurationToTX();
  }
}

bool isConfigComplete() {
  return !testConfig.filename.isEmpty() && 
         testConfig.totalWeight > 0 && 
         testConfig.windSpeed >= 0 && 
         testConfig.height > 0;
}

void showConfiguration() {
  Serial.println("\n=== CURRENT TEST CONFIGURATION ===");
  Serial.print("Filename: ");
  if (testConfig.filename.isEmpty()) {
    Serial.println("❌ Not set");
  } else {
    Serial.println("✅ " + testConfig.filename + ".txt");
  }
  
  Serial.print("Total Weight: ");
  if (testConfig.totalWeight <= 0) {
    Serial.println("❌ Not set");
  } else {
    Serial.println("✅ " + String(testConfig.totalWeight, 1) + " kg");
  }
  
  Serial.print("Wind Speed: ");
  if (testConfig.windSpeed < 0) {
    Serial.println("❌ Not set");
  } else {
    Serial.println("✅ " + String(testConfig.windSpeed, 1) + " m/s");
  }
  
  Serial.print("Height: ");
  if (testConfig.height <= 0) {
    Serial.println("❌ Not set");
  } else {
    Serial.println("✅ " + String(testConfig.height, 1) + " m");
  }
  
  Serial.print("Status: ");
  if (isConfigComplete()) {
    Serial.println("✅ Complete - Ready for FLIGHT mode");
  } else {
    Serial.println("❌ Incomplete - Set missing parameters");
  }
  Serial.println("=====================================\n");
}

// Helper function to send a single LoRa command immediately
void sendLoRaCommandNow(String command) {
  Serial.println("📡 Transmitting: " + command);
  
  if (SERIAL_TEST_MODE) {
    Serial.println("   (SERIAL TEST MODE - No actual LoRa transmission)");
    return;
  }
  
  // Send immediately via LoRa
  LoRa.beginPacket();
  LoRa.print(command);
  LoRa.endPacket();
  
  Serial.println("   ✅ LoRa packet transmitted");
  
  // Update counters
  commandPacketCount++;
  lastTXResponse = "Command sent: " + command;
  lastCommandTime = millis();
}

void sendConfigurationToTX() {
  if (!isConfigComplete()) {
    Serial.println("❌ Configuration incomplete - cannot send to TX");
    return;
  }
  
  Serial.println("📤 Sending configuration to TX...");
  Serial.println("   📋 Configuration Details:");
  Serial.println("   • Filename: " + testConfig.filename + ".txt");
  Serial.println("   • Weight: " + String(testConfig.totalWeight, 2) + " kg");
  Serial.println("   • Wind: " + String(testConfig.windSpeed, 2) + " m/s");
  Serial.println("   • Height: " + String(testConfig.height, 2) + " m");
  Serial.println("");
  
  // Send configuration commands immediately and synchronously
  Serial.println("� Step 1: Initialize configuration mode");
  sendLoRaCommandNow("CONFIG_START");
  delay(500);  // Allow TX to enter config mode
  
  Serial.println("� Step 2: Send filename");  
  sendLoRaCommandNow("FILENAME:" + testConfig.filename);
  delay(300);  // Allow TX to process filename
  
  Serial.println("� Step 3: Send weight");
  sendLoRaCommandNow("WEIGHT:" + String(testConfig.totalWeight, 2));
  delay(300);  // Allow TX to process weight
  
  Serial.println("� Step 4: Send wind speed");
  sendLoRaCommandNow("WIND:" + String(testConfig.windSpeed, 2));
  delay(300);  // Allow TX to process wind speed
  
  Serial.println("� Step 5: Send height");
  sendLoRaCommandNow("HEIGHT:" + String(testConfig.height, 2));
  delay(300);  // Allow TX to process height
  
  Serial.println("� Step 6: Finalize configuration");
  sendLoRaCommandNow("CONFIG_READY");
  delay(200);  // Allow final processing
  
  configurationSent = true;
  Serial.println("");
  Serial.println("✅ Configuration transmission sequence complete!");
  Serial.println("🔍 Check TX serial monitor - should show all 6 commands received");
  Serial.println("📊 Commands sent: CONFIG_START → FILENAME → WEIGHT → WIND → HEIGHT → CONFIG_READY");
}
