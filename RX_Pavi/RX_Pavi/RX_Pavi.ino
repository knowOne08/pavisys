#include <SPI.h>
#include <LoRa.h>

// Pin mapping for ESP32-C3 with LoRa
#define NSS   D7   // IO7
#define RST   D1   // IO2
#define DIO0  D0   // IO3

// === ESP32-C3 STABILITY CONFIGURATION ===
// ESP32-C3 has less RAM and different memory management than ESP32-WROOM-32
#ifdef CONFIG_IDF_TARGET_ESP32C3
  #define ESP32_C3_FIXES_ENABLED true
  #define ESP32_C3_EMERGENCY_MODE true  // Aggressive memory conservation
  #pragma message "ESP32-C3 EMERGENCY STABILITY MODE enabled"
#else
  #define ESP32_C3_FIXES_ENABLED false
  #define ESP32_C3_EMERGENCY_MODE false
#endif

// === GROUND STATION CONFIGURATION ===
// Test mode - set to true to test serial only, false for full LoRa operation
#define SERIAL_TEST_MODE false

// System state variables
String lastTXResponse = "Waiting for TX response...";
String connectionStatus = "Disconnected";
unsigned long lastCommandTime = 0;
String pendingCommand = "";         // Command to be sent immediately
String commandBuffer = "";

// === MENU SYSTEM VARIABLES ===
bool menuMode = false;              // Whether we're in menu navigation mode
String menuInput = "";              // Current menu input buffer
enum MenuLevel {
  MENU_MAIN,
  MENU_CONFIG,
  MENU_FLIGHT,
  MENU_DATA,
  MENU_CALIBRATION,
  MENU_WIFI,
  MENU_SET_PARAMS
};
MenuLevel currentMenuLevel = MENU_MAIN;

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

// === FUNCTION DECLARATIONS ===
void sendLoRaCommandNow(String command);
void sendConfigurationToTX();
void enterMenuMode();
void exitMenuMode();
void processMenuInput(String input);
bool attemptFlightModeSwitch();
void checkMemoryStatus(); // ESP32-C3 memory monitoring

void setup() {
  Serial.begin(115200);
  // delay(2000);
  
  Serial.println("=== RX: LoRa Ground Station (Serial Only) ===");
  Serial.println("ESP32-C3 initializing...");
  
  // ESP32-C3 Memory Status
  uint32_t freeHeap = ESP.getFreeHeap();
  Serial.print("Free heap: "); Serial.println(freeHeap);
  
  #if ESP32_C3_FIXES_ENABLED
  if (freeHeap < 200000) {  // Less than 200KB
    Serial.println("⚠️  WARNING: Low memory detected on ESP32-C3");
    Serial.println("💡 ESP32-C3 stability fixes are ENABLED");
  }
  
  // ESP32-C3 Power Management - reduce CPU frequency for maximum stability
  setCpuFrequencyMhz(40);  // Drastically reduce CPU frequency for maximum stability
  Serial.println("CPU frequency set to 40MHz for maximum stability");
  #endif
  
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
  
  // Configure LoRa settings to match TX - ULTRA CONSERVATIVE for ESP32-C3
  LoRa.setSpreadingFactor(12);    // SF12 for maximum reliability
  LoRa.setSignalBandwidth(125E3); // 125 kHz bandwidth  
  LoRa.setCodingRate4(8);         // CR 4/8 for maximum error correction
  LoRa.setPreambleLength(8);      // 8 symbol preamble
  LoRa.setSyncWord(0x12);         // Custom sync word (matches TX)
  // LoRa.setTxPower(5);             // CONSERVATIVE: 5dBm for ESP32-C3 stability
  LoRa.enableCrc();               // Enable CRC
  
  Serial.println("LoRa configured: SF12, BW125, CR4/8, TxPower=5dBm (ESP32-C3 Conservative)");
  
  Serial.println("🚁 *** RX READY - GROUND STATION WITH MENU SYSTEM ***");
  Serial.println("💡 Starting in CONFIG mode - set your test parameters");
  Serial.println("📚 Type 'help' for text commands OR 'menu' for interactive menus");
  Serial.println("===============================================");
  
  // Initialize configuration with defaults
  testConfig.filename = "";
  testConfig.totalWeight = 0.0;
  testConfig.windSpeed = 0.0;
  testConfig.height = 0.0;
  testConfig.isComplete = false;
  
  // Show current mode and basic help
  printCurrentMode();
  Serial.println("\n💡 Quick options:");
  Serial.println("   • Type 'menu' for easy interactive menus");
  Serial.println("   • Type 'help' for text command list");
  Serial.println("   • Type commands directly (e.g., 'set filename test1')");
}

void loop() {
  // ESP32-C3 Memory monitoring
  checkMemoryStatus();
  
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
  Serial.println("\n=== GROUND STATION COMMAND HELP ===");
  Serial.println("💡 TIP: Type 'menu' for easy interactive menus!");
  Serial.println("");
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
  Serial.println("  menu                   - 🎛️ Enter interactive menu system");
  Serial.println("  mode config            - Switch to CONFIG mode");
  Serial.println("  mode flight            - Switch to FLIGHT mode");  
  Serial.println("  mode data              - Switch to DATA RECOVERY mode");
  Serial.println("  mode calibration       - Switch to LOAD CALIBRATION mode");
  Serial.println("  ping                   - Test LoRa connection");
  Serial.println("  status                 - Show system status");
  Serial.println("  help                   - Show this help");
  Serial.println("======================================");
  Serial.println("💡 EASY MODE: Type 'menu' for point-and-click navigation!");
  Serial.println("💡 TEXT MODE: Commands are intuitive - just type what you want to do!");
}

void processUserCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (menuMode) {
        if (menuInput.length() > 0) {
          processMenuInput(menuInput);
          menuInput = "";
        }
      } else {
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
      }
    } else {
      if (menuMode) {
        menuInput += c;
        Serial.print(c); // Echo menu input
      } else {
        commandBuffer += c;
      }
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
  else if (cmd == "menu") {
    enterMenuMode();
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

bool attemptFlightModeSwitch() {
  // Check if configuration is complete
  if (!isConfigComplete()) {
    Serial.println("\n❌ === FLIGHT MODE BLOCKED ===");
    Serial.println("Configuration is incomplete! Missing parameters:");
    
    if (testConfig.filename.isEmpty()) {
      Serial.println("  • Filename: Not set");
    }
    if (testConfig.totalWeight <= 0) {
      Serial.println("  • Weight: Not set");
    }
    if (testConfig.windSpeed < 0) {
      Serial.println("  • Wind Speed: Not set");
    }
    if (testConfig.height <= 0) {
      Serial.println("  • Height: Not set");
    }
    
    Serial.println("\n💡 Complete the configuration first:");
    Serial.println("   1. Use 'Set Test Parameters' option");
    Serial.println("   2. Or set missing parameters individually");
    Serial.println("=======================================");
    return false;
  }
  
  // Show configuration and ask for confirmation
  Serial.println("\n🚀 === ENTERING FLIGHT MODE ===");
  Serial.println("Current configuration will be used:");
  Serial.println("╔════════════════════════════════╗");
  Serial.println("║        FLIGHT CONFIG           ║");
  Serial.println("╠════════════════════════════════╣");
  Serial.println("║ Filename: " + String(testConfig.filename) + ".txt");
  Serial.println("║ Weight:   " + String(testConfig.totalWeight, 1) + " kg");
  Serial.println("║ Wind:     " + String(testConfig.windSpeed, 1) + " m/s");
  Serial.println("║ Height:   " + String(testConfig.height, 1) + " m");
  Serial.println("╚════════════════════════════════╝");
  Serial.println("");
  Serial.println("⚠️  In FLIGHT mode you can:");
  Serial.println("   • Start/Stop data logging");
  Serial.println("   • Fire pyro channels");
  Serial.println("   • Reset sensor origins");
  Serial.println("");
  Serial.print("✅ Proceed to FLIGHT mode? (y/n): ");
  
  String confirmation = getMenuTextInput();
  confirmation.toLowerCase();
  
  if (confirmation == "y" || confirmation == "yes") {
    Serial.println("\n🚀 Switching to FLIGHT mode...");
    switchMode(MODE_FLIGHT);
    
    // Send configuration to TX if not already sent
    if (!configurationSent) {
      Serial.println("📤 Auto-sending configuration to TX...");
      sendConfigurationToTX();
      configurationSent = true;
    }
    
    Serial.println("✅ FLIGHT mode activated - Ready for operations!");
    return true;
  } else {
    Serial.println("\n❌ Flight mode entry cancelled");
    Serial.println("💡 Staying in current mode - use menus to modify config if needed");
    return false;
  }
}

void switchMode(GroundStationMode newMode) {
  // Direct mode switching (used internally, no validation)
  GroundStationMode previousMode = currentMode;
  currentMode = newMode;
  
  if (previousMode != newMode) {
    Serial.print("✅ Mode switched: ");
    switch(previousMode) {
      case MODE_CONFIG: Serial.print("CONFIG"); break;
      case MODE_FLIGHT: Serial.print("FLIGHT"); break;
      case MODE_DATA_RECOVERY: Serial.print("DATA RECOVERY"); break;
      case MODE_LOAD_CALIBRATION: Serial.print("LOAD CALIBRATION"); break;
    }
    Serial.print(" → ");
    printCurrentMode();
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
  
  // ESP32-C3 ULTRA-SAFE MODE: Minimal transmission attempt
  Serial.println("🔧 ESP32-C3 ULTRA-SAFE MODE: Minimal transmission attempt");

  // ESP32-C3 Memory check before operation
  if (ESP32_C3_FIXES_ENABLED) {
    Serial.print("Free heap before: ");
    Serial.println(ESP.getFreeHeap());
  }

  // ESP32-C3 CRITICAL: Use fixed char array to avoid String operations during transmission
  char commandMsg[100];
  command.toCharArray(commandMsg, sizeof(commandMsg));

  Serial.print("📡 Sending: ");
  Serial.println(commandMsg);

  // Ultra-conservative transmission with maximum safety
  yield();
  delay(100);
  
  Serial.println("Step 1: beginPacket()");
  LoRa.beginPacket();
  yield();
  delay(100);
  
  Serial.println("Step 2: writing data");
  // Write data byte by byte to be extra safe
  int len = strlen(commandMsg);
  for (int i = 0; i < len; i++) {
    LoRa.write(commandMsg[i]);
    if (i % 5 == 0) {
      yield(); // Feed watchdog every 5 characters
      delayMicroseconds(100);
    }
  }
  
  yield();
  delay(200); // Long delay before the critical endPacket
  
  Serial.println("Step 3: endPacket() - CRITICAL");
  yield();
  
  LoRa.endPacket();
  
  yield();
  Serial.println("SUCCESS: Packet sent!");
  
  // Update counters
  commandPacketCount++;
  lastTXResponse = "Command sent: " + command;
  lastCommandTime = millis();

  if (ESP32_C3_FIXES_ENABLED) {
    Serial.print("Free heap after: ");
    Serial.println(ESP.getFreeHeap());
  }
}

void sendConfigurationToTX() {
  if (!isConfigComplete()) {
    Serial.println("❌ Configuration incomplete - cannot send to TX");
    Serial.println("📋 Missing parameters:");
    if (testConfig.filename.isEmpty()) Serial.println("   • Filename");
    if (testConfig.totalWeight <= 0) Serial.println("   • Weight");
    if (testConfig.windSpeed < 0) Serial.println("   • Wind Speed");
    if (testConfig.height <= 0) Serial.println("   • Height");
    return;
  }
  
  Serial.println("📤 Sending COMBINED configuration to TX...");
  Serial.println("   📋 Configuration Details:");
  Serial.println("   • Filename: '" + testConfig.filename + "'");
  Serial.println("   • Weight: " + String(testConfig.totalWeight, 2) + " kg");
  Serial.println("   • Wind: " + String(testConfig.windSpeed, 2) + " m/s");
  Serial.println("   • Height: " + String(testConfig.height, 2) + " m");
  Serial.println("");
  
  // Validate filename before sending
  if (testConfig.filename.isEmpty()) {
    Serial.println("❌ CRITICAL ERROR: Filename is empty! Cannot send configuration.");
    return;
  }
  
  // Create combined CONFIG message (matches updated TX protocol)
  String combinedConfig = "CONFIG:" + testConfig.filename + "," + 
                         String((int)testConfig.totalWeight) + "," +
                         String((int)testConfig.windSpeed) + "," +
                         String((int)testConfig.height);
  
  Serial.println("📡 Sending COMBINED CONFIG: " + combinedConfig);
  Serial.println("� This matches the updated TX protocol that handles combined CONFIG packets");
  
  // Send using ultra-safe method
  sendLoRaCommandNow(combinedConfig);
  
  configurationSent = true;
  Serial.println("");
  Serial.println("✅ Configuration transmission complete!");
  Serial.println("🔍 Check TX serial monitor for:");
  Serial.println("   • 'LoRa packet received' message");
  Serial.println("   • 'RX Command: CONFIG:...' message");
  Serial.println("   • 'Combined CONFIG packet received' message");
  Serial.println("   • 'Configuration complete - ready to start' message");
  Serial.println("");
  Serial.println("⚠️  If TX doesn't show these messages, there may be LoRa communication issues.");
}

// === MENU SYSTEM IMPLEMENTATION ===

void enterMenuMode() {
  menuMode = true;
  currentMenuLevel = MENU_MAIN;
  Serial.println("\n🎛️ === ENTERING INTERACTIVE MENU MODE ===");
  Serial.println("💡 Use numbers to navigate, 'b' to go back, 'q' to quit menus");
  printMainMenu();
}

void exitMenuMode() {
  menuMode = false;
  Serial.println("\n✅ Exited menu mode - back to text commands");
  Serial.println("💡 Type 'menu' to return to menus or 'help' for text commands\n");
}

void processMenuInput(String input) {
  input.trim();
  input.toLowerCase();
  
  // Global menu commands
  if (input == "q" || input == "quit") {
    exitMenuMode();
    return;
  }
  else if (input == "b" || input == "back") {
    handleBackNavigation();
    return;
  }
  else if (input == "h" || input == "help") {
    printMenuHelp();
    return;
  }
  
  // Handle menu-specific input
  switch (currentMenuLevel) {
    case MENU_MAIN:
      processMainMenuInput(input);
      break;
    case MENU_CONFIG:
      processConfigMenuInput(input);
      break;
    case MENU_FLIGHT:
      processFlightMenuInput(input);
      break;
    case MENU_DATA:
      processDataMenuInput(input);
      break;
    case MENU_CALIBRATION:
      processCalibrationMenuInput(input);
      break;
    case MENU_SET_PARAMS:
      processSetParamsMenuInput(input);
      break;
  }
}

void handleBackNavigation() {
  switch (currentMenuLevel) {
    case MENU_MAIN:
      exitMenuMode();
      break;
    case MENU_CONFIG:
    case MENU_FLIGHT:
    case MENU_DATA:
    case MENU_CALIBRATION:
      // When going back to main menu, don't auto-switch modes
      // Let the user see their current mode on the main menu
      currentMenuLevel = MENU_MAIN;
      Serial.println("🔙 Returning to main menu...");
      printMainMenu();
      break;
    case MENU_SET_PARAMS:
      currentMenuLevel = MENU_CONFIG;
      // Ensure we're in config mode when in config menu
      if (currentMode != MODE_CONFIG) {
        Serial.println("🔄 Returning to CONFIG mode...");
        switchMode(MODE_CONFIG);
      }
      printConfigMenu();
      break;
  }
}

void printMenuHelp() {
  Serial.println("\n📚 === MENU NAVIGATION HELP ===");
  Serial.println("Numbers (1,2,3...): Select menu option");
  Serial.println("'b' or 'back':     Go back to previous menu");
  Serial.println("'q' or 'quit':     Exit menu mode");
  Serial.println("'h' or 'help':     Show this help");
  Serial.println("===============================");
  
  // Reprint current menu
  switch (currentMenuLevel) {
    case MENU_MAIN: printMainMenu(); break;
    case MENU_CONFIG: printConfigMenu(); break;
    case MENU_FLIGHT: printFlightMenu(); break;
    case MENU_DATA: printDataMenu(); break;
    case MENU_CALIBRATION: printCalibrationMenu(); break;
    case MENU_SET_PARAMS: printSetParamsMenu(); break;
  }
}

// === MAIN MENU ===
void printMainMenu() {
  Serial.println("\n🎛️ === MAIN MENU ===");
  printCurrentMode();
  
  // Show config status
  if (isConfigComplete()) {
    Serial.println("Config Status: ✅ Complete");
  } else {
    Serial.println("Config Status: ❌ Incomplete");
  }
  Serial.println("");
  
  Serial.println("Menu Navigation (will switch modes automatically):");
  Serial.println("1. 📋 Configuration Setup → CONFIG mode");
  Serial.println(String("2. 🚀 Flight Operations → FLIGHT mode ") + (isConfigComplete() ? "✅" : "❌"));
  Serial.println("3. 💾 Data Recovery → DATA RECOVERY mode");
  Serial.println("4. ⚖️  Load Cell Calibration → CALIBRATION mode");
  Serial.println("");
  Serial.println("Quick Actions (stay in current mode):");
  Serial.println("5. 📡 Test LoRa Connection");
  Serial.println("6. 📊 System Status");
  Serial.println("");
  Serial.println("'q' = Quit menus  |  'h' = Help");
  Serial.print("Choice: ");
}

void processMainMenuInput(String input) {
  int choice = input.toInt();
  
  switch (choice) {
    case 1:
      // Switch to CONFIG mode and menu
      if (currentMode != MODE_CONFIG) {
        Serial.println("🔄 Switching to CONFIG mode...");
        switchMode(MODE_CONFIG);
      }
      currentMenuLevel = MENU_CONFIG;
      printConfigMenu();
      break;
    case 2:
      // Switch to FLIGHT mode with validation
      if (attemptFlightModeSwitch()) {
        currentMenuLevel = MENU_FLIGHT;
        printFlightMenu();
      } else {
        // Stay on main menu if flight mode switch failed
        printMainMenu();
      }
      break;
    case 3:
      // Switch to DATA RECOVERY mode and menu
      if (currentMode != MODE_DATA_RECOVERY) {
        Serial.println("🔄 Switching to DATA RECOVERY mode...");
        switchMode(MODE_DATA_RECOVERY);
      }
      currentMenuLevel = MENU_DATA;
      printDataMenu();
      break;
    case 4:
      // Switch to LOAD CALIBRATION mode and menu
      if (currentMode != MODE_LOAD_CALIBRATION) {
        Serial.println("🔄 Switching to LOAD CALIBRATION mode...");
        switchMode(MODE_LOAD_CALIBRATION);
      }
      currentMenuLevel = MENU_CALIBRATION;
      printCalibrationMenu();
      break;
    case 5:
      Serial.println("\n📡 Testing LoRa connection...");
      sendCommandToFlightComputer("PING");
      Serial.println("✅ PING sent - check TX response");
      printMainMenu();
      break;
    case 6:
      printGroundStationStatus();
      printMainMenu();
      break;
    default:
      Serial.println("❌ Invalid choice. Enter 1-6, 'b' for back, or 'q' to quit.");
      Serial.print("Choice: ");
      break;
  }
}

// === CONFIGURATION MENU ===
void printConfigMenu() {
  Serial.println("\n📋 === CONFIGURATION SETUP ===");
  
  // Show current config status
  Serial.println("Current Configuration:");
  Serial.println("• Filename: " + (testConfig.filename.isEmpty() ? "❌ Not set" : "✅ " + testConfig.filename));
  Serial.println("• Weight: " + (testConfig.totalWeight <= 0 ? "❌ Not set" : "✅ " + String(testConfig.totalWeight, 1) + " kg"));
  Serial.println("• Wind Speed: " + (testConfig.windSpeed < 0 ? "❌ Not set" : "✅ " + String(testConfig.windSpeed, 1) + " m/s"));
  Serial.println("• Height: " + (testConfig.height <= 0 ? "❌ Not set" : "✅ " + String(testConfig.height, 1) + " m"));
  Serial.println("");
  
  Serial.println("Options:");
  Serial.println("1. 📝 Set Test Parameters");
  Serial.println("2. 📋 Show Full Configuration");
  Serial.println("3. 📤 Send Config to TX");
  Serial.println("4. 🚀 Enter Flight Mode");
  Serial.println("");
  Serial.println("'b' = Back to Main  |  'q' = Quit menus");
  Serial.print("Choice: ");
}

void processConfigMenuInput(String input) {
  int choice = input.toInt();
  
  switch (choice) {
    case 1:
      currentMenuLevel = MENU_SET_PARAMS;
      printSetParamsMenu();
      break;
    case 2:
      showConfiguration();
      printConfigMenu();
      break;
    case 3:
      if (isConfigComplete()) {
        Serial.println("📤 Sending configuration to TX...");
        sendConfigurationToTX();
        configurationSent = true;
        Serial.println("✅ Configuration sent successfully!");
      } else {
        Serial.println("❌ Configuration incomplete - cannot send to TX");
        Serial.println("💡 Use option 1 to set missing parameters");
      }
      printConfigMenu();
      break;
    case 4:
      // Attempt to switch to flight mode with validation and confirmation
      if (attemptFlightModeSwitch()) {
        currentMenuLevel = MENU_FLIGHT;
        printFlightMenu();
      } else {
        // Stay in config menu if flight mode switch failed
        printConfigMenu();
      }
      break;
    default:
      Serial.println("❌ Invalid choice. Enter 1-4, 'b' for back, or 'q' to quit.");
      Serial.print("Choice: ");
      break;
  }
}

// === SET PARAMETERS SUBMENU ===
void printSetParamsMenu() {
  Serial.println("\n📝 === SET TEST PARAMETERS ===");
  Serial.println("1. ⚙️ Set All Parameters (Quick Setup)");
  Serial.println("2. 📁 Set Filename Only");
  Serial.println("3. ⚖️  Set Total Weight Only");
  Serial.println("4. 🌬️  Set Wind Speed Only");
  Serial.println("5. 📏 Set Height Only");
  Serial.println("");
  Serial.println("'b' = Back to Config  |  'q' = Quit menus");
  Serial.print("Choice: ");
}

void processSetParamsMenuInput(String input) {
  int choice = input.toInt();
  
  switch (choice) {
    case 1:
      promptForAllParameters();
      break;
    case 2:
      promptForFilename();
      break;
    case 3:
      promptForWeight();
      break;
    case 4:
      promptForWindSpeed();
      break;
    case 5:
      promptForHeight();
      break;
    default:
      Serial.println("❌ Invalid choice. Enter 1-5, 'b' for back, or 'q' to quit.");
      Serial.print("Choice: ");
      break;
  }
}

// === FLIGHT OPERATIONS MENU ===
void printFlightMenu() {
  Serial.println("\n🚀 === FLIGHT OPERATIONS ===");
  
  // Show current mode status
  Serial.print("Current Mode: ");
  printCurrentMode();
  
  if (currentMode != MODE_FLIGHT) {
    Serial.println("⚠️ Warning: Menu and mode mismatch!");
    Serial.println("💡 This should not happen - please report this issue");
    Serial.println("");
  }
  
  // Show flight status
  if (flightModeActive) {
    Serial.println("Flight Status: 🟢 ACTIVE LOGGING - Data recording in progress");
  } else {
    Serial.println("Flight Status: 🟡 Ready to start - Configuration loaded");
  }
  
  // Show current config summary
  if (isConfigComplete()) {
    Serial.println("Config: ✅ " + testConfig.filename + " | " + String(testConfig.totalWeight, 1) + "kg | " + String(testConfig.height, 1) + "m");
  } else {
    Serial.println("Config: ❌ Incomplete (this should not happen in flight mode!)");
  }
  Serial.println("");
  
  Serial.println("1. 🚀 Start Data Logging");
  Serial.println("2. 🛑 Stop Data Logging");
  Serial.println("3. 🎯 Reset Sensor Origins");
  Serial.println("4. 💥 Fire Pyro Channel");
  Serial.println("5. 📋 Return to Config Mode");
  Serial.println("");
  Serial.println("'b' = Back to Main  |  'q' = Quit menus");
  Serial.print("Choice: ");
}

void processFlightMenuInput(String input) {
  int choice = input.toInt();
    switch (choice) {
    case 1:
      Serial.println("🚀 Starting data logging...");
      sendCommandToFlightComputer("FLIGHT_START");
      flightModeActive = true;
      printFlightMenu();
      break;
    case 2:
      Serial.println("🛑 Stopping data logging...");
      sendCommandToFlightComputer("FLIGHT_STOP");
      flightModeActive = false;
      printFlightMenu();
      break;
    case 3:
      Serial.println("🎯 Resetting sensor zero points...");
      sendCommandToFlightComputer("SENSOR_RESET");
      printFlightMenu();
      break;
    case 4:
      showPyroMenu();
      break;
    case 5:
      // Return to config mode with confirmation if flight is active
      if (flightModeActive) {
        Serial.println("\n⚠️ === FLIGHT ACTIVE WARNING ===");
        Serial.println("Data logging is currently active!");
        Serial.println("Switching to CONFIG mode will stop flight operations.");
        Serial.print("Continue anyway? (y/n): ");
        
        String confirmation = getMenuTextInput();
        confirmation.toLowerCase();
        
        if (confirmation == "y" || confirmation == "yes") {
          Serial.println("🛑 Stopping flight operations...");
          sendCommandToFlightComputer("FLIGHT_STOP");
          flightModeActive = false;
          Serial.println("🔄 Switching to CONFIG mode...");
          switchMode(MODE_CONFIG);
          currentMenuLevel = MENU_CONFIG;
          printConfigMenu();
        } else {
          Serial.println("❌ Mode switch cancelled - staying in FLIGHT mode");
          printFlightMenu();
        }
      } else {
        Serial.println("🔄 Returning to CONFIG mode...");
        switchMode(MODE_CONFIG);
        currentMenuLevel = MENU_CONFIG;
        printConfigMenu();
      }
      break;
    default:
      Serial.println("❌ Invalid choice. Enter 1-5, 'b' for back, or 'q' to quit.");
      Serial.print("Choice: ");
      break;
  }
}

// === DATA RECOVERY MENU ===
void printDataMenu() {
  Serial.println("\n💾 === DATA RECOVERY ===");
  Serial.println("1. 📶 Start WiFi on TX");
  Serial.println("2. 📶 Stop WiFi on TX");
  Serial.println("3. 📁 Request File List from TX");
  Serial.println("4. 🌐 Open WiFi Instructions");
  Serial.println("");
  Serial.println("'b' = Back to Main  |  'q' = Quit menus");
  Serial.print("Choice: ");
}

void processDataMenuInput(String input) {
  int choice = input.toInt();
  
  switch (choice) {
    case 1:
      Serial.println("📶 Starting WiFi on TX for file download...");
      sendCommandToFlightComputer("WIFI_START");
      Serial.println("💡 TX should start WiFi SoftAP - check TX serial output");
      printDataMenu();
      break;
    case 2:
      Serial.println("📶 Stopping WiFi on TX...");
      sendCommandToFlightComputer("WIFI_STOP");
      printDataMenu();
      break;
    case 3:
      Serial.println("📁 Requesting file list from TX...");
      sendCommandToFlightComputer("FILE_LIST");
      printDataMenu();
      break;
    case 4:
      printWiFiInstructions();
      printDataMenu();
      break;
    default:
      Serial.println("❌ Invalid choice. Enter 1-4, 'b' for back, or 'q' to quit.");
      Serial.print("Choice: ");
      break;
  }
}

// === CALIBRATION MENU ===
void printCalibrationMenu() {
  Serial.println("\n⚖️ === LOAD CELL CALIBRATION ===");
  Serial.println("1. 🔧 Start Calibration Process");
  Serial.println("2. ⚖️  Set Zero Point (No Load)");
  Serial.println("3. 📏 Set Known Weight Point");
  Serial.println("4. 💾 Save Calibration");
  Serial.println("5. 🧪 Test Current Calibration");
  Serial.println("");
  Serial.println("'b' = Back to Main  |  'q' = Quit menus");
  Serial.print("Choice: ");
}

void processCalibrationMenuInput(String input) {
  int choice = input.toInt();
  
  switch (choice) {
    case 1:
      Serial.println("⚖️ Starting load cell calibration process...");
      sendCommandToFlightComputer("CALIB_START");
      printCalibrationMenu();
      break;
    case 2:
      Serial.println("⚖️ Setting zero point (ensure no load on cell)...");
      sendCommandToFlightComputer("CALIB_ZERO");
      printCalibrationMenu();
      break;
    case 3:
      promptForCalibrationWeight();
      break;
    case 4:
      Serial.println("⚖️ Saving calibration constants...");
      sendCommandToFlightComputer("CALIB_SAVE");
      printCalibrationMenu();
      break;
    case 5:
      Serial.println("⚖️ Testing current calibration...");
      sendCommandToFlightComputer("CALIB_TEST");
      printCalibrationMenu();
      break;
    default:
      Serial.println("❌ Invalid choice. Enter 1-5, 'b' for back, or 'q' to quit.");
      Serial.print("Choice: ");
      break;
  }
}

// === PARAMETER INPUT FUNCTIONS ===
void promptForAllParameters() {
  Serial.println("\n🚀 === QUICK SETUP - ALL PARAMETERS ===");
  Serial.println("Enter all test parameters in sequence:");
  Serial.println("(Press Enter after each parameter, or type 'skip' to keep current value)\n");
  
  // 1. Filename
  Serial.print("📁 Filename");
  if (!testConfig.filename.isEmpty()) {
    Serial.print(" (current: " + testConfig.filename + ")");
  }
  Serial.print(": ");
  
  String filename = getMenuTextInput();
  filename.toLowerCase();
  if (filename.length() > 0 && filename != "skip") {
    testConfig.filename = filename;
    Serial.println("   ✅ Filename set to: " + filename + ".txt");
  } else {
    Serial.println("   ⏭️  Filename unchanged");
  }
  
  // 2. Weight
  Serial.print("\n⚖️  Total Weight (kg)");
  if (testConfig.totalWeight > 0) {
    Serial.print(" (current: " + String(testConfig.totalWeight, 1) + " kg)");
  }
  Serial.print(": ");
  
  String weightInput = getMenuTextInput();
  weightInput.toLowerCase();
  if (weightInput.length() > 0 && weightInput != "skip") {
    float weight = weightInput.toFloat();
    if (weight > 0) {
      testConfig.totalWeight = weight;
      Serial.println("   ✅ Weight set to: " + String(weight, 2) + " kg");
    } else {
      Serial.println("   ❌ Invalid weight - keeping previous value");
    }
  } else {
    Serial.println("   ⏭️  Weight unchanged");
  }
  
  // 3. Wind Speed
  Serial.print("\n🌬️  Wind Speed (m/s)");
  if (testConfig.windSpeed >= 0) {
    Serial.print(" (current: " + String(testConfig.windSpeed, 1) + " m/s)");
  }
  Serial.print(": ");
  
  String windInput = getMenuTextInput();
  windInput.toLowerCase();
  if (windInput.length() > 0 && windInput != "skip") {
    float wind = windInput.toFloat();
    if (wind >= 0) {
      testConfig.windSpeed = wind;
      Serial.println("   ✅ Wind speed set to: " + String(wind, 2) + " m/s");
    } else {
      Serial.println("   ❌ Invalid wind speed - keeping previous value");
    }
  } else {
    Serial.println("   ⏭️  Wind speed unchanged");
  }
  
  // 4. Height
  Serial.print("\n📏 Height (m)");
  if (testConfig.height > 0) {
    Serial.print(" (current: " + String(testConfig.height, 1) + " m)");
  }
  Serial.print(": ");
  
  String heightInput = getMenuTextInput();
  heightInput.toLowerCase();
  if (heightInput.length() > 0 &&  heightInput != "skip") {
    float height = heightInput.toFloat();
    if (height > 0) {
      testConfig.height = height;
      Serial.println("   ✅ Height set to: " + String(height, 2) + " m");
    } else {
      Serial.println("   ❌ Invalid height - keeping previous value");
    }
  } else {
    Serial.println("   ⏭️  Height unchanged");
  }
  
  // Summary
  Serial.println("\n🎯 === CONFIGURATION SUMMARY ===");
  Serial.println("• Filename: " + (testConfig.filename.isEmpty() ? "❌ Not set" : "✅ " + testConfig.filename + ".txt"));
  Serial.println("• Weight: " + (testConfig.totalWeight <= 0 ? "❌ Not set" : "✅ " + String(testConfig.totalWeight, 1) + " kg"));
  Serial.println("• Wind Speed: " + (testConfig.windSpeed < 0 ? "❌ Not set" : "✅ " + String(testConfig.windSpeed, 1) + " m/s"));
  Serial.println("• Height: " + (testConfig.height <= 0 ? "❌ Not set" : "✅ " + String(testConfig.height, 1) + " m"));
  Serial.println("");
  
  if (isConfigComplete()) {
    Serial.println("🎉 Configuration is COMPLETE and ready!");
    Serial.println("💡 You can now send this config to TX or enter flight mode");
  } else {
    Serial.println("⚠️  Configuration is still incomplete");
    Serial.println("💡 Use individual parameter options to set missing values");
  }
  
  Serial.println("================================");
  
  // Return to config menu
  currentMenuLevel = MENU_CONFIG;
  printConfigMenu();
}

void promptForFilename() {
  Serial.println("\n📁 === SET FILENAME ===");
  Serial.println("Enter test filename (will be saved as filename.txt):");
  Serial.print("Filename: ");
  
  String filename = getMenuTextInput();
  if (filename.length() > 0) {
    testConfig.filename = filename;
    Serial.println("✅ Filename set to: " + filename + ".txt");
  } else {
    Serial.println("❌ Invalid filename");
  }
  
  currentMenuLevel = MENU_CONFIG;
  printConfigMenu();
}

void promptForWeight() {
  Serial.println("\n⚖️ === SET TOTAL WEIGHT ===");
  Serial.println("Enter total weight in kilograms:");
  Serial.print("Weight (kg): ");
  
  String input = getMenuTextInput();
  float weight = input.toFloat();
  if (weight > 0) {
    testConfig.totalWeight = weight;
    Serial.println("✅ Weight set to: " + String(weight, 2) + " kg");
  } else {
    Serial.println("❌ Invalid weight");
  }
  
  currentMenuLevel = MENU_CONFIG;
  printConfigMenu();
}

void promptForWindSpeed() {
  Serial.println("\n🌬️ === SET WIND SPEED ===");
  Serial.println("Enter wind speed in m/s:");
  Serial.print("Wind speed (m/s): ");
  
  String input = getMenuTextInput();
  float wind = input.toFloat();
  if (wind >= 0) {
    testConfig.windSpeed = wind;
    Serial.println("✅ Wind speed set to: " + String(wind, 2) + " m/s");
  } else {
    Serial.println("❌ Invalid wind speed");
  }
  
  currentMenuLevel = MENU_CONFIG;
  printConfigMenu();
}

void promptForHeight() {
  Serial.println("\n📏 === SET HEIGHT ===");
  Serial.println("Enter height in meters:");
  Serial.print("Height (m): ");
  
  String input = getMenuTextInput();
  float height = input.toFloat();
  if (height > 0) {
    testConfig.height = height;
    Serial.println("✅ Height set to: " + String(height, 2) + " m");
  } else {
    Serial.println("❌ Invalid height");
  }
  
  currentMenuLevel = MENU_CONFIG;
  printConfigMenu();
}

void promptForCalibrationWeight() {
  Serial.println("\n📏 === SET CALIBRATION WEIGHT ===");
  Serial.println("Place known weight on load cell, then enter its value:");
  Serial.print("Calibration weight (kg): ");
  
  String input = getMenuTextInput();
  float weight = input.toFloat();
  if (weight > 0) {
    Serial.println("⚖️ Setting calibration weight: " + String(weight, 2) + " kg");
    sendCommandToFlightComputer("CALIB_WEIGHT:" + String(weight, 2));
    Serial.println("✅ Calibration weight command sent");
  } else {
    Serial.println("❌ Invalid weight");
  }
  
  printCalibrationMenu();
}

void showPyroMenu() {
  Serial.println("\n💥 === PYRO CHANNEL SELECTION ===");
  Serial.println("⚠️  WARNING: This will fire a pyro channel!");
  Serial.println("");
  Serial.println("1. 💥 Fire Pyro Channel 1");
  Serial.println("2. 💥 Fire Pyro Channel 2");
  Serial.println("3. 💥 Fire Pyro Channel 3");
  Serial.println("4. 💥 Fire Pyro Channel 4");
  Serial.println("5. ❌ Cancel");
  Serial.println("");
  Serial.print("Choice (1-5): ");
  
  String input = getMenuTextInput();
  int choice = input.toInt();
  
  if (choice >= 1 && choice <= 4) {
    String pyroCmd = "PYRO" + String(choice);
    Serial.println("💥 *** PYRO COMMAND WARNING ***");
    Serial.println("About to send: " + pyroCmd);
    Serial.println("This will fire a pyro channel on the flight computer!");
    Serial.print("Type 'FIRE' to confirm or anything else to cancel: ");
    
    String confirmation = getMenuTextInput();
    confirmation.toUpperCase();
    
    if (confirmation == "FIRE") {
      Serial.println("💥 FIRING PYRO CHANNEL " + String(choice) + "!");
      sendCommandToFlightComputer(pyroCmd);
    } else {
      Serial.println("❌ Pyro command cancelled");
    }
  } else if (choice == 5) {
    Serial.println("❌ Pyro operation cancelled");
  } else {
    Serial.println("❌ Invalid choice");
  }
  
  printFlightMenu();
}

void printWiFiInstructions() {
  Serial.println("\n🌐 === WIFI CONNECTION INSTRUCTIONS ===");
  Serial.println("1. Send 'Start WiFi on TX' command (option 1)");
  Serial.println("2. Wait for TX to start WiFi SoftAP");
  Serial.println("3. On your computer/phone, look for WiFi network:");
  Serial.println("   • Network: 'Pavi-FlightComputer'");
  Serial.println("   • Password: 'pavitest123'");
  Serial.println("4. Connect to this network");
  Serial.println("5. Open browser and go to: http://192.168.4.1 \nOR\n");
  Serial.println("6. Open browser and go to: http://paviflightdata.local");
  Serial.println("7. Use the web interface to download files");
  Serial.println("8. When done, use 'Stop WiFi on TX' (option 2)");
  Serial.println("================================================");
}

String getMenuTextInput() {
  String input = "";
  unsigned long timeout = millis() + 30000; // 30 second timeout
  
  while (millis() < timeout) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        Serial.println(); // New line
        break;
      } else if (c == '\b' || c == 127) { // Backspace
        if (input.length() > 0) {
          input.remove(input.length() - 1);
          Serial.print("\b \b"); // Clear character
        }
      } else {
        input += c;
        Serial.print(c); // Echo
      }
    }
  }
  
  if (millis() >= timeout) {
    Serial.println("\n❌ Input timeout");
  }
  
  input.trim();
  return input;
}

// ESP32-C3 Memory Monitoring Function
void checkMemoryStatus() {
  #if ESP32_C3_FIXES_ENABLED
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 10000) { // Check every 10 seconds
    lastCheck = millis();
    uint32_t freeHeap = ESP.getFreeHeap();
    uint32_t minFreeHeap = ESP.getMinFreeHeap();
    
    if (freeHeap < 100000) { // Less than 100KB
      Serial.println("🚨 CRITICAL: Low memory detected!");
      Serial.print("   Free: "); Serial.print(freeHeap); Serial.println(" bytes");
      Serial.print("   Min ever: "); Serial.print(minFreeHeap); Serial.println(" bytes");
      Serial.println("   Consider reducing String operations or restarting");
    } else if (freeHeap < 150000) { // Less than 150KB
      Serial.println("⚠️  Warning: Memory getting low");
      Serial.print("   Free: "); Serial.print(freeHeap); Serial.println(" bytes");
    }
  }
  #endif
}
