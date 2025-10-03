#include <SPI.h>
#include <LoRa.h>
#include <SD.h>
#include <FS.h>

// ESP32-C3 specific includes
#ifdef CONFIG_IDF_TARGET_ESP32C3
  #include "esp_task_wdt.h"
#endif

// === CONFIGURATION ===
#define NSS   D7   // IO7
#define RST   D1   // IO2  
#define DIO0  D0   // IO3
#define SD_CS D8   // SD card chip select

// ESP32-C3 memory optimization
#ifdef CONFIG_IDF_TARGET_ESP32C3
  #define ESP32_C3_MODE true
  #pragma message "ESP32-C3 optimized mode enabled"
#else
  #define ESP32_C3_MODE false
#endif

// === SYSTEM STATE ===
enum GroundStationMode { MODE_CONFIG, MODE_FLIGHT, MODE_DATA, MODE_CALIBRATION };
GroundStationMode currentMode = MODE_CONFIG;

// Enhanced menu system
enum MenuLevel {
  MENU_MAIN,
  MENU_CONFIG,
  MENU_FLIGHT, 
  MENU_DATA,
  MENU_CALIBRATION,
  MENU_SET_PARAMS,
  MENU_WIFI
};
MenuLevel currentMenuLevel = MENU_MAIN;

struct TestConfig {
  String filename = "";
  float totalWeight = -1;
  float windSpeed = -1;
  float height = -1;
  bool pressureFilter = false;
  bool accelFilter = false;
  bool gyroFilter = false;
  bool magFilter = false;
  bool gpsFilter = false;
  
  bool isComplete() { 
    return filename != "" && totalWeight > 0 && windSpeed >= 0 && height > 0; 
  }
  
  void clear() {
    filename = "";
    totalWeight = -1;
    windSpeed = -1;
    height = -1;
    pressureFilter = false;
    accelFilter = false;
    gyroFilter = false;
    magFilter = false;
    gpsFilter = false;
  }
} config;

struct TelemetryData {
  float altitude = 0, velocity = 0, weight = 0;
  float accelX = 0, accelY = 0, accelZ = 0;
  float gyroX = 0, gyroY = 0, gyroZ = 0;
  float temperature = 0;
  int flightState = 0, rssi = 0;
  unsigned long lastUpdate = 0;
} telemetry;

// === SYSTEM VARIABLES ===
bool menuMode = false, flightActive = false, configSent = false;
String lastResponse = "Waiting for TX response...";
String connectionStatus = "Disconnected";
String commandBuffer = "", menuInput = "";
unsigned long lastCommandTime = 0;
int telemetryCount = 0, commandCount = 0;
bool showSerialData = true, fcConnected = false;




// function declarations
void showDataMenu(); 
void showMainMenu();
void showConfigMenu();
void showFlightMenu();
void showCalibrationMenu();
void showWiFiMenu();
void showSystemStatus();
void showESP32Diagnostics();
void showHelpMenu(); 

// === CORE FUNCTIONS ===
void setup() {
  Serial.begin(115200);
  delay(2000);  // Longer startup delay for ESP32-C3
  
  Serial.println("\n🚀 PAVI GROUND STATION RX READY 🚀");
  Serial.println("===================================");
  Serial.println("📡 Commands are sent to TX via LoRa");
  Serial.println("📊 Data is received and logged locally");
  
  if (ESP32_C3_MODE) {
    Serial.println("ESP32-C3 Ultra-Conservative Anti-Reset Mode");
    Serial.print("Free heap at startup: ");
    Serial.println(ESP.getFreeHeap());
    
    // ESP32-C3 CRITICAL: Disable brownout detector that can cause resets
    #ifdef ESP32C3
      // Note: Brownout detector disable requires bootloader modification
      // Instead, we use ultra-low TX power and conservative timing
      Serial.println("ESP32-C3 Reset Prevention: Ultra-low power mode");
    #endif
    
    // ESP32-C3 Watchdog Fix: Use software feeding instead of reconfiguration
    // Note: Hardware watchdog reconfiguration requires complex struct setup in newer ESP-IDF
    // Instead, we rely on frequent yield() calls throughout the code for stability
    Serial.println("Using software watchdog feeding strategy");
    
    // ESP32-C3 Power Management: MINIMAL power consumption
    setCpuFrequencyMhz(20);  // Even lower CPU frequency for maximum stability
    Serial.println("CPU frequency set to 20MHz for maximum watchdog stability");
    
    // ESP32-C3: Additional power management
    delay(500);
    yield();
    Serial.println("ESP32-C3 initialization complete");
  }
  
  
  // ESP32-C3 Fix: Initialize SPI with explicit pins and longer delays
  SPI.begin();
  delay(500); // Much longer SPI settling time


  // Initialize SD card for local data logging
  // Serial.println("Initializing SD card...");
  // if (!SD.begin(SD_CS)) {
  //   Serial.println("⚠️  SD card initialization failed - local logging disabled");
  // } else {
  //   Serial.println("✅ SD card ready for local data logging");
  // }
  
  Serial.println("Initializing LoRa...");
  
  // Initialize LoRa with ESP32-C3 ultra-conservative settings
  LoRa.setPins(NSS, RST, DIO0);
  
  // ESP32-C3 CRITICAL: Set minimal TX power BEFORE begin()
  delay(100);
  
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa initialization failed!");
    while (1) {
      delay(1000);
      Serial.println("Retrying LoRa initialization...");
      if (LoRa.begin(433E6)) break;
    }
  }
  
  // ESP32-C3 Fix: MINIMAL TX POWER for stability (set immediately after begin)
  // LoRa.setTxPower(2);             // ABSOLUTE MINIMUM for ESP32-C3 stability
  delay(100);
  
  // ESP32-C3 Fix: Ultra-conservative LoRa settings
  LoRa.setSpreadingFactor(12);    // Maximum reliability
  LoRa.setSignalBandwidth(125E3); // Standard bandwidth
  LoRa.setCodingRate4(8);         // Maximum error correction
  LoRa.enableCrc();               // Enable CRC
  LoRa.setSyncWord(0x12);         // Custom sync word
  
  // ESP32-C3: Additional stability settings
  LoRa.setPreambleLength(8);      // Longer preamble for better sync
  LoRa.setTxPower(2);             // Re-confirm MINIMUM TX power
  
  delay(500); // Long LoRa settling time
  
  Serial.println("LoRa initialized with ultra-conservative settings");
  Serial.println("TX Power: 2dBm (MINIMUM possible for ESP32-C3 stability)");
  Serial.println("ESP32-C3 Anti-Reset Transmission Mode: ENABLED");
  
  if (ESP32_C3_MODE) {
    Serial.print("Free heap after LoRa init: ");
    Serial.println(ESP.getFreeHeap());
  }
  
  showMainMenu();
}

void loop() {
  // ESP32-C3: Very frequent yield calls for watchdog stability
  yield();
  
  processUserInput();
  
  yield(); // Feed watchdog between operations
  
  handleLoRaReceive();
  
  yield(); // Feed watchdog after operations
  
  // ESP32-C3: Less frequent status checks to avoid watchdog issues
  if (ESP32_C3_MODE) {
    static unsigned long lastStatusCheck = 0;
    if (millis() - lastStatusCheck > 60000) {  // Every minute instead of 30s
      esp32c3StatusCheck();
      lastStatusCheck = millis();
      yield(); // Feed watchdog after status check
    }
    
    // Memory check - less frequent
    static unsigned long lastMemCheck = 0;
    if (millis() - lastMemCheck > 120000) {  // Every 2 minutes
      checkMemory();
      lastMemCheck = millis();
      yield(); // Feed watchdog after memory check
    }
  }
  
  delay(20); // Slightly longer delay for stability
  yield(); // Final yield before loop restart
}

// === INPUT PROCESSING ===
void processUserInput() {
  if (Serial.available()) {
    yield(); // Feed watchdog before processing
    
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    yield(); // Feed watchdog after string operations
    
    if (input.length() == 0) return;
    
    // Always process as menu input (number-based)
    processMenuInput(input);
    
    yield(); // Feed watchdog after command processing
  }
}

void processCommand(String cmd) {
  cmd.toLowerCase();
  
  // Global commands
  if (cmd == "help" || cmd == "h") {
    printHelp();
  } else if (cmd == "menu" || cmd == "m") {
    enterMenu();
  } else if (cmd == "status" || cmd == "s") {
    printStatus();
  } else if (cmd == "mode") {
    printCurrentMode();
  } else if (cmd.startsWith("mode ")) {
    switchMode(cmd.substring(5));
  } else if (cmd == "reset" && ESP32_C3_MODE) {
    emergencyLoRaReset();
  } else if (cmd == "check" && ESP32_C3_MODE) {
    esp32c3StatusCheck();
  } else {
    processContextCommand(cmd);
  }
}

void processContextCommand(String cmd) {
  switch (currentMode) {
    case MODE_CONFIG:
      processConfigCommand(cmd);
      break;
    case MODE_FLIGHT:
      processFlightCommand(cmd);
      break;
    case MODE_DATA:
      processDataCommand(cmd);
      break;
    case MODE_CALIBRATION:
      processCalCommand(cmd);
      break;
  }
}

// === MODE-SPECIFIC COMMANDS ===
void processConfigCommand(String cmd) {
  if (cmd.startsWith("filename ") || cmd.startsWith("f ")) {
    config.filename = cmd.substring(cmd.indexOf(' ') + 1);
    Serial.println("✅ Filename stored locally: " + config.filename);
  } else if (cmd.startsWith("weight ") || cmd.startsWith("w ")) {
    config.totalWeight = cmd.substring(cmd.indexOf(' ') + 1).toFloat();
    Serial.println("✅ Weight stored locally: " + String(config.totalWeight) + "g");
  } else if (cmd.startsWith("wind ")) {
    config.windSpeed = cmd.substring(5).toFloat();
    Serial.println("✅ Wind speed stored locally: " + String(config.windSpeed) + " m/s");
  } else if (cmd.startsWith("height ") || cmd.startsWith("h ")) {
    config.height = cmd.substring(cmd.indexOf(' ') + 1).toFloat();
    Serial.println("✅ Height stored locally: " + String(config.height) + "m");
  } else if (cmd == "show" || cmd == "config") {
    showLocalConfig();
  } else if (cmd == "send") {
    sendConfigToTX();
  } else if (cmd == "clear") {
    config.clear();
    Serial.println("✅ Local configuration cleared");
  } else {
    Serial.println("Config commands: filename <name>, weight <g>, wind <m/s>, height <m>, show, send, clear");
    Serial.println("💡 All changes are stored locally until you use 'send'");
  }
}

void processFlightCommand(String cmd) {
  if (cmd == "start" || cmd == "begin") {
    sendLoRaCommand("START_LOGGING");
    Serial.println("Flight recording started");
    flightActive = true;
  } else if (cmd == "stop" || cmd == "end") {
    sendLoRaCommand("STOP_LOGGING");
    Serial.println("Flight recording stopped");
    flightActive = false;
  } else if (cmd.startsWith("pyro")) {
    String pyroCmd = cmd;
    pyroCmd.replace("pyro", "PYRO");
    sendLoRaCommand(pyroCmd);
    Serial.println("Pyro command sent: " + pyroCmd);
  } else if (cmd == "status") {
    sendLoRaCommand("STATUS");
  } else {
    Serial.println("Flight commands: start, stop, pyro1-4, status");
  }
}

void processDataCommand(String cmd) {
  if (cmd == "wifi") {
    sendLoRaCommand("START_WIFI");
    Serial.println("WiFi access point started on TX");
    Serial.println("Connect to: PaviFlightData or PaviFlightData-2");
    Serial.println("Access: http://192.168.4.1");
  } else if (cmd == "stop") {
    sendLoRaCommand("STOP_WIFI");
    Serial.println("WiFi stopped on TX");
  } else if (cmd == "list") {
    sendLoRaCommand("LIST_FILES");
  } else {
    Serial.println("Data commands: wifi, stop, list");
  }
}

void processCalCommand(String cmd) {
  if (cmd == "tare") {
    sendLoRaCommand("CAL_TARE");
  } else if (cmd.startsWith("cal ")) {
    float weight = cmd.substring(4).toFloat();
    sendLoRaCommand("CAL_WEIGHT:" + String(weight));
  } else if (cmd == "save") {
    sendLoRaCommand("CAL_SAVE");
  } else if (cmd == "status") {
    sendLoRaCommand("CAL_STATUS");
  } else {
    Serial.println("Calibration commands: tare, cal <weight>, save, status");
  }
}

// === MENU SYSTEM ===
void enterMenu() {
  menuMode = true;
  Serial.println("\n=== MENU MODE ===");
  printMainMenu();
}

void exitMenu() {
  menuMode = false;
  Serial.println("Exited menu mode");
  printHelp();
}

void processMenuInput(String input) {
  int choice = input.toInt();
  
  switch (currentMenuLevel) {
    case MENU_MAIN:
      handleMainMenu(choice);
      break;
    case MENU_CONFIG:
      handleConfigMenu(choice, input);
      break;
    case MENU_FLIGHT:
      handleFlightMenu(choice);
      break;
    case MENU_DATA:
      handleDataMenu(choice);
      break;
    case MENU_CALIBRATION:
      handleCalibrationMenu(choice);
      break;
    case MENU_WIFI:
      handleWiFiMenu(choice);
      break;
    case MENU_SET_PARAMS:
      handleParameterMenu(choice, input);
      break;
  }
}

void handleMainMenu(int choice) {
  switch (choice) {
    case 1:
      showConfigMenu();
      break;
    case 2:
      showFlightMenu();
      break;
    case 3:
      showDataMenu();
      break;
    case 4:
      showCalibrationMenu();
      break;
    case 5:
      showWiFiMenu();
      break;
    case 6:
      showSystemStatus();
      break;
    case 7:
      if (ESP32_C3_MODE) showESP32Diagnostics();
      else Serial.println("❌ Invalid option");
      break;
    case 0:
      showHelpMenu();
      break;
    case 99:
      showMainMenu();
      break;
    default:
      Serial.println("❌ Invalid option. Enter number 0-" + String(ESP32_C3_MODE ? "7" : "6"));
      break;
  }
}

// === CONFIGURATION MENU ===
void showConfigMenu() {
  currentMenuLevel = MENU_CONFIG;
  Serial.println("\n╔═══════════════════════════════════════╗");
  Serial.println("║        ⚙️  TX CONFIGURATION ⚙️        ║");
  Serial.println("╠═══════════════════════════════════════╣");
  Serial.println("║  1️⃣  - Set All Parameters (Guided)   ║");
  Serial.println("║  2️⃣  - Set Data Filename              ║");
  Serial.println("║  3️⃣  - Set Total Weight (g)           ║");
  Serial.println("║  4️⃣  - Set Wind Speed (m/s)           ║");
  Serial.println("║  5️⃣  - Set Drop Height (m)            ║");
  Serial.println("║  6️⃣  - Toggle Sensor Filters          ║");
  Serial.println("║  7️⃣  - Show Current Config            ║");
  Serial.println("║  8️⃣  - Send Config to TX              ║");
  Serial.println("║  9️⃣  - Clear Configuration            ║");
  Serial.println("║  0️⃣  - Back to Main Menu              ║");
  Serial.println("╚═══════════════════════════════════════╝");
  Serial.println("📝 Enter configuration option:");
}

void handleConfigMenu(int choice, String input) {
  switch (choice) {
    case 1:
      setAllParameters();
      break;
    case 2:
      Serial.println("📝 Enter filename (without extension):");
      waitForConfigInput("FILENAME");
      break;
    case 3:
      Serial.println("⚖️ Enter total weight in grams:");
      waitForConfigInput("WEIGHT");
      break;
    case 4:
      Serial.println("💨 Enter wind speed in m/s:");
      waitForConfigInput("WIND");
      break;
    case 5:
      Serial.println("📏 Enter drop height in meters:");
      waitForConfigInput("HEIGHT");
      break;
    case 6:
      showFilterMenu();
      break;
    case 7:
      showLocalConfig();
      break;
    case 8:
      sendConfigToTX();
      break;
    case 9:
      config.clear();
      Serial.println("✅ Local configuration cleared");
      showConfigMenu();
      break;
    case 0:
      showMainMenu();
      break;
    default:
      Serial.println("❌ Invalid option. Enter 0-9");
      break;
  }
}

void showFilterMenu() {
  Serial.println("\n╔═══════════════════════════════════════╗");
  Serial.println("║          🔧 SENSOR FILTERS 🔧         ║");
  Serial.println("╠═══════════════════════════════════════╣");
  Serial.println("║  1️⃣  - Toggle Pressure Filter         ║");
  Serial.println("║  2️⃣  - Toggle Acceleration Filter     ║");
  Serial.println("║  3️⃣  - Toggle Gyroscope Filter        ║");
  Serial.println("║  4️⃣  - Toggle Magnetometer Filter     ║");
  Serial.println("║  5️⃣  - Toggle GPS Filter              ║");
  Serial.println("║  6️⃣  - Show Current Filter Status     ║");
  Serial.println("║  0️⃣  - Back to Config Menu            ║");
  Serial.println("╚═══════════════════════════════════════╝");
  Serial.println("🔧 Enter filter option:");
  
  // Handle filter menu inline
  while (!Serial.available()) { delay(10); yield(); }
  String input = Serial.readStringUntil('\n');
  input.trim();
  int filterChoice = input.toInt();
  
  switch (filterChoice) {
    case 1:
      config.pressureFilter = !config.pressureFilter;
      Serial.println("✅ Pressure filter: " + String(config.pressureFilter ? "ENABLED" : "DISABLED"));
      break;
    case 2:
      config.accelFilter = !config.accelFilter;
      Serial.println("✅ Acceleration filter: " + String(config.accelFilter ? "ENABLED" : "DISABLED"));
      break;
    case 3:
      config.gyroFilter = !config.gyroFilter;
      Serial.println("✅ Gyroscope filter: " + String(config.gyroFilter ? "ENABLED" : "DISABLED"));
      break;
    case 4:
      config.magFilter = !config.magFilter;
      Serial.println("✅ Magnetometer filter: " + String(config.magFilter ? "ENABLED" : "DISABLED"));
      break;
    case 5:
      config.gpsFilter = !config.gpsFilter;
      Serial.println("✅ GPS filter: " + String(config.gpsFilter ? "ENABLED" : "DISABLED"));
      break;
    case 6:
      Serial.println("\n� Current Filter Settings:");
      Serial.println("   • Pressure: " + String(config.pressureFilter ? "ON" : "OFF"));
      Serial.println("   • Accelerometer: " + String(config.accelFilter ? "ON" : "OFF"));
      Serial.println("   • Gyroscope: " + String(config.gyroFilter ? "ON" : "OFF"));
      Serial.println("   • Magnetometer: " + String(config.magFilter ? "ON" : "OFF"));
      Serial.println("   • GPS: " + String(config.gpsFilter ? "ON" : "OFF"));
      break;
    case 0:
      showConfigMenu();
      return;
    default:
      Serial.println("❌ Invalid option");
      break;
  }
  Serial.println("💡 Use option 9 to send complete configuration to TX");
  delay(1500);
  showConfigMenu();
}

// === FLIGHT OPERATIONS MENU ===
void showFlightMenu() {
  currentMenuLevel = MENU_FLIGHT;
  Serial.println("\n╔═══════════════════════════════════════╗");
  Serial.println("║       🚀 FLIGHT OPERATIONS 🚀        ║");
  Serial.println("╠═══════════════════════════════════════╣");
  Serial.println("║  1️⃣  - Start Data Logging            ║");
  Serial.println("║  2️⃣  - Stop Data Logging             ║");
  Serial.println("║  3️⃣  - Fire Pyro 1                   ║");
  Serial.println("║  4️⃣  - Fire Pyro 2                   ║");
  Serial.println("║  5️⃣  - Fire Pyro 3                   ║");
  Serial.println("║  6️⃣  - Fire Pyro 4                   ║");
  Serial.println("║  7️⃣  - Reset Altitude Offset         ║");
  Serial.println("║  8️⃣  - TX System Status              ║");
  Serial.println("║  0️⃣  - Back to Main Menu              ║");
  Serial.println("╚═══════════════════════════════════════╝");
  Serial.println("🚀 Enter flight operation:");
}

void handleFlightMenu(int choice) {
  switch (choice) {
    case 1:
      sendLoRaCommand("START_LOGGING");
      Serial.println("📡 Starting data logging on TX...");
      Serial.println("📊 TX will begin collecting sensor data");
      break;
    case 2:
      sendLoRaCommand("STOP_LOGGING");
      Serial.println("📡 Stopping data logging on TX...");
      break;
    case 3:
      sendLoRaCommand("PYRO1");
      Serial.println("📡 🔥 FIRING PYRO 1");
      break;
    case 4:
      sendLoRaCommand("PYRO2");
      Serial.println("� 🔥 FIRING PYRO 2");
      break;
    case 5:
      sendLoRaCommand("PYRO3");
      Serial.println("📡 🔥 FIRING PYRO 3");
      break;
    case 6:
      sendLoRaCommand("PYRO4");
      Serial.println("📡 🔥 FIRING PYRO 4");
      break;
    case 7:
      sendLoRaCommand("SENSOR_RESET");
      Serial.println("📡 Resetting altitude offset and sensor zeros...");
      break;
    case 8:
      sendLoRaCommand("STATUS");
      Serial.println("📡 Requesting TX system status...");
      break;
    case 0:
      showMainMenu();
      break;
    default:
      Serial.println("❌ Invalid option. Enter 0-8");
      break;
  }
}

// === DATA MANAGEMENT MENU ===
void showDataMenu() {
  currentMenuLevel = MENU_DATA;
  Serial.println("\n╔═══════════════════════════════════════╗");
  Serial.println("║       📊 DATA MANAGEMENT 📊          ║");
  Serial.println("╠═══════════════════════════════════════╣");
  Serial.println("║  1️⃣  - Start WiFi Hotspot            ║");
  Serial.println("║  2️⃣  - Stop WiFi Hotspot             ║");
  Serial.println("║  3️⃣  - WiFi Status                   ║");
  Serial.println("║  4️⃣  - Show Connection Info          ║");
  Serial.println("║  0️⃣  - Back to Main Menu              ║");
  Serial.println("╚═══════════════════════════════════════╝");
  Serial.println("🌐 WiFi Hotspot: PaviFlightData / PaviFlightData-2");
  Serial.println("📱 URL: http://192.168.4.1");
  Serial.println("📁 Enter data management option:");
}

void handleDataMenu(int choice) {
  switch (choice) {
    case 1:
      sendLoRaCommand("START_WIFI");
      Serial.println("📡 Starting WiFi hotspot on TX...");
      Serial.println("🌐 TX will create WiFi hotspot for data download");
      Serial.println("� Connect to: PaviFlightData or PaviFlightData-2");
      Serial.println("🌐 Then visit: http://192.168.4.1");
      break;
    case 2:
      sendLoRaCommand("STOP_WIFI");
      Serial.println("📡 Stopping WiFi hotspot on TX...");
      break;
    case 3:
      sendLoRaCommand("WIFI_STATUS");
      Serial.println("📡 Requesting WiFi status from TX...");
      break;
    case 4:
      Serial.println("📱 Connection Information:");
      Serial.println("   🌐 SSID: PaviFlightData or PaviFlightData-2");
      Serial.println("   🔐 Password: (Open network)");
      Serial.println("   � URL: http://192.168.4.1");
      Serial.println("   📁 Access files through web browser");
      break;
    case 0:
      showMainMenu();
      break;
    default:
      Serial.println("❌ Invalid option. Enter 0-4");
      break;
  }
}

void showLocalFileMenu() {
  Serial.println("\n📁 Local RX File Management:");
  Serial.println("1️⃣ - List local files");
  Serial.println("2️⃣ - Delete local file");
  Serial.println("0️⃣ - Back to data menu");
  
  while (!Serial.available()) { delay(10); yield(); }
  String input = Serial.readStringUntil('\n');
  input.trim();
  int choice = input.toInt();
  
  switch (choice) {
    case 1:
      listLocalFiles();
      break;
    case 2:
      Serial.println("Enter filename to delete:");
      while (!Serial.available()) { delay(10); yield(); }
      input = Serial.readStringUntil('\n');
      input.trim();
      deleteLocalFile(input);
      break;
    case 0:
      showDataMenu();
      return;
    default:
      Serial.println("❌ Invalid option");
      break;
  }
  showDataMenu();
}

// === CALIBRATION MENU ===
void showCalibrationMenu() {
  currentMenuLevel = MENU_CALIBRATION;
  Serial.println("\n╔═══════════════════════════════════════╗");
  Serial.println("║        🎯 LOAD CELL CALIBRATION 🎯   ║");
  Serial.println("╠═══════════════════════════════════════╣");
  Serial.println("║  1️⃣  - Tare Load Cell (Zero)         ║");
  Serial.println("║  2️⃣  - Calibrate with Known Weight    ║");
  Serial.println("║  3️⃣  - Save Calibration               ║");
  Serial.println("║  4️⃣  - Test Current Calibration      ║");
  Serial.println("║  0️⃣  - Back to Main Menu              ║");
  Serial.println("╚═══════════════════════════════════════╝");
  Serial.println("⚖️ Enter calibration option:");
}

void handleCalibrationMenu(int choice) {
  switch (choice) {
    case 1:
      sendLoRaCommand("CAL_TARE");
      Serial.println("📡 Taring load cell (setting zero point)...");
      Serial.println("⚖️ Ensure no weight is on the load cell");
      break;
    case 2:
      Serial.println("⚖️ Enter known calibration weight in grams:");
      waitForCalInput("CAL_WEIGHT");
      break;
    case 3:
      sendLoRaCommand("CAL_SAVE");
      Serial.println("📡 Saving load cell calibration...");
      break;
    case 4:
      sendLoRaCommand("CAL_STATUS");
      Serial.println("📡 Testing current calibration...");
      break;
    case 0:
      showMainMenu();
      break;
    default:
      Serial.println("❌ Invalid option. Enter 0-4");
      break;
  }
}

// === WIFI & FILE MENU ===
void showWiFiMenu() {
  currentMenuLevel = MENU_WIFI;
  Serial.println("\n╔═══════════════════════════════════════╗");
  Serial.println("║        🌐 TX WIFI & FILES 🌐         ║");
  Serial.println("╠═══════════════════════════════════════╣");
  Serial.println("║  1️⃣  - Start WiFi Hotspot            ║");
  Serial.println("║  2️⃣  - Stop WiFi Hotspot             ║");
  Serial.println("║  3️⃣  - Show WiFi Status              ║");
  Serial.println("║  4️⃣  - Get Download URL              ║");
  Serial.println("║  5️⃣  - WiFi File Browser             ║");
  Serial.println("║  6️⃣  - Restart WiFi Service          ║");
  Serial.println("║  0️⃣  - Back to Main Menu              ║");
  Serial.println("╚═══════════════════════════════════════╝");
  Serial.println("🌐 Enter WiFi option:");
}

void handleWiFiMenu(int choice) {
  switch (choice) {
    case 1:
      sendLoRaCommand("START_WIFI");
      Serial.println("📡 Starting WiFi hotspot on TX...");
      Serial.println("🌐 SSID: PAVI_TX");
      Serial.println("🔐 Password: pavi2024");
      Serial.println("📱 Connect and visit: http://192.168.4.1");
      break;
    case 2:
      sendLoRaCommand("STOP_WIFI");
      Serial.println("📡 Stopping WiFi hotspot on TX...");
      break;
    case 3:
      sendLoRaCommand("WIFI_STATUS");
      Serial.println("📡 Requesting WiFi status from TX...");
      break;
    case 4:
      Serial.println("🌐 TX Download URL: http://192.168.4.1");
      Serial.println("📱 Connect to PAVI_TX WiFi first");
      break;
    case 5:
      sendLoRaCommand("WIFI_FILES");
      Serial.println("📡 Requesting WiFi file browser status...");
      break;
    case 6:
      sendLoRaCommand("RESTART_WIFI");
      Serial.println("📡 Restarting WiFi service on TX...");
      break;
    case 0:
      showMainMenu();
      break;
    default:
      Serial.println("❌ Invalid option. Enter 0-6");
      break;
  }
}

// === LORA COMMUNICATION ===
void handleLoRaReceive() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();
    }
    
    // Log received data with RSSI
    int rssi = LoRa.packetRssi();
    Serial.println("📡 RX << " + received + " (RSSI: " + String(rssi) + "dBm)");
    
    // Process received data
    processReceivedData(received);
    
    yield(); // ESP32-C3 stability
  }
}

void processReceivedData(String data) {
  // Handle different types of responses from TX
  if (data.startsWith("STATUS:")) {
    Serial.println("📊 TX Status: " + data.substring(7));
  } else if (data.startsWith("CONFIG:")) {
    Serial.println("⚙️ TX Config: " + data.substring(7));
  } else if (data.startsWith("FILES:")) {
    Serial.println("📁 TX Files: " + data.substring(6));
  } else if (data.startsWith("CAL:")) {
    Serial.println("🎯 Calibration: " + data.substring(4));
  } else if (data.startsWith("WIFI:")) {
    Serial.println("🌐 WiFi Status: " + data.substring(5));
  } else if (data.startsWith("ERROR:")) {
    Serial.println("❌ TX Error: " + data.substring(6));
  } else if (data.startsWith("OK:")) {
    Serial.println("✅ TX Response: " + data.substring(3));
  } else {
    // Generic sensor data or unknown response
    Serial.println("📊 TX Data: " + data);
  }
  
  // Log to SD card if it's sensor data
  if (data.indexOf(",") > 0 && !data.startsWith("STATUS") && !data.startsWith("CONFIG")) {
    logDataToSD(data);
  }
}

void logDataToSD(String data) {
  static String currentLogFile = "";
  
  // Create new log file if needed
  if (currentLogFile == "") {
    currentLogFile = "/RX_LOG_" + String(millis()) + ".csv";
    File file = SD.open(currentLogFile, FILE_WRITE);
    if (file) {
      file.println("timestamp,rssi,data");
      file.close();
      Serial.println("📝 Created log file: " + currentLogFile);
    }
  }
  
  // Append data
  File file = SD.open(currentLogFile, FILE_APPEND);
  if (file) {
    String logEntry = String(millis()) + "," + String(LoRa.packetRssi()) + "," + data;
    file.println(logEntry);
    file.close();
  }
}

// === HELPER FUNCTIONS ===
void waitForConfigInput(String type) {
  Serial.println("(Enter value, or 'back' to return to menu)");
  while (!Serial.available()) { delay(10); yield(); }
  String input = Serial.readStringUntil('\n');
  input.trim();
  
  if (input.equalsIgnoreCase("back")) {
    showConfigMenu();
    return;
  }
  
  // Store locally instead of sending to TX immediately
  if (type == "FILENAME") {
    config.filename = input;
    Serial.println("✅ Filename stored locally: " + config.filename);
  } else if (type == "WEIGHT") {
    config.totalWeight = input.toFloat();
    Serial.println("✅ Weight stored locally: " + String(config.totalWeight) + "g");
  } else if (type == "WIND") {
    config.windSpeed = input.toFloat();
    Serial.println("✅ Wind speed stored locally: " + String(config.windSpeed) + " m/s");
  } else if (type == "HEIGHT") {
    config.height = input.toFloat();
    Serial.println("✅ Height stored locally: " + String(config.height) + "m");
  }
  
  Serial.println("💡 Use option 9 to send complete configuration to TX");
  delay(1500);
  showConfigMenu();
}

void waitForDataInput(String type) {
  Serial.println("(Enter value, or 'back' to return to menu)");
  while (!Serial.available()) { delay(10); yield(); }
  String input = Serial.readStringUntil('\n');
  input.trim();
  
  if (input.equalsIgnoreCase("back")) {
    showDataMenu();
    return;
  }
  
  if (type == "FORMAT_CONFIRM") {
    if (input == "CONFIRM") {
      sendLoRaCommand("FORMAT_SD");
      Serial.println("📡 Formatting TX SD card...");
    } else {
      Serial.println("❌ Format cancelled");
    }
  } else {
    String command = type + ":" + input;
    sendLoRaCommand(command);
    Serial.println("📡 Processing " + input + "...");
  }
  showDataMenu();
}

void waitForCalInput(String type) {
  Serial.println("(Enter value, or 'back' to return to menu)");
  while (!Serial.available()) { delay(10); yield(); }
  String input = Serial.readStringUntil('\n');
  input.trim();
  
  if (input.equalsIgnoreCase("back")) {
    showCalibrationMenu();
    return;
  }
  
  if (type == "CAL_WEIGHT") {
    String command = "CAL_WEIGHT:" + input;
    sendLoRaCommand(command);
    Serial.println("📡 Calibrating with " + input + "g weight...");
    Serial.println("⚖️ Place the exact weight on the load cell");
  } else {
    String command = type + ":" + input;
    sendLoRaCommand(command);
    Serial.println("📡 Setting " + type + " = " + input);
  }
  showCalibrationMenu();
}

void showSystemStatus() {
  Serial.println("\n╔═══════════════════════════════════════╗");
  Serial.println("║           📊 SYSTEM STATUS 📊         ║");
  Serial.println("╚═══════════════════════════════════════╝");
  
  // Local RX status
  Serial.println("📡 RX Ground Station Status:");
  Serial.println("   • LoRa: Connected (433MHz)");
  if (ESP32_C3_MODE) {
    Serial.println("   • ESP32-C3 Mode: ENABLED");
    Serial.println("   • Free Heap: " + String(ESP.getFreeHeap()) + " bytes");
  }
  Serial.println("   • SD Card: " + String(SD.exists("/") ? "Ready" : "Error"));
  
  // Request TX status
  Serial.println("\n📡 Requesting TX status...");
  sendLoRaCommand("FULL_STATUS");
  
  Serial.println("\nPress any key to return to main menu...");
  while (!Serial.available()) { delay(10); yield(); }
  Serial.read();
  showMainMenu();
}

void showESP32Diagnostics() {
  if (!ESP32_C3_MODE) return;
  
  Serial.println("\n╔═══════════════════════════════════════╗");
  Serial.println("║        🔧 ESP32-C3 DIAGNOSTICS 🔧    ║");
  Serial.println("╚═══════════════════════════════════════╝");
  
  Serial.println("💾 Memory Status:");
  Serial.println("   • Free Heap: " + String(ESP.getFreeHeap()) + " bytes");
  Serial.println("   • Heap Size: " + String(ESP.getHeapSize()) + " bytes");
  Serial.println("   • CPU Frequency: " + String(getCpuFrequencyMhz()) + " MHz");
  
  Serial.println("\n🔄 Reset Information:");
  Serial.println("   • Reset Reason: Available via ESP32 API");
  
  Serial.println("\n📡 LoRa Status:");
  Serial.println("   • TX Power: 2dBm (minimum for stability)");
  Serial.println("   • Frequency: 433MHz");
  Serial.println("   • Spreading Factor: 12 (max reliability)");
  
  esp32c3StatusCheck();
  
  Serial.println("\nPress any key to return to main menu...");
  while (!Serial.available()) { delay(10); yield(); }
  Serial.read();
  showMainMenu();
}

void showHelpMenu() {
  Serial.println("\n╔═══════════════════════════════════════╗");
  Serial.println("║            📖 HELP & ABOUT 📖         ║");
  Serial.println("╚═══════════════════════════════════════╝");
  
  Serial.println("🚀 PAVI Ground Station RX v2.0");
  Serial.println("   Optimized Remote TX Control Interface");
  Serial.println("");
  Serial.println("📡 How it works:");
  Serial.println("   • RX sends commands to TX via LoRa");
  Serial.println("   • TX responds with status/data");
  Serial.println("   • All operations are menu-driven");
  Serial.println("");
  Serial.println("🎛️ Menu Navigation:");
  Serial.println("   • Enter numbers to select options");
  Serial.println("   • Type '99' anytime for main menu");
  Serial.println("   • Type 'back' in input prompts to return");
  Serial.println("");
  Serial.println("⚠️ Important Notes:");
  Serial.println("   • Ensure TX is powered and in range");
  Serial.println("   • WiFi downloads require close proximity");
  Serial.println("   • Calibrations need TX to be stable");
  
  Serial.println("\nPress any key to return to main menu...");
  while (!Serial.available()) { delay(10); yield(); }
  Serial.read();
  showMainMenu();
}

void sendLoRaCommand(String command) {
  if (ESP32_C3_MODE) {
    // ESP32-C3 ultra-conservative transmission
    yield();
    LoRa.beginPacket();
    yield();
    LoRa.print(command);
    yield();
    LoRa.endPacket();
    yield();
    Serial.println("📡 Sent: " + command);
    delay(100); // Short delay for ESP32-C3 stability
  } else {
    LoRa.beginPacket();
    LoRa.print(command);
    LoRa.endPacket();
    Serial.println("📡 Sent: " + command);
  }
  
  // Resume receiving
  LoRa.receive();
}

void listLocalFiles() {
  Serial.println("\n📁 Local RX Files:");
  File root = SD.open("/");
  if (!root) {
    Serial.println("❌ Failed to open SD card");
    return;
  }
  
  int fileCount = 0;
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    
    if (!entry.isDirectory()) {
      Serial.println("   📄 " + String(entry.name()) + " (" + String(entry.size()) + " bytes)");
      fileCount++;
    }
    entry.close();
  }
  
  if (fileCount == 0) {
    Serial.println("   (No files found)");
  }
  root.close();
}

void deleteLocalFile(String filename) {
  if (SD.exists(filename)) {
    if (SD.remove(filename)) {
      Serial.println("✅ Deleted: " + filename);
    } else {
      Serial.println("❌ Failed to delete: " + filename);
    }
  } else {
    Serial.println("❌ File not found: " + filename);
  }
}

// === MISSING FUNCTIONS ===
void printHelp() {
  Serial.println("\n=== PAVI GROUND STATION ===");
  Serial.println("Current Mode: CONFIG");
  Serial.println("\nGlobal Commands:");
  Serial.println("help/h     - Show this help");
  Serial.println("menu/m     - Enter menu mode"); 
  Serial.println("status/s   - Show system status");
  Serial.println("mode       - Show current mode");
  if (ESP32_C3_MODE) {
    Serial.println("reset      - Emergency LoRa reset (ESP32-C3 only)");
    Serial.println("check      - ESP32-C3 status check");
  }
  Serial.println();
}

void printStatus() {
  Serial.println("\n=== GROUND STATION STATUS ===");
  Serial.println("Connection: " + connectionStatus);
  Serial.println("Last Response: " + lastResponse);
  Serial.println("RSSI: " + String(telemetry.rssi) + " dBm");
  Serial.println("Packets - TX: " + String(commandCount) + ", RX: " + String(telemetryCount));
  Serial.println("Config Status: " + String(config.isComplete() ? "Complete" : "Incomplete"));
  
  if (ESP32_C3_MODE) {
    Serial.println("Free Heap: " + String(ESP.getFreeHeap()) + " bytes");
  }
}

void printCurrentMode() {
  Serial.println("Current mode: CONFIG");
}

void switchMode(String modeName) {
  Serial.println("Mode switching disabled in menu system");
}

void showConfig() {
  Serial.println("\n=== CURRENT TEST CONFIGURATION ===");
  Serial.print("Filename: ");
  if (config.filename.isEmpty()) {
    Serial.println("NOT SET");
  } else {
    Serial.println(config.filename);
  }
  
  Serial.print("Total Weight: ");
  if (config.totalWeight <= 0) {
    Serial.println("NOT SET");
  } else {
    Serial.println(String(config.totalWeight) + "g");
  }
  
  Serial.print("Wind Speed: ");
  if (config.windSpeed < 0) {
    Serial.println("NOT SET");
  } else {
    Serial.println(String(config.windSpeed) + " m/s");
  }
  
  Serial.print("Height: ");
  if (config.height <= 0) {
    Serial.println("NOT SET");
  } else {
    Serial.println(String(config.height) + "m");
  }
  
  Serial.print("Status: ");
  if (config.isComplete()) {
    Serial.println("COMPLETE");
  } else {
    Serial.println("INCOMPLETE");
  }
  Serial.println("=====================================\n");
}

void sendConfig() {
  if (!config.isComplete()) {
    Serial.println("ERROR: Configuration incomplete!");
    showConfig();
    return;
  }
  
  String configData = "CONFIG:" + config.filename + "," + 
                     String((int)config.totalWeight) + "," +
                     String((int)config.windSpeed) + "," +
                     String((int)config.height);
  
  sendLoRaCommand(configData);
  Serial.println("Configuration sent to TX");
  configSent = true;
}

void clearConfig() {
  config.filename = "";
  config.totalWeight = -1;
  config.windSpeed = -1;
  config.height = -1;
  configSent = false;
  Serial.println("Configuration cleared");
}

void printMainMenu() {
  showMainMenu();
}

void checkMemory() {
  if (ESP32_C3_MODE) {
    Serial.println("Free heap: " + String(ESP.getFreeHeap()) + " bytes");
  }
}

void emergencyLoRaReset() {
  if (ESP32_C3_MODE) {
    Serial.println("🚨 EMERGENCY: LoRa module reset attempt");
    
    // Hardware reset sequence with watchdog feeding
    digitalWrite(RST, LOW);
    yield();
    delay(50);
    yield();
    digitalWrite(RST, HIGH);
    yield();
    delay(200);
    yield();
    
    // Re-initialize with minimal settings
    if (LoRa.begin(433E6)) {
      yield();
      LoRa.setTxPower(2);  // Absolute minimum power
      yield();
      LoRa.setSpreadingFactor(12);
      yield();
      LoRa.enableCrc();
      yield();
      Serial.println("✅ Emergency LoRa reset successful");
    } else {
      Serial.println("❌ Emergency LoRa reset failed");
    }
    yield();
  }
}

void esp32c3StatusCheck() {
  if (ESP32_C3_MODE) {
    yield(); // Feed watchdog at start
    
    unsigned long freeHeap = ESP.getFreeHeap();
    if (freeHeap < 200000) {  // Low memory warning
      Serial.println("⚠️ ESP32-C3 LOW MEMORY WARNING: " + String(freeHeap) + " bytes");
    }
    
    yield(); // Feed watchdog mid-function
    
    // Check for any stuck states - but be less aggressive
    static unsigned long lastLoopTime = 0;
    unsigned long now = millis();
    if (now - lastLoopTime > 10000) {  // Increased threshold to 10s
      Serial.println("⚠️ ESP32-C3 SLOW LOOP DETECTED");
      lastLoopTime = now; // Reset the timer to avoid spam
    } else if (lastLoopTime == 0) {
      lastLoopTime = now; // Initialize on first run
    }
    
    yield(); // Feed watchdog at end
  }
}

void handleParameterMenu(int choice, String input) {
  // Placeholder function - redirects to config menu
  Serial.println("Parameter menu not implemented - use config menu instead");
  showConfigMenu();
}

// === MAIN MENU FUNCTION ===
void showMainMenu() {
  currentMenuLevel = MENU_MAIN;
  Serial.println("\n╔═══════════════════════════════════════╗");
  Serial.println("║     🚀 PAVI GROUND STATION RX 🚀      ║");
  Serial.println("║      Remote TX Control Interface      ║");
  Serial.println("╠═══════════════════════════════════════╣");
  Serial.println("║  1️⃣  - TX Configuration Menu (Local)  ║");
  Serial.println("║  2️⃣  - Flight Operations Menu         ║");
  Serial.println("║  3️⃣  - Data Management Menu           ║");
  Serial.println("║  4️⃣  - TX Calibration Menu            ║");
  Serial.println("║  5️⃣  - TX WiFi & File Menu            ║");
  Serial.println("║  6️⃣  - System Status                  ║");
  if (ESP32_C3_MODE) {
    Serial.println("║  7️⃣  - ESP32-C3 Diagnostics           ║");
  }
  Serial.println("║  0️⃣  - Help & About                   ║");
  Serial.println("╚═══════════════════════════════════════╝");
  Serial.println("📡 Config Status: " + String(config.isComplete() ? "✅ Ready to Send" : "⚠️ Incomplete"));
  Serial.println("📡 Enter menu number to control TX remotely:");
}

// === NEW CONFIGURATION FUNCTIONS ===
void setAllParameters() {
  Serial.println("\n╔═══════════════════════════════════════╗");
  Serial.println("║     🔧 GUIDED CONFIGURATION SETUP 🔧  ║");
  Serial.println("║         (Type 'skip' to keep current) ║");
  Serial.println("╚═══════════════════════════════════════╝");
  
  // Filename
  Serial.println("\n📝 Step 1/4: Data Filename");
  Serial.println("Current: " + String(config.filename != "" ? config.filename : "Not set"));
  Serial.println("Enter filename (without extension):");
  while (!Serial.available()) { delay(10); yield(); }
  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input != "skip" && input != "") {
    config.filename = input;
    Serial.println("✅ Filename set: " + config.filename);
  } else if (input == "skip") {
    Serial.println("⏭️ Filename unchanged");
  }
  
  // Weight
  Serial.println("\n⚖️ Step 2/4: Total Weight");
  Serial.println("Current: " + String(config.totalWeight > 0 ? String(config.totalWeight) + "g" : "Not set"));
  Serial.println("Enter total weight in grams:");
  while (!Serial.available()) { delay(10); yield(); }
  input = Serial.readStringUntil('\n');
  input.trim();
  if (input != "skip" && input != "") {
    config.totalWeight = input.toFloat();
    Serial.println("✅ Weight set: " + String(config.totalWeight) + "g");
  } else if (input == "skip") {
    Serial.println("⏭️ Weight unchanged");
  }
  
  // Wind Speed
  Serial.println("\n💨 Step 3/4: Wind Speed");
  Serial.println("Current: " + String(config.windSpeed >= 0 ? String(config.windSpeed) + " m/s" : "Not set"));
  Serial.println("Enter wind speed in m/s:");
  while (!Serial.available()) { delay(10); yield(); }
  input = Serial.readStringUntil('\n');
  input.trim();
  if (input != "skip" && input != "") {
    config.windSpeed = input.toFloat();
    Serial.println("✅ Wind speed set: " + String(config.windSpeed) + " m/s");
  } else if (input == "skip") {
    Serial.println("⏭️ Wind speed unchanged");
  }
  
  // Height
  Serial.println("\n📏 Step 4/4: Drop Height");
  Serial.println("Current: " + String(config.height > 0 ? String(config.height) + "m" : "Not set"));
  Serial.println("Enter drop height in meters:");
  while (!Serial.available()) { delay(10); yield(); }
  input = Serial.readStringUntil('\n');
  input.trim();
  if (input != "skip" && input != "") {
    config.height = input.toFloat();
    Serial.println("✅ Height set: " + String(config.height) + "m");
  } else if (input == "skip") {
    Serial.println("⏭️ Height unchanged");
  }
  
  Serial.println("\n🎉 Configuration setup complete!");
  Serial.println("📋 Review your settings with option 7, then send with option 8");
  
  delay(2000);
  showConfigMenu();
}

void showLocalConfig() {
  Serial.println("\n╔═══════════════════════════════════════╗");
  Serial.println("║        📋 LOCAL CONFIGURATION 📋     ║");
  Serial.println("╚═══════════════════════════════════════╝");
  
  Serial.println("📄 Filename: " + String(config.filename != "" ? config.filename : "❌ Not set"));
  Serial.println("⚖️ Total Weight: " + String(config.totalWeight > 0 ? String(config.totalWeight) + "g" : "❌ Not set"));
  Serial.println("💨 Wind Speed: " + String(config.windSpeed >= 0 ? String(config.windSpeed) + " m/s" : "❌ Not set"));
  Serial.println("📏 Drop Height: " + String(config.height > 0 ? String(config.height) + "m" : "❌ Not set"));
  
  Serial.println("\n🔧 Sensor Filters:");
  Serial.println("   • Pressure: " + String(config.pressureFilter ? "ON" : "OFF"));
  Serial.println("   • Accelerometer: " + String(config.accelFilter ? "ON" : "OFF"));
  Serial.println("   • Gyroscope: " + String(config.gyroFilter ? "ON" : "OFF"));
  Serial.println("   • Magnetometer: " + String(config.magFilter ? "ON" : "OFF"));
  Serial.println("   • GPS: " + String(config.gpsFilter ? "ON" : "OFF"));
  
  Serial.println(String("\n") + String(config.isComplete() ? "✅ Configuration is COMPLETE - ready to send!" : "⚠️ Configuration is INCOMPLETE"));
  
  Serial.println("\nPress any key to return to config menu...");
  while (!Serial.available()) { delay(10); yield(); }
  Serial.read();
  showConfigMenu();
}

void sendConfigToTX() {
  if (!config.isComplete()) {
    Serial.println("❌ Cannot send incomplete configuration!");
    Serial.println("Missing: ");
    if (config.filename == "") Serial.println("   • Filename");
    if (config.totalWeight <= 0) Serial.println("   • Total Weight");
    if (config.windSpeed < 0) Serial.println("   • Wind Speed");  
    if (config.height <= 0) Serial.println("   • Drop Height");
    
    Serial.println("\nPress any key to return to config menu...");
    while (!Serial.available()) { delay(10); yield(); }
    Serial.read();
    showConfigMenu();
    return;
  }
  
  // Send combined config packet to TX
  String configPacket = "CONFIG:" + config.filename + "," + 
                        String(config.totalWeight, 2) + "," + 
                        String(config.windSpeed, 2) + "," + 
                        String(config.height, 2);
  
  sendLoRaCommand(configPacket);
  
  Serial.println("\n📡 SENDING COMPLETE CONFIGURATION TO TX");
  Serial.println("╔═══════════════════════════════════════╗");
  Serial.println("║           📦 CONFIG PACKET 📦         ║");
  Serial.println("╠═══════════════════════════════════════╣");
  Serial.println("║ 📄 Filename: " + config.filename);
  Serial.println("║ ⚖️ Weight: " + String(config.totalWeight) + "g");
  Serial.println("║ 💨 Wind: " + String(config.windSpeed) + " m/s");
  Serial.println("║ 📏 Height: " + String(config.height) + "m");
  Serial.println("╚═══════════════════════════════════════╝");
  Serial.println("✅ Configuration sent! TX should confirm receipt.");
  
  // Also send filter settings if any are enabled
  if (config.pressureFilter || config.accelFilter || config.gyroFilter || config.magFilter || config.gpsFilter) {
    Serial.println("\n📡 Sending filter settings...");
    if (config.pressureFilter) sendLoRaCommand("ENABLE_PRESSURE_FILTER");
    if (config.accelFilter) sendLoRaCommand("ENABLE_ACCEL_FILTER");
    if (config.gyroFilter) sendLoRaCommand("ENABLE_GYRO_FILTER");
    if (config.magFilter) sendLoRaCommand("ENABLE_MAG_FILTER");
    if (config.gpsFilter) sendLoRaCommand("ENABLE_GPS_FILTER");
  }
  
  delay(2000);
  showConfigMenu();
}
