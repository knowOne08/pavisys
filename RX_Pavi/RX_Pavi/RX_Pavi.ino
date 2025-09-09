#include <SPI.h>
#include <LoRa.h>

// Pin mapping for your schematic
#define NSS   D7   // IO7
#define RST   D1  // IO2
#define DIO0  D0  // IO3

// Test mode - set to true to test serial only, false for full LoRa operation
#define SERIAL_TEST_MODE false

// LoRa Communication States  
enum LoRaState {
  LORA_NORMAL_MODE,     // RX receives telemetry, TX sends
  LORA_COMMAND_MODE     // RX sends commands, TX listens  
};
LoRaState loraState = LORA_NORMAL_MODE;
unsigned long lastStateChange = 0;
#define COMMAND_MODE_DURATION 3000  // 3 seconds for command mode
#define NORMAL_MODE_DURATION 10000  // 10 seconds for normal mode
String pendingCommand = "";         // Command waiting to be sent

// === PARACHUTE TEST GROUND STATION ===
// Ground station operating modes
enum GroundStationMode {
  MODE_TELEMETRY,      // Receiving telemetry data
  MODE_COMMAND,        // Sending commands to flight computer
  MODE_MONITOR         // Monitoring flight status
};

// System state variables  
GroundStationMode currentMode = MODE_TELEMETRY;
unsigned long lastTelemetryTime = 0;
unsigned long lastCommandTime = 0;
int telemetryPacketCount = 0;
int commandPacketCount = 0;
bool flightComputerConnected = false;
String commandBuffer = "";

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
  
  // Add a delay to ensure serial is ready
  delay(2000);
  
  Serial.println("=== PARACHUTE TEST GROUND STATION ===");
  Serial.println("Flight Computer Remote Control & Telemetry System");
  Serial.println("Serial communication established");
  
  if (SERIAL_TEST_MODE) {
    Serial.println("RUNNING IN SERIAL TEST MODE");
    Serial.println("LoRa initialization SKIPPED for testing");
    Serial.println("If you see this, serial communication works!");
    Serial.println("Change SERIAL_TEST_MODE to false to enable LoRa");
    Serial.println("============================");
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
  Serial.println("Ground station operational");
  
  printGroundStationHelp();
  currentMode = MODE_TELEMETRY;
}

void loop() {
  // Process user commands from serial
  processUserCommands();
  
  // Handle LoRa state machine
  if (!SERIAL_TEST_MODE) {
    handleLoRaStateMachine();
  }
  
  // Display telemetry in monitor mode  
  if (currentMode == MODE_MONITOR) {
    displayTelemetryDashboard();
  }
  
  // Test mode heartbeat
  if (SERIAL_TEST_MODE) {
    static unsigned long lastHeartbeat = 0;
    static int testCount = 0;
    
    if (millis() - lastHeartbeat > 5000) {  // Every 5 seconds
      lastHeartbeat = millis();
      Serial.print("Ground station test mode - heartbeat #");
      Serial.println(testCount++);
    }
  }
}

// === GROUND STATION FUNCTIONS ===

void printGroundStationHelp() {
  Serial.println("\n=== GROUND STATION COMMANDS ===");
  Serial.println("TELEMETRY MODE:");
  Serial.println("  MONITOR - Switch to live telemetry dashboard");
  Serial.println("  QUIET - Switch to quiet telemetry mode");
  Serial.println("");
  Serial.println("FLIGHT COMPUTER COMMANDS:");
  Serial.println("  CMD <command> - Send command to flight computer");
  Serial.println("  STATUS - Request flight computer status");
  Serial.println("  CONFIG - Start flight configuration");
  Serial.println("  START - Start data recording");
  Serial.println("  STOP - Stop data recording"); 
  Serial.println("  FILES - List files on flight computer");
  Serial.println("  DOWNLOAD <filename> - Download file from flight computer");
  Serial.println("");
  Serial.println("PYRO COMMANDS (DANGER - USE WITH CAUTION):");
  Serial.println("  PYRO1, PYRO2, PYRO3, PYRO4 - Fire pyro channels");
  Serial.println("  ARM - Arm pyro system");
  Serial.println("  DISARM - Disarm pyro system");
  Serial.println("");
  Serial.println("SYSTEM COMMANDS:");
  Serial.println("  HELP - Show this help");
  Serial.println("  RSSI - Show signal strength");
  Serial.println("  PING - Test communication");
  Serial.println("  SERIALTOGGLE - Toggle serial data output");
  Serial.println("=====================================\n");
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
  cmd.toUpperCase();
  cmd.trim();
  
  Serial.print("Ground Station Command: "); Serial.println(cmd);
  
  // Ground station mode commands
  if (cmd == "HELP") {
    printGroundStationHelp();
  }
  else if (cmd == "MONITOR") {
    currentMode = MODE_MONITOR;
    Serial.println("Switched to MONITOR mode - Live telemetry dashboard");
  }
  else if (cmd == "QUIET" || cmd == "TELEMETRY") {
    currentMode = MODE_TELEMETRY;
    Serial.println("Switched to TELEMETRY mode - Quiet telemetry logging");
  }
  else if (cmd == "RSSI") {
    showSignalStatus();
  }
  else if (cmd == "PING") {
    sendCommandToFlightComputer("STATUS");
  }
  
  // Direct flight computer commands
  else if (cmd == "STATUS" || cmd == "CONFIG" || cmd == "START" || cmd == "STOP" || 
           cmd == "FILES" || cmd == "SPACE" || cmd == "FORMAT") {
    sendCommandToFlightComputer(cmd);
  }
  else if (cmd.startsWith("DOWNLOAD ")) {
    String filename = cmd.substring(9);
    sendCommandToFlightComputer("DOWNLOAD " + filename);
  }
  else if (cmd.startsWith("DELETE ")) {
    String filename = cmd.substring(7);
    sendCommandToFlightComputer("DELETE " + filename);
  }
  else if (cmd.startsWith("CMD ")) {
    String command = cmd.substring(4);
    sendCommandToFlightComputer(command);
  }
  
  // Pyro commands (high priority)
  else if (cmd.startsWith("PYRO") && cmd.length() == 5) {
    sendPyroCommand(cmd);
  }
  else if (cmd == "ARM") {
    Serial.println("PYRO SYSTEM ARMED - DANGER: Pyro channels can now fire!");
    // You could add additional arming logic here
  }
  else if (cmd == "DISARM") {
    Serial.println("PYRO SYSTEM DISARMED - Safe mode");
    // You could add additional disarming logic here
  }
  
  else {
    Serial.println("ERROR: Unknown command. Type HELP for available commands.");
  }
}

void sendCommandToFlightComputer(String command) {
  if (SERIAL_TEST_MODE) {
    Serial.print("TEST MODE: Would send command: ");
    Serial.println(command);
    return;
  }
  
  Serial.print("Sending to flight computer: ");
  Serial.println(command);
  
  // Send command via LoRa
  LoRa.beginPacket();
  LoRa.print("CMD:" + command);
  LoRa.endPacket();
  
  Serial.print("LoRa packet sent: CMD:");
  Serial.println(command);
  
  commandPacketCount++;
  lastCommandTime = millis();
}

void sendPyroCommand(String pyroCmd) {
  Serial.println("=== PYRO COMMAND WARNING ===");
  Serial.print("About to send: ");
  Serial.println(pyroCmd);
  Serial.println("This will FIRE a pyro channel!");
  Serial.print("Type 'FIRE' to confirm: ");
  
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

void handleLoRaCommunication() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    telemetryPacketCount++;
    
    // Read packet
    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();
    }
    
    // Update connection status
    flightComputerConnected = true;
    lastTelemetryTime = millis();
    
    // Parse telemetry data
    parseTelemetryPacket(received, LoRa.packetRssi());
    
    // Display based on current mode
    if (currentMode == MODE_TELEMETRY) {
      displayTelemetryLine(received, telemetryPacketCount, LoRa.packetRssi());
    }
  }
}

void parseTelemetryPacket(String packet, int rssi) {
  // Update RSSI and timestamp
  telemetry.rssi = rssi;
  telemetry.lastUpdate = millis();
  
  // Parse packet format: H:12.5|V:-2.1|W:2.45|A:0.12,0.05,-0.98|G:1.2,-0.5,0.8|T:23.4|S:2
  int startIndex = 0;
  int delimiterIndex = 0;
  
  while (delimiterIndex != -1) {
    delimiterIndex = packet.indexOf('|', startIndex);
    String segment;
    
    if (delimiterIndex == -1) {
      segment = packet.substring(startIndex);
    } else {
      segment = packet.substring(startIndex, delimiterIndex);
    }
    
    if (segment.length() > 0) {
      parseTelemetrySegment(segment);
    }
    
    startIndex = delimiterIndex + 1;
  }
}

void parseTelemetrySegment(String segment) {
  int colonIndex = segment.indexOf(':');
  if (colonIndex == -1) return;
  
  String type = segment.substring(0, colonIndex);
  String value = segment.substring(colonIndex + 1);
  
  if (type == "H") {
    telemetry.altitude = value.toFloat();
  }
  else if (type == "V") {
    telemetry.verticalVelocity = value.toFloat();
  }
  else if (type == "W") {
    telemetry.weight = value.toFloat();
  }
  else if (type == "T") {
    telemetry.temperature = value.toFloat();
  }
  else if (type == "S") {
    telemetry.flightState = value.toInt();
  }
  else if (type == "A") {
    // Parse accelerometer: x,y,z
    int comma1 = value.indexOf(',');
    int comma2 = value.indexOf(',', comma1 + 1);
    if (comma1 != -1 && comma2 != -1) {
      telemetry.accelX = value.substring(0, comma1).toFloat();
      telemetry.accelY = value.substring(comma1 + 1, comma2).toFloat();
      telemetry.accelZ = value.substring(comma2 + 1).toFloat();
    }
  }
  else if (type == "G") {
    // Parse gyroscope: x,y,z
    int comma1 = value.indexOf(',');
    int comma2 = value.indexOf(',', comma1 + 1);
    if (comma1 != -1 && comma2 != -1) {
      telemetry.gyroX = value.substring(0, comma1).toFloat();
      telemetry.gyroY = value.substring(comma1 + 1, comma2).toFloat();
      telemetry.gyroZ = value.substring(comma2 + 1).toFloat();
    }
  }
}

void displayTelemetryLine(String packet, int count, int rssi) {
  if (!showSerialData) return;
  Serial.print("RX[");
  Serial.print(count);
  Serial.print("] ");
  Serial.print("RSSI:");
  Serial.print(rssi);
  Serial.print(" | ");
  Serial.println(packet);
}

void displayTelemetryDashboard() {
  if (!showSerialData) return;
  static unsigned long lastDisplay = 0;
  
  // Update dashboard every 2 seconds
  if (millis() - lastDisplay > 2000) {
    lastDisplay = millis();
    
    Serial.println("\n=== TELEMETRY DASHBOARD ===");
    
    // Flight state
    Serial.print("Flight State: ");
    switch (telemetry.flightState) {
      case 1: Serial.println("CONFIGURED"); break;
      case 2: Serial.println("RECORDING"); break;
      case 3: Serial.println("STOPPED"); break;
      default: Serial.println("UNKNOWN"); break;
    }
    
    // Connection status
    Serial.print("Connection: ");
    if (flightComputerConnected) {
      unsigned long timeSinceLastPacket = millis() - lastTelemetryTime;
      if (timeSinceLastPacket < 5000) {
        Serial.print("CONNECTED (");
        Serial.print(telemetry.rssi);
        Serial.println(" dBm)");
      } else {
        Serial.println("TIMEOUT");
        flightComputerConnected = false;
      }
    } else {
      Serial.println("DISCONNECTED");
    }
    
    // Primary flight data
    Serial.println("\n--- FLIGHT DATA ---");
    Serial.print("Altitude: ");
    Serial.print(telemetry.altitude, 1);
    Serial.println(" m");
    
    Serial.print("Vertical Velocity: ");
    Serial.print(telemetry.verticalVelocity, 2);
    Serial.println(" m/s");
    
    Serial.print("Weight: ");
    Serial.print(telemetry.weight, 2);
    Serial.println(" kg");
    
    Serial.print("Temperature: ");
    Serial.print(telemetry.temperature, 1);
    Serial.println(" °C");
    
    // IMU data
    Serial.println("\n--- SENSOR DATA ---");
    Serial.print("Acceleration (m/s²): X:");
    Serial.print(telemetry.accelX, 2);
    Serial.print(" Y:");
    Serial.print(telemetry.accelY, 2);
    Serial.print(" Z:");
    Serial.println(telemetry.accelZ, 2);
    
    Serial.print("Gyroscope (rad/s): X:");
    Serial.print(telemetry.gyroX, 3);
    Serial.print(" Y:");
    Serial.print(telemetry.gyroY, 3);
    Serial.print(" Z:");
    Serial.println(telemetry.gyroZ, 3);
    
    // Statistics
    Serial.println("\n--- STATISTICS ---");
    Serial.print("Telemetry Packets: ");
    Serial.println(telemetryPacketCount);
    Serial.print("Commands Sent: ");
    Serial.println(commandPacketCount);
    
    Serial.println("\nType 'QUIET' to exit dashboard, 'HELP' for commands");
    Serial.println("==========================================");
  }
}

void checkConnectionStatus() {
  static bool wasConnected = false;
  
  // Check for connection timeout
  if (flightComputerConnected && (millis() - lastTelemetryTime > 10000)) {
    flightComputerConnected = false;
    if (showSerialData) Serial.println("WARNING: Lost connection to flight computer!");
  }
  
  // Detect new connection
  if (!wasConnected && flightComputerConnected) {
    if (showSerialData) Serial.println("Flight computer connected!");
  }
  
  wasConnected = flightComputerConnected;
}

void showSignalStatus() {
  Serial.println("\n=== SIGNAL STATUS ===");
  Serial.print("Flight Computer: ");
  if (flightComputerConnected) {
    Serial.print("CONNECTED (RSSI: ");
    Serial.print(telemetry.rssi);
    Serial.println(" dBm)");
    
    unsigned long timeSince = (millis() - lastTelemetryTime) / 1000;
    Serial.print("Last packet: ");
    Serial.print(timeSince);
    Serial.println(" seconds ago");
  } else {
    Serial.println("DISCONNECTED");
  }
  
  Serial.print("Packets received: ");
  Serial.println(telemetryPacketCount);
  Serial.print("Commands sent: ");
  Serial.println(commandPacketCount);
  Serial.println("====================\n");
}
