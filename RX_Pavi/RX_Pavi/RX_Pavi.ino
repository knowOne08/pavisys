#include <SPI.h>
#include <LoRa.h>

// Pin mapping for your schematic
#define NSS   D7   // IO7
#define RST   D1  // IO2
#define DIO0  D0  // IO3

// Test mode - set to true to test serial only, false for full LoRa operation
#define SERIAL_TEST_MODE false

// Simple command/response system - no state machine needed
String pendingCommand = "";         // Command to be sent immediately

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
  
  Serial.println("=== RX: GROUND STATION COMMAND SENDER ===");
  Serial.println("Pure command/response system - no continuous telemetry");
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
  
  // Test LoRa module responsiveness
  Serial.print(">>> LoRa module test - Frequency: ");
  // Serial.print(LoRa.getFrequency());
  Serial.println(" Hz");
  Serial.print(">>> Spreading Factor: ");
  // Serial.println(LoRa.getSpreadingFactor());
  Serial.print(">>> Signal Bandwidth: ");
  // Serial.println(LoRa.getSignalBandwidth());
  
  Serial.println("*** RX READY FOR LORA TEST ***");
  Serial.println("*** Type PING to test LoRa transmission ***");
  Serial.println("*** Watch TX serial monitor for reception ***");
  
  printGroundStationHelp();
}

void loop() {
  // Process user commands from serial
  processUserCommands();
  
  // RX is now a pure command sender - no continuous telemetry
  if (!SERIAL_TEST_MODE) {
    handleCommandSending();
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
  Serial.println("\n=== LORA FLIGHT CONTROL COMMANDS ===");
  Serial.println("Ground station sends commands to flight computer");
  Serial.println("No response expected - check TX serial monitor");
  Serial.println("");
  Serial.println("FLIGHT CONTROL:");
  Serial.println("  PING - Test LoRa connection");
  Serial.println("  START - Begin flight configuration");
  Serial.println("  STOP - Stop data recording");
  Serial.println("  RESET - Reset sensor origins");
  Serial.println("  STATUS - Get system status");
  Serial.println("");
  Serial.println("PYRO CHANNELS (USE WITH CAUTION!):");
  Serial.println("  PYRO1, PYRO2, PYRO3, PYRO4 - Fire pyro channels");
  Serial.println("");
  Serial.println("FLIGHT CONFIGURATION (after START):");
  Serial.println("  FILENAME:test1 - Set log filename");
  Serial.println("  WEIGHT:2.5 - Set total weight in kg");
  Serial.println("  WIND:3.2 - Set wind speed in m/s");
  Serial.println("  HEIGHT:100 - Set initial height in m");
  Serial.println("  CONFIRM - Start recording with parameters");
  Serial.println("  CANCEL - Cancel configuration");
  Serial.println("");
  Serial.println("SYSTEM:");
  Serial.println("  HELP - Show this help");
  Serial.println("======================================\n");
  Serial.println("Commands sent immediately when entered");
  Serial.println("Watch TX serial monitor for reception!");
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
  
  // Flight control commands
  if (cmd == "HELP") {
    printGroundStationHelp();
  }
  else if (cmd == "PING") {
    sendCommandToFlightComputer("PING");
  }
  else if (cmd == "START") {
    Serial.println("=== STARTING FLIGHT CONFIGURATION ===");
    Serial.println("Sending START command to TX...");
    Serial.println("After TX responds, send these configuration commands:");
    Serial.println("- FILENAME:your_test_name");
    Serial.println("- WEIGHT:2.5 (kg)");
    Serial.println("- WIND:3.0 (m/s)"); 
    Serial.println("- HEIGHT:50 (meters)");
    Serial.println("- CONFIRM (to start recording)");
    Serial.println("======================================");
    sendCommandToFlightComputer("START");
  }
  else if (cmd == "STOP") {
    sendCommandToFlightComputer("STOP");
  }
  else if (cmd == "RESET") {
    sendCommandToFlightComputer("RESET");
  }
  else if (cmd == "STATUS") {
    sendCommandToFlightComputer("STATUS");
  }
  else if (cmd.startsWith("PYRO")) {
    if (cmd == "PYRO1" || cmd == "PYRO2" || cmd == "PYRO3" || cmd == "PYRO4") {
      sendPyroCommand(cmd);
    } else {
      Serial.println("ERROR: Invalid pyro command. Use PYRO1, PYRO2, PYRO3, or PYRO4");
    }
  }
  else if (cmd.startsWith("FILENAME:") || cmd.startsWith("WEIGHT:") || 
           cmd.startsWith("WIND:") || cmd.startsWith("HEIGHT:") || 
           cmd == "CONFIRM" || cmd == "CANCEL") {
    Serial.println("Sending flight configuration command: " + cmd);
    sendCommandToFlightComputer(cmd);
  }
  // Legacy test commands
  else if (cmd == "HELLO" || cmd == "TEST") {
    sendCommandToFlightComputer(cmd);
  }
  
  // Any other command - just send it as-is for testing
  else {
    Serial.println("Sending custom command: " + cmd);
    sendCommandToFlightComputer(cmd);
  }
}

void sendCommandToFlightComputer(String command) {
  if (SERIAL_TEST_MODE) {
    Serial.print("TEST MODE: Would send command: ");
    Serial.println(command);
    return;
  }
  
  // Send command immediately
  pendingCommand = command;
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

void handleCommandSending() {
  // SIMPLE TEST: Just send commands, don't wait for responses
  
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
