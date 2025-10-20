#include <HX711_ADC.h>
#include <BluetoothSerial.h>

BluetoothSerial BTSerial;  

// HX711 Pins
const int HX711_dout = 21; 
const int HX711_sck  = 19; 

HX711_ADC LoadCell(HX711_dout, HX711_sck);

// Calibration factor
float calibrationFactor = 1.0;

// Timing
unsigned long t = 0;

void setup() {
  Serial.begin(115200);
  BTSerial.begin("ESP32_LoadCell");  

  Serial.println("\nESP32 HX711 Load Cell Setup");
  BTSerial.println("\nESP32 HX711 Load Cell Setup (Bluetooth)");

  LoadCell.begin();
  LoadCell.setCalFactor(calibrationFactor); 

  unsigned long stabilizingTime = 2000;
  bool _tare = true;
  LoadCell.start(stabilizingTime, _tare);

  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("Error: Check HX711 wiring or pins!");
    BTSerial.println("Error: Check HX711 wiring or pins!");
    while (1);
  }

  while (!LoadCell.update());
  Serial.println("Startup complete. Send 'r' to calibrate.");
  BTSerial.println("Startup complete. Send 'r' to calibrate.");
  Serial.println("WARNING: Calibration factor is 1.0 - you MUST calibrate first!");
  BTSerial.println("WARNING: Calibration factor is 1.0 - you MUST calibrate first!");
  Serial.println("Dynamic readings (no filtering) - Raw HX711 data:");
  BTSerial.println("Dynamic readings (no filtering) - Raw HX711 data:");
}

void loop() {
  static boolean newDataReady = false;

  if (LoadCell.update()) newDataReady = true;

  if (newDataReady && millis() > t + 50) {  // Faster update rate for dynamic readings
    float raw_grams = LoadCell.getData();

    // Debug: Check if we're getting valid data
    if (isnan(raw_grams)) {
      Serial.println("Warning: NaN reading from load cell - check calibration!");
      BTSerial.println("Warning: NaN reading from load cell - check calibration!");
      
      // Show raw value for debugging
      long rawValue = LoadCell.getRawValue();
      Serial.print("Raw ADC value: ");
      Serial.print(rawValue);
      Serial.print(" | Cal Factor: ");
      Serial.println(calibrationFactor);
      
      newDataReady = false;
      t = millis();
      return;
    }

    // Convert to Newtons (raw, no filtering)
    float thrust_newton = (raw_grams / 1000.0) * 9.81;
    
    // Convert to kg
    float mass_kg = raw_grams / 1000.0;

    // Output raw dynamic data
    String dataMsg = "Mass (g): " + String(raw_grams, 1) + 
                     " | Mass (kg): " + String(mass_kg, 3) +
                     " | Thrust (N): " + String(thrust_newton, 3);

    Serial.println(dataMsg);
    BTSerial.println(dataMsg);

    newDataReady = false;
    t = millis();
  }

  // Handle Serial & Bluetooth commands
  if (Serial.available()) handleCommand(Serial.read());
  if (BTSerial.available()) handleCommand(BTSerial.read());

  if (LoadCell.getTareStatus()) {
    Serial.println("Tare complete.");
    BTSerial.println("Tare complete.");
  }
}

void handleCommand(char cmd) {
  if (cmd == 't') LoadCell.tareNoDelay();
  else if (cmd == 'r') calibrate();
  else if (cmd == 'c') changeCalFactor();
}

void calibrate() {
  Serial.println("\n--- Calibration Start ---");
  BTSerial.println("\n--- Calibration Start ---");
  Serial.println("Remove all weight, then send 't' to tare.");
  BTSerial.println("Remove all weight, then send 't' to tare.");

  while (true) {
    LoadCell.update();
    if (Serial.available() && Serial.read() == 't') LoadCell.tareNoDelay();
    if (BTSerial.available() && BTSerial.read() == 't') LoadCell.tareNoDelay();
    if (LoadCell.getTareStatus()) {
      Serial.println("Tare done.");
      BTSerial.println("Tare done.");
      break;
    }
  }

  Serial.println("Now place known weight and send its mass in grams (e.g. 1000.0):");
  BTSerial.println("Now place known weight and send its mass in grams (e.g. 1000.0):");

  float known_mass = 0;
  while (true) {
    LoadCell.update();
    if (Serial.available()) known_mass = Serial.parseFloat();
    else if (BTSerial.available()) known_mass = BTSerial.parseFloat();
    if (known_mass > 0) {
      Serial.print("Received known mass: ");
      Serial.println(known_mass);
      BTSerial.print("Received known mass: ");
      BTSerial.println(known_mass);
      break;
    }
  }

  LoadCell.refreshDataSet();
  calibrationFactor = LoadCell.getNewCalibration(known_mass);
  LoadCell.setCalFactor(calibrationFactor);

  Serial.print("New calibration factor: ");
  Serial.println(calibrationFactor, 4);
  BTSerial.print("New calibration factor: ");
  BTSerial.println(calibrationFactor, 4);
  Serial.println("--- Calibration Complete ---");
  BTSerial.println("--- Calibration Complete ---");
}

void changeCalFactor() {
  Serial.println("\n--- Change Calibration Factor ---");
  BTSerial.println("\n--- Change Calibration Factor ---");

  Serial.print("Current factor: ");
  Serial.println(calibrationFactor);
  BTSerial.print("Current factor: ");
  BTSerial.println(calibrationFactor);

  Serial.println("Send new calibration factor (e.g. 696.0):");
  BTSerial.println("Send new calibration factor (e.g. 696.0):");

  float newFactor = 0;
  while (true) {
    if (Serial.available()) newFactor = Serial.parseFloat();
    else if (BTSerial.available()) newFactor = BTSerial.parseFloat();

    if (newFactor > 0) {
      calibrationFactor = newFactor;
      LoadCell.setCalFactor(calibrationFactor);
      Serial.print("New factor set: ");
      Serial.println(calibrationFactor);
      BTSerial.print("New factor set: ");
      BTSerial.println(calibrationFactor);
      break;
    }
  }

  Serial.println("--- Factor Change Done ---");
  BTSerial.println("--- Factor Change Done ---");
}