/*
 * HX711_Raw Calibration Example - Finding Calibration Factor
 * This example helps you find the calibration factor for your load cell
 * to convert raw values to actual weight measurements.
 * 
 * Process:
 * 1. Run this code with no weight on the scale
 * 2. Follow the prompts to place known weights
 * 3. The code will calculate and display the calibration factor
 * 
 * Connections:
 * HX711 VDD -> Arduino 5V (or 3.3V)
 * HX711 VCC -> Arduino 5V (or 3.3V) 
 * HX711 GND -> Arduino GND
 * HX711 DT (DOUT) -> Arduino Pin 3
 * HX711 SCK (PD_SCK) -> Arduino Pin 2
 */

#include "HX711_Raw.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 21;
const int LOADCELL_SCK_PIN = 19;

HX711_Raw scale;

// Calibration variables
float knownWeight = 0;  // Weight of calibration object in grams (or your preferred unit)
long rawReading = 0;
long zeroReading = 0;
float calibrationFactor = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("HX711_Raw Calibration Tool");
  Serial.println("==========================");
  Serial.println();
  
  // Initialize the HX711
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_gain(128);  // Use 128 gain for maximum sensitivity
  
  Serial.println("Initialization complete!");
  
  // Wait for HX711 to be ready
  while (!scale.is_ready()) {
    Serial.println("Waiting for HX711...");
    delay(500);
  }
  
  Serial.println("HX711 is ready!");
  Serial.println();
  
  // Start calibration process
  calibrateScale();
}

void loop() {
  // After calibration, continuously display weight
  if (scale.is_ready()) {
    // Get raw value and convert to weight
    long rawValue = scale.read_average(5);
    float weight = (rawValue - zeroReading) / calibrationFactor;
    
    Serial.print("Raw: ");
    Serial.print(rawValue);
    Serial.print(" | Weight: ");
    Serial.print(weight, 2);
    Serial.println(" g");
    
    delay(1000);
  }
}

void calibrateScale() {
  Serial.println("=== CALIBRATION PROCESS ===");
  Serial.println();
  
  // Step 1: Zero/Tare the scale
  Serial.println("STEP 1: Remove all weight from the scale");
  Serial.println("Press any key and hit Enter when ready...");
  waitForInput();
  
  // Take zero reading
  Serial.println("Taking zero reading...");
  zeroReading = scale.read_average(20);  // Take average of 20 readings for stability
  Serial.print("Zero reading: ");
  Serial.println(zeroReading);
  Serial.println();
  
  // Step 2: Place known weight
  Serial.println("STEP 2: Place a known weight on the scale");
  Serial.print("Enter the weight in grams: ");
  knownWeight = getFloatInput();
  Serial.print("You entered: ");
  Serial.print(knownWeight);
  Serial.println(" grams");
  Serial.println();
  
  Serial.println("Press any key and hit Enter when weight is placed...");
  waitForInput();
  
  // Take loaded reading
  Serial.println("Taking loaded reading...");
  rawReading = scale.read_average(20);  // Take average of 20 readings
  Serial.print("Loaded reading: ");
  Serial.println(rawReading);
  Serial.println();
  
  // Calculate calibration factor
  long difference = rawReading - zeroReading;
  calibrationFactor = (float)difference / knownWeight;
  
  Serial.println("=== CALIBRATION RESULTS ===");
  Serial.print("Zero reading: ");
  Serial.println(zeroReading);
  Serial.print("Loaded reading: ");
  Serial.println(rawReading);
  Serial.print("Difference: ");
  Serial.println(difference);
  Serial.print("Known weight: ");
  Serial.print(knownWeight);
  Serial.println(" g");
  Serial.print("Calibration factor: ");
  Serial.println(calibrationFactor, 6);
  Serial.println();
  
  // Test the calibration
  Serial.println("=== CALIBRATION TEST ===");
  float calculatedWeight = difference / calibrationFactor;
  Serial.print("Calculated weight: ");
  Serial.print(calculatedWeight, 2);
  Serial.println(" g");
  Serial.print("Error: ");
  Serial.print(abs(calculatedWeight - knownWeight), 2);
  Serial.println(" g");
  Serial.println();
  
  // Provide code snippet for user
  Serial.println("=== USE THESE VALUES IN YOUR CODE ===");
  Serial.println("scale.set_offset(" + String(zeroReading) + ");");
  Serial.println("scale.set_scale(" + String(calibrationFactor, 6) + ");");
  Serial.println();
  Serial.println("Or use these raw values:");
  Serial.println("Zero reading: " + String(zeroReading));
  Serial.println("Calibration factor: " + String(calibrationFactor, 6));
  Serial.println();
  
  Serial.println("Calibration complete! Now showing live weight readings...");
  Serial.println("========================================");
}

void waitForInput() {
  while (Serial.available() > 0) {
    Serial.read();  // Clear input buffer
  }
  while (Serial.available() == 0) {
    delay(100);  // Wait for input
  }
  while (Serial.available() > 0) {
    Serial.read();  // Clear input buffer
  }
}

float getFloatInput() {
  while (Serial.available() > 0) {
    Serial.read();  // Clear input buffer
  }
  while (Serial.available() == 0) {
    delay(100);  // Wait for input
  }
  return Serial.parseFloat();
}
