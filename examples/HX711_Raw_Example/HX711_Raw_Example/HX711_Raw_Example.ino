/*
 * HX711_Raw Example - Reading Raw Values in Kilograms
 * This example demonstrates how to read raw values from the HX711 load cell amplifier
 * using the HX711_Raw library for maximum responsiveness.
 * 
 * IMPORTANT: To get accurate weight readings in kg, you must first run the 
 * HX711_Calibration.ino example to determine your calibration factor and offset.
 * Then use scale.set_offset() and scale.set_scale() with your calibrated values.
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

void setup() {
  Serial.begin(115200);
  Serial.println("HX711_Raw Example - Reading Raw Values in Kilograms");
  Serial.println("===================================================");
  
  // Initialize the HX711
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  
  // Set gain (128 = Channel A with 128 gain, 64 = Channel A with 64 gain, 32 = Channel B with 32 gain)
  scale.set_gain(128);
  
  // Check for calibration mode in first 2 seconds
  Serial.println("Send 'c' within 2 seconds to enter calibration mode...");
  unsigned long startTime = millis();
  bool calibrationMode = false;
  
  while (millis() - startTime < 2000) {
    if (Serial.available()) {
      char input = Serial.read();
      if (input == 'c' || input == 'C') {
        calibrationMode = true;
        break;
      }
    }
    delay(10);
  }
  
  if (calibrationMode) {
    Serial.println("Entering calibration mode...");
    calibrateScale();
  } else {
    // CALIBRATION VALUES - Replace these with your actual calibrated values
    // Run calibration mode or use known values
    scale.set_offset(61467); 
    scale.set_scale(1.336289);
    Serial.println("Using stored calibration values...");
  }
  
  Serial.println("Initialization complete!");
  Serial.println("Waiting for HX711 to be ready...");
  
  // Wait for HX711 to be ready
  while (!scale.is_ready()) {
    delay(100);
  }
  
  Serial.println("HX711 is ready!");
  
  // Debug: Show current calibration values
  Serial.println("=== CURRENT CALIBRATION VALUES ===");
  Serial.print("Offset: ");
  Serial.println(scale.get_offset());
  Serial.print("Scale: ");
  Serial.println(scale.get_scale(), 6);
  Serial.println("==================================");
  
  Serial.println("Reading raw values every 500ms...");
  Serial.println("Format: Raw Value | Average(10) | Weight(kg) | Timestamp(ms)");
  Serial.println("===============================================================");
}

void loop() {
  // Check if HX711 is ready
  if (scale.is_ready()) {
    
    // Read single raw value (fastest, no averaging)
    long rawValue = scale.read();
    
    // Read average of 10 readings (more stable but slower)
    long averageValue = scale.read_average(10);
    
    // Convert to weight in kg (you'll need to set calibration values)
    // For now, showing raw conversion - use calibration example first
    float weightKg = scale.get_units(5);  // Average of 5 readings, converted to kg
    
    // Calculate weight manually for debugging
    long offsetValue = scale.get_offset();
    float scaleValue = scale.get_scale();
    float manualWeight = (averageValue - offsetValue) / scaleValue;
    
    // Print the values
    Serial.print("Raw: ");
    Serial.print(rawValue);
    Serial.print(" | Avg(10): ");
    Serial.print(averageValue);
    Serial.print(" | Weight: ");
    Serial.print(weightKg, 3);
    Serial.print(" kg | Manual: ");
    Serial.print(manualWeight, 3);
    Serial.print(" kg | Time: ");
    Serial.print(millis());
    Serial.println(" ms");
    
  } else {
    Serial.println("HX711 not ready - check connections");
  }
  
  delay(500);  // Read every 500ms
}

/*
 * Calibration subroutine - Additions for calibration mode
 */

void calibrateScale() {
  Serial.println();
  Serial.println("=== CALIBRATION MODE ===");
  Serial.println();
  
  // Wait for HX711 to be ready
  while (!scale.is_ready()) {
    Serial.println("Waiting for HX711...");
    delay(500);
  }
  
  // Step 1: Zero/Tare the scale
  Serial.println("STEP 1: Remove all weight from the scale");
  Serial.println("Press any key and hit Enter when ready...");
  waitForInput();
  
  // Take zero reading
  Serial.println("Taking zero reading...");
  long zeroReading = scale.read_average(20);  // Take average of 20 readings for stability
  Serial.print("Zero reading: ");
  Serial.println(zeroReading);
  Serial.println();
  
  // Step 2: Place known weight
  Serial.println("STEP 2: Place a known weight on the scale");
  Serial.print("Enter the weight in kilograms: ");
  float knownWeight = getFloatInput();
  Serial.print("You entered: ");
  Serial.print(knownWeight);
  Serial.println(" kg");
  Serial.println();
  
  Serial.println("Press any key and hit Enter when weight is placed...");
  waitForInput();
  
  // Take loaded reading
  Serial.println("Taking loaded reading...");
  long rawReading = scale.read_average(20);  // Take average of 20 readings
  Serial.print("Loaded reading: ");
  Serial.println(rawReading);
  Serial.println();
  
  // Calculate calibration factor
  long difference = rawReading - zeroReading;
  float calibrationFactor = (float)difference / knownWeight;
  
  Serial.println("=== CALIBRATION RESULTS ===");
  Serial.print("Zero reading: ");
  Serial.println(zeroReading);
  Serial.print("Loaded reading: ");
  Serial.println(rawReading);
  Serial.print("Difference: ");
  Serial.println(difference);
  Serial.print("Known weight: ");
  Serial.print(knownWeight);
  Serial.println(" kg");
  Serial.print("Calibration factor: ");
  Serial.println(calibrationFactor, 6);
  Serial.println();
  
  // Test the calibration
  Serial.println("=== CALIBRATION TEST ===");
  float calculatedWeight = difference / calibrationFactor;
  Serial.print("Calculated weight: ");
  Serial.print(calculatedWeight, 3);
  Serial.println(" kg");
  Serial.print("Error: ");
  Serial.print(abs(calculatedWeight - knownWeight), 3);
  Serial.println(" kg");
  Serial.println();
  
  // Apply calibration
  scale.set_offset(zeroReading);
  scale.set_scale(calibrationFactor);
  
  // Provide code snippet for user
  Serial.println("=== UPDATE YOUR CODE WITH THESE VALUES ===");
  Serial.println("scale.set_offset(" + String(zeroReading) + ");");
  Serial.println("scale.set_scale(" + String(calibrationFactor, 6) + ");");
  Serial.println();
  
  Serial.println("Calibration applied! Continuing with normal operation...");
  Serial.println("================================================");
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
