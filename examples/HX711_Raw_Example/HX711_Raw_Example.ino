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
const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 2;

HX711_Raw scale;

void setup() {
  Serial.begin(115200);
  Serial.println("HX711_Raw Example - Reading Raw Values in Kilograms");
  Serial.println("===================================================");
  
  // Initialize the HX711
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  
  // Set gain (128 = Channel A with 128 gain, 64 = Channel A with 64 gain, 32 = Channel B with 32 gain)
  scale.set_gain(128);
  
  // CALIBRATION VALUES - Replace these with your actual calibrated values
  // Run HX711_Calibration.ino first to get these values
  // scale.set_offset(YOUR_ZERO_READING);     // Example: scale.set_offset(12345);
  // scale.set_scale(YOUR_CALIBRATION_FACTOR); // Example: scale.set_scale(2280.5);
  
  Serial.println("Initialization complete!");
  Serial.println("NOTE: For accurate kg readings, set calibration values above!");
  Serial.println("Waiting for HX711 to be ready...");
  
  // Wait for HX711 to be ready
  while (!scale.is_ready()) {
    delay(100);
  }
  
  Serial.println("HX711 is ready!");
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
    
    // Print the values
    Serial.print("Raw: ");
    Serial.print(rawValue);
    Serial.print(" | Avg(10): ");
    Serial.print(averageValue);
    Serial.print(" | Weight: ");
    Serial.print(weightKg, 3);
    Serial.print(" kg | Time: ");
    Serial.print(millis());
    Serial.println(" ms");
    
    // Optional: Print in different formats
    if (rawValue != 0) {  // Only print if we got a valid reading
      Serial.print("  -> Hex: 0x");
      Serial.print(rawValue, HEX);
      Serial.print(" | Binary: ");
      Serial.println(rawValue, BIN);
    } else {
      Serial.println("  -> Reading failed or timeout");
    }
    
  } else {
    Serial.println("HX711 not ready - check connections");
  }
  
  delay(500);  // Read every 500ms
}
