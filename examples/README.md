# HX711_Raw Library Examples

This folder contains example code for using the HX711_Raw library with load cells.

## Examples Included

### 1. HX711_Raw_Example.ino
**Purpose**: Demonstrates basic usage of the HX711_Raw library to read raw values from a load cell and display weights in kilograms.

**Features**:
- Reads single raw values (fastest response)
- Reads averaged values (more stable)
- Converts to weight in kilograms (when calibrated)
- Displays values in decimal, hex, and binary formats
- Shows timing information

**Use Case**: Perfect for applications requiring maximum responsiveness and access to unfiltered data with weight display in kg.

### 2. HX711_Calibration.ino
**Purpose**: Interactive calibration tool to find the calibration factor for your specific load cell setup, working in kilograms.

**Features**:
- Step-by-step calibration process
- Input known weights in kilograms
- Calculates calibration factor automatically
- Tests calibration accuracy
- Provides code snippets for your projects
- Live weight display in kilograms after calibration

**Use Case**: Run this once to calibrate your load cell for kilogram measurements, then use the calculated values in your main project.

## Hardware Connections

For both examples, use these connections:

```
HX711 Pin    Arduino Pin
---------    -----------
VDD          5V (or 3.3V)
VCC          5V (or 3.3V)
GND          GND
DT (DOUT)    Pin 3
SCK (PD_SCK) Pin 2
```

## Load Cell Connections

Connect your load cell to the HX711:

```
Load Cell Wire    HX711 Pin
--------------    ---------
Red (E+)          E+
Black (E-)        E-
White (A-)        A-
Green (A+)        A+
```

*Note: Wire colors may vary by manufacturer. Check your load cell datasheet.*

## Usage Instructions

### For Raw Values Example:
1. Upload `HX711_Raw_Example.ino` to your Arduino
2. Open Serial Monitor (115200 baud)
3. Observe raw values being displayed every 500ms
4. Apply weight to see values change

### For Calibration:
1. Upload `HX711_Calibration.ino` to your Arduino
2. Open Serial Monitor (115200 baud)
3. Follow the step-by-step instructions:
   - Remove all weight from scale
   - Place a known weight (e.g., 0.1kg, 0.5kg, 1kg, 2kg)
   - Enter the exact weight value in kilograms
4. Note down the calibration factor and zero reading
5. Use these values in your main project

### Using Calibration Results:

After calibration, use the results in your code like this:

```cpp
#include "HX711_Raw.h"

HX711_Raw scale;

void setup() {
  scale.begin(3, 2);  // DOUT, SCK pins
  
  // Use your calibration values here
  scale.set_offset(YOUR_ZERO_READING);
  scale.set_scale(YOUR_CALIBRATION_FACTOR);
}

void loop() {
  if (scale.is_ready()) {
    float weight = scale.get_units();  // Weight in kilograms
    Serial.print("Weight: ");
    Serial.print(weight, 3);
    Serial.println(" kg");
  }
  delay(100);
}
```

## Tips for Better Results

1. **Stable Surface**: Mount your load cell on a stable, level surface
2. **Good Connections**: Ensure all wires are securely connected
3. **Power Supply**: Use a stable power supply (avoid USB power for precision)
4. **Temperature**: Allow system to warm up for 5-10 minutes for stable readings
5. **Calibration Weight**: Use a weight close to your expected measurement range
6. **Multiple Points**: For higher accuracy, calibrate with multiple known weights

## Troubleshooting

**"HX711 not ready" message**:
- Check wiring connections
- Verify power supply
- Ensure load cell is properly connected to HX711

**Inconsistent readings**:
- Check for loose connections
- Ensure stable mounting
- Move away from vibration sources
- Use averaging (`read_average()`) for stability

**Zero drift**:
- Allow warm-up time
- Re-tare the scale periodically
- Check for temperature variations
