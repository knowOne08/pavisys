/*
 * HX711_Raw.h - Raw HX711 Implementation WITHOUT Internal Filtering
 * Based on HX711 library but removes all internal smoothing/filtering
 * For PAVI Flight Computer - Maximum responsiveness and raw data access
 */

#ifndef HX711_RAW_H
#define HX711_RAW_H

#include "Arduino.h"

class HX711_Raw {
private:
  byte PD_SCK;    // Power Down and Serial Clock Input Pin
  byte DOUT;      // Serial Data Output Pin
  byte GAIN;      // Amplification factor
  long OFFSET;    // Used for tare weight
  float SCALE;    // Used to return weight in desired units

public:
  HX711_Raw();

  // Initialize HX711
  void begin(byte dout, byte pd_sck, byte gain = 128);

  // Check if HX711 is ready for reading
  bool is_ready();

  // Set gain (128 = Channel A, 32 = Channel B, 64 = Channel A with different gain)
  void set_gain(byte gain = 128);

  // Read raw 24-bit value from HX711 (NO FILTERING)
  long read();

  // Read average of samples (if you want some averaging, but you control it)
  long read_average(byte times = 10);

  // Get current raw reading minus offset
  long get_value(byte times = 1);

  // Get reading converted to units using scale factor
  float get_units(byte times = 1);

  // Set tare/zero point
  void tare(byte times = 10);

  // Set scale factor for unit conversion
  void set_scale(float scale = 1.f);

  // Get current scale factor
  float get_scale();

  // Set offset manually
  void set_offset(long offset = 0);

  // Get current offset
  long get_offset();

  // Power down HX711
  void power_down();

  // Power up HX711
  void power_up();
};

#endif
