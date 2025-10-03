/*
 * HX711_Raw.cpp - Raw HX711 Implementation WITHOUT Internal Filtering
 * Based on HX711 library but removes all internal smoothing/filtering
 * For PAVI Flight Computer - Maximum responsiveness and raw data access
 */

#include "HX711_Raw.h"

HX711_Raw::HX711_Raw() {
  SCALE = 1.f;
  OFFSET = 0;
  GAIN = 128;
}

void HX711_Raw::begin(byte dout, byte pd_sck, byte gain) {
  PD_SCK = pd_sck;
  DOUT = dout;

  pinMode(PD_SCK, OUTPUT);
  pinMode(DOUT, INPUT);

  set_gain(gain);
}

bool HX711_Raw::is_ready() {
  return digitalRead(DOUT) == LOW;
}

void HX711_Raw::set_gain(byte gain) {
  switch (gain) {
    case 128:   // Channel A, 128 gain
      GAIN = 1;
      break;
    case 64:    // Channel A, 64 gain  
      GAIN = 3;
      break;
    case 32:    // Channel B, 32 gain
      GAIN = 2;
      break;
  }
}

long HX711_Raw::read() {
  // Wait for HX711 to be ready
  unsigned long start = millis();
  while (!is_ready()) {
    if (millis() - start > 1000) return 0; // 1 second timeout
    yield();
  }

  unsigned long value = 0;
  uint8_t data[3] = { 0 };
  uint8_t filler = 0x00;

  // Disable interrupts for precise timing
  noInterrupts();

  // Read 24 bits
  for (uint8_t i = 0; i < 3; i++) {
    for (uint8_t j = 0; j < 8; j++) {
      digitalWrite(PD_SCK, HIGH);
      bitWrite(data[i], 7 - j, digitalRead(DOUT));
      digitalWrite(PD_SCK, LOW);
    }
  }

  // Set gain for next reading
  for (unsigned int i = 0; i < GAIN; i++) {
    digitalWrite(PD_SCK, HIGH);
    digitalWrite(PD_SCK, LOW);
  }

  interrupts();

  // Construct 24-bit signed value
  if (data[2] & 0x80) {
    filler = 0xFF;
  } else {
    filler = 0x00;
  }

  // Construct signed long from 24-bit data
  value = (static_cast<unsigned long>(filler) << 24
         | static_cast<unsigned long>(data[2]) << 16
         | static_cast<unsigned long>(data[1]) << 8
         | static_cast<unsigned long>(data[0]));

  return static_cast<long>(value);
}

long HX711_Raw::read_average(byte times) {
  long sum = 0;
  byte validReadings = 0;
  
  for (byte i = 0; i < times; i++) {
    long reading = read();
    if (reading != 0) {  // Skip failed readings
      sum += reading;
      validReadings++;
    }
  }
  
  if (validReadings > 0) {
    return sum / validReadings;
  }
  return 0;
}

long HX711_Raw::get_value(byte times) {
  return read_average(times) - OFFSET;
}

float HX711_Raw::get_units(byte times) {
  return get_value(times) / SCALE;
}

void HX711_Raw::tare(byte times) {
  long sum = read_average(times);
  set_offset(sum);
}

void HX711_Raw::set_scale(float scale) {
  SCALE = scale;
}

float HX711_Raw::get_scale() {
  return SCALE;
}

void HX711_Raw::set_offset(long offset) {
  OFFSET = offset;
}

long HX711_Raw::get_offset() {
  return OFFSET;
}

void HX711_Raw::power_down() {
  digitalWrite(PD_SCK, LOW);
  digitalWrite(PD_SCK, HIGH);
}

void HX711_Raw::power_up() {
  digitalWrite(PD_SCK, LOW);
}
