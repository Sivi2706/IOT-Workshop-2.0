#pragma once
#include <Arduino.h>

// Measurements supplied by the 6-pin BMP280 barometer.
struct BaroData {
  float pressurePa = 0.0f;       // Atmospheric pressure in pascals (Pa)
  float temperatureC = 0.0f;     // Barometer temperature (degree C)
  float altitudeM = 0.0f;        // HEIGHT CHANGE from startup (meters)
  float referencePressurePa = 0.0f; // Pressure measured at startup
  uint32_t uptimeMs = 0;
  bool valid = false;
};

bool bmpBegin();         // Detect BMP280 at 0x76 or 0x77; baseline pressure
bool bmpStart();         // Background reads around 20 Hz
BaroData bmpGetLatest(); // Copy newest BMP280 readings for Firebase JSON
