#pragma once
#include <Arduino.h>

struct ImuData {
  uint32_t uptimeMs = 0;
  float qw=1, qx=0, qy=0, qz=0;
  float ax=0, ay=0, az=0;
  float gx=0, gy=0, gz=0;
  bool stationary=false;
  bool valid=false;
};

bool imuBegin();              // Initialize I2C registers
bool imuCalibrate();          // Keep sensor still at startup
bool imuStart();              // Start independent 100 Hz sampling
ImuData imuGetLatest();      // Copy latest approximately 50 Hz snapshot
