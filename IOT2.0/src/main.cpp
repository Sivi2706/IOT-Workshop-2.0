
/*
  Nano + MPU6050 => quaternion orientation relative to calibration origin.
  BENCH-TOP VISUALIZATION ONLY: NO MOTORS, SERVOS, OR DEPLOYMENT OUTPUTS.

  Arduino Nano (ATmega328P): A4=SDA, A5=SCL, GND=GND.
  Power MPU6050 at voltage supported by YOUR breakout; bare chips need 3.3 V.
  USB serial: 115200, 50 Hz packet stream. Direct I2C; only Wire.h needed.

  Commands (send newline):
    C = re-calibrate gyro bias (hold still ~1.5 s), recapture origin
    Z = zero current attitude as origin (hold still briefly)
    ? = print help

  Packet (CSV):
    Q,millis,qw,qx,qy,qz,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps

  q = world-frame rotation RELATIVE TO THE CALIBRATION ORIGIN.
  World +Z is vertical. Yaw is relative/gyro-only and can drift.
*/
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// Adjustable constants
static const uint32_t SERIAL_BAUD = 115200;
static const uint32_t SENSOR_PERIOD_US = 10000UL;  // 100 Hz
static const uint8_t OUTPUT_EVERY_N = 2;           // 50 Hz
static const uint16_t CAL_SAMPLES = 300;
static const float KP = 2.8f;     // Mahony proportional accelerometer feedback
static const float KI = 0.01f;    // slow gyro bias correction when near 1g

static uint8_t mpuAddr = 0x68; // auto-check 0x69 if required
static float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
static float qw = 1, qx = 0, qy = 0, qz = 0;
static float refW = 1, refX = 0, refY = 0, refZ = 0;
static float ix = 0, iy = 0, iz = 0;
static uint32_t prevSampleUs = 0;
static uint8_t outputCount = 0;
static bool sensorOK = false;

struct Sample {
  float ax, ay, az; // acceleration in g
  float gx, gy, gz; // angular speed in degrees per second
};

static bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool readReg(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(mpuAddr, (uint8_t)1) != 1) return false;
  value = Wire.read();
  return true;
}

static bool readSample(Sample &out) {
  Wire.beginTransmission(mpuAddr);
  Wire.write((uint8_t)0x3B); // ACCEL_XOUT_H through GYRO_ZOUT_L
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(mpuAddr, (uint8_t)14) != 14) return false;

  // Read bytes in strict order; don't rely on C++ operand evaluation order.
  uint8_t raw[14];
  for (uint8_t i=0; i<14; ++i) raw[i] = (uint8_t)Wire.read();
  int16_t ax = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
  int16_t ay = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
  int16_t az = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);
  int16_t gx = (int16_t)(((uint16_t)raw[8] << 8) | raw[9]);
  int16_t gy = (int16_t)(((uint16_t)raw[10] << 8) | raw[11]);
  int16_t gz = (int16_t)(((uint16_t)raw[12] << 8) | raw[13]);

  // ±2 g => 16384 LSB/g. ±250 °/s => 131 LSB/(°/s).
  out.ax = ax / 16384.0f;
  out.ay = ay / 16384.0f;
  out.az = az / 16384.0f;
  out.gx = gx / 131.0f;
  out.gy = gy / 131.0f;
  out.gz = gz / 131.0f;
  return true;
}

static bool initMPU() {
  uint8_t who = 0;
  mpuAddr = 0x68;
  if (!readReg(0x75, who) || who != 0x68) {
    mpuAddr = 0x69;
    if (!readReg(0x75, who) || who != 0x68) return false;
  }
  if (!writeReg(0x6B, 0x01)) return false; // wake; PLL clock
  delay(100);
  if (!writeReg(0x19, 9)) return false;    // 1kHz/10 = 100 Hz sample rate
  if (!writeReg(0x1A, 0x03)) return false; // low-pass filter (~44 Hz accel)
  if (!writeReg(0x1B, 0x00)) return false; // ±250 deg/s gyro
  if (!writeReg(0x1C, 0x00)) return false; // ±2g accel
  Serial.print(F("# MPU6050 WHO_AM_I=0x68, I2C addr=0x"));
  Serial.println((unsigned int)mpuAddr, HEX);
  return true;
}

static void normalizeQuat(float &w, float &x, float &y, float &z) {
  float norm = sqrtf(w*w + x*x + y*y + z*z);
  if (norm < 1e-8f) { w=1; x=0; y=0; z=0; return; }
  w /= norm; x /= norm; y /= norm; z /= norm;
}

// Determine initial board-to-world rotation based on stationary gravity vector.
// yaw cannot be determined using the MPU6050 alone; initial yaw is arbitrary.
static void initQuatFromAcceleration(float ax, float ay, float az) {
  const float norm = sqrtf(ax*ax + ay*ay + az*az);
  if (norm < 0.4f) { qw=1; qx=0; qy=0; qz=0; return; }
  ax /= norm; ay /= norm; az /= norm;
  if (az < -0.999f) {
    qw=0; qx=1; qy=0; qz=0;
  } else {
    // Shortest rotation that maps measured specific-force direction to +Z.
    qw = 1.0f + az;
    qx = ay;
    qy = -ax;
    qz = 0;
    normalizeQuat(qw,qx,qy,qz);
  }
}

static void captureReference() {
  refW = qw; refX = qx; refY = qy; refZ = qz;
  Serial.println(F("# ORIGIN_ZEROED"));
}

static bool calibrate() {
  Serial.println(F("# CALIBRATING: keep board COMPLETELY STILL for ~1.5 seconds"));
  double sx=0, sy=0, sz=0;
  double ax=0, ay=0, az=0;
  Sample s;
  for (uint16_t i=0; i<CAL_SAMPLES; i++) {
    if (!readSample(s)) return false;
    sx += s.gx; sy += s.gy; sz += s.gz;
    ax += s.ax; ay += s.ay; az += s.az;
    delay(5);
  }
  gyroBiasX = (float)(sx / CAL_SAMPLES);
  gyroBiasY = (float)(sy / CAL_SAMPLES);
  gyroBiasZ = (float)(sz / CAL_SAMPLES);
  const float mx=(float)(ax/CAL_SAMPLES);
  const float my=(float)(ay/CAL_SAMPLES);
  const float mz=(float)(az/CAL_SAMPLES);
  const float an=sqrtf(mx*mx+my*my+mz*mz);
  if (an < 0.75f || an > 1.25f) {
    Serial.println(F("# ERROR: calibration requires approximately 1g; keep still and retry C"));
    return false;
  }
  initQuatFromAcceleration(mx,my,mz);
  ix=iy=iz=0;
  captureReference();
  Serial.print(F("# READY gyro bias dps: "));
  Serial.print(gyroBiasX,3); Serial.print(',');
  Serial.print(gyroBiasY,3); Serial.print(',');
  Serial.println(gyroBiasZ,3);
  prevSampleUs = micros();
  return true;
}

// Quaternion update with Mahony IMU feedback: gyroscope integrated, with
// gravity-driven correction for roll/pitch when |acceleration| is near 1 g.
// Under significant linear acceleration, gyro integration continues alone.
static void updateAHRS(const Sample &s, float dt) {
const float DEG2RAD = 0.01745329251994f;
float gx = (s.gx - gyroBiasX) * DEG2RAD;
float gy = (s.gy - gyroBiasY) * DEG2RAD;
float gz = (s.gz - gyroBiasZ) * DEG2RAD;
  float ax=s.ax, ay=s.ay, az=s.az;
  const float an = sqrtf(ax*ax + ay*ay + az*az);
  if (an > 0.80f && an < 1.20f) {
    ax/=an; ay/=an; az/=an;
    const float vx=2.0f*(qx*qz - qw*qy);
    const float vy=2.0f*(qw*qx + qy*qz);
    const float vz=qw*qw - qx*qx - qy*qy + qz*qz;
    const float ex=ay*vz - az*vy;
    const float ey=az*vx - ax*vz;
    const float ez=ax*vy - ay*vx;
    ix += KI*ex*dt; iy += KI*ey*dt; iz += KI*ez*dt;
    gx += KP*ex+ix;
    gy += KP*ey+iy;
    gz += KP*ez+iz;
  }
  const float ow=qw, ox=qx, oy=qy, oz=qz;
  const float hdt=0.5f*dt;
  qw = ow + (-ox*gx-oy*gy-oz*gz)*hdt;
  qx = ox + ( ow*gx+oy*gz-oz*gy)*hdt;
  qy = oy + ( ow*gy-ox*gz+oz*gx)*hdt;
  qz = oz + ( ow*gz+ox*gy-oy*gx)*hdt;
  normalizeQuat(qw,qx,qy,qz);
}

static void printTelemetry(const Sample &s) {
  // qRelative = qCurrent * inverse(qReference), conjugate for unit quaternions
  float w=qw*refW+qx*refX+qy*refY+qz*refZ;
  float x=-qw*refX+qx*refW-qy*refZ+qz*refY;
  float y=-qw*refY+qx*refZ+qy*refW-qz*refX;
  float z=-qw*refZ-qx*refY+qy*refX+qz*refW;
  normalizeQuat(w,x,y,z);
  Serial.print(F("Q,"));
  Serial.print(millis()); Serial.print(',');
  Serial.print(w,6); Serial.print(',');
  Serial.print(x,6); Serial.print(',');
  Serial.print(y,6); Serial.print(',');
  Serial.print(z,6); Serial.print(',');
  Serial.print(s.ax,3); Serial.print(',');
  Serial.print(s.ay,3); Serial.print(',');
  Serial.print(s.az,3); Serial.print(',');
  Serial.print(s.gx-gyroBiasX,2); Serial.print(',');
  Serial.print(s.gy-gyroBiasY,2); Serial.print(',');
  Serial.println(s.gz-gyroBiasZ,2);
}

static void handleCommands() {
  while (Serial.available()) {
    const char cmd=(char)Serial.read();
    if (cmd=='C' || cmd=='c') {
      if (!calibrate()) Serial.println(F("# ERROR: calibration failed"));
      outputCount=0;
    } else if (cmd=='Z' || cmd=='z') {
      captureReference();
    } else if (cmd=='?') {
      Serial.println(F("# C=calibrate still; Z=zero relative orientation; ?=help"));
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin(); // Nano: SDA=A4, SCL=A5
  Wire.setClock(400000UL);
  delay(500);
  Serial.println(F("# NANO MPU6050 3D ROCKET ORIENTATION DEMO"));
  sensorOK=initMPU();
  if (!sensorOK) {
    Serial.println(F("# FATAL: no MPU6050 detected at 0x68/0x69, verify wiring"));
    while (true) { delay(1000); }
  }
  if (!calibrate()) {
    Serial.println(F("# FATAL: calibration failed; reset Nano with sensor stationary"));
    while (true) { delay(1000); }
  }
}

void loop() {
  handleCommands();
  const uint32_t now=micros();
  if ((uint32_t)(now-prevSampleUs) < SENSOR_PERIOD_US) return;
  const uint32_t elapsed=now-prevSampleUs;
  prevSampleUs=now;
  const float dt=elapsed * 1.0e-6f;
  if (dt < 0.002f || dt > 0.15f) return;
  Sample s;
  if (!readSample(s)) {
    Serial.println(F("# ERROR: sensor I2C read failed"));
    return;
  }
  updateAHRS(s,dt);
  if (++outputCount >= OUTPUT_EVERY_N) {
    outputCount=0;
    printTelemetry(s);
  }
}
