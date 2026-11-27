// MPU6050 driver and quaternion orientation fusion.
// Workshop participants normally do not need to edit this file.
#include "mpu6050.h"
#include <Wire.h>
#include <math.h>

constexpr uint8_t MPU = 0x68;
constexpr uint8_t SDA_PIN = 21, SCL_PIN = 22;
constexpr uint32_t SAMPLE_US = 10000;  // 100 Hz
constexpr uint16_t CAL_SAMPLES = 300;
constexpr float DEG_TO_RAD_F = 0.0174532925199433f;
constexpr float MAHONY_KP = 2.0f;

struct Imu { float ax, ay, az, gx, gy, gz; };
struct Quaternion { float w, x, y, z; };
struct Snapshot {
  uint32_t uptime;
  float quat[4], accel[3], gyro[3];
  bool stationary, valid;
};
static Quaternion q = {1,0,0,0}, origin = {1,0,0,0};
static float biasX=0,biasY=0,biasZ=0;
static bool stationary=false;
static uint8_t stillSamples=0,decimation=0;
static uint32_t lastSampleUs=0;
static Snapshot latest={};
static portMUX_TYPE sensorMux = portMUX_INITIALIZER_UNLOCKED;

static bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU);
  Wire.write(reg); Wire.write(value);
  return Wire.endTransmission() == 0;
}
static bool readReg(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(MPU); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, (uint8_t)1) != 1) return false;
  value = Wire.read(); return true;
}
static bool readImu(Imu &s) {
  Wire.beginTransmission(MPU); Wire.write((uint8_t)0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, (uint8_t)14) != 14) return false;
  uint8_t b[14];
  for (int i = 0; i < 14; ++i) b[i] = Wire.read();
  auto to16 = [&](int i) -> int16_t {
    return (int16_t)((uint16_t(b[i]) << 8) | b[i+1]);
  };
  s.ax = to16(0)/16384.0f; s.ay = to16(2)/16384.0f;
  s.az = to16(4)/16384.0f;
  s.gx = to16(8)/131.0f; s.gy = to16(10)/131.0f;
  s.gz = to16(12)/131.0f;
  return true;
}
bool imuBegin() {
  static bool busReady = false;
  if (!busReady) {
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);
    Wire.setTimeOut(100);
    busReady = true;
    Serial.println("# I2C initialized: SDA=21 SCL=22, 100kHz");
  }
  Serial.println("# Checking MPU6050 at 0x68");
  Wire.beginTransmission(MPU);
  uint8_t err = Wire.endTransmission(true);
  if (err != 0) {
    Serial.printf("# MPU6050 no ACK (I2C error %u)\n", err);
    return false;
  }
  uint8_t who = 0;
  if (!readReg(0x75, who) || who != 0x68) {
    Serial.printf("# WHO_AM_I failed; value=0x%02X\n", who);
    return false;
  }
  if (!writeReg(0x6B,0x01)) return false;  // wake
  delay(100);
  if (!writeReg(0x19,0x09)) return false;  // divider
  if (!writeReg(0x1A,0x03)) return false;  // low-pass
  if (!writeReg(0x1B,0x00)) return false;  // gyro +/-250 dps
  if (!writeReg(0x1C,0x00)) return false;  // accel +/-2g
  Serial.println("# MPU6050 initialized at 0x68");
  return true;
}

// ---- Quaternion attitude estimation ----
static Quaternion unit(Quaternion p) {
  float n = sqrtf(p.w*p.w + p.x*p.x + p.y*p.y + p.z*p.z);
  if (n < 1e-8f) return {1,0,0,0};
  return {p.w/n,p.x/n,p.y/n,p.z/n};
}
static Quaternion mul(Quaternion a, Quaternion b) {
  return {
    a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
    a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
    a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
    a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w
  };
}
static Quaternion conj(Quaternion a) { return {a.w,-a.x,-a.y,-a.z}; }
static Quaternion gravityQuat(float ax,float ay,float az) {
  float n = sqrtf(ax*ax + ay*ay + az*az);
  if (n < 0.4f) return {1,0,0,0};
  ax/=n; ay/=n; az/=n;
  if (az < -0.999f) return {0,1,0,0};
  return unit({1.0f+az,ay,-ax,0});
}
bool imuCalibrate() {
  Serial.println("# CALIBRATING - keep MPU6050 still for ~1.5 s");
  double sx=0,sy=0,sz=0,sax=0,say=0,saz=0;
  double gyroSq=0,accSq=0;
  for (int i=0;i<CAL_SAMPLES;i++) {
    Imu s;
    if (!readImu(s)) return false;
    sx+=s.gx; sy+=s.gy; sz+=s.gz;
    sax+=s.ax; say+=s.ay; saz+=s.az;
    gyroSq+=s.gx*s.gx+s.gy*s.gy+s.gz*s.gz;
    accSq+=s.ax*s.ax+s.ay*s.ay+s.az*s.az;
    delay(5);
  }
  float gx=sx/CAL_SAMPLES, gy=sy/CAL_SAMPLES, gz=sz/CAL_SAMPLES;
  float ax=sax/CAL_SAMPLES, ay=say/CAL_SAMPLES, az=saz/CAL_SAMPLES;
  float norm=sqrtf(ax*ax+ay*ay+az*az);
  double gyroVar=gyroSq/CAL_SAMPLES-(gx*gx+gy*gy+gz*gz);
  double accVar=accSq/CAL_SAMPLES-(ax*ax+ay*ay+az*az);
  if (norm<0.8f || norm>1.2f || gyroVar>1.0 || accVar>0.0012) {
    Serial.println("# CAL ERROR - hold sensor still");
    return false;
  }
  biasX=gx; biasY=gy; biasZ=gz;
  q=gravityQuat(ax,ay,az); origin=q;
  stationary=false; stillSamples=0;
  lastSampleUs=micros();
  Serial.printf("# READY gyro bias: %.3f, %.3f, %.3f\n",biasX,biasY,biasZ);
  return true;
}
static Imu updateAttitude(const Imu &s,float dt) {
  Imu out=s;
  float gx=s.gx-biasX,gy=s.gy-biasY,gz=s.gz-biasZ;
  float norm=sqrtf(s.ax*s.ax+s.ay*s.ay+s.az*s.az);
  float rate=sqrtf(gx*gx+gy*gy+gz*gz);
  bool still=(fabsf(norm-1.0f)<0.045f && rate<0.45f);
  if (still) { if(stillSamples<30) ++stillSamples; }
  else stillSamples=0;
  stationary=(stillSamples>=30);
  if (stationary) {
    biasX+=0.0025f*(s.gx-biasX);
    biasY+=0.0025f*(s.gy-biasY);
    biasZ+=0.0025f*(s.gz-biasZ);
    gx=gy=gz=0;
  }
  out.gx=gx; out.gy=gy; out.gz=gz;
  gx*=DEG_TO_RAD_F; gy*=DEG_TO_RAD_F; gz*=DEG_TO_RAD_F;
  if (norm>0.80f && norm<1.20f) {
    float ax=s.ax/norm,ay=s.ay/norm,az=s.az/norm;
    float vx=2*(q.x*q.z-q.w*q.y);
    float vy=2*(q.w*q.x+q.y*q.z);
    float vz=q.w*q.w-q.x*q.x-q.y*q.y+q.z*q.z;
    float ex=ay*vz-az*vy,ey=az*vx-ax*vz,ez=ax*vy-ay*vx;
    float err=sqrtf(ex*ex+ey*ey+ez*ez);
    if (!stationary || err>0.010f) {
      gx+=MAHONY_KP*ex; gy+=MAHONY_KP*ey; gz+=MAHONY_KP*ez;
    }
  }
  Quaternion d=mul(q,{0,gx,gy,gz});
  q=unit({q.w+0.5f*d.w*dt,q.x+0.5f*d.x*dt,
          q.y+0.5f*d.y*dt,q.z+0.5f*d.z*dt});
  return out;
}
static void saveSnapshot(const Imu &s) {
  Quaternion v=unit(mul(conj(origin),q));
  Snapshot next={};
  next.uptime=millis();
  next.quat[0]=v.w; next.quat[1]=v.x;
  next.quat[2]=v.y; next.quat[3]=v.z;
  next.accel[0]=s.ax; next.accel[1]=s.ay; next.accel[2]=s.az;
  next.gyro[0]=s.gx; next.gyro[1]=s.gy; next.gyro[2]=s.gz;
  next.stationary=stationary; next.valid=true;
  portENTER_CRITICAL(&sensorMux); latest=next; portEXIT_CRITICAL(&sensorMux);
}



// Sensor sampling runs in its own FreeRTOS task; HTTPS can block without
// pausing the 100 Hz IMU acquisition.
static TaskHandle_t imuTaskHandle = nullptr;
static void imuTask(void *) {
  lastSampleUs = micros();
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    uint32_t now = micros();
    uint32_t dtUs = now - lastSampleUs;
    lastSampleUs = now;
    float dt = dtUs * 1.0e-6f;
    Imu raw;
    if (dt >= 0.002f && dt <= 0.15f && readImu(raw)) {
      Imu corrected = updateAttitude(raw, dt);
      if (++decimation >= 2) { decimation=0; saveSnapshot(corrected); }
    }
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
  }
}

bool imuStart() {
  if (imuTaskHandle != nullptr) return true;
  return xTaskCreatePinnedToCore(
    imuTask, "imu100hz", 4096, nullptr, 2, &imuTaskHandle, 1
  ) == pdPASS;
}

ImuData imuGetLatest() {
  Snapshot s;
  portENTER_CRITICAL(&sensorMux);
  s = latest;
  portEXIT_CRITICAL(&sensorMux);
  ImuData out = {};
  out.uptimeMs=s.uptime;
  out.qw=s.quat[0]; out.qx=s.quat[1];
  out.qy=s.quat[2]; out.qz=s.quat[3];
  out.ax=s.accel[0]; out.ay=s.accel[1]; out.az=s.accel[2];
  out.gx=s.gyro[0]; out.gy=s.gyro[1]; out.gz=s.gyro[2];
  out.stationary=s.stationary;
  out.valid=s.valid;
  return out;
}
