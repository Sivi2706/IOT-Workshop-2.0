/*
  Arduino Nano V3 + MPU6050: orientation relative to initial sensor frame.
  PlatformIO project; bench-only 3D visualizer. No output actuators.

  A4=SDA, A5=SCL, GND=GND; power/logic levels MUST suit the MPU6050 breakout.
  100 Hz sensor, 50 Hz USB telemetry, 115200 baud.

  Q,t_ms,qw,qx,qy,qz,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,still

  The quaternion is q_delta = conjugate(q_at_zero) * q_now.
  This expresses the motion in the sensor's CALIBRATED reference axes.
  Python maps sensor axes to rocket axes with --nose-axis and --turn.

  Commands: C=calibrate gyro again while motionless, Z=current pose is zero,
            ?=help. Keep the sensor still during boot or C.
*/
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

static const uint32_t BAUD = 115200UL;
static const uint32_t SAMPLE_PERIOD_US = 10000UL; // 100 Hz
static const uint8_t REPORT_DECIMATION = 2;      // 50 Hz
static const uint16_t CAL_COUNT = 300;           // 1.5 sec at 5ms/sample
static const float RAD_PER_DEGREE = 0.0174532925199433f;

// Gravity correction; yaw has NO absolute reference with MPU6050 alone.
static const float MAHONY_KP = 2.0f;
// Zero angular-rate update: small rates are treated as noise *only after*
// repeated stationary evidence. Deliberately slow rotations can be lost.
static const float STILL_RATE_DPS = 0.45f;
static const float STILL_ACCEL_TOL_G = 0.045f;
static const uint8_t STILL_CONFIRM_SAMPLES = 30; // 0.30 s
static const float STILL_BIAS_ALPHA = 0.0025f;   // ~4 s time constant
static const float STILL_CORRECTION_DEADBAND = 0.010f;

static uint8_t addr = 0x68;
struct Sample { float ax,ay,az,gx,gy,gz; };
struct Quat { float w,x,y,z; };
static Quat q = {1,0,0,0}, qZero = {1,0,0,0};
static float bx=0, by=0, bz=0;
static uint8_t stillCount=0;
static bool isStill=false;
static uint8_t outputCount=0;
static uint32_t lastTickUs=0;

static bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg); Wire.write(value);
  return Wire.endTransmission() == 0;
}
static bool readReg(uint8_t reg,uint8_t &v) {
  Wire.beginTransmission(addr); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr,(uint8_t)1) != 1) return false;
  v=(uint8_t)Wire.read(); return true;
}
static bool readSample(Sample &s) {
  Wire.beginTransmission(addr); Wire.write((uint8_t)0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr,(uint8_t)14) != 14) return false;
  uint8_t b[14];
  for (uint8_t i=0; i<14; ++i) b[i]=(uint8_t)Wire.read();
  const int16_t ax=(int16_t)(((uint16_t)b[0]<<8)|b[1]);
  const int16_t ay=(int16_t)(((uint16_t)b[2]<<8)|b[3]);
  const int16_t az=(int16_t)(((uint16_t)b[4]<<8)|b[5]);
  const int16_t gx=(int16_t)(((uint16_t)b[8]<<8)|b[9]);
  const int16_t gy=(int16_t)(((uint16_t)b[10]<<8)|b[11]);
  const int16_t gz=(int16_t)(((uint16_t)b[12]<<8)|b[13]);
  s.ax=ax/16384.0f; s.ay=ay/16384.0f; s.az=az/16384.0f;
  s.gx=gx/131.0f; s.gy=gy/131.0f; s.gz=gz/131.0f;
  return true;
}
static bool initSensor() {
  uint8_t chip=0;
  addr=0x68;
  if (!readReg(0x75,chip) || chip!=0x68) {
    addr=0x69;
    if (!readReg(0x75,chip) || chip!=0x68) return false;
  }
  if (!writeReg(0x6B,0x01)) return false;
  delay(100);
  if (!writeReg(0x19,0x09)) return false; // divider: 100 Hz
  if (!writeReg(0x1A,0x03)) return false; // DLPF (~44 Hz)
  if (!writeReg(0x1B,0x00)) return false; // gyro +/-250 deg/s
  if (!writeReg(0x1C,0x00)) return false; // accel +/-2 g
  Serial.print(F("# MPU6050 found at 0x"));
  Serial.println((unsigned int)addr,HEX);
  return true;
}
static Quat normalized(Quat v) {
  const float n=sqrtf(v.w*v.w+v.x*v.x+v.y*v.y+v.z*v.z);
  if (n<1.0e-8f) return {1,0,0,0};
  v.w/=n; v.x/=n; v.y/=n; v.z/=n; return v;
}
static Quat conjugate(Quat v) { return {v.w,-v.x,-v.y,-v.z}; }
static Quat multiply(Quat a, Quat b) {
  return {a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
          a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
          a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
          a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};
}
static Quat orientationFromGravity(float ax,float ay,float az) {
  float n=sqrtf(ax*ax+ay*ay+az*az);
  if(n<0.4f) return {1,0,0,0};
  ax/=n; ay/=n; az/=n;
  // Rotation mapping measured stationary specific-force +g into world +Z.
  if(az < -0.999f) return {0,1,0,0};
  return normalized({1.0f+az,ay,-ax,0});
}
static void zeroHere() {
  qZero=q;
  Serial.println(F("# ORIGIN_ZEROED: local sensor axes at this pose"));
}
static bool calibrate() {
  Serial.println(F("# CALIBRATING - hold IMU and breadboard STILL for 1.5 seconds"));
  double gx=0,gy=0,gz=0,ax=0,ay=0,az=0;
  double gx2=0,gy2=0,gz2=0,ax2=0,ay2=0,az2=0;
  Sample s;
  for(uint16_t i=0;i<CAL_COUNT;i++) {
    if(!readSample(s)) return false;
    gx+=s.gx; gy+=s.gy; gz+=s.gz;
    ax+=s.ax; ay+=s.ay; az+=s.az;
    gx2+=s.gx*s.gx; gy2+=s.gy*s.gy; gz2+=s.gz*s.gz;
    ax2+=s.ax*s.ax; ay2+=s.ay*s.ay; az2+=s.az*s.az;
    delay(5);
  }
  const double invN=1.0/(double)CAL_COUNT;
  const float agx=(float)(gx*invN),agy=(float)(gy*invN),agz=(float)(gz*invN);
  const float aax=(float)(ax*invN),aay=(float)(ay*invN),aaz=(float)(az*invN);
  const float an=sqrtf(aax*aax+aay*aay+aaz*aaz);
  const double gyrVar=(gx2+gy2+gz2)*invN-(agx*agx+agy*agy+agz*agz);
  const double accVar=(ax2+ay2+az2)*invN-(aax*aax+aay*aay+aaz*aaz);
  if (an<0.8f || an>1.2f || gyrVar>1.0 || accVar>0.0012) {
    Serial.println(F("# CAL ERROR: moved during calibration; hold still and send C"));
    return false;
  }
  bx=agx; by=agy; bz=agz;
  q=orientationFromGravity(aax,aay,aaz);
  stillCount=0; isStill=false;
  zeroHere();
  lastTickUs=micros();
  Serial.print(F("# READY gyro bias deg/s: "));
  Serial.print(bx,3);Serial.print(',');Serial.print(by,3);Serial.print(',');Serial.println(bz,3);
  return true;
}
static void update(const Sample &s,float dt, Sample &corrected) {
  corrected=s;
  float gx=s.gx-bx,gy=s.gy-by,gz=s.gz-bz;
  const float rate=sqrtf(gx*gx+gy*gy+gz*gz);
  const float an=sqrtf(s.ax*s.ax+s.ay*s.ay+s.az*s.az);
  const bool possibleStill=(fabsf(an-1.0f)<STILL_ACCEL_TOL_G && rate<STILL_RATE_DPS);
  if(possibleStill) { if(stillCount<STILL_CONFIRM_SAMPLES) ++stillCount; }
  else stillCount=0;
  isStill=(stillCount>=STILL_CONFIRM_SAMPLES);
  if(isStill) {
    // Slowly follow warm-up/temperature gyro bias; no orientation integration
    // from noise while motionless. No compass => moving yaw still drifts.
    bx+=(s.gx-bx)*STILL_BIAS_ALPHA;
    by+=(s.gy-by)*STILL_BIAS_ALPHA;
    bz+=(s.gz-bz)*STILL_BIAS_ALPHA;
    gx=gy=gz=0.0f;
  }
  corrected.gx=gx; corrected.gy=gy; corrected.gz=gz;
  gx*=RAD_PER_DEGREE; gy*=RAD_PER_DEGREE; gz*=RAD_PER_DEGREE;
  // Project Earth +Z (gravity specific force) into the sensor body axes.
  if(an>0.80f && an<1.20f) {
    const float ax=s.ax/an,ay=s.ay/an,az=s.az/an;
    const float vx=2.0f*(q.x*q.z-q.w*q.y);
    const float vy=2.0f*(q.w*q.x+q.y*q.z);
    const float vz=q.w*q.w-q.x*q.x-q.y*q.y+q.z*q.z;
    const float ex=ay*vz-az*vy;
    const float ey=az*vx-ax*vz;
    const float ez=ax*vy-ay*vx;
    const float error=sqrtf(ex*ex+ey*ey+ez*ez);
    // Avoid visible jitter once the stationary pose has converged.
    if(!isStill || error>STILL_CORRECTION_DEADBAND) {
      gx+=MAHONY_KP*ex;
      gy+=MAHONY_KP*ey;
      gz+=MAHONY_KP*ez;
    }
  }
  const Quat omega={0,gx,gy,gz};
  const Quat dq=multiply(q,omega); // q_dot = 0.5 * q * omega_body
  q=normalized({q.w+0.5f*dq.w*dt,q.x+0.5f*dq.x*dt,
                q.y+0.5f*dq.y*dt,q.z+0.5f*dq.z*dt});
}
static void printPacket(const Sample &s) {
  // Reference is on LEFT: sensor-frame delta, NOT world-frame delta.
  const Quat out=normalized(multiply(conjugate(qZero),q));
  Serial.print(F("Q,")); Serial.print(millis());Serial.print(',');
  Serial.print(out.w,6);Serial.print(',');
  Serial.print(out.x,6);Serial.print(',');
  Serial.print(out.y,6);Serial.print(',');
  Serial.print(out.z,6);Serial.print(',');
  Serial.print(s.ax,3);Serial.print(',');
  Serial.print(s.ay,3);Serial.print(',');
  Serial.print(s.az,3);Serial.print(',');
  Serial.print(s.gx,3);Serial.print(',');
  Serial.print(s.gy,3);Serial.print(',');
  Serial.print(s.gz,3);Serial.print(',');
  Serial.println(isStill ? 1 : 0);
}
static void commands() {
  while(Serial.available()>0) {
    const char c=(char)Serial.read();
    if(c=='Z' || c=='z') zeroHere();
    else if(c=='C' || c=='c') {
      if(!calibrate()) Serial.println(F("# CALIBRATION FAILED: retry with C"));
      outputCount=0;
    } else if(c=='?') {
      Serial.println(F("# C=full calibration while still; Z=reset origin; ?=help"));
    }
  }
}
void setup() {
  Serial.begin(BAUD);
  Wire.begin(); // Nano hardware I2C: SDA=A4, SCL=A5
  Wire.setClock(400000UL);
  delay(450);
  Serial.println(F("# NANO MPU6050 ROCKET ATTITUDE V2 (sensor-frame quaternion)"));
  if(!initSensor()) {
    Serial.println(F("# FATAL: MPU6050 not found at 0x68 or 0x69"));
    while(true) delay(1000);
  }
  while(!calibrate()) {
    Serial.println(F("# Retry automatic calibration in 2 seconds..."));
    delay(2000);
  }
}
void loop() {
  commands();
  const uint32_t now=micros();
  if((uint32_t)(now-lastTickUs)<SAMPLE_PERIOD_US) return;
  const uint32_t elapsed=now-lastTickUs;
  lastTickUs=now;
  const float dt=(float)elapsed*1.0e-6f;
  if(dt<0.002f || dt>0.15f) return;
  Sample s,adjusted;
  if(!readSample(s)) {
    Serial.println(F("# ERROR: MPU6050 I2C read failed")); return;
  }
  update(s,dt,adjusted);
  if(++outputCount>=REPORT_DECIMATION) {
    outputCount=0;
    printPacket(adjusted);
  }
}
