
/*
 ESP32 + MPU6050 -> Firebase
 Same anonymous authentication and database rules
 as the original Nano Python publisher.

 I2C: SDA=21, SCL=22, address=0x68
 Sensor: 100 Hz
 Firebase: approximately 2 Hz

 Arduino-ESP32 2.0.17
 Requires ArduinoJson 7
 No ESP32CertBundle library required.
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <math.h>
#include <time.h>

// =====================================================
// 1. CONFIGURATION
// =====================================================

static const char* WIFI_SSID =
    "REDMI Note 17 5G";

static const char* WIFI_PASSWORD =
    "G2706pls";

static const char* API_KEY =
    "AIzaSyBgCv_x3peG3M3-h0d2aRhUP8hegv6kZWo";

static const char* DB_URL =
    "https://iot-2-e109b-default-rtdb.asia-southeast1.firebasedatabase.app";

constexpr uint8_t MPU = 0x68;
constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;

constexpr uint32_t SAMPLE_US = 10000;
constexpr uint32_t CLOUD_MS = 500;
constexpr int CAL_SAMPLES = 300;

constexpr float DEG_TO_RAD_F =
    0.0174532925199433f;

constexpr float MAHONY_KP = 2.0f;

// =====================================================
// 2. SENSOR STRUCTURES
// =====================================================

struct Imu {
    float ax, ay, az;
    float gx, gy, gz;
};

struct Quaternion {
    float w, x, y, z;
};

struct Snapshot {
    uint32_t uptime;
    float quat[4];
    float accel[3];
    float gyro[3];
    bool stationary;
    bool valid;
};

static Quaternion q = {1,0,0,0};
static Quaternion origin = {1,0,0,0};

static float biasX = 0;
static float biasY = 0;
static float biasZ = 0;

static bool stationary = false;
static uint8_t stillSamples = 0;
static uint8_t decimation = 0;

static uint32_t lastSampleUs = 0;
static Snapshot latest = {};

static portMUX_TYPE sensorMux =
    portMUX_INITIALIZER_UNLOCKED;

// =====================================================
// 3. MPU6050 I2C
// =====================================================

static bool writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

static bool readReg(uint8_t reg, uint8_t &value) {
    Wire.beginTransmission(MPU);
    Wire.write(reg);

    if (Wire.endTransmission(false) != 0)
        return false;

    if (Wire.requestFrom(MPU, (uint8_t)1) != 1)
        return false;

    value = Wire.read();
    return true;
}

static bool readImu(Imu &s) {
    Wire.beginTransmission(MPU);
    Wire.write((uint8_t)0x3B);

    if (Wire.endTransmission(false) != 0)
        return false;

    if (Wire.requestFrom(MPU, (uint8_t)14) != 14)
        return false;

    uint8_t b[14];

    for (int i=0; i<14; i++)
        b[i] = Wire.read();

    auto to16 = [&](int i) -> int16_t {
        return (int16_t)(
            (uint16_t(b[i]) << 8) | b[i+1]
        );
    };

    s.ax = to16(0)/16384.0f;
    s.ay = to16(2)/16384.0f;
    s.az = to16(4)/16384.0f;

    s.gx = to16(8)/131.0f;
    s.gy = to16(10)/131.0f;
    s.gz = to16(12)/131.0f;

    return true;
}

static bool initMpu() {
    Serial.println("# Checking MPU6050 at 0x68");

    Wire.beginTransmission(MPU);

    uint8_t err = Wire.endTransmission(true);

    if (err != 0) {
        Serial.printf(
            "# MPU6050 no ACK, error %u\n", err
        );
        return false;
    }

    uint8_t who = 0;

    if (!readReg(0x75,who) || who != 0x68) {
        Serial.printf(
            "# WHO_AM_I failed: 0x%02X\n", who
        );
        return false;
    }

    if (!writeReg(0x6B,0x01)) return false;
    delay(100);

    if (!writeReg(0x19,0x09)) return false;
    if (!writeReg(0x1A,0x03)) return false;
    if (!writeReg(0x1B,0x00)) return false;
    if (!writeReg(0x1C,0x00)) return false;

    Serial.println("# MPU6050 initialized at 0x68");
    return true;
}

// =====================================================
// 4. QUATERNION ESTIMATION
// =====================================================

static Quaternion unit(Quaternion p) {
    float n = sqrtf(
        p.w*p.w+p.x*p.x+p.y*p.y+p.z*p.z
    );

    if (n<1e-8f) return {1,0,0,0};

    return {
        p.w/n,p.x/n,p.y/n,p.z/n
    };
}

static Quaternion mul(Quaternion a, Quaternion b) {
    return {
        a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
        a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
        a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
        a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w
    };
}

static Quaternion conj(Quaternion a) {
    return {a.w,-a.x,-a.y,-a.z};
}

static Quaternion gravityQuat(float ax,float ay,float az) {
    float n = sqrtf(ax*ax+ay*ay+az*az);

    if (n<0.4f) return {1,0,0,0};

    ax/=n; ay/=n; az/=n;

    if (az<-0.999f) return {0,1,0,0};

    return unit({1.0f+az,ay,-ax,0});
}

static bool calibrate() {
    Serial.println("# CALIBRATING - hold MPU6050 still");

    double sx=0,sy=0,sz=0;
    double sax=0,say=0,saz=0;
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

    float gx=sx/CAL_SAMPLES;
    float gy=sy/CAL_SAMPLES;
    float gz=sz/CAL_SAMPLES;

    float ax=sax/CAL_SAMPLES;
    float ay=say/CAL_SAMPLES;
    float az=saz/CAL_SAMPLES;

    float norm=sqrtf(ax*ax+ay*ay+az*az);

    double gyroVar=gyroSq/CAL_SAMPLES-
        (gx*gx+gy*gy+gz*gz);

    double accVar=accSq/CAL_SAMPLES-
        (ax*ax+ay*ay+az*az);

    if (norm<0.8f || norm>1.2f ||
        gyroVar>1.0 || accVar>0.0012) {
        Serial.println("# CAL ERROR - keep still");
        return false;
    }

    biasX=gx; biasY=gy; biasZ=gz;
    q=gravityQuat(ax,ay,az);
    origin=q;

    stationary=false;
    stillSamples=0;
    lastSampleUs=micros();

    Serial.printf(
        "# READY gyro bias: %.3f, %.3f, %.3f\n",
        biasX,biasY,biasZ
    );

    return true;
}

static Imu updateAttitude(const Imu &s,float dt) {
    Imu out=s;

    float gx=s.gx-biasX;
    float gy=s.gy-biasY;
    float gz=s.gz-biasZ;

    float norm=sqrtf(
        s.ax*s.ax+s.ay*s.ay+s.az*s.az
    );

    float rate=sqrtf(gx*gx+gy*gy+gz*gz);

    bool still =
        fabsf(norm-1.0f)<0.045f && rate<0.45f;

    if (still) {
        if (stillSamples<30) ++stillSamples;
    } else {
        stillSamples=0;
    }

    stationary=(stillSamples>=30);

    if (stationary) {
        biasX+=0.0025f*(s.gx-biasX);
        biasY+=0.0025f*(s.gy-biasY);
        biasZ+=0.0025f*(s.gz-biasZ);
        gx=gy=gz=0;
    }

    out.gx=gx;
    out.gy=gy;
    out.gz=gz;

    gx*=DEG_TO_RAD_F;
    gy*=DEG_TO_RAD_F;
    gz*=DEG_TO_RAD_F;

    if (norm>0.8f && norm<1.2f) {
        float ax=s.ax/norm;
        float ay=s.ay/norm;
        float az=s.az/norm;

        float vx=2*(q.x*q.z-q.w*q.y);
        float vy=2*(q.w*q.x+q.y*q.z);
        float vz=q.w*q.w-q.x*q.x-q.y*q.y+q.z*q.z;

        float ex=ay*vz-az*vy;
        float ey=az*vx-ax*vz;
        float ez=ax*vy-ay*vx;

        float err=sqrtf(ex*ex+ey*ey+ez*ez);

        if (!stationary || err>0.010f) {
            gx+=MAHONY_KP*ex;
            gy+=MAHONY_KP*ey;
            gz+=MAHONY_KP*ez;
        }
    }

    Quaternion d=mul(q,{0,gx,gy,gz});

    q=unit({
        q.w+0.5f*d.w*dt,
        q.x+0.5f*d.x*dt,
        q.y+0.5f*d.y*dt,
        q.z+0.5f*d.z*dt
    });

    return out;
}

static void saveSnapshot(const Imu &s) {
    Quaternion v=unit(mul(conj(origin),q));

    Snapshot next={};

    next.uptime=millis();

    next.quat[0]=v.w;
    next.quat[1]=v.x;
    next.quat[2]=v.y;
    next.quat[3]=v.z;

    next.accel[0]=s.ax;
    next.accel[1]=s.ay;
    next.accel[2]=s.az;

    next.gyro[0]=s.gx;
    next.gyro[1]=s.gy;
    next.gyro[2]=s.gz;

    next.stationary=stationary;
    next.valid=true;

    portENTER_CRITICAL(&sensorMux);
    latest=next;
    portEXIT_CRITICAL(&sensorMux);
}

// =====================================================
// 5. GOOGLE ROOT CERTIFICATES — VERIFIED TLS
// =====================================================

// Public Google Trust Services roots R1, R2, R3, R4.
// Certificate checking remains enabled.

static const char GOOGLE_ROOTS[] PROGMEM = R"GTS(
-----BEGIN CERTIFICATE-----
MIIFVzCCAz+gAwIBAgINAgPlk28xsBNJiGuiFzANBgkqhkiG9w0BAQwFADBHMQsw
CQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2VzIExMQzEU
MBIGA1UEAxMLR1RTIFJvb3QgUjEwHhcNMTYwNjIyMDAwMDAwWhcNMzYwNjIyMDAw
MDAwWjBHMQswCQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZp
Y2VzIExMQzEUMBIGA1UEAxMLR1RTIFJvb3QgUjEwggIiMA0GCSqGSIb3DQEBAQUA
A4ICDwAwggIKAoICAQC2EQKLHuOhd5s73L+UPreVp0A8of2C+X0yBoJx9vaMf/vo
27xqLpeXo4xL+Sv2sfnOhB2x+cWX3u+58qPpvBKJXqeqUqv4IyfLpLGcY9vXmX7w
Cl7raKb0xlpHDU0QM+NOsROjyBhsS+z8CZDfnWQpJSMHobTSPS5g4M/SCYe7zUjw
TcLCeoiKu7rPWRnWr4+wB7CeMfGCwcDfLqZtbBkOtdh+JhpFAz2weaSUKK0Pfybl
qAj+lug8aJRT7oM6iCsVlgmy4HqMLnXWnOunVmSPlk9orj2XwoSPwLxAwAtcvfaH
szVsrBhQf4TgTM2S0yDpM7xSma8ytSmzJSq0SPly4cpk9+aCEI3oncKKiPo4Zor8
Y/kB+Xj9e1x3+naH+uzfsQ55lVe0vSbv1gHR6xYKu44LtcXFilWr06zqkUspzBmk
MiVOKvFlRNACzqrOSbTqn3yDsEB750Orp2yjj32JgfpMpf/VjsPOS+C12LOORc92
wO1AK/1TD7Cn1TsNsYqiA94xrcx36m97PtbfkSIS5r762DL8EGMUUXLeXdYWk70p
aDPvOmbsB4om3xPXV2V4J95eSRQAogB/mqghtqmxlbCluQ0WEdrHbEg8QOB+DVrN
VjzRlwW5y0vtOUucxD/SVRNuJLDWcfr0wbrM7Rv1/oFB2ACYPTrIrnqYNxgFlQID
AQABo0IwQDAOBgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4E
FgQU5K8rJnEaK0gnhS9SZizv8IkTcT4wDQYJKoZIhvcNAQEMBQADggIBAJ+qQibb
C5u+/x6Wki4+omVKapi6Ist9wTrYggoGxval3sBOh2Z5ofmmWJyq+bXmYOfg6LEe
QkEzCzc9zolwFcq1JKjPa7XSQCGYzyI0zzvFIoTgxQ6KfF2I5DUkzps+GlQebtuy
h6f88/qBVRRiClmpIgUxPoLW7ttXNLwzldMXG+gnoot7TiYaelpkttGsN/H9oPM4
7HLwEXWdyzRSjeZ2axfG34arJ45JK3VmgRAhpuo+9K4l/3wV3s6MJT/KYnAK9y8J
ZgfIPxz88NtFMN9iiMG1D53Dn0reWVlHxYciNuaCp+0KueIHoI17eko8cdLiA6Ef
MgfdG+RCzgwARWGAtQsgWSl4vflVy2PFPEz0tv/bal8xa5meLMFrUKTX5hgUvYU/
Z6tGn6D/Qqc6f1zLXbBwHSs09dR2CQzreExZBfMzQsNhFRAbd03OIozUhfJFfbdT
6u9AWpQKXCBfTkBdYiJ23//OYb2MI3jSNwLgjt7RETeJ9r/tSQdirpLsQBqvFAnZ
0E6yove+7u7Y/9waLd64NnHi/Hm3lCXRSHNboTXns5lndcEZOitHTtNCjv0xyBZm
2tIMPNuzjsmhDYAPexZ3FL//2wmUspO8IFgV6dtxQ/PeEMMA3KgqlbbC1j+Qa3bb
bP6MvPJwNQzcmRk13NfIRmPVNnGuV/u3gm3c
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIFVzCCAz+gAwIBAgINAgPlrsWNBCUaqxElqjANBgkqhkiG9w0BAQwFADBHMQsw
CQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2VzIExMQzEU
MBIGA1UEAxMLR1RTIFJvb3QgUjIwHhcNMTYwNjIyMDAwMDAwWhcNMzYwNjIyMDAw
MDAwWjBHMQswCQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZp
Y2VzIExMQzEUMBIGA1UEAxMLR1RTIFJvb3QgUjIwggIiMA0GCSqGSIb3DQEBAQUA
A4ICDwAwggIKAoICAQDO3v2m++zsFDQ8BwZabFn3GTXd98GdVarTzTukk3LvCvpt
nfbwhYBboUhSnznFt+4orO/LdmgUud+tAWyZH8QiHZ/+cnfgLFuv5AS/T3KgGjSY
6Dlo7JUle3ah5mm5hRm9iYz+re026nO8/4Piy33B0s5Ks40FnotJk9/BW9BuXvAu
MC6C/Pq8tBcKSOWIm8Wba96wyrQD8Nr0kLhlZPdcTK3ofmZemde4wj7I0BOdre7k
RXuJVfeKH2JShBKzwkCX44ofR5GmdFrS+LFjKBC4swm4VndAoiaYecb+3yXuPuWg
f9RhD1FLPD+M2uFwdNjCaKH5wQzpoeJ/u1U8dgbuak7MkogwTZq9TwtImoS1mKPV
+3PBV2HdKFZ1E66HjucMUQkQdYhMvI35ezzUIkgfKtzra7tEscszcTJGr61K8Yzo
dDqs5xoic4DSMPclQsciOzsSrZYuxsN2B6ogtzVJV+mSSeh2FnIxZyuWfoqjx5RW
Ir9qS34BIbIjMt/kmkRtWVtd9QCgHJvGeJeNkP+byKq0rxFROV7Z+2et1VsRnTKa
G73VululycslaVNVJ1zgyjbLiGH7HrfQy+4W+9OmTN6SpdTi3/UGVN4unUu0kzCq
gc7dGtxRcw1PcOnlthYhGXmy5okLdWTK1au8CcEYof/UVKGFPP0UJAOyh9OktwID
AQABo0IwQDAOBgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4E
FgQUu//KjiOfT5nK2+JopqUVJxce2Q4wDQYJKoZIhvcNAQEMBQADggIBAB/Kzt3H
vqGf2SdMC9wXmBFqiN495nFWcrKeGk6c1SuYJF2ba3uwM4IJvd8lRuqYnrYb/oM8
0mJhwQTtzuDFycgTE1XnqGOtjHsB/ncw4c5omwX4Eu55MaBBRTUoCnGkJE+M3DyC
B19m3H0Q/gxhswWV7uGugQ+o+MePTagjAiZrHYNSVc61LwDKgEDg4XSsYPWHgJ2u
NmSRXbBoGOqKYcl3qJfEycel/FVL8/B/uWU9J2jQzGv6U53hkRrJXRqWbTKH7QMg
yALOWr7Z6v2yTcQvG99fevX4i8buMTolUVVnjWQye+mew4K6Ki3pHrTgSAai/Gev
HyICc/sgCq+dVEuhzf9gR7A/Xe8bVr2XIZYtCtFenTgCR2y59PYjJbigapordwj6
xLEokCZYCDzifqrXPW+6MYgKBesntaFJ7qBFVHvmJ2WZICGoo7z7GJa7Um8M7YNR
TOlZ4iBgxcJlkoKM8xAfDoqXvneCbT+PHV28SSe9zE8P4c52hgQjxcCMElv924Sg
JPFI/2R80L5cFtHvma3AH/vLrrw4IgYmZNralw4/KBVEqE8AyvCazM90arQ+POuV
7LXTWtiBmelDGDfrs7vRWGJB82bSj6p4lVQgw1oudCvV0b4YacCs1aTPObpRhANl
6WLAYv7YTVWW4tAR+kg0Eeye7QUd5MjWHYbL
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIICCTCCAY6gAwIBAgINAgPluILrIPglJ209ZjAKBggqhkjOPQQDAzBHMQswCQYD
VQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2VzIExMQzEUMBIG
A1UEAxMLR1RTIFJvb3QgUjMwHhcNMTYwNjIyMDAwMDAwWhcNMzYwNjIyMDAwMDAw
WjBHMQswCQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2Vz
IExMQzEUMBIGA1UEAxMLR1RTIFJvb3QgUjMwdjAQBgcqhkjOPQIBBgUrgQQAIgNi
AAQfTzOHMymKoYTey8chWEGJ6ladK0uFxh1MJ7x/JlFyb+Kf1qPKzEUURout736G
jOyxfi//qXGdGIRFBEFVbivqJn+7kAHjSxm65FSWRQmx1WyRRK2EE46ajA2ADDL2
4CejQjBAMA4GA1UdDwEB/wQEAwIBhjAPBgNVHRMBAf8EBTADAQH/MB0GA1UdDgQW
BBTB8Sa6oC2uhYHP0/EqEr24Cmf9vDAKBggqhkjOPQQDAwNpADBmAjEA9uEglRR7
VKOQFhG/hMjqb2sXnh5GmCCbn9MN2azTL818+FsuVbu/3ZL3pAzcMeGiAjEA/Jdm
ZuVDFhOD3cffL74UOO0BzrEXGhF16b0DjyZ+hOXJYKaV11RZt+cRLInUue4X
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIICCTCCAY6gAwIBAgINAgPlwGjvYxqccpBQUjAKBggqhkjOPQQDAzBHMQswCQYD
VQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2VzIExMQzEUMBIG
A1UEAxMLR1RTIFJvb3QgUjQwHhcNMTYwNjIyMDAwMDAwWhcNMzYwNjIyMDAwMDAw
WjBHMQswCQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2Vz
IExMQzEUMBIGA1UEAxMLR1RTIFJvb3QgUjQwdjAQBgcqhkjOPQIBBgUrgQQAIgNi
AATzdHOnaItgrkO4NcWBMHtLSZ37wWHO5t5GvWvVYRg1rkDdc/eJkTBa6zzuhXyi
QHY7qca4R9gq55KRanPpsXI5nymfopjTX15YhmUPoYRlBtHci8nHc8iMai/lxKvR
HYqjQjBAMA4GA1UdDwEB/wQEAwIBhjAPBgNVHRMBAf8EBTADAQH/MB0GA1UdDgQW
BBSATNbrdP9JNqPV2Py1PsVq8JQdjDAKBggqhkjOPQQDAwNpADBmAjEA6ED/g94D
9J+uHXqnLrmvT/aDHQ4thQEd0dlq7A/Cr8deVl5c1RxYIigL9zC2L7F8AjEA8GE8
p/SgguMh1YQdc4acLa/KNJvxn7kjNuK8YAOdgLOaVsjh4rsUecrNIdSUtUlD
-----END CERTIFICATE-----
)GTS";

// =====================================================
// 6. FIREBASE AUTHENTICATION
// =====================================================

static Preferences prefs;

static String idToken;
static String refreshToken;
static String firebaseUid;

static uint32_t renewAtMs=0;
static bool signedIn=false;
static int lastHttpStatus=0;

static bool https(
    const String &url,
    const String &data,
    const char *contentType,
    String &response,
    const char *verb="POST"
) {
    WiFiClientSecure client;
    client.setCACert(GOOGLE_ROOTS);

    HTTPClient http;

    response="";
    lastHttpStatus=0;

    if (!http.begin(client,url)) {
        lastHttpStatus=-100;
        Serial.println("# HTTP begin failed");
        return false;
    }

    http.setTimeout(10000);
    http.addHeader("Content-Type",contentType);

    int code=http.sendRequest(
        verb,(uint8_t*)data.c_str(),data.length()
    );

    lastHttpStatus=code;

    if (code>0) response=http.getString();

    http.end();

    if (code>=200 && code<300)
        return true;

    Serial.printf(
        "# HTTPS %s error: %d\n",verb,code
    );

    if (code>0 && response.length())
        Serial.println(response.substring(0,240));

    return false;
}

static bool parseAuth(
    const String &body,
    bool refresh
) {
    JsonDocument doc;

    if (deserializeJson(doc,body)) {
        Serial.println("# Invalid auth JSON");
        return false;
    }

    String newId =
        doc[refresh?"id_token":"idToken"].as<String>();

    String newRefresh =
        doc[refresh?"refresh_token":"refreshToken"]
        .as<String>();

    String uid =
        doc[refresh?"user_id":"localId"].as<String>();

    String expiry =
        doc[refresh?"expires_in":"expiresIn"]
        .as<String>();

    int seconds=expiry.toInt();
    if (seconds<=0) seconds=3600;

    if (newId.isEmpty() ||
        newRefresh.isEmpty() ||
        uid.isEmpty())
        return false;

    idToken=newId;
    refreshToken=newRefresh;
    firebaseUid=uid;

    prefs.putString("refresh",refreshToken);

    renewAtMs=millis()+
        (uint32_t)max(60,seconds-120)*1000UL;

    signedIn=true;

    Serial.printf(
        "# Firebase anonymous UID: %s\n",
        firebaseUid.c_str()
    );

    Serial.printf(
        "# Database path: /nanoTelemetry/%s/latest\n",
        firebaseUid.c_str()
    );

    return true;
}

static String urlEncode(const String &s) {
    const char* hex="0123456789ABCDEF";
    String out;

    for (size_t i=0;i<s.length();i++) {
        uint8_t c=(uint8_t)s[i];

        if ((c>='A'&&c<='Z') ||
            (c>='a'&&c<='z') ||
            (c>='0'&&c<='9') ||
            c=='-' || c=='_' ||
            c=='.' || c=='~') {
            out+=(char)c;
        } else {
            out+='%';
            out+=hex[c>>4];
            out+=hex[c&15];
        }
    }

    return out;
}

static bool signInFirebase() {
    String response;

    String prior=prefs.isKey("refresh")
        ? prefs.getString("refresh","")
        : "";

    // Same refresh-token flow as Nano's publish.py
    if (prior.length()) {
        String url=String(
            "https://securetoken.googleapis.com/v1/token?key="
        )+API_KEY;

        String body=
            "grant_type=refresh_token&refresh_token="+
            urlEncode(prior);

        if (https(
            url,body,
            "application/x-www-form-urlencoded",
            response
        ) && parseAuth(response,true))
            return true;

        // Do not discard UID on network/TLS failure.
        if (lastHttpStatus!=400 &&
            lastHttpStatus!=401)
            return false;
    }

    // Same anonymous sign-in endpoint as Nano.
    String url=String(
        "https://identitytoolkit.googleapis.com/v1/accounts:signUp?key="
    )+API_KEY;

    if (!https(
        url,
        "{\"returnSecureToken\":true}",
        "application/json",
        response
    ))
        return false;

    return parseAuth(response,false);
}

// =====================================================
// 7. TELEMETRY JSON — NANO RULES COMPATIBILITY
// =====================================================

static String packetJson(
    const Snapshot &s,
    uint32_t seq
) {
    JsonDocument doc;

    doc["schema_version"]=1;
    doc["source"]="esp32_mpu6050";
    doc["sequence"]=seq;

    // Legacy Nano field required by existing rules.
    // Both fields contain ESP32 millis().
    doc["nano_uptime_ms"]=s.uptime;
    doc["esp32_uptime_ms"]=s.uptime;

    JsonArray qa=
        doc["quaternion_wxyz"].to<JsonArray>();

    for (float v:s.quat) qa.add(v);

    JsonArray aa=
        doc["acceleration_g_xyz"].to<JsonArray>();

    for (float v:s.accel) aa.add(v);

    JsonArray ga=
        doc["gyro_dps_xyz"].to<JsonArray>();

    for (float v:s.gyro) ga.add(v);

    doc["stationary"]=s.stationary;

    doc["server_received_unix_ms"][".sv"]=
        "timestamp";

    String out;
    serializeJson(doc,out);
    return out;
}

// =====================================================
// 8. FIREBASE CLOUD TASK
// =====================================================

static void cloudTask(void *) {
    prefs.begin("fb_anon",false);

    uint32_t seq=0;
    uint32_t previous=0;
    uint32_t lastAttempt=0;
    uint32_t lastWifiAttempt=0;

    bool printedWifi=false;
    bool printedClock=false;

    for (;;) {
        if (WiFi.status()!=WL_CONNECTED) {
            signedIn=false;
            printedWifi=false;

            if (millis()-lastWifiAttempt>=10000) {
                Serial.println("# Reconnecting Wi-Fi");
                WiFi.reconnect();
                lastWifiAttempt=millis();
            }

            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (!printedWifi) {
            Serial.print("# Wi-Fi IP: ");
            Serial.println(WiFi.localIP());
            printedWifi=true;
        }

        // Clock must be set before verifying HTTPS.
        if (time(nullptr)<1793491200) {
            configTime(
                0,0,
                "pool.ntp.org",
                "time.google.com"
            );

            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (!printedClock) {
            time_t now=time(nullptr);
            Serial.print("# UTC clock: ");
            Serial.print(ctime(&now));
            printedClock=true;
        }

        if (!signedIn ||
            (int32_t)(millis()-renewAtMs)>=0) {

            if (!signInFirebase()) {
                Serial.println(
                    "# Firebase auth failed; retrying"
                );
                vTaskDelay(pdMS_TO_TICKS(3000));
                continue;
            }
        }

        if (millis()-lastAttempt<CLOUD_MS) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        lastAttempt=millis();

        Snapshot s;

        portENTER_CRITICAL(&sensorMux);
        s=latest;
        portEXIT_CRITICAL(&sensorMux);

        if (!s.valid || s.uptime==previous)
            continue;

        // Same UID-scoped path as Nano's publish.py.
        String url=String(DB_URL)+
            "/nanoTelemetry/"+firebaseUid+
            "/latest.json?auth="+idToken+
            "&print=silent";

        String response;

        if (https(
            url,packetJson(s,seq+1),
            "application/json",
            response,"PUT"
        )) {
            previous=s.uptime;

            Serial.printf(
                "# SENT seq=%lu uptime=%lu\n",
                (unsigned long)++seq,
                (unsigned long)previous
            );
        } else {
            if (lastHttpStatus==401)
                signedIn=false;

            if (lastHttpStatus==403)
                Serial.println(
                    "# Check nanoTelemetry rules"
                );

            vTaskDelay(pdMS_TO_TICKS(2500));
        }
    }
}

// =====================================================
// 9. ESP32 SETUP AND LOOP
// =====================================================

void setup() {
    Serial.begin(115200);
    delay(400);

    Serial.println(
        "# ESP32_NANO_AUTH_PEM_V1"
    );

    Serial.println(
        "# ESP32 MPU6050 -> Firebase Anonymous"
    );

    Wire.begin(SDA_PIN,SCL_PIN);
    Wire.setClock(100000);
    Wire.setTimeOut(100);

    while (!initMpu()) {
        Serial.println(
            "# MPU6050 retry in 2 seconds"
        );
        delay(2000);
    }

    while (!calibrate()) {
        Serial.println(
            "# Retry calibration in 2 seconds"
        );
        delay(2000);
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID,WIFI_PASSWORD);

    Serial.println("# Connecting Wi-Fi");

    BaseType_t result=xTaskCreatePinnedToCore(
        cloudTask,"firebase",12288,
        nullptr,1,nullptr,0
    );

    if (result!=pdPASS)
        Serial.println(
            "# ERROR starting Firebase task"
        );
}

void loop() {
    uint32_t now=micros();

    if ((uint32_t)(now-lastSampleUs)<SAMPLE_US) {
        delay(1);
        return;
    }

    uint32_t dtUs=now-lastSampleUs;
    lastSampleUs=now;

    float dt=dtUs*1.0e-6f;

    if (dt<0.002f || dt>0.15f)
        return;

    Imu raw;

    if (!readImu(raw)) {
        Serial.println(
            "# MPU6050 I2C read failed"
        );
        return;
    }

    Imu corrected=updateAttitude(raw,dt);

    if (++decimation>=2) {
        decimation=0;
        saveSnapshot(corrected);
    }
}
