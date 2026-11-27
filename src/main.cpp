// ============================================================
// IOT WORKSHOP: ESP32 MPU6050 + BMP280 <-> FIREBASE
// Students: follow the numbered steps to see IoT data flow.
// ============================================================
#include <Arduino.h>
#include <ArduinoJson.h>
#include "mpu6050.h"
#include "bmp280.h"
#include "firebase.h"

constexpr unsigned long UPLOAD_EVERY_MS = 500; // target: 2 uploads/sec
unsigned long lastAttempt = 0;
uint32_t sequence = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n# IOT-WORKSHOP MODULAR ESP32 BMP280 V2");

  // STEP 1: Start the motion sensor (sampling continues separately).
  while (!imuBegin()) delay(2000);
  while (!imuCalibrate()) delay(2000); // Keep the MPU6050 still

  // STEP 2: Start the optional 6-pin BMP280 on the same I2C wires.
  // The IMU and Firebase still work if BMP280 is unplugged.
  bool bmpFound = bmpBegin();

  // Sensor background tasks start after both devices are initialized.
  if (!imuStart()) { Serial.println("# Cannot start IMU task"); while (true) delay(1000); }
  if (bmpFound && !bmpStart()) Serial.println("# Cannot start BMP280 task");

  // STEP 3: Connect to Wi-Fi and sign in anonymously to Firebase.
  // Firebase remembers its anonymous UID across restarts.
  wifiConnect();
  firebaseBegin();
  Serial.println("# Type R in Serial Monitor to RECEIVE latest data");
}

void loop() {
  // STEP 4: Every 500 ms, get the latest sensor measurements.
  if (millis() - lastAttempt >= UPLOAD_EVERY_MS) {
    lastAttempt = millis();
    ImuData imu = imuGetLatest();
    BaroData baro = bmpGetLatest();

    if (imu.valid) {
      // STEP 5: Build a JSON packet (this is what travels over Wi-Fi).
      JsonDocument packet;
      packet["sequence"] = sequence + 1;
      packet["source"] = baro.valid ? "esp32_mpu6050_bmp280" : "esp32_mpu6050";
      packet["nano_uptime_ms"] = imu.uptimeMs; // Nano-rule compatibility
      packet["esp32_uptime_ms"] = imu.uptimeMs;
      packet["stationary"] = imu.stationary;

      JsonArray q = packet["quaternion_wxyz"].to<JsonArray>();
      q.add(imu.qw); q.add(imu.qx); q.add(imu.qy); q.add(imu.qz);
      JsonArray a = packet["acceleration_g_xyz"].to<JsonArray>();
      a.add(imu.ax); a.add(imu.ay); a.add(imu.az);
      JsonArray g = packet["gyro_dps_xyz"].to<JsonArray>();
      g.add(imu.gx); g.add(imu.gy); g.add(imu.gz);
      // BMP280 reports pressure, temperature and height above power-on position.
      // If disconnected, omit these optional fields (no fake zeros).
      if (baro.valid) {
        packet["baro_pressure_pa"] = baro.pressurePa;
        packet["baro_temperature_c"] = baro.temperatureC;
        packet["baro_altitude_m"] = baro.altitudeM;
        packet["baro_reference_pressure_pa"] = baro.referencePressurePa;
      }

      // Firebase replaces this special field with its server timestamp.
      packet["server_received_unix_ms"][".sv"] = "timestamp";

      String json;
      serializeJson(packet, json);

      // STEP 6: SEND the packet with an HTTPS PUT request.
      // The database rules allow this anonymous user to write its own path.
      if (firebaseReady()) {
        String path = "/nanoTelemetry/" + firebaseUID() + "/latest";
        unsigned long started = millis();
        Serial.println("# SEND (HTTP PUT): " + path);
        int status = firebasePut(path, json);
        Serial.printf("# HTTP %d | %lu ms\n", status, millis() - started);
        if (status == 200 || status == 204) {
          ++sequence; // Only count confirmed writes
          Serial.printf("# SENT seq=%lu sensor_ms=%lu\n",
                        (unsigned long)sequence, (unsigned long)imu.uptimeMs);
        } else Serial.println("# Upload not confirmed; will retry next interval");
      }
    }
  }

  // STEP 7: RECEIVE data from Firebase using an HTTPS GET request.
  // In the Serial Monitor, type R and press Enter.
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'R' || command == 'r') {
      if (firebaseReady()) {
        String path = "/nanoTelemetry/" + firebaseUID() + "/latest";
        String received;
        Serial.println("# RECEIVE (HTTP GET): " + path);
        int status = firebaseGet(path, received);
        Serial.printf("# HTTP %d\n", status);
        if (status == 200) {
          Serial.println("# Firebase replied with JSON:");
          Serial.println(received);
        } else Serial.println("# GET failed; check Wi-Fi/authentication");
      } else Serial.println("# Cannot authenticate to Firebase yet");
    }
    while (Serial.available()) Serial.read(); // discard newline characters
  }
  delay(5);
}
