// Six-pin BMP280 I2C barometer. Both sensors share ESP32 GPIO21/22.
// The Adafruit driver handles Bosch pressure and temperature compensation.
#include "bmp280.h"
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <math.h>

static Adafruit_BMP280 bmp(&Wire);
static BaroData latest = {};
static float baselinePa = 0.0f;
static TaskHandle_t baroTaskHandle = nullptr;
static portMUX_TYPE baroMux = portMUX_INITIALIZER_UNLOCKED;

bool bmpBegin() {
  // Wire.begin(21,22) is already called by imuBegin() in setup().
  uint8_t address = 0;
  if (bmp.begin(0x76)) address = 0x76;
  else if (bmp.begin(0x77)) address = 0x77;
  else {
    Serial.println("# BMP280 not found at 0x76/0x77; barometer optional");
    return false;
  }

  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X4,
    Adafruit_BMP280::FILTER_X4,
    Adafruit_BMP280::STANDBY_MS_63
  );

  // Average a few static readings as the launch-pad altitude reference.
  float total = 0.0f;
  int count = 0;
  for (int i = 0; i < 12; ++i) {
    delay(80);
    float pressure = bmp.readPressure();
    if (isfinite(pressure) && pressure >= 30000.0f && pressure <= 110000.0f) {
      total += pressure;
      ++count;
    }
  }
  if (count < 5) {
    Serial.println("# BMP280 pressure baseline invalid");
    return false;
  }
  baselinePa = total / count;
  Serial.printf("# BMP280 detected at 0x%02X; reference pressure %.1f Pa\n",
                address, baselinePa);
  Serial.println("# BMP280 height is RELATIVE to startup, not GPS altitude");
  return true;
}

static void bmpTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    float pressure = bmp.readPressure();
    float temp = bmp.readTemperature();

    BaroData sample = {};
    sample.uptimeMs = millis();
    sample.referencePressurePa = baselinePa;
    sample.valid = isfinite(pressure) && isfinite(temp) &&
                   pressure >= 30000.0f && pressure <= 110000.0f &&
                   temp >= -40.0f && temp <= 85.0f && baselinePa > 0.0f;
    if (sample.valid) {
      sample.pressurePa = pressure;
      sample.temperatureC = temp;
      // Approximate barometric height above the initial sensor position.
      sample.altitudeM = 44330.0f *
          (1.0f - powf(pressure / baselinePa, 0.1903f));
    }
    portENTER_CRITICAL(&baroMux);
    latest = sample;
    portEXIT_CRITICAL(&baroMux);

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(50)); // target 20 Hz
  }
}

bool bmpStart() {
  if (baselinePa <= 0.0f) return false;
  if (baroTaskHandle) return true;
  return xTaskCreatePinnedToCore(
    bmpTask, "baro20hz", 4096, nullptr, 1, &baroTaskHandle, 1
  ) == pdPASS;
}

BaroData bmpGetLatest() {
  BaroData out;
  portENTER_CRITICAL(&baroMux);
  out = latest;
  portEXIT_CRITICAL(&baroMux);
  return out;
}
