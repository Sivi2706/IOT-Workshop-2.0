# IOT-Workshop-2.0 — ESP32 + MPU6050 + 6-pin BMP280 → Firebase → Ground Station

A beginner-focused ESP32 firmware with a **readable `src/main.cpp`** that shows
JSON construction, **HTTP PUT** (send), HTTP status, and **HTTP GET** (receive),
plus BMP280 pressure, temperature and relative altitude.

Original Nano USB publisher and Python dashboard retained from the supplied project.

## Folder structure

```text
IOT_Workshop_2_Beginner_Modular/
  platformio.ini
  firebase.config.json
  firebase.rules.json
  requirements.txt
  Nano_original_reference.cpp       # reference only (NOT compiled)
  src/
    main.cpp                        # BEGIN HERE: send/receive demonstration
    mpu6050.cpp / mpu6050.h         # I2C + quaternion calculations
    bmp280.cpp / bmp280.h           # Barometer, temperature, relative altitude
    firebase.cpp / firebase.h       # verified HTTPS + anonymous auth
    secrets.h / secrets.example.h   # enter Wi-Fi locally
  ground_station/
    dashboard.py                    # original dashboard, + ESP32 cloud option
    esp32_cloud_receiver.py         # anonymous read-only cloud viewer
    ...                             # original Nano publisher/helper scripts
```

## Hardware

ESP32 DevKit (ESP-WROOM-32) → GY-521 MPU6050:

| ESP32 | MPU6050 module |
|---|---|
| GPIO21 | SDA |
| GPIO22 | SCL |
| 3V3 | VCC |
| GND | GND |

The MPU6050 is probed at **0x68**. Mount the sensor securely and keep it stationary
while calibrating. Use a **2.4 GHz Wi-Fi** network.

## 6-pin BMP280 wiring — same I2C bus as the MPU6050

Connect **both** sensor boards to the same SDA (GPIO21) and SCL (GPIO22) lines:

| ESP32 pin | GY-521 MPU6050 | 6-pin BMP280 module |
|---|---|---|
| 3V3 | VCC | VCC / VIN (3.3 V recommended) |
| GND | GND | GND |
| GPIO21 | SDA | SDA / SDI |
| GPIO22 | SCL | SCL / SCK |
| 3V3 | — | CSB / CS (tie HIGH for I2C) |
| GND | — | SDO (choose address **0x76**) |

**Important:** Use the printed pin labels on your BMP280 board; 6-pin breakout
pin **order varies**. If your board has built-in CSB and SDO pull-ups,
connecting SDO to 3V3 gives **0x77** instead. Firmware detects both.
Do **not** feed 5 V to a bare 3.3 V-only BMP280 module; some breakouts have
regulators, others do not. A 3.3 V supply is a safer default for this ESP32 lab.
If your 6-pin module has its own I2C pull-ups, they must pull to 3.3 V, not 5 V.
If your module already ties CSB HIGH internally, leave it as-is; an external
connection is not needed. If the SDO pin already has a pull-up or pull-down,
use that defined state or wire SDO explicitly to select the address.

**What the barometer measures:** atmospheric pressure (Pa) and chip temperature
(°C). The program averages pressure during startup and calculates
**relative altitude (m)** from subsequent pressure changes. This is an
approximate pressure-based height change, NOT GPS altitude or absolute
height above sea level; wind, pressure drift and indoor air effects matter.
Keep the BMP280 still for a second at startup to establish its reference.
It samples at about **20 Hz** independently of the network's target 2 Hz upload.
If the BMP280 is disconnected, the IMU/Firebase path still operates and
BMP280 JSON fields are **omitted**, not replaced by misleading zeros.

## First-time installation (Windows, VS Code + PlatformIO)

1. **Make a backup** of your existing working PlatformIO project.
2. Extract this ZIP into a new folder and open **that folder** in VS Code.
3. Edit `src/secrets.h`: replace Wi-Fi placeholders with the local SSID and password.
   Do not commit `src/secrets.h` to Git.
4. Attach the ESP32 on COM5 (change `upload_port` in `platformio.ini` if needed).
5. From the PlatformIO Terminal, run:

```powershell
pio run -e esp32_firebase
pio run -e esp32_firebase -t upload --upload-port COM5
pio device monitor -p COM5 -b 115200
```

If `pio` isn't on PATH, use:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32_firebase
```

**Expected messages:** Wi-Fi connected, Firebase UID, then `# SEND (HTTP PUT)`,
`# HTTP 204`, and `# SENT`. The HTTP 204 response means the server accepted the
write and returned **no body**. The code avoids `getString()` on a 204 response.

While the monitor is open, type **R** and press Enter to execute an **HTTP GET**.
The ESP32 should report HTTP 200 and print the JSON received from Firebase.

**IMPORTANT:** The sketch targets **Arduino-ESP32 2.0.17** through PlatformIO's
`espressif32` platform. It uses the verified `setCACert()` root PEM approach that
worked in the original ESP32_NANO_AUTH_PEM_V1 firmware. It does not use
`ESP32CertBundle` and does not disable certificate checking.

## Why the main.cpp is beginner-friendly

1. `imuGetLatest()` returns a snapshot while a **separate 100 Hz sensor task**
   continues recording measurements. `bmpGetLatest()` reads a separate 20 Hz
   barometer snapshot.
2. Students assemble `JsonDocument` fields into a telemetry packet, including
   optional `baro_pressure_pa`, `baro_temperature_c`, `baro_altitude_m` and
   `baro_reference_pressure_pa`. The existing Nano-compatible required fields
   are preserved.
3. `firebasePut(path, json)` sends the packet over HTTPS (REST API).
4. `HTTP 204` confirms the database accepted the upload; packet sequence increments
   **only for confirmed uploads**.
5. Typing **R** calls `firebaseGet()` (HTTPS GET) to retrieve that same JSON.
6. The goal is **2 Hz**, not a guaranteed hard-real-time 500 ms rate; HTTPS
   setup, network latency and Firebase can delay each request. The firmware prints
   actual elapsed request duration.

## Firebase anonymous identity and access rules

The ESP32 uses the same **anonymous Firebase REST authentication** as the original
Nano Python publisher, and stores its refresh token in ESP32 nonvolatile storage.
Its telemetry is written to:

`/nanoTelemetry/ESP32_ANONYMOUS_UID/latest`

This intentionally retains the Nano packet's `nano_uptime_ms` field for your
existing schema validation, even though its value is actually ESP32 uptime.
The packet also contains `esp32_uptime_ms` to clarify the source.

* Your existing Nano rules already allow the **ESP32 itself** to read and write
  its **own** `/nanoTelemetry/UID/latest` record. You do **not** need to change
  the rules to demonstrate HTTP PUT and GET on the same ESP32.
* The **Python ground station** signs in with its **own** anonymous UID, so it
  cannot read another device's data unless you explicitly authorize it.
* To use the cloud dashboard, use the provided `firebase.rules.json` and replace
  **both** `REPLACE_ESP32_UID` and `REPLACE_VIEWER_UID` with the corresponding
  real UIDs before publishing those rules in Firebase Console → Realtime Database
  → Rules. Do not publish placeholder values or use `".read": true`.

## Python ground station (no USB required for ESP32 data)

Install original dashboard requirements:

```powershell
py -m pip install -r requirements.txt
```

Register a persistent **anonymous viewer account**:

```powershell
py ground_station/esp32_cloud_receiver.py --register
```

The command prints a viewer UID. Replace `REPLACE_VIEWER_UID` in the supplied
rules, and replace `REPLACE_ESP32_UID` with the UID printed by your ESP32 after
login. Publish the updated rules in Firebase Console.

Launch the existing live graphs / 3D viewer:

```powershell
py ground_station/dashboard.py --esp32-cloud --esp32-uid YOUR_ESP32_UID --view imu --nose-axis +z
```

To troubleshoot rules without the GUI:

```powershell
py ground_station/esp32_cloud_receiver.py --uid YOUR_ESP32_UID
```

This displays a JSON snapshot if the viewer has read permission. The Python
receiver stores its refresh token locally at `ground_station/viewer_token.json`.
That file is excluded from Git and should remain private.

For the ESP32 cloud dashboard, switch to **SHOW TELEMETRY VIEW** to see live
BMP280 **ALTITUDE** (relative to start), **PRESSURE** (hPa) and
**TEMPERATURE** (°C). The cloud receiver converts Firebase JSON into the
original `Q` IMU packet and an optional `B` barometer packet. Older Nano-only
streams remain compatible. Vertical velocity and RSSI are **not invented**;
they remain N/A.

The **original Nano dashboard modes** still work, e.g.:

```powershell
py ground_station/dashboard.py --demo
py ground_station/dashboard.py --firebase
py ground_station/dashboard.py --port COM3
```

## Workshop troubleshooting

| Symptom | What to check |
|---|---|
| Sensor not found | MPU6050 address 0x68, SDA=21, SCL=22, 3V3, GND |
| BMP280 not found | Check CSB HIGH, SDO LOW (0x76) or HIGH (0x77), 3V3 power and shared I2C wiring |
| Wi-Fi timeout | Network is 2.4 GHz and credentials are correct |
| TLS X509 error | NTP clock and Google root certificate chain |
| HTTP 401 / 403 | Anonymous Auth enabled; UID matches Firebase Rules |
| HTTP 204 | Successful upload without response body |
| HTTP 200 on GET | Successful read and JSON response |
| ~76 seconds between writes | Never call getString() for HTTP 204 (already fixed) |
| Viewer HTTP 401 | Viewer token expired; re-register if needed |
| Viewer HTTP 403 | Viewer UID not authorized for that specific ESP32 UID |

## Security and scope

This is a **bench demonstration using Wi-Fi instead of radio**. Not flight-tested
rocket avionics. The public Firebase Web API key identifies the project, but
Wi-Fi credentials, refresh tokens and ID tokens must remain private. Avoid
`setInsecure()` for authenticated requests. Trusted roots might need updating
if Google's deployed certificate chain changes.

## Verification

Python scripts were syntax-tested, including the optional BMP280 cloud packet;
packet conversion was checked against the
original `attitude_math.parse_packet`, and the packaged source files were checked
for expected entry points. **Hardware upload, real-world 2 Hz rate and Firebase
cloud dashboard access must still be validated on your ESP32/PC.**
