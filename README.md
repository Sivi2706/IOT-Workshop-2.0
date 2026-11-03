# Nano MPU6050 Live Ground Station — V1/V2-Compatible

**Project:** IOT2.0  
**Current status:** Physical Arduino Nano + MPU6050 telemetry over USB Serial  
**Future scope:** OpenRocket simulation → Firebase Realtime Database → ground station  
**Document version:** 1.0 · **Updated:** 4 November 2026

_Live bench-test ground station: the left window displays the simplified 3D rocket; the right window shows physical IMU plots and orientation graphics._  
[View the original GitHub-hosted screenshot](https://github.com/user-attachments/assets/5eca60ba-9c1b-44d9-a969-27716c93c03b)

> [!IMPORTANT]  
> **Current, working data path:** Arduino Nano V3 + MPU6050 → USB Serial → Python ground station. 
## 1. Purpose and present system

The purpose is to visualize movement of a breadboard-mounted **Arduino Nano V3 + MPU6050** in a rocket-themed ground station. The Nano estimates orientation and transmits accelerometer, gyroscope, and quaternion readings to a Windows PC, where Python displays live plots and the rocket's relative attitude.

### Current physical bench test

```
MPU6050 ──I²C──> Arduino Nano V3 ──USB Serial / COM3──> Python dashboard
                                                        ├── Live IMU plots
                                                        ├── Pitch / yaw / axial-roll views
                                                        └── Simplified 3D rocket
```

### Planned cloud simulation

```
OpenRocket simulation ──> CSV export ──> Python playback publisher
                                                │
                                         Firebase Realtime Database
                                                │
                                      Python cloud ground station
                                                ├── Flight graphs
                                                ├── 3D attitude
                                                └── Trajectory (if position data exists)
```

**Source distinction:** Physical sensor data must not be labeled OpenRocket simulation data. A Firebase-based simulation does not represent actual radio-frequency reception. Neither path controls ignition, CO₂ release, ejection, deployment, or other flight-critical hardware.

## 2. Current project files

Place `ground_station/` alongside `src/` in the **existing working PlatformIO project**:

```
IOT2.0/
├── platformio.ini                  # Keep your working Nano settings
├── src/
│   └── main.cpp                   # Keep the current working firmware
├── ground_station/
│   ├── dashboard.py               # Launch USB, demo, or CSV mode
│   ├── live_dashboard.py          # V1/V2 Serial receiver and live graphs
│   ├── attitude_math.py          # Quaternion math and axis mapping
│   ├── telemetry.py              # Offline OpenRocket CSV importer
│   ├── requirements.txt          # Python dependencies
│   └── synthetic_demo_not_openrocket.csv
├── assets/
│   └── live_ground_station.png    # Image used at the top of this README
└── tests/
    └── test_live.py              # Optional receiver tests
```

The compatibility release can contain reference copies of `src/main.cpp` and `platformio.ini`. **Do not overwrite your working copies** simply to install the receiver fix. The V1/V2 packet-compatibility change belongs in `ground_station/live_dashboard.py`.

## 3. Hardware and software

|Item|Current use|
|---|---|
|Arduino Nano V3 / ATmega328P|Reads the MPU6050 and outputs USB Serial data|
|MPU6050|3-axis accelerometer and 3-axis gyroscope|
|Nano A4|I²C SDA|
|Nano A5|I²C SCL|
|GND and suitable supply|Common ground; voltage appropriate for the sensor breakout|
|USB data cable|Connects the Nano to the Windows PC|
|VS Code + PlatformIO|Builds/uploads firmware and provides the Serial Monitor|
|Windows Python 3 + `pyserial`|Runs the dashboard and receives Serial packets|

> [!CAUTION]  
> Check the **specific MPU6050 breakout board** before powering it. A bare MPU6050 uses 3.3 V logic and is not 5 V tolerant. Add level shifting where required.

The Nano has **no built-in Wi-Fi**. The observed Serial port for this setup is **COM3**, but Windows may assign another port after reconnection.

## 4. Start the live physical dashboard in VS Code

From a terminal opened in the **correct `IOT2.0` project root**:

1. Keep the Nano connected by USB, with the MPU6050 wired and the working firmware uploaded.
    
2. Close the PlatformIO/Arduino **Serial Monitor** and any other Python program using COM3. Closing the monitor does **not** stop the Nano firmware.
    
3. Install the Python Serial library if it is not already installed:
    
    ```
    py -m pip install pyserial
    ```
    
4. Start the dashboard:
    
    ```
    py ground_station/dashboard.py --port COM3 --nose-axis=+z
    ```
    
5. Keep the breadboard still while the Nano restarts/calibrates after its Serial port opens. Once calibration finishes, valid `Q,...` packets should begin to arrive.
    

### Other commands

**Open the original eight-panel telemetry layout:**

```
py ground_station/dashboard.py --port COM3 --view telemetry
```

**Run without automatically opening the separate 3D rocket window:**

```
py ground_station/dashboard.py --port COM3 --no-3d
```

**Record physical sensor readings:**

```
py ground_station/dashboard.py --port COM3 --record ground_station/nano_live_log.csv
```

**Test the GUI without the Nano (synthetic demo, not OpenRocket data):**

```
py ground_station/dashboard.py --demo
```

**Replay an exported OpenRocket CSV locally (not through COM3):**

```
py ground_station/dashboard.py --csv "C:\path\to\openrocket_export.csv"
```

### Diagnosing Serial communication

Use **PlatformIO → Project Tasks → Monitor** in VS Code. If `pio` is not recognized in PowerShell, use the full executable path:

```
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device monitor --port COM3 --baud 115200
```

Press **Ctrl+C** to exit the monitor before starting the dashboard. Keep the Nano powered. If the firmware must be uploaded, use the already verified PlatformIO environment/bootloader and active project folder.

## 5. Serial packet formats and compatibility

### V1 firmware: 12 fields (including `Q`)

Your working Nano produced:

```
# NANO MPU6050 3D ROCKET ORIENTATION DEMO
# MPU6050 WHO_AM_I=0x68, I2C addr=0x68
# CALIBRATING: keep board COMPLETELY STILL for ~1.5 seconds
# ORIGIN_ZEROED
# READY gyro bias dps: 0.079,-3.026,-0.269
Q,2366,1.000000,0.000027,0.000011,0.000002,0.153,0.009,1.092,0.02,0.06,0.02
```

**V1 packet definition:**

```
Q,time_ms,qw,qx,qy,qz,ax_g,ay_g,az_g,gx_deg_s,gy_deg_s,gz_deg_s
```

### V2 firmware: 13 fields

The newer firmware appends `stationary` (0 = moving, 1 = stationary):

```
Q,time_ms,qw,qx,qy,qz,ax_g,ay_g,az_g,gx_deg_s,gy_deg_s,gz_deg_s,stationary
```

### Compatibility rules

- The patched `ground_station/live_dashboard.py` accepts **both** packet lengths.
- For **V1**, `STATIONARY: ?` is expected because no stationary flag is transmitted.
- **V1** sends a calibration-relative quaternion using its original reference convention. The V2 sensor-to-model mounting transform is not automatically applicable to V1. In particular, `--nose-axis` and `--turn` cannot reliably correct V1's mounting axes; validate an axis-mapping change against physical motion.
- The working V1 firmware outputs roughly one `Q,...` packet every **20 ms (50 Hz)**. Screen refresh may be slower.
- Lines beginning with `#` are **status messages**, not telemetry packets.
- `time_ms` measures time since Nano startup, **not** OpenRocket simulated flight time.

## 6. Dashboard functions and measurement limits

|Control or view|Function|
|---|---|
|**IMU VIEW**|Acceleration X/Y/Z and gyroscope-related live trends|
|**SHOW TELEMETRY VIEW**|Flight-style panels; missing measurements remain **N/A**|
|**Pitch / +X, Yaw / +Y, Axial roll / +Z**|Relative orientation angles calculated from incoming quaternion data|
|**3D ROCKET**|Simplified rocket rotates with estimated attitude; no geographic position|
|**ZERO ORIGIN (Z)**|Requests a new orientation zero from the Nano|
|**CALIBRATE (C)**|Re-estimates gyroscope bias; keep breadboard still|
|**CLEAR GRAPHS**|Clears local plot history without re-calibrating the Nano|
|`--record`|Saves incoming readings as a CSV on the PC|

**Not available from an MPU6050 alone:** altitude, vertical velocity, atmospheric pressure, ambient temperature, RSSI, latitude/longitude, or a physical rocket trajectory. Never fill these channels with invented values.

The MPU6050 has **no magnetometer**, so yaw/heading is not absolute and can drift. Example `ax=0.153 g` and `az=1.092 g` values are accelerometer measurements—not altitude or position.

### Troubleshooting

|Symptom|Action|
|---|---|
|Dashboard stays at 0 packets and reports **legacy 12-field packet**|Replace `ground_station/live_dashboard.py` with the compatible receiver file|
|Repeated `# NANO MPU6050...` startup messages|Check for USB/board resets, power stability, reset behavior, and wiring; repetition alone does not identify the cause|
|Sensor initializes but no `Q,...` lines arrive|Look for `# CALIBRATING`, `# READY`, and I²C errors; hold sensor still during calibration|
|COM port busy / access denied|Close the Serial Monitor and any other program using the same COM port|
|Dashboard immediately exits|Run from PowerShell to inspect the traceback, working directory, and Python dependencies|
|Orientation axes do not match the breadboard|Check physical sensor mounting. V1/V2 quaternion conventions differ; validate axes before changing firmware|
|Flight telemetry panels show N/A|Expected without additional sensors or simulated flight fields|

## 7. Future development — OpenRocket simulation through Firebase

### Overall goal

Reuse the same ground-station visual interface with a selectable data source: **physical USB Serial**, **offline OpenRocket CSV**, or **cloud-fed OpenRocket simulation**. The cloud path replaces the _proposed telemetry transport_ for a workshop demonstration; it is **not a test of RF performance or flight communications reliability**.

```
OpenRocket (.ork)
    │ Run simulation / choose export channels
    ▼
Exported CSV
    ├──────────────────> Offline dashboard (validation baseline)
    ▼
Python CSV parser + unit normalization
    ▼
Time-controlled playback publisher
    │ HTTPS + authenticated credentials
    ▼
Firebase Realtime Database
    ├── /workshop/rocket/demo01/latest
    └── /workshop/rocket/demo01/events     (optional future history)
    │ Authenticated snapshot reader / listener
    ▼
Ground-station adapter
    ▼
Existing dashboard
    ├── Flight graphs and events
    ├── 3D attitude (only where source data supports it)
    └── Trajectory (only with position data)

Future optional control path:
NFC tag → phone → authenticated simulation command → Firebase → PC publisher
```

### Implementation phases

|Phase|Work|Definition of done|Status|
|---|---|---|---|
|**0 — Physical baseline**|Nano + MPU6050 over USB; accept V1/V2 packets|Correct live IMU/attitude values and saved test log|Compatibility implemented; user-side dashboard confirmation pending|
|**1 — OpenRocket export**|Export a complete flight with selected channels|Preserve original CSV; document headers, units, and omissions|Planned|
|**2 — Offline replay**|Normalize units; replay by simulation timestamps|Altitude/velocity values agree with CSV at key events|Reader exists; validation against actual export pending|
|**3 — Firebase publisher**|Authenticated Python publisher; 2–5 Hz initial rate|Named session visible in Firebase; error/reconnect behavior checked|Separate prototype; not integrated here|
|**4 — Firebase ground station**|Receive cloud samples using same canonical data model|Second PC shows correct replay, freshness, and disconnection state|Separate prototype; integration pending|
|**5 — Workshop/NFC**|Compare sources; NFC controls simulation playback|Secure start/pause/reset; no cloud-controlled deployment outputs|Planned|

### Suggested OpenRocket export fields

Choose the relevant variables in **Flight simulations → Plot / export → Export data**, when available in your OpenRocket version:

|OpenRocket variable|Proposed canonical field|Notes|
|---|---|---|
|Time (s)|`sim_time_s`|Required simulation clock|
|Altitude (m)|`altitude_m`|Vertical height|
|Vertical velocity (m/s)|`vertical_velocity_m_s`|Signed ascent/descent rate|
|Air pressure (Pa)|`pressure_pa`|Display mbar as Pa ÷ 100|
|Air temperature (K or °C)|`temperature_c`|If kelvin, subtract 273.15|
|Vertical orientation / zenith|`zenith_rad`|Tilt, where exported|
|Lateral orientation / azimuth|`azimuth_rad`|Direction of tilt, where exported|
|Roll angle or roll rate|`roll_rad`|A rate alone needs integration assumptions|
|East / North of launch (m)|`east_m`, `north_m`|Needed for horizontal trajectory|
|Events|`events`|Source-provided or explicitly labeled derived events|

Azimuth and zenith are **not automatically identical** to a full three-axis Euler orientation. Reconstruct 3D attitude only using a documented and tested conversion. With only altitude, show a **vertical-height display**, not an invented geographic track. OpenRocket simulations do **not** contain measured RF RSSI.

### Proposed shared Firebase telemetry schema

_The example values below are illustrative, not observations from the Nano or an OpenRocket flight._

```
{
  "schema_version": 1,
  "session_id": "demo01",
  "source": "openrocket_simulation",
  "sequence": 42,
  "sim_time_s": 8.4,
  "altitude_m": 628.2,
  "vertical_velocity_m_s": 37.5,
  "pressure_pa": null,
  "temperature_c": null,
  "attitude": {
    "zenith_rad": null,
    "azimuth_rad": null,
    "roll_rad": null
  },
  "position": {
    "east_m": null,
    "north_m": null
  },
  "phase": "COAST",
  "rssi_dbm": null
}
```

Use `null` for unavailable fields. Include schema version, source, session ID, sequence number, and **well-defined timestamps** to distinguish fresh telemetry from repeated or stale samples. Server receipt time and simulation time refer to different clocks.

### Firebase design and security

- Use **Firebase Realtime Database** if reusing the earlier REST-based Python publisher/client; this is distinct from Firestore.
- Create a test Firebase project and record its **regional database URL**.
- Authenticate publishers and viewers separately; restrict write permissions to the publisher's session and read permissions to authorized viewers. **Do not enable open public read/write rules.**
- Store credentials in environment variables or a secret store, **not** in `main.cpp`, `platformio.ini`, screenshots, or a Git repository.
- A `/latest` node is simple but overwrites earlier data. Polling may miss intermediate samples. Add a bounded history or event store if complete plots are required.
- Start around **2–5 published updates per second** while validating latency and database usage; the Nano's 50 Hz USB cadence is not a Firebase delivery guarantee.
- Show **STALE/DISCONNECTED** after data becomes old. Do not silently substitute synthetic data as live measurement.
- **Never** connect Firebase/NFC controls to real ignition, CO₂, ejection, separation, or other flight-critical outputs. NFC can safely control a simulation player or indicator.

### Planned source abstraction

```
class TelemetrySource:
    def next_frame(self):
        """Return a validated canonical telemetry frame, or None."""
        raise NotImplementedError

# SerialNanoSource     -> physical Q packets (current)
# OpenRocketCSVSource  -> offline simulation replay (prototype)
# FirebaseSource       -> authenticated cloud snapshots (future integration)
```

The unified dashboard should prominently identify **SOURCE: NANO**, **SOURCE: OPENROCKET CSV**, or **SOURCE: FIREBASE SIMULATION**. Never merge Nano boot time with OpenRocket `sim_time_s` as though they were the same clock.

## 8. Acceptance checklist

### Current USB-compatible version

- [ ] `py ground_station/dashboard.py --port COM3` opens the live interface.
- [ ] PlatformIO/Arduino Serial Monitor is closed and COM3 is free.
- [ ] `# READY...` is followed by continuous `Q,...` data.
- [ ] Dashboard packet count increases beyond zero.
- [ ] Stationary orientation changes only minimally; V1 does not need a `stationary` flag.
- [ ] Physical breadboard rotations correspond to the attitude graphics.
- [ ] `--record` creates a readable physical telemetry CSV.
- [ ] Unavailable channels remain **N/A**.

### Future OpenRocket/Firebase milestones

- [ ] Exported CSV values and units are verified against OpenRocket.
- [ ] Offline and cloud replay agree at selected simulation timestamps.
- [ ] Source/session labels and missing fields are correct.
- [ ] Ground station marks stale telemetry after a network interruption.
- [ ] Firebase Security Rules block unauthorized writes.
- [ ] Cloud/NFC commands cannot trigger physical deployment hardware.

## 9. References and documentation

- [OpenRocket — Advanced flight simulation and export](https://openrocket.readthedocs.io/en/latest/user_guide/advanced_flight_simulation.html)
- [Firebase Realtime Database — REST API](https://firebase.google.com/docs/database/rest/start)
- [Firebase — REST authentication](https://firebase.google.com/docs/database/rest/auth)
- [Firebase — Read and write data](https://firebase.google.com/docs/database/web/read-and-write)
- [Firebase — Security Rules](https://firebase.google.com/docs/database/security)
- [Rocket telemetry dashboard inspiration — Model Rocket Telemetry, Part 2](https://johnjonesfour.com/2020/10/04/model-rocket-telemetry-part-2/)