<img width="593" height="362" alt="Screenshot 2026-11-04 050755" src="https://github.com/user-attachments/assets/5eca60ba-9c1b-44d9-a969-27716c93c03b" />


Nano MPU6050 Live Ground Station — V1/V2-Compatible
Project: IOT2.0 | Status: Bench-top physical telemetry dashboard; Firebase/OpenRocket expansion planned
Document version: 1.0 | Updated: 4 November 2026
Read this first: This README documents the V1/V2-compatible live dashboard. Your current working Arduino Nano firmware sends 12-field Q,... packets over USB Serial. The patched dashboard accepts them; you do not need to upload a new main.cpp just to use the dashboard. Future OpenRocket-to-Firebase work is a separate, staged development plan—not functionality that is already running in the physical Nano.

1. Purpose and present system
The aim is to display actual motion of a breadboard-mounted Arduino Nano V3 + MPU6050 using a rocket-themed ground station. The Nano estimates relative orientation, sends accelerometer, gyroscope and quaternion data to a Windows PC, and the Python dashboard displays live plots and rocket attitude graphics.
CURRENT, PHYSICAL BENCH TEST (working data path)
MPU6050 --I2C--> Arduino Nano V3 --USB serial / COM3--> Python dashboard
                                                     |-- live IMU plots
                                                     |-- pitch / yaw / axial-roll views
                                                     `-- simplified 3D rocket

FUTURE, SIMULATION/CLOUD PATH (development roadmap)
OpenRocket simulation --> CSV --> Python playback publisher
                                      |
                                      v
                             Firebase Realtime Database
                                      |
                                      v
                         Python/cloud ground station
Important distinction: The first path uses physical sensor readings. The second will use simulated flight data. Neither should be mislabeled as an RF-received rocket flight. Neither controls ignition, ejection, deployment, or other flight-critical actuators.
2. Current project files
Place ground_station/ next to src/, inside the existing, working PlatformIO project:
IOT2.0/
├── platformio.ini                  # keep your existing working Nano settings
├── src/
│   └── main.cpp                    # keep the current WORKING firmware
├── ground_station/
│   ├── dashboard.py                # ONE launch command for USB / demo / CSV
│   ├── live_dashboard.py           # V1 and V2 Serial receiver + live graphs
│   ├── attitude_math.py            # quaternion math and axis mapping
│   ├── telemetry.py                # OpenRocket CSV importer (offline mode)
│   ├── requirements.txt            # pyserial
│   └── synthetic_demo_not_openrocket.csv
└── tests/
    └── test_live.py                # optional automated receiver tests
The compatibility ZIP also contains reference copies of src/main.cpp and platformio.ini. Do not overwrite your own working files with these just to install the dashboard patch. The exact fix for the previous legacy 12-field packet message is in ground_station/live_dashboard.py.
3. Hardware and software
Item	Current use
Arduino Nano V3 / ATmega328P	Reads the IMU and outputs USB Serial data
MPU6050	3-axis accelerometer + 3-axis gyroscope
Nano A4 / A5	I2C SDA / SCL respectively
Common GND; compatible supply	Use voltage and I2C logic levels appropriate to your specific breakout; bare MPU6050 is not 5 V tolerant
USB data cable	Connects Nano to the Windows PC
VS Code + PlatformIO	Builds/uploads the existing firmware
Windows Python 3	Runs the dashboard; pyserial is the additional dependency


The Nano does not have built-in Wi-Fi. The current dashboard uses COM3, based on the latest observed connection; your Windows-assigned port could change after reconnection.
4. Start the live physical dashboard (VS Code)
From the VS Code terminal opened in the correct IOT2.0 project root:
1. Verify your Nano firmware is already uploaded and the MPU6050 is connected.
2. Close any PlatformIO Serial Monitor, Arduino Serial Monitor, or old rocket_viewer.py window using the port. Only one process can normally own COM3 at a time. Closing the monitor does not stop the firmware; keep the Nano powered by USB.
3. Install the Python dependency if needed:
   py -m pip install pyserial
4. Start the dashboard (current observed port):
   py ground_station/dashboard.py --port COM3 --nose-axis=+z
5. Leave the breadboard stationary at startup, because opening the serial port may reset the Nano and trigger calibration. Wait until Q,... packets begin.
Other useful commands:
# Start with the original eight-panel telemetry headings
py ground_station/dashboard.py --port COM3 --view telemetry

# Do not auto-open the additional 3D rocket window
py ground_station/dashboard.py --port COM3 --no-3d

# Record physical packets to CSV while showing the dashboard
py ground_station/dashboard.py --port COM3 --record ground_station/nano_live_log.csv

# Inspect the user interface without plugging in a Nano;
# demo data is SYNTHETIC, not physically measured and not from OpenRocket.
py ground_station/dashboard.py --demo

# Offline OpenRocket file playback; does NOT use COM3
py ground_station/dashboard.py --csv "C:\\path\\to\\openrocket_export.csv"
To open the monitor when diagnosing the Nano, use PlatformIO → Project Tasks → Monitor in VS Code. If pio is not recognized in PowerShell, use the installed executable directly:
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device monitor --port COM3 --baud 115200
Press Ctrl+C to close that monitor before launching Python. If an upload is needed, use the working environment/bootloader and the correct project folder; do not switch bootloader settings just to debug the Python dashboard.
5. Exactly which serial packets are supported?
Your current V1 firmware (12 CSV fields including Q):
# NANO MPU6050 3D ROCKET ORIENTATION DEMO
# MPU6050 WHO_AM_I=0x68, I2C addr=0x68
# CALIBRATING: keep board COMPLETELY STILL for ~1.5 seconds
# ORIGIN_ZEROED
# READY gyro bias dps: 0.079,-3.026,-0.269
Q,2366,1.000000,0.000027,0.000011,0.000002,0.153,0.009,1.092,0.02,0.06,0.02
Field order:
Q, time_ms, qw, qx, qy, qz, ax_g, ay_g, az_g, gx_deg_s, gy_deg_s, gz_deg_s
Newer V2 firmware (13 fields): same fields plus a final stationary flag (0 = moving, 1 = stationary).

Compatibility behavior:
- Both 12-field V1 and 13-field V2 packets are accepted by the patched live_dashboard.py.
- On V1, STATIONARY shows ? because the firmware does not transmit that flag. This is not a dashboard error.
- V1 transmits a calibration-relative, world-frame quaternion; the dashboard displays it without applying V2's sensor-to-model mounting transform. --nose-axis / --turn cannot fix V1 sensor mounting axes reliably. V2 supports those options.
- Q packets arrive approximately every 20 ms / 50 Hz from the current firmware. The GUI draws at a lower rate.
- # ... lines are status messages, not telemetry packets.
- Nano time is time since its boot, not an OpenRocket flight timestamp.
6. Dashboard functions and measurement limits
Control / view	What it does
IMU VIEW	Actual acceleration X/Y/Z and gyroscope-related trends
SHOW TELEMETRY VIEW	Original flight-style panel layout; missing sensors display N/A
PITCH / +X, YAW / +Y, AXIAL ROLL / +Z	Relative orientation angles calculated from incoming quaternion
3D ROCKET	Rotates simplified rocket geometry using relative attitude; not geographic position
ZERO ORIGIN (Z)	Requests the Nano to treat the present pose as a new orientation origin
CALIBRATE (C)	Re-estimates gyroscope bias; hold the board motionless
CLEAR GRAPHS	Clears local plot history without recalibrating the Nano
--record	Saves incoming data as a CSV on the PC


Unavailable from the present MPU6050-only setup: altitude, vertical velocity, air pressure, ambient air temperature, RSSI, latitude/longitude, and true rocket trajectory. Do not fill these with invented values. MPU6050-based yaw has no magnetometer/absolute heading reference and can drift over time; the V1 firmware also does not report stationary detection. ax=0.153, az=1.092 in your sample are measured accelerations in g, not position/height.
Troubleshooting quick reference
Symptom	Check
Dashboard shows 0 packets and legacy 12-field packet	Replace only ground_station/live_dashboard.py with the V1/V2-compatible file.
Repeated # NANO MPU6050... startup messages	The board may be resetting; verify USB power/data cable, reset behavior, MPU wiring and monitor connection. Repetition alone does not prove the cause.
Sensor initializes but no Q,... packets	Check for # CALIBRATING, # READY, sensor I2C errors; hold still during calibration.
Access denied / COM port busy	Close PlatformIO/Arduino Serial Monitor and the older Python viewer; check the current COM port.
Dashboard opens and immediately exits	Run from a terminal to read errors; confirm the working folder and pyserial installation.
Attitude moves but axes don't match the breadboard	Check how the MPU6050 is physically mounted. V1 mapping differs from V2; upgrade to V2 only when prepared to validate the new axis convention.
Telemetry headings show N/A	Expected: MPU6050 does not provide the required flight/atmospheric/radio channels.


7. Future development: OpenRocket simulation through Firebase
Overall goal
Repurpose this same ground-station dashboard so its data source can be switched from USB Serial to OpenRocket flight-simulation telemetry delivered via Firebase Realtime Database. Retain offline CSV playback as the debugging/reference path. This substitutes a cloud route for a proposed RF data link in the workshop demonstration; it does not demonstrate actual RF performance or flight communications resilience.
                   SIMULATED TELEMETRY / CLOUD PIPELINE

 OpenRocket (.ork)
       |
       | run simulation, export selected flight channels
       v
 OpenRocket-exported CSV
       |
       v
 Python CSV parser/normalizer --> Offline dashboard (validation baseline)
       |
       v
 Time-controlled publisher (simulated flight clock)
       |
       | HTTPS; authenticated user/token
       v
 Firebase Realtime Database
   /workshop/rocket/demo01/latest
   /workshop/rocket/demo01/events         [later phase, optional]
       |
       | authenticated snapshot reader / listener
       v
 Python ground-station data adapter
       |
       v
 Existing dashboard: graphs + phase/event labels + 3D orientation
       + trajectory view IF East/North/altitude data is available

 Later option: NFC tag -> phone app -> authenticated playback-control
 command -> Firebase commands -> PC publisher (simulation only)
Implementation phases and completion criteria
Phase	Work	Definition of done	Status
0 — Physical baseline	Nano + MPU6050 over USB; fix packet compatibility	Dashboard accepts V1/V2 packets, shows actual IMU and attitude values; save a short test log	Implemented in compatibility release; user-side live dashboard confirmation pending
1 — OpenRocket data export	Run one complete simulation, export time, altitude, vertical velocity, pressure, temperature, orientation, and East/North positions if supported	Save original CSV; document column headers/units and missing channels	Planned
2 — Offline replay baseline	Import CSV, map units and fields, replay samples against their simulation timestamps	Altitude/velocity plots match the exported CSV at key events; missing fields remain N/A	Offline reader exists; validate against your actual OpenRocket CSV
3 — Firebase publisher	Write a Python process that reads the validated CSV and publishes telemetry at a selected rate (e.g., 2–5 Hz initially), preserving source time and sequence number	Authorized publisher writes to a named demo session; latest changes in Firebase console; reconnect/error handling checked	Optional publisher code exists in separate OpenRocket package; not integrated with this live folder
4 — Firebase ground station	Read database updates, translate JSON into the same dashboard data model, display freshness/latency and source	Dashboard on a second computer displays the replay with correct units and source label; disconnect/reconnect and missing data handled	Optional cloud viewer exists separately; integration/validation pending
5 — Workshop demonstration	Compare local CSV, cloud replay and actual MPU6050 sources; introduce NFC/phone control of simulation playback only	Clearly distinct sources, controlled start/pause/reset, and no cloud path to physical ejection/deployment	Future enhancement


Export fields to request from OpenRocket
OpenRocket offers a CSV flight-data export under Flight simulations → Plot / export → Export data. Select the relevant variables when present in your OpenRocket version:
Parameter	Dashboard field	Notes
Time (s)	time_s	Required; authoritative simulated time
Altitude (m)	altitude_m	Flight-height trace
Vertical velocity (m/s)	vertical_velocity_m_s	Signed ascent/descent velocity
Air pressure (Pa)	pressure_pa	Dashboard may convert to mbar, Pa / 100
Air temperature (K or °C)	temperature_c	Record unit; for kelvin convert K - 273.15
Vertical orientation / zenith	zenith_rad	Tilt angle, if exported
Lateral orientation / azimuth	azimuth_rad	Direction of tilt, if exported
Roll angle or roll rate	roll_rad	If only rate is available, integration needs explicit assumptions
East / North of launch (m)	east_m, north_m	Needed for horizontal rocket-position/ground-track graphics
Events (launch, burnout, apogee, landing)	events	From simulation event export or derived/clearly labeled logic


Do not imply that azimuth/zenith are necessarily the same as a full 3-axis Euler pitch/yaw/roll orientation. For 3D reconstruction, use a clear, tested conversion and mark approximations. If only altitude is exported, display vertical motion, not a fake 3D geographical track. OpenRocket does not intrinsically provide measured RF RSSI, so show N/A for signal strength.
Proposed common telemetry JSON schema (design target, not current output)
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
The numbers above are illustrative schema examples, not measured Nano values or results of an OpenRocket flight. Use null rather than fabricated values for channels not exported. Keep source, session_id, schema_version, sequence, and timestamps so receivers can distinguish genuine fresh samples from repeated or stale ones. Add a server receive timestamp when implementing the cloud subscriber/publisher and specify which clock it refers to.
Firebase design and security
- Use Firebase Realtime Database, not Firestore, if reusing the existing Python REST-based publisher/client from the prior OpenRocket package.
- Create a test Firebase project; record the regional database URL and configure Firebase Authentication.
- Start with a publisher account that may write only its demo session and a viewer account that can only read. Define restrictive Realtime Database Security Rules; never set open public read/write rules for a classroom deployment.
- Store credentials in session environment variables or a local secret store, not in main.cpp, a checked-in platformio.ini, screenshots or GitHub.
- Use /latest for low-complexity monitoring, but remember that repeated writes overwrite history. With a polling subscriber, intermediate samples can be skipped. Add a bounded /events or /samples history path and replay/paging logic if lossless plots are required.
- Start with approximately 2–5 updates per second to simplify testing and control database usage. The Nano's 50 Hz USB stream is not the same as a 50 Hz Firebase transport guarantee.
- When Internet access fails, mark data STALE/DISCONNECTED. Continue local CSV replay as a separate fallback; never silently substitute synthetic live readings.
- Do not send Firebase/NFC/cloud commands to a rocket's ignition, CO2, ejection, separation, or other flight-critical actuators. For an NFC workshop exercise, use authenticated commands that operate a simulation player or a low-risk indicator only.
Planned Python interfaces (illustrative)
# Future abstraction: one dashboard, distinct telemetry sources.
class TelemetrySource:
    def next_frame(self):
        """Return validated canonical telemetry, or None when no update."""
        raise NotImplementedError

# SerialNanoSource    -> parses real Q packets (present)
# OpenRocketCSVSource -> reads/export-time replay (offline code present)
# FirebaseSource      -> receives authenticated cloud snapshots (future integration)
Do not mix the V1 Nano packet clock with an OpenRocket sim_time_s or claim a live Nano packet is an OpenRocket flight result. Show a visible SOURCE: NANO / OPENROCKET CSV / FIREBASE SIMULATION label in the final unified app.
8. Test/acceptance checklist
Current compatible USB version
- [ ] py ground_station/dashboard.py --port COM3 opens the live UI.
- [ ] PlatformIO Serial Monitor is closed, so COM3 is not locked.
- [ ] Nano # READY... is followed by Q,... lines.
- [ ] Bottom status changes from 0 packets to increasing received count.
- [ ] At rest, orientation changes only minimally; no requirement for a STATIONARY flag with V1.
- [ ] Moving the breadboard changes the expected attitude graphic.
- [ ] --record creates a readable CSV of real samples.
- [ ] No telemetry field is populated from a sensor not fitted.
Future Firebase/OpenRocket milestone
- [ ] CSV values and units validated against an OpenRocket export.
- [ ] Offline replay and cloud replay match at selected simulation times.
- [ ] Dataset/session/source labels and missing-value handling are correct.
- [ ] Ground station marks stale data during simulated network disconnect.
- [ ] Security rules prohibit unauthorized writes.
- [ ] No cloud command can trigger a physical deployment action.
9. Useful source documentation
- OpenRocket, advanced flight simulation and exporting data: https://openrocket.readthedocs.io/en/latest/user_guide/advanced_flight_simulation.html
- Firebase Realtime Database setup: https://firebase.google.com/docs/database/rest/start
- Firebase Realtime Database REST authentication: https://firebase.google.com/docs/database/rest/auth
- Firebase Realtime Database read/write listeners: https://firebase.google.com/docs/database/web/read-and-write
- Firebase Security Rules: https://firebase.google.com/docs/database/security
- Reference dashboard inspiration: https://johnjonesfour.com/2020/10/04/model-rocket-telemetry-part-2/
Version boundary: This README describes the V1/V2-compatible physical Nano USB ground station and a future OpenRocket → Firebase integration plan. It does not imply that the current Nano firmware, Firebase publisher, and Firebase ground-station client are already one integrated system.
