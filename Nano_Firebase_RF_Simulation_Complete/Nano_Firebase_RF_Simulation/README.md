# Arduino Nano MPU6050 → Firebase → Ground Station (RF-style Packet Simulation)

This is a **working-source integration package** based on the uploaded
`Nano_MPU6050_Live_Ground_Station` project. It preserves the established
`main.cpp` (50 Hz USB `Q` packets) and the existing IMU graphs and 3D rocket
window. The older `IOT2.0` firmware also works because both its 12-field V1
and the newer 13-field V2 packets are accepted.

**Note:** A classic Nano has no Wi-Fi. The computer connected by USB acts as
the **Firebase transmitter bridge**. Firebase is a **cloud transport standing
in for an RF data link**, not a real radio. Real RSSI, SNR, over-the-air
packet loss and propagation latency are *not measured*.

```text
MPU6050 --I2C--> Arduino Nano --USB 115200--> PC 1 Python publisher
                                              |
                                              | HTTPS / Firebase Auth
                                              v
                          Firebase Realtime Database
                            /workshop/rocket/bench01/
                                latest
                                recent/000 ... recent/119
                                meta
                                              |
                                              | authenticated REST polling
                                              v
                             PC 2 (or PC 1): Python ground station
                                live IMU plots + 3D rocket
                                sequence gap / stale indication
```

## 1. Hardware

- Nano V3 (ATmega328P) + MPU6050: **SDA=A4, SCL=A5, GND=GND**.
- Supply and logic-level shifting must suit your particular MPU6050 breakout.
  Bare MPU6050 sensor inputs are 3.3 V, not 5 V tolerant.
- Nano USB into PC 1, Arduino driver and serial port working.
- PC 1 and PC 2 need internet access. They may be **the same PC** for a demo.
- The working firmware is already in `src/main.cpp`. No change or re-upload
  is necessary if `Q,...` packets are arriving from your Nano. Preserve your
  own working `platformio.ini` settings and upload/bootloader configuration.

## 2. Configure Firebase (Realtime Database, not Firestore)

1. Open [Firebase Console](https://console.firebase.google.com/), make a project,
   and create a **Realtime Database**. Copy the complete *database URL*; it
   may end in `.firebasedatabase.app` or `.firebaseio.com`.
2. Under **Authentication → Sign-in method**, enable **Email/Password**.
3. Under **Authentication → Users**, create two users: one publisher, one viewer.
   Copy their distinct **User UIDs**.
4. In **Realtime Database → Rules**, paste `firebase.rules.json` and replace
   `REPLACE_WITH_PUBLISHER_UID` and `REPLACE_WITH_VIEWER_UID` with the actual
   UIDs. Publish the rules. These rules deny public read/write, allow only the
   two identities to read, and only the publisher to write.
5. Get your Firebase **Web API key** under the Firebase project settings.
6. Copy `.env.example` to `.env` in this project's root. Fill in your Firebase
   URL, API key, and both users' email/password. `.env` is excluded by
   `.gitignore`. **Do not share `.env`, passwords, or Auth tokens.**

The demo grants these accounts access to `workshop/rocket/$station`. For a
larger deployment, scope each station to specific accounts and harden
schema validation, session management and credential distribution.

## 3. Install in VS Code / PowerShell

Open a terminal at this package's root, then:

```powershell
py -m pip install -r ground_station/requirements.txt
Copy-Item .env.example .env
notepad .env
```

Fill the `.env` file in Notepad, then save.

### Terminal 1: Nano-to-Firebase transmitter

Close PlatformIO Serial Monitor before starting (the COM port cannot be opened
by two apps at once). Attach Nano, keep it still during boot calibration, then:

```powershell
py ground_station/firebase_publish.py --port COM3 --station bench01 --hz 5
```

Swap `COM3` for the port on your Windows PC. You should see startup status and
lines similar to `SENT seq=000009 ...`. Firebase should contain
`workshop/rocket/bench01/latest` and up to 120 packets under `/recent`.

### Terminal 2: Firebase-to-ground-station receiver

On the same PC or on another PC with the same project and corresponding
Firebase viewer credentials:

```powershell
py ground_station/dashboard.py --firebase --station bench01 --view imu --nose-axis +z
```

This opens the **same eight-card IMU dashboard and 3D rocket view** used for
USB physical tests. Cloud packets preserve the source quaternion and gyro
measurements. The receiver disables `C` and `Z` commands because calibration
and zeroing need the local USB connection. No remote control of deployment
hardware is implemented.

Receiver flags include `CLOUD: CONNECTED`, `SEQ`, `GAPS` and `STALE`.
GAPS counts missing **simulated transport sequence numbers** from an observed
run. It does not mean RF packet error rate was physically measured.

### Optional controlled demonstration of dropped/delayed packets

```powershell
py ground_station/firebase_publish.py --port COM3 --station bench01 --hz 5 --sim-drop 0.15 --sim-latency-ms 100
```

`--sim-drop 0.15` independently skips approximately 15% of generated cloud
packets (randomized; not exactly 15% in any short run). Sequence numbers still
advance, so missing packets appear as gaps. `--sim-latency-ms 100` adds a
software delay; it is not real RF latency. These options are **disabled by
default**.

### Existing direct physical USB mode still works

Stop the publisher (Ctrl+C) first to release COM3:

```powershell
py ground_station/dashboard.py --port COM3 --nose-axis +z
```

### Other launch options

```powershell
py ground_station/dashboard.py --firebase --station bench01 --no-3d
py ground_station/dashboard.py --firebase --station bench01 --view telemetry
py ground_station/dashboard.py --firebase --station bench01 --record ground_station/cloud_capture.csv
py ground_station/dashboard.py --demo
```

## 4. Data packet structure

The Firebase node `workshop/rocket/bench01/latest` is a JSON packet with:

```json
{
  "schema_version": 1,
  "station": "bench01",
  "session_id": "abc123example",
  "sequence": 42,
  "source": "physical_nano_mpu6050",
  "transport": "firebase_cloud_rf_simulation",
  "nano_uptime_ms": 2466,
  "client_sent_unix_ms": 1790000000000,
  "server_received_unix_ms": 1790000000100,
  "quaternion_wxyz": [1, 0, 0, 0],
  "acceleration_g_xyz": [0.153, 0.009, 1.092],
  "gyro_dps_xyz": [0.02, 0.06, 0.02],
  "stationary": true,
  "packet_csv": "Q,2466,1.000000,0.000027,0.000011,0.000002,0.153,0.009,1.092,0.02,0.06,0.02,1",
  "rf_metrics": {"rssi_dbm": null, "snr_db": null, "physical_radio_used": false}
}
```

All values shown here are *illustrative*. The code uses a Firebase server-side
timestamp placeholder when writing, so `server_received_unix_ms` is set by
Firebase. Nano uptime, wall-clock time and Firebase receipt time are
**different clocks**. The dashboard uses Nano time for graphs and a local
monotonic clock for staleness.

The 50 Hz Nano USB serial stream is **sampled** at 5 Hz for cloud publishing
by default. The 120-slot ring retains up to ~24 seconds of published frames.
Packets older than this may be lost to a viewer that disconnects too long;
the viewer displays sequence gaps after resuming. This demo does not guarantee
lossless transmission and does not claim actual RF reliability.

## 5. Tests (offline)

```powershell
py -m unittest discover -s tests -v
```

## 6. Troubleshooting

| Symptom | Diagnosis |
|---|---|
| COM3 access denied | Close Serial Monitor or the old USB dashboard before publisher starts. |
| No `Q,...` packets | Check power/I2C and wait for Nano's still calibration. |
| Authentication error | Enable Email/Password and check `.env` user credentials and Web API key. |
| Firebase HTTP 401/403 | Check Realtime Database rules and the correct Auth user UID. |
| Query says `Index not defined` | Publish `firebase.rules.json` with `.indexOn: ["sequence"]` at `/recent`. |
| Graph stale | Check the publisher's terminal, internet connectivity, and latest sequence. |
| Viewer misses >120 frames | Ring buffer overwritten during disconnection; expected limitation. |
| No RSSI / altitude / pressure | MPU6050 cannot measure those; this link has no physical radio. |
| Ground station app won't start | Run from PowerShell to inspect traceback; ensure Python Tkinter is installed. |

Official technical references:
- [Firebase Realtime Database REST API](https://firebase.google.com/docs/database/rest/start)
- [Firebase REST authentication](https://firebase.google.com/docs/database/rest/auth)
- [Firebase Email/Password Auth REST API](https://firebase.google.com/docs/reference/rest/auth)
- [Firebase security rules](https://firebase.google.com/docs/database/security)

**Safety boundary:** This is a workshop/bench-test telemetry visualization.
Nothing in the cloud is wired to ignition, ejection, deployment, separation,
pyrotechnics or other flight-critical control outputs.
