# Live Arduino Nano / MPU6050 telemetry ground station

This is **the physical MPU6050 version** of the black eight-panel ground-station dashboard, adapted from the OpenRocket CSV viewer. It shows LIVE USB data from the **unchanged `src/main.cpp` V2 Nano firmware**, not simulation playback or Firebase. The firmware sends `Q` quaternion packets at 50 Hz and sensor samples at 100 Hz. Nothing in this project activates a servo or ejection system.

## Add to your existing PlatformIO project

Keep your **working** `IOT2.0/src/main.cpp` and `IOT2.0/platformio.ini` untouched. Copy this ZIP's `ground_station` folder into your current project so it is a sibling of `src`:

```text
IOT2.0/
├── platformio.ini           # your working existing file; don't replace
├── src/
│   ├── main.cpp             # your working existing V2 code; don't replace
│   └── ...                  # older Python files can stay
└── ground_station/
    ├── dashboard.py         # ONE entrypoint: CSV, demo, or live USB
    ├── live_dashboard.py    # live serial input and plots
    ├── attitude_math.py     # sensor mounting to rocket axes
    ├── telemetry.py         # existing OpenRocket CSV import
    └── synthetic_demo_not_openrocket.csv
```

The included `src/main.cpp` and `platformio.ini` are only a **reference copy** for creating a new complete project, not files you must overwrite in your already working Nano setup.

## Run on Windows / VS Code

1. Upload your already-working V2 firmware to the Nano through PlatformIO if not already loaded.
2. Close **PlatformIO Serial Monitor**, **Arduino Serial Monitor**, and any previous `rocket_viewer.py` process; COM4 can only be opened by one program at a time.
3. In the VS Code terminal, from the `IOT2.0` project root:

```powershell
py -m pip install pyserial
py ground_station/dashboard.py --port COM4 --nose-axis +z
```

The black dashboard and a separate 3D rocket window open, using **actual Nano data**. If you do not want the separate 3D rocket to appear initially, add `--no-3d`.

If the MPU6050's +X or +Y axis points towards the imaginary rocket nose instead, use:

```powershell
py ground_station/dashboard.py --port COM4 --nose-axis +x
py ground_station/dashboard.py --port COM4 --nose-axis +y --turn 90
```

The `--turn` argument accepts `0`, `90`, `180`, or `270` degrees. These are **fixed mounting offsets**, not filters or substitutions for measured movement. The Nano always reports the same sensor-frame quaternion; Python transforms it into a model-frame quaternion.

### What appears on the dashboard?

The **eight-cell grid** retains the flight-style design. It opens in **IMU VIEW** by default, because the physical MPU6050 can measure these quantities now:

| Top-left five charts (IMU VIEW) | Bottom-right three panels |
|---|---|
| Acceleration X, Y, Z (g) | Relative pitch / model X rotation |
| Gyroscope angular-speed magnitude (°/s) | Relative yaw / model Y rotation |
| Gyroscope Z (°/s) | Relative axial roll / model Z rotation |

Click **SHOW TELEMETRY VIEW** to see the original headings (signal, altitude, velocity, temperature, pressure, pitch, yaw, roll). The first five will show **N/A — sensor not installed**. Do **not** infer rocket altitude, vertical velocity, temperature, pressure or RF signal strength from a stationary MPU6050. Later those channels can be populated from a BMP280/GPS/LoRa module or the OpenRocket/Firebase source.

- **ZERO ORIGIN (Z):** commands the physical Nano to make its current orientation the new reference; graphs restart after the Nano confirms.
- **CALIBRATE (C):** the physical Nano recalibrates gyro bias (hold board absolutely motionless). It is normal for packets to pause for about 1.5 seconds.
- **3D ROCKET:** displays a 3D geometric attitude view synchronized with the real Nano quaternion. It is orientation, **not physical position**.
- **CLEAR GRAPHS:** only resets local plotting history; does not change the Nano's zero or calibration.
- The blue banner shows **LIVE SENSOR TIME**, not true rocket flight time.
- A status line reports `LIVE/STALE`, packet count, and stationary detection.

To save the raw live packet stream while displaying it:

```powershell
py ground_station/dashboard.py --port COM4 --record ground_station/nano_live_log.csv
```

## Keep your original OpenRocket modes

```powershell
py ground_station/dashboard.py --demo
py ground_station/dashboard.py --csv "C:\path\to\openrocket.csv"
```

The same dashboard entrypoint supports all three inputs. The Nano mode currently uses **USB Serial**, not RF and not Firebase. There is no dependence on internet/cloud for this demo.

## Technical data and limits

Expected V2 packet from `main.cpp`:

```text
Q,12345,1.000000,0.000000,0.000000,0.000000,0.010,-0.030,1.000,0.00,0.00,0.00,1
```

Fields are time in milliseconds since Nano boot; unit quaternion `(w,x,y,z)` relative to the firmware's calibration origin; accelerations in g; bias-corrected gyro speeds in degrees/s; and a 1/0 stationary flag. Packet reception and rendering are decoupled so the 50-Hz sender doesn't block the interface. The user interface plots an approximately 45-second rolling history and updates about 14 times/sec.

Because the MPU6050 lacks a magnetometer, its yaw/heading is **relative** and may drift. The displayed pitch/yaw/roll labels are **rotations about the model X/Y/Z axes**, derived from the quaternion for readability. The 3D rocket uses the full quaternion, avoiding Euler angle interpolation/gimbal lock for rendering. Body axis alignment is essential: use `--nose-axis` and `--turn` for the physical mount.

If you see **legacy 12-field packet** in status, your Nano still has old V1 firmware; upload the V2 firmware that outputs the additional stationary flag.

## Tests

```powershell
py -m unittest discover -s tests -v
```

A successful Python data/GUI test does not constitute validation on your physical Nano. This is a bench-top demonstrator, not a safety-critical flight computer.
