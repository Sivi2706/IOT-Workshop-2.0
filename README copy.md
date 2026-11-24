# Nano to Firebase — Original Ground Station

The original eight-panel Python dashboard and 3D model run with the Arduino Nano V3 and MPU6050. The PC reads Nano packets over USB and uploads them to Firebase using Anonymous Authentication. The graphs receive each packet after Firebase confirms its write.

## Start on Windows

1. Open this folder in VS Code and upload `src/main.cpp` using PlatformIO if the Nano does not already emit V2 `Q,...` packets at 115200 baud. The default PlatformIO target uses the old Nano bootloader.
2. In Firebase Console for project `iot-2-e109b`, enable Anonymous Authentication and add the `nanoTelemetry` rule from `firebase.rules.json` to the existing Realtime Database rules. Reuse the existing rule if it is already present. The included `firebase.config.json` carries the package’s existing Web API key and database URL. Edit those two values together if you switch projects. The PC creates an anonymous session at startup and refreshes its ID token as needed.
3. Close Serial Monitor. Keep the MPU6050 still during startup. Run these commands from this project folder:

   ```powershell
   py -m pip install -r requirements.txt
   py ground_station/dashboard.py
   ```

The dashboard detects the Nano by checking for a valid packet. To select a known port, use `py ground_station/dashboard.py --firebase-port COM3`. The terminal prints the database path and confirmed sequence numbers. An internet connection is required for cloud mode.

## Other modes

- USB only, including sensor calibration and zeroing: `py ground_station/dashboard.py --port COM3`
- Offline sample playback (generates sample data on first run): `py ground_station/dashboard.py --demo`
- Existing OpenRocket CSV playback: `py ground_station/dashboard.py --csv "path/to/file.csv"`
- Keep the dashboard but hide the separate live 3D window at startup: `py ground_station/dashboard.py --no-3d`

Replace COM3 with the actual port. For sensor detection problems, verify that SDA is connected to A4, SCL to A5, ground is shared, and the sensor module has the appropriate supply. Reopen Serial Monitor at 115200 baud to read the sensor startup diagnostics, then close it before starting the dashboard again.
