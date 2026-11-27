"""Read ESP32-published Firebase packets as a virtual serial feed for the existing dashboard.

Set up:
    py ground_station/esp32_cloud_receiver.py --register
    # Add the printed viewer UID to the Firebase rules alongside ESP32 UID.
    py ground_station/dashboard.py --esp32-cloud --esp32-uid ESP32_UID --view imu

This module does not make the database public. It authenticates the viewer anonymously
and requires the Firebase database rules to permit that exact viewer UID to read the
exact ESP32 UID. Tokens are stored locally in ground_station/viewer_token.json.
"""
from __future__ import annotations

import argparse
import json
import math
import threading
import time
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parent.parent
CONFIG_FILE = ROOT / "firebase.config.json"
TOKEN_FILE = Path(__file__).resolve().parent / "viewer_token.json"


def request_json(method: str, url: str, data=None, *, form=False):
    body = None
    headers = {}
    if data is not None:
        if form:
            body = urlencode(data).encode("utf-8")
            headers["Content-Type"] = "application/x-www-form-urlencoded"
        else:
            body = json.dumps(data).encode("utf-8")
            headers["Content-Type"] = "application/json"
    request = Request(url, data=body, headers=headers, method=method)
    try:
        with urlopen(request, timeout=10) as response:
            return json.load(response)
    except HTTPError as exc:
        detail = exc.read(250).decode("utf-8", "replace")
        raise RuntimeError(f"Firebase HTTP {exc.code}: {detail}") from None
    except (URLError, TimeoutError) as exc:
        raise RuntimeError(f"Cannot reach Firebase: {exc}") from None


class ViewerSession:
    """Anonymous viewer identity, persisted across dashboard restarts."""
    def __init__(self):
        config = json.loads(CONFIG_FILE.read_text(encoding="utf-8-sig"))
        self.api_key = config["apiKey"].strip()
        self.base_url = config["databaseURL"].strip().rstrip("/")
        self.uid = ""
        self.token = ""
        self.refresh = ""
        self.expiry = 0.0
        if TOKEN_FILE.exists():
            saved = json.loads(TOKEN_FILE.read_text(encoding="utf-8"))
            self.refresh = saved.get("refresh_token", "")
            self.uid = saved.get("uid", "")
        self.ensure_token()

    def _save(self):
        TOKEN_FILE.write_text(
            json.dumps({"uid": self.uid, "refresh_token": self.refresh}, indent=2),
            encoding="utf-8",
        )

    def ensure_token(self):
        if self.token and time.monotonic() < self.expiry:
            return
        if self.refresh:
            url = "https://securetoken.googleapis.com/v1/token?" + urlencode({"key": self.api_key})
            try:
                reply = request_json("POST", url, {
                    "grant_type": "refresh_token",
                    "refresh_token": self.refresh,
                }, form=True)
                self.uid = reply["user_id"]
                self.token = reply["id_token"]
                self.refresh = reply["refresh_token"]
                self.expiry = time.monotonic() + max(30, int(reply["expires_in"]) - 120)
                self._save()
                return
            except RuntimeError as exc:
                # Network errors must never destroy the existing identity.
                if "HTTP 400" not in str(exc) and "HTTP 401" not in str(exc):
                    raise
                self.refresh = ""
        url = "https://identitytoolkit.googleapis.com/v1/accounts:signUp?" + urlencode({"key": self.api_key})
        reply = request_json("POST", url, {"returnSecureToken": True})
        self.uid = reply["localId"]
        self.token = reply["idToken"]
        self.refresh = reply["refreshToken"]
        self.expiry = time.monotonic() + max(30, int(reply["expiresIn"]) - 120)
        self._save()

    def latest(self, esp32_uid):
        self.ensure_token()
        path = f"/nanoTelemetry/{esp32_uid}/latest.json?" + urlencode({"auth": self.token})
        return request_json("GET", self.base_url + path)


def frame_to_packet(frame):
    """Convert Firebase telemetry back to the dashboard's existing Q packet schema."""
    if not isinstance(frame, dict):
        return None
    try:
        t = int(frame["esp32_uptime_ms"] if "esp32_uptime_ms" in frame else frame["nano_uptime_ms"])
        parts = [frame["quaternion_wxyz"], frame["acceleration_g_xyz"], frame["gyro_dps_xyz"]]
        if [len(v) for v in parts] != [4, 3, 3]:
            return None
        values = [float(v) for group in parts for v in group]
        if not all(math.isfinite(v) for v in values):
            return None
        still = int(bool(frame.get("stationary", False)))
        return f"Q,{t}," + ",".join(f"{v:.7g}" for v in values) + f",{still}"
    except (KeyError, ValueError, TypeError, OverflowError):
        return None


def frame_to_baro_packet(frame):
    """Optional BMP280 packet: B,uptime_ms,pressure_hPa,temperature_C,relative_alt_m."""
    if not isinstance(frame, dict):
        return None
    try:
        t = int(frame.get("esp32_uptime_ms", frame.get("nano_uptime_ms")))
        pa = float(frame["baro_pressure_pa"])
        c = float(frame["baro_temperature_c"])
        h = float(frame["baro_altitude_m"])
        if not all(math.isfinite(v) for v in (pa, c, h)) or pa <= 0:
            return None
        return f"B,{t},{pa / 100:.5f},{c:.4f},{h:.4f}"
    except (KeyError, TypeError, ValueError, OverflowError):
        return None


class Esp32CloudAdapter:
    """Implements the serial-like interface used by LiveGroundStation."""
    is_cloud = True

    def __init__(self, esp32_uid, poll_interval=0.5):
        self.esp32_uid = esp32_uid
        self.poll_interval = max(0.3, poll_interval)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._buffer = bytearray()
        self.is_open = True
        self.last_seen_seq = None
        self.sequence_gaps = 0
        self.cloud_state = "CONNECTING"
        self.last_error = None
        self._worker_thread = threading.Thread(target=self._worker, daemon=True)
        self._worker_thread.start()

    @property
    def in_waiting(self):
        with self._lock:
            return len(self._buffer)

    def read(self, size=1):
        with self._lock:
            result = bytes(self._buffer[:size])
            del self._buffer[:size]
        return result

    def write(self, _data):
        raise OSError("Firebase cloud view is read-only. Use ESP32 serial monitor for commands.")

    def close(self):
        self._stop.set()
        self.is_open = False
        self._worker_thread.join(timeout=0.5)

    def _push(self, line):
        with self._lock:
            if len(self._buffer) > 60000:
                self._buffer.clear()
            self._buffer.extend((line + "\n").encode("ascii", errors="replace"))

    def _worker(self):
        try:
            session = ViewerSession()
            self._push(f"# VIEWER_UID={session.uid} ESP32_UID={self.esp32_uid}")
            while not self._stop.is_set():
                try:
                    data = session.latest(self.esp32_uid)
                    if isinstance(data, dict):
                        seq = int(data["sequence"])
                        packet = frame_to_packet(data)
                        if packet is not None and seq != self.last_seen_seq:
                            if self.last_seen_seq is not None and seq > self.last_seen_seq+1:
                                self.sequence_gaps += seq-self.last_seen_seq-1
                            self.last_seen_seq = seq
                            self.cloud_state = "CONNECTED"
                            self.last_error = None
                            # Push BMP280 data before its matching IMU packet.
                            baro_packet = frame_to_baro_packet(data)
                            if baro_packet is not None:
                                self._push(baro_packet)
                            self._push(packet)
                        elif packet is None:
                            self.cloud_state = "INVALID PACKET"
                    else:
                        self.cloud_state = "NO DATA"
                except (RuntimeError, OSError, ValueError, KeyError) as exc:
                    self.last_error = str(exc)
                    self.cloud_state = "READ FAILED"
                    self._push(f"# Firebase read error: {exc}")
                self._stop.wait(self.poll_interval)
        except (RuntimeError, OSError, ValueError, KeyError) as exc:
            self.last_error = str(exc)
            self.cloud_state = "AUTH FAILED"
            self._push(f"# Viewer sign-in failed: {exc}")


def main():
    parser = argparse.ArgumentParser(description="Firebase anonymous viewer for ESP32 telemetry")
    parser.add_argument("--register", action="store_true", help="Register/read persistent viewer UID")
    parser.add_argument("--uid", help="ESP32 publisher UID for one-time packet test")
    args = parser.parse_args()
    try:
        session = ViewerSession()
        print("Ground-station anonymous viewer UID:", session.uid)
        if args.uid:
            frame = session.latest(args.uid)
            print("Latest telemetry:", json.dumps(frame, indent=2))
            print("Dashboard Q packet:", frame_to_packet(frame))
            print("Dashboard B packet:", frame_to_baro_packet(frame))
        elif not args.register:
            parser.print_help()
    except (RuntimeError, OSError, ValueError, KeyError) as exc:
        parser.exit(1, f"Firebase error: {exc}\n")

if __name__ == "__main__":
    main()
