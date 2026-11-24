"""Publish Arduino Nano MPU6050 samples from USB serial to Firebase."""
import argparse
import json
import math
import time
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode, urlsplit
from urllib.request import Request, urlopen

from attitude_math import parse_packet

CONFIG_PATH = Path(__file__).resolve().parent.parent / "firebase.config.json"


def load_firebase_config():
    try:
        config = json.loads(CONFIG_PATH.read_text(encoding="utf-8-sig"))
        api_key = config["apiKey"].strip()
        database_url = config["databaseURL"].strip().rstrip("/")
        url = urlsplit(database_url)
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as error:
        raise RuntimeError(f"Check apiKey and databaseURL in {CONFIG_PATH.name}: {error}") from None

    if not api_key:
        raise RuntimeError(f"Set the Firebase Web API key in {CONFIG_PATH.name}")
    if url.scheme != "https" or not url.hostname or url.netloc != url.hostname or (
        not url.hostname.endswith((".firebaseio.com", ".firebasedatabase.app"))
        or url.path or url.query or url.fragment
    ):
        raise RuntimeError(f"Set databaseURL to the Firebase Realtime Database HTTPS root in {CONFIG_PATH.name}")
    return api_key, database_url


def request_json(method, url, payload, *, form=False):
    body = urlencode(payload).encode() if form else json.dumps(payload, allow_nan=False).encode()
    content_type = "application/x-www-form-urlencoded" if form else "application/json"
    request = Request(url, data=body, headers={"Content-Type": content_type}, method=method)
    try:
        with urlopen(request, timeout=8) as response:
            return json.load(response)
    except HTTPError as error:
        try:
            detail = json.load(error).get("error", {})
            message = detail.get("message", error.reason) if isinstance(detail, dict) else detail
        except (ValueError, AttributeError):
            message = error.reason
        raise RuntimeError(f"Firebase HTTP {error.code}: {message}") from None
    except (URLError, TimeoutError):
        raise RuntimeError("Firebase connection failed or timed out") from None


class AnonymousSession:
    def __init__(self):
        self.api_key, self.database_url = load_firebase_config()
        key = urlencode({"key": self.api_key})
        account = request_json(
            "POST", f"https://identitytoolkit.googleapis.com/v1/accounts:signUp?{key}",
            {"returnSecureToken": True},
        )
        self.uid = account["localId"]
        self.token = account["idToken"]
        self.refresh_token = account["refreshToken"]
        self.expiry = time.monotonic() + int(account["expiresIn"]) - 60

    def publish(self, frame):
        if time.monotonic() >= self.expiry:
            result = request_json(
                "POST", f"https://securetoken.googleapis.com/v1/token?{urlencode({'key': self.api_key})}",
                {"grant_type": "refresh_token", "refresh_token": self.refresh_token}, form=True,
            )
            self.token = result["id_token"]
            self.refresh_token = result["refresh_token"]
            self.expiry = time.monotonic() + int(result["expires_in"]) - 60
        query = urlencode({"auth": self.token})
        request_json("PUT", f"{self.database_url}/nanoTelemetry/{self.uid}/latest.json?{query}", frame)


def make_frame(line, sequence):
    try:
        parsed = parse_packet(line)
    except (ValueError, OverflowError, ZeroDivisionError):
        return None
    if parsed is None:
        return None
    uptime, quaternion, acceleration, gyro, stationary = parsed
    if uptime < 0 or not all(map(math.isfinite, (*quaternion, *acceleration, *gyro))):
        return None
    return {
        "sequence": sequence,
        "nano_uptime_ms": uptime,
        "quaternion_wxyz": list(quaternion),
        "acceleration_g_xyz": list(acceleration),
        "gyro_dps_xyz": list(gyro),
        "stationary": stationary,
        "server_received_unix_ms": {".sv": "timestamp"},
    }


def open_nano(selected_port, stop, notify):
    import serial
    from serial.tools.list_ports import comports

    candidates = [port for port in comports() if port.vid is not None]
    candidates.sort(key=lambda port: (port.vid not in {0x1A86, 0x0403, 0x10C4, 0x2341, 0x2A03}, port.device))
    names = [selected_port] if selected_port else [port.device for port in candidates]
    problems = []
    for name in names:
        if stop():
            break
        notify(f"Checking {name} at 115200 baud…")
        try:
            device = serial.Serial(name, 115200, timeout=0.1)
        except serial.SerialException as error:
            problems.append(f"{name}: {error}")
            continue
        matched = False
        last_line = "No serial output"
        try:
            deadline = time.monotonic() + 12
            while not stop() and time.monotonic() < deadline:
                line = device.readline().decode("ascii", errors="replace").strip()
                if line:
                    last_line = line
                if line.startswith("#"):
                    notify(f"{name}: {line}")
                if line.startswith("# FATAL"):
                    break
                if line.startswith("Q,") and make_frame(line, 1) is not None:
                    matched = True
                    return device
        finally:
            if not matched:
                device.close()
        problems.append(f"{name}: {last_line}")
    detail = " | ".join(problems) if problems else "No USB serial ports detected"
    raise RuntimeError(f"No valid Nano Q packet. {detail}")


def run_publisher(selected_port, hz, *, stop=lambda: False, notify=print, on_sample=None):
    notify("Opening Nano. Keep the MPU6050 still during startup calibration.")
    with open_nano(selected_port, stop, notify) as port:
        if stop():
            return
        notify(f"Connected to {port.port}. Signing in anonymously…")
        session = AnonymousSession()
        notify(f"Firebase path: /nanoTelemetry/{session.uid}/latest")
        sequence = 0
        next_send = 0.0
        while not stop():
            line = port.readline().decode("ascii", errors="replace").strip()
            if line.startswith("#"):
                notify(line)
            if not line.startswith("Q,") or time.monotonic() < next_send:
                continue
            frame = make_frame(line, sequence + 1)
            if frame is None:
                continue
            try:
                session.publish(frame)
                sequence += 1
                notify(f"SENT seq={sequence} | Nano uptime={frame['nano_uptime_ms']} ms")
                if on_sample is not None:
                    on_sample(frame, line)
            except RuntimeError as error:
                notify(str(error))
            finally:
                port.reset_input_buffer()
                next_send = time.monotonic() + 1 / hz


def parse_args(description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--port", help="Override USB auto-detection, e.g. COM3")
    parser.add_argument("--hz", type=float, default=2, help="Maximum Firebase updates per second (up to 5)")
    args = parser.parse_args()
    if not 0 < args.hz <= 5:
        parser.error("--hz must be greater than 0 and at most 5")
    return args


def main():
    args = parse_args(__doc__)
    try:
        run_publisher(args.port, args.hz)
    except KeyboardInterrupt:
        print("Stopped.")
    except (OSError, RuntimeError, ImportError, ValueError, KeyError) as error:
        raise SystemExit(f"Publisher error: {error}") from None


if __name__ == "__main__":
    main()
