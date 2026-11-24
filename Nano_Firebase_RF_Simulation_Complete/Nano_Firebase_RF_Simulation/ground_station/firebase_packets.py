"""Nano V1/V2 packet validation and cloud envelope conversion."""
from __future__ import annotations

import time
from attitude_math import parse_packet


SCHEMA_VERSION = 1
RING_SIZE = 120  # 120 packets (~24 s at 5 Hz) retained, not unbounded history


def encode(raw_line: str, session_id: str, sequence: int, *, station: str):
    parsed = parse_packet(raw_line)
    if parsed is None:
        raise ValueError('Not a valid Nano Q packet')
    nano_ms, quaternion, accel, gyro, stationary = parsed
    if nano_ms < 0 or sequence < 0:
        raise ValueError('Negative sequence or Nano timestamp')
    return {
        'schema_version': SCHEMA_VERSION,
        'station': station,
        'session_id': session_id,
        'sequence': sequence,
        'source': 'physical_nano_mpu6050',
        'transport': 'firebase_cloud_rf_simulation',
        'nano_uptime_ms': nano_ms,
        'client_sent_unix_ms': int(time.time() * 1000),
        'server_received_unix_ms': {'.sv': 'timestamp'},
        'quaternion_wxyz': [round(v, 8) for v in quaternion],
        'acceleration_g_xyz': list(accel),
        'gyro_dps_xyz': list(gyro),
        'stationary': stationary,
        'packet_csv': raw_line.strip(),
        'rf_metrics': {'rssi_dbm': None, 'snr_db': None, 'physical_radio_used': False},
    }


def decode(frame: dict):
    """Return original physical Nano CSV line or reject unsupported/untrusted frame."""
    if not isinstance(frame, dict):
        return None
    if frame.get('schema_version') != SCHEMA_VERSION:
        return None
    if frame.get('source') != 'physical_nano_mpu6050':
        return None
    line = frame.get('packet_csv')
    if not isinstance(line, str) or len(line) > 250:
        return None
    packet = parse_packet(line)
    if packet is None or packet[0] != frame.get('nano_uptime_ms'):
        return None
    return line
