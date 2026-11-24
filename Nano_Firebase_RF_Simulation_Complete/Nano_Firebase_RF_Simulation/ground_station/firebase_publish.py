"""Arduino Nano USB Serial -> Firebase RTDB RF-style telemetry packet simulator.

Run: py ground_station/firebase_publish.py --port COM3 --hz 5
USB Serial stays 50 Hz. This publisher intentionally sends a sampled 2-5 Hz cloud
stream and may optionally simulate packet loss, but it is NOT an RF radio.
"""
from __future__ import annotations

import argparse
import random
import time
import uuid

from attitude_math import parse_packet
from firebase_packets import RING_SIZE, encode
from firebase_transport import FirebaseREST, station_path


def publish_loop(port: str, baud: int, station: str, hz: float, drop_prob: float,
                 sim_latency_ms: float):
    import serial

    path = station_path(station)
    client = FirebaseREST('PUBLISHER')
    run_id = uuid.uuid4().hex[:12]
    period = 1.0 / hz
    last_send = time.monotonic()
    last_line = None
    last_nano_ms = None
    raw_packets = 0
    published = 0
    simulated_drops = 0
    failed = 0
    sequence = 0
    print(f'Firebase publisher for {station}; authenticating...')
    try:
        client.request('PATCH', path, payload={
            'latest': None, 'recent': None,
            'meta': {'session_id': run_id, 'source': 'physical_nano_mpu6050',
                     'transport': 'firebase_cloud_rf_simulation',
                     'started_unix_ms': {'.sv': 'timestamp'}}})
        print(f'Connected to Firebase as publisher UID {client.uid}; session={run_id}')
        print(f'Opening {port} at {baud} baud; keep MPU6050 still during calibration.')
        with serial.Serial(port, baudrate=baud, timeout=0.05) as device:
            while True:
                incoming = device.readline()
                if incoming:
                    line = incoming.decode('ascii', errors='replace').strip()
                    if line.startswith('#'):
                        print(f'NANO: {line}')
                        if 'ORIGIN_ZEROED' in line or 'CALIBRATING' in line:
                            last_line = None
                    elif line.startswith('Q,') and parse_packet(line) is not None:
                        raw_packets += 1
                        last_line = line
                        last_nano_ms = parse_packet(line)[0]
                now = time.monotonic()
                if now - last_send < period:
                    continue
                last_send = now
                if last_line is None:
                    continue
                # Every attempted simulated RF packet receives a sequence number;
                # intentional drops are visible as sequence gaps at the receiver.
                frame = encode(last_line, run_id, sequence, station=station)
                sequence += 1
                last_line = None  # never republish old physical measurements
                if random.random() < drop_prob:
                    simulated_drops += 1
                    if sequence % 10 == 0:
                        print(f'SIMULATED RF LOSS: {simulated_drops} packets dropped')
                    continue
                if sim_latency_ms:
                    time.sleep(sim_latency_ms / 1000.0)
                slot = (sequence - 1) % RING_SIZE
                try:
                    client.request('PATCH', path, payload={
                        'latest': frame,
                        f'recent/{slot:03d}': frame,
                    })
                    published += 1
                    if published == 1 or published % 10 == 0:
                        print(f'SENT seq={sequence-1:06d}, nano_ms={last_nano_ms}, '
                              f'published={published}, raw={raw_packets}, '
                              f'sim_dropped={simulated_drops}, errors={failed}')
                except (RuntimeError, ConnectionError) as exc:
                    failed += 1
                    print(f'FIREBASE ERROR: {exc}; next physical packet will be used')
    except KeyboardInterrupt:
        print('Publisher stopped.')
    finally:
        client.close()


def main():
    ap = argparse.ArgumentParser(description='Physical Nano -> Firebase RF-style packet simulation')
    ap.add_argument('--port', default='COM3', help='Nano USB serial port; change if necessary')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--station', default='bench01')
    ap.add_argument('--hz', type=float, default=5.0, help='Cloud updates per second (2-5 recommended)')
    ap.add_argument('--sim-drop', type=float, default=0.0,
                    help='Artificial packet loss probability from 0 to 1; default no added loss')
    ap.add_argument('--sim-latency-ms', type=float, default=0.0,
                    help='Artificial per-packet delay, not measured RF propagation time')
    args = ap.parse_args()
    if not 0 < args.hz <= 10:
        ap.error('--hz must be >0 and <=10 (Firebase update rate)')
    if not 0 <= args.sim_drop <= 1:
        ap.error('--sim-drop must be between 0 and 1')
    if not 0 <= args.sim_latency_ms <= 1000:
        ap.error('--sim-latency-ms must be between 0 and 1000')
    try:
        publish_loop(args.port, args.baud, args.station, args.hz,
                     args.sim_drop, args.sim_latency_ms)
    except (ValueError, OSError, RuntimeError, ConnectionError) as exc:
        ap.exit(1, f'PUBLISHER ERROR: {exc}\n')


if __name__ == '__main__':
    main()
