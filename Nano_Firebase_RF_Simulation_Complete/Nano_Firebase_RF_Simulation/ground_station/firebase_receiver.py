"""Background Firebase poller exposed as a read-only serial-like object.

Reuses the verified NanoSerialPump, LiveFlight and 3D viewer without making
network calls on Tkinter's UI thread. Does NOT implement radio receive hardware.
"""
from __future__ import annotations

import json
import threading
import time

from firebase_packets import RING_SIZE, decode
from firebase_transport import FirebaseREST, station_path


class FirebaseSerialAdapter:
    """Serial-compatible, non-commandable byte stream fed from Firebase RTDB."""
    is_cloud = True

    def __init__(self, station='bench01', poll_interval=0.35):
        self.base = station_path(station)
        if poll_interval < 0.15:
            raise ValueError('poll_interval must be at least 0.15 seconds')
        self.poll_interval = poll_interval
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._buffer = bytearray()
        self.is_open = True
        self.last_error = None
        self.last_seen_seq = None
        self.sequence_gaps = 0
        self.active_session = None
        self.cloud_state = 'CONNECTING'
        self.thread = threading.Thread(target=self._worker, name='FirebaseReceiver', daemon=True)
        self.thread.start()

    @property
    def in_waiting(self):
        with self._lock:
            return len(self._buffer)

    def read(self, size=1):
        with self._lock:
            data = bytes(self._buffer[:size])
            del self._buffer[:size]
        return data

    def write(self, _data):
        raise OSError('Firebase cloud receiver is read-only; C/Z commands require local USB')

    def close(self):
        self.is_open = False
        self._stop.set()
        if self.thread.is_alive():
            self.thread.join(timeout=0.75)

    def _push(self, line):
        with self._lock:
            self._buffer.extend((line.rstrip('\r\n') + '\n').encode('ascii', errors='replace'))
            if len(self._buffer) > 60000:
                self._buffer.clear()
                self._buffer.extend(b'# WARNING: firebase UI queue overflow; data discarded\n')

    def _consume(self, snapshots):
        if not isinstance(snapshots, dict):
            return
        frames = sorted((x for x in snapshots.values()
                         if isinstance(x, dict) and type(x.get('sequence')) is int),
                        key=lambda x: x.get('sequence', -1))
        for frame in frames:
            if frame.get('session_id') != self.active_session:
                continue
            sequence = frame.get('sequence')
            if type(sequence) is not int or sequence < 0:
                continue
            if self.last_seen_seq is not None and sequence <= self.last_seen_seq:
                continue
            line = decode(frame)
            if line is None:
                continue
            if self.last_seen_seq is not None:
                self.sequence_gaps += max(0, sequence - self.last_seen_seq - 1)
            self.last_seen_seq = sequence
            self._push(line)

    def _worker(self):
        client = None
        try:
            client = FirebaseREST('VIEWER')
            while not self._stop.is_set():
                try:
                    latest = client.request('GET', self.base + '/latest')
                    if not isinstance(latest, dict):
                        self.cloud_state = 'WAITING FOR PUBLISHER'
                        self._stop.wait(self.poll_interval)
                        continue
                    session_id = latest.get('session_id')
                    latest_seq = latest.get('sequence')
                    if not isinstance(session_id, str) or type(latest_seq) is not int:
                        raise ValueError('Malformed Firebase /latest packet')
                    if session_id != self.active_session:
                        self.active_session = session_id
                        with self._lock:
                            self._buffer.clear()  # discard queued previous-session bytes
                        self.last_seen_seq = None
                        self.sequence_gaps = 0
                        self._push('# ORIGIN_ZEROED: NEW FIREBASE SESSION')
                    # Query only newer records, so every packet can be plotted.
                    # The 120-slot ring bounds storage; long outages may lose history.
                    first_seq = max(0, latest_seq - (RING_SIZE - 1)) if self.last_seen_seq is None else self.last_seen_seq + 1
                    query = {'orderBy': json.dumps('sequence'),
                             'startAt': str(first_seq), 'limitToFirst': str(RING_SIZE)}
                    recent = client.request('GET', self.base + '/recent', params=query)
                    self._consume(recent)
                    self.cloud_state = 'CONNECTED'
                    self.last_error = None
                    self._stop.wait(self.poll_interval)
                except (RuntimeError, ConnectionError, ValueError, KeyError) as exc:
                    self.cloud_state = 'NETWORK ERROR'
                    self.last_error = str(exc)
                    # Do not reset cursor during reconnect; find missing packets in ring.
                    self._stop.wait(max(1.5, self.poll_interval))
        except (RuntimeError, ConnectionError, ValueError) as exc:
            self.cloud_state = 'AUTH/CONFIG ERROR'
            self.last_error = str(exc)
        finally:
            if client is not None:
                client.close()
