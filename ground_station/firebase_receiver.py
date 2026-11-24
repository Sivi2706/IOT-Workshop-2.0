"""Feed the original dashboard with Nano packets after Firebase confirms storage."""
import threading

from publish import run_publisher


class FirebaseSerialAdapter:
    is_cloud = True
    sequence_gaps = 0

    def __init__(self, port, hz):
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._buffer = bytearray()
        self.is_open = True
        self.cloud_state = "DETECTING NANO"
        self.last_seen_seq = None
        self.last_error = None
        self._thread = threading.Thread(target=self._worker, args=(port, hz), daemon=True)
        self._thread.start()

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
        raise OSError("Use direct USB mode for Nano calibration and zeroing commands.")

    def close(self):
        self.is_open = False
        self._stop.set()
        self._thread.join(timeout=0.75)

    def _push(self, line):
        with self._lock:
            if len(self._buffer) > 60000:
                self._buffer.clear()
            self._buffer.extend((line + "\n").encode("ascii", errors="replace"))

    def _notify(self, message):
        print(message, flush=True)
        if message.startswith(("Firebase HTTP", "Firebase connection")):
            self.last_error = message
            self.cloud_state = "WRITE FAILED"
        elif not message.startswith("SENT"):
            self._push(message if message.startswith("#") else f"# {message}")

    def _sample(self, frame, line):
        self.last_error = None
        self.last_seen_seq = frame["sequence"]
        self.cloud_state = "CONNECTED"
        self._push(line)

    def _worker(self, port, hz):
        try:
            run_publisher(port, hz, stop=self._stop.is_set, notify=self._notify, on_sample=self._sample)
        except (OSError, RuntimeError, ImportError, ValueError, KeyError) as error:
            if not self._stop.is_set():
                self.last_error = str(error)
                self.cloud_state = "STOPPED"
                self._push(f"# ERROR: {error}")
