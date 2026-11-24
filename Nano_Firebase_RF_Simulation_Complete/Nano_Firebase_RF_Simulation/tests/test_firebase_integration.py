"""Offline tests; never contact Firebase or an Arduino."""
import sys
import threading
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ground_station'))
from attitude_math import parse_packet
from firebase_packets import encode, decode, RING_SIZE
from firebase_receiver import FirebaseSerialAdapter
from live_dashboard import NanoSerialPump, LiveFlight

V1 = 'Q,2366,1.000000,0.000027,0.000011,0.000002,0.153,0.009,1.092,0.02,0.06,0.02'
V2 = 'Q,2466,1.000000,0.000027,0.000011,0.000002,0.153,0.009,1.092,0.02,0.06,0.02,1'


class DummySerial:
    def __init__(self):
        self.buffer=bytearray()
    @property
    def in_waiting(self):
        return len(self.buffer)
    def read(self,n):
        b=bytes(self.buffer[:n]); del self.buffer[:n]; return b


class TestNanoPacket(unittest.TestCase):
    def test_legacy_and_new_firmware_packets_are_supported(self):
        self.assertIsNotNone(parse_packet(V1))
        self.assertIsNone(parse_packet(V1)[-1])
        self.assertTrue(parse_packet(V2)[-1])
        for line in (V1,V2):
            f=encode(line, 'abc', 42, station='bench01')
            self.assertEqual(decode(f), line)
            self.assertEqual(f['gyro_dps_xyz'], [0.02, 0.06, 0.02])
            self.assertEqual(f['source'], 'physical_nano_mpu6050')
            self.assertEqual(f['rf_metrics']['rssi_dbm'], None)
            self.assertFalse(f['rf_metrics']['physical_radio_used'])
            self.assertEqual(f['sequence'], 42)
            self.assertEqual(f['server_received_unix_ms'], {'.sv': 'timestamp'})

    def test_reject_invalid_packets(self):
        self.assertIsNone(parse_packet('Q,5,NaN,0,0,0,0,0,0,0,0,0'))
        bad=encode(V1, 'abc', 42, station='bench01')
        bad['nano_uptime_ms']=0
        self.assertIsNone(decode(bad))

    def test_serial_pump_routes_cloud_packet_to_existing_3d_math(self):
        physical = DummySerial()
        physical.buffer.extend((V1+'\n'+V2+'\n').encode())
        pump=NanoSerialPump(physical)
        packets,statuses=pump.poll()
        self.assertEqual(len(packets),2)
        self.assertEqual(packets[0][0],2366)
        self.assertEqual(packets[1][0],2466)
        flight=LiveFlight()
        for p in packets: flight.add(p)
        self.assertEqual(flight.received,2)
        self.assertEqual(flight.frames[-1].stationary,True)
        self.assertAlmostEqual(flight.frames[-1].gyro[0],0.02)

    def test_receiver_counts_sequence_gaps(self):
        # Build an adapter without spawning network polling thread.
        a=FirebaseSerialAdapter.__new__(FirebaseSerialAdapter)
        a._lock=threading.Lock()
        a._buffer=bytearray()
        a.active_session='abc'
        a.last_seen_seq=None
        a.sequence_gaps=0
        samples={str(k):encode(V2,'abc',k,station='bench01') for k in [4,6,7]}
        a._consume(samples)
        self.assertEqual(a.last_seen_seq,7)
        self.assertEqual(a.sequence_gaps,1)
        self.assertEqual(a.in_waiting,3*(len(V2)+1))
        pump=NanoSerialPump(a)
        packets,_=pump.poll()
        self.assertEqual(len(packets),3)
        a._consume(samples)
        self.assertEqual(a.in_waiting,0)  # duplicates ignored

    def test_ring_is_bounded(self):
        self.assertEqual(RING_SIZE, 120)
        self.assertEqual(len({i % RING_SIZE for i in range(1000)}), RING_SIZE)


if __name__=='__main__':
    unittest.main()
