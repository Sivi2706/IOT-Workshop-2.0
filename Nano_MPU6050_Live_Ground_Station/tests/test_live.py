import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1] / 'ground_station'))

from attitude_math import q_from_euler
from live_dashboard import NanoSerialPump, LiveFlight

class DummySerial:
    def __init__(self):self.buffer=bytearray();self.out=bytearray()
    @property
    def in_waiting(self):return len(self.buffer)
    def read(self,n):
        part=self.buffer[:n];del self.buffer[:n]
        return bytes(part)
    def write(self,bytes_):self.out.extend(bytes_)

class TestPhysicalIntegration(unittest.TestCase):
    def test_split_serial_frame_and_comment(self):
        s=DummySerial();pump=NanoSerialPump(s)
        data=b'# NANO MPU6050 ROCKET ATTITUDE V2\r\nQ,1000,1,0,0,0,0,0,1,0,0,0,1\r\n'
        split_at=data.index(b'\n')+1
        s.buffer.extend(data[:split_at])
        frames,msg=pump.poll()
        self.assertEqual(frames,[])
        self.assertEqual(len(msg),1)
        s.buffer.extend(data[split_at:])
        frames,msg=pump.poll()
        self.assertEqual(len(frames),1)
        self.assertEqual(frames[0][0],1000)
        self.assertTrue(frames[0][-1])

    def test_legacy_packets_rejected_explicitly(self):
        s=DummySerial();pump=NanoSerialPump(s)
        s.buffer.extend(b'Q,1000,1,0,0,0,0,0,1,0,0,0\n')
        frames,msg=pump.poll()
        self.assertEqual(frames,[])
        self.assertIn('legacy',msg[0])

    def test_model_axis_mapping_with_roll(self):
        f=LiveFlight(nose_axis='+z')
        q=q_from_euler(0,0,math.radians(60))
        p=(1000,q,(0,0,1),(0,0,5),False)
        fr=f.add(p)
        self.assertAlmostEqual(fr.roll,60,places=4)
        self.assertAlmostEqual(fr.pitch,0,places=4)
        self.assertEqual(fr.get('altitude'),None)
        self.assertEqual(fr.get('temperature'),None)

    def test_nose_x_maps_sensor_x_rotation_to_axial_roll(self):
        f=LiveFlight(nose_axis='+x')
        fr=f.add((1000,q_from_euler(math.radians(35),0,0),(1,0,0),(0,0,0),False))
        self.assertAlmostEqual(fr.roll,35,places=4)

    def test_window_and_timestamps(self):
        f=LiveFlight()
        for i in range(300):
            f.add((1000+i*20,(1,0,0,0),(0.5,0.2,1),(0,0,0),False))
        self.assertAlmostEqual(f.seconds,5.98,places=3)
        h=f.history('ax',seconds=2,limit=20)
        self.assertLessEqual(len(h),21)
        self.assertGreaterEqual(min(t for t,v in h),3.98)
        self.assertEqual(f.sample(0).quaternion,(1,0,0,0))

    def test_reboot_resets_plot_history(self):
        f=LiveFlight()
        f.add((100000,(1,0,0,0),(0,0,1),(0,0,0),True))
        f.add((100020,(1,0,0,0),(0,0,1),(0,0,0),True))
        f.add((200,(1,0,0,0),(0,0,1),(0,0,0),True))
        self.assertEqual(len(f.frames),1)
        self.assertEqual(f.seconds,0)

if __name__=='__main__':unittest.main()
