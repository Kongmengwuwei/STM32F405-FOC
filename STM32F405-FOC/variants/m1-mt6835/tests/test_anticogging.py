import importlib.util
from pathlib import Path
import unittest
import struct
import numpy as np

path=Path(__file__).resolve().parents[1]/'tools/bench/anticogging.py'
spec=importlib.util.spec_from_file_location('cog_bench',path)
cog=importlib.util.module_from_spec(spec);spec.loader.exec_module(cog)

class MapTests(unittest.TestCase):
    def samples(self, sign):
        theta=(np.arange(720)*.5+.03*sign)%360
        current=.3*np.sin(theta*np.pi/180*84)+sign*.1
        return np.column_stack([theta,current,np.zeros(720),np.zeros(720)])

    def test_bidirectional_friction_and_actual_angles(self):
        table,quality=cog.build_map([self.samples(1),self.samples(-1)])
        expected=.3*np.sin(np.arange(720)*.5*np.pi/180*84)
        self.assertLess(np.sqrt(np.mean((table-expected)**2)),.006)
        self.assertLess(abs(table.mean()),1e-6)
        self.assertGreater(quality['correlation'],.99)
        self.assertAlmostEqual(quality['friction_bias_a'],.1,places=3)

    def test_incomplete_unrepeatable_or_excessive_rejected(self):
        with self.assertRaises(RuntimeError):
            cog.build_map([self.samples(1)[:-1],self.samples(-1)])
        a=self.samples(1);b=self.samples(-1);b[:,1]=-b[:,1]
        with self.assertRaises(RuntimeError):cog.build_map([a,b])
        a=self.samples(1);b=self.samples(-1);a[:,1]*=10;b[:,1]*=10
        with self.assertRaises(RuntimeError):cog.build_map([a,b])

    def test_drop_between_usb_reads_is_rejected(self):
        class SerialFixture:
            data=b''
            @property
            def in_waiting(self):return len(self.data)
            def read(self,n):
                result=self.data[:n];self.data=self.data[n:];return result
        link=cog.Link.__new__(cog.Link)
        link.s=SerialFixture();link.pending=bytearray();link.group=6;link.last_sequence=None
        def feed(first):
            frames=[]
            for n in range(first,first+30):
                row=[0.]*24;row[8]=12;row[22]=n*400;row[23]=n*4
                frames.append(struct.pack('<25f',*row,float('inf')))
            link.s.data=b''.join(frames)
        feed(0);self.assertEqual(len(link.rows(.01)),30)
        feed(31)
        with self.assertRaisesRegex(RuntimeError,'between reads'):link.rows(.01)

if __name__=='__main__':unittest.main()
