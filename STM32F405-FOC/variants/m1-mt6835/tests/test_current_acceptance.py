"""Acceptance must reject a plausible fit with a poor/stalled current loop."""
import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/bench'))
from current_acceptance import analyse_current


def samples(tau=.001, bias=0, ringing=0):
    t=np.arange(1250)*.0002
    x=np.zeros((len(t),12),dtype='<f4')
    words=x.view('<u4')
    words[:,0]=(np.arange(len(t),dtype=np.uint32)*200+0xffff00)&0xffffff
    words[:,1]=(5<<24)+np.arange(len(t),dtype=np.uint32)*2
    measured=0.
    for n,now in enumerate(t):
        target=0 if now<.04 else (.2 if now<.11 or now>=.18 else -.2)
        if target:words[n,0]|=4<<24
        x[n,2]=target
        measured+=(target-measured)*(1-np.exp(-.0002/tau))
        x[n,9]=measured+bias+ringing*np.sin(2*np.pi*200*now)
    return words


class Acceptance(unittest.TestCase):
    def test_good_loop_and_timestamp_wrap(self):
        r=analyse_current(samples(),.2)
        self.assertTrue(r['passed'])
        self.assertLess(max(s['t90_s'] for s in r['response']),.003)

    def test_slow_bias_and_ringing(self):
        for values in [dict(tau=.03),dict(bias=.06),dict(ringing=.12)]:
            with self.subTest(values=values),self.assertRaises(RuntimeError):
                analyse_current(samples(**values),.2)

    def test_adc_noise_reference_and_low_snr(self):
        rng=np.random.default_rng(42)
        w=samples();x=w.view('<f4')
        x[:,4]+=rng.normal(0,.02,len(x));x[:,9]+=rng.normal(0,.02,len(x))
        self.assertTrue(analyse_current(w,.2)['passed'])
        x[:,4]+=rng.normal(0,.10,len(x))
        with self.assertRaisesRegex(RuntimeError,'signal/noise'):analyse_current(w,.2)

    def test_loss_and_fault(self):
        words=samples()
        with self.assertRaises(RuntimeError):analyse_current(np.delete(words,400,axis=0),.2)
        words[400,0]|=3<<27
        with self.assertRaises(RuntimeError):analyse_current(words,.2)


if __name__=='__main__':unittest.main()
