"""Short plastic-arm comparison. Run from the firmware root with VOFA disconnected.
Requires serial, numpy and the active group-6 telemetry layout. Always stops in finally.
Experimental capture bounds do not change firmware protection policy.
"""
import sys,time,json,argparse
from pathlib import Path
import serial,numpy as np
sys.path.insert(0,'variants/m1-mt6835/tools/bench')
from overview_probe import TAIL
parser=argparse.ArgumentParser()
parser.add_argument('out',nargs='?',default='build/loaded-arm-comparison')
parser.add_argument('--max-iq-ref',type=float,default=1.8)
args=parser.parse_args()
out=Path(args.out);out.mkdir(exist_ok=True)
frames=[];buf=bytearray();events=[]
def sample(s,seconds):
    start=len(frames);last=time.monotonic();end=last+seconds
    while time.monotonic()<end:
        buf.extend(s.read(max(1,min(s.in_waiting,65536))))
        while True:
            at=buf.find(TAIL)
            if at<0:break
            raw=bytes(buf[:at]);del buf[:at+4]
            if len(raw)!=96:continue
            x=np.frombuffer(raw,dtype='<f4').copy();frames.append(x);last=time.monotonic()
            if not np.isfinite(x).all() or x[13]!=0 or not 8<x[8]<12.6 or abs(x[1])>120 or abs(x[4])>args.max_iq_ref or abs(x[5])>3.0:
                raise RuntimeError('Outside short loaded test bounds '+str(x[[1,4,8,12,13]]))
        if time.monotonic()-last>.5:raise RuntimeError('Telemetry stalled')
    return np.array(frames[start:])
with serial.Serial('COM8',1000000,timeout=.003,write_timeout=.2) as s:
    try:
        s.write(b'stop\nsend 6\n');pre=sample(s,.5)
        if not np.all(pre[:,12]==0):raise RuntimeError('Not idle')
        s.write(b'zero\nclear\n');sample(s,.2)
        for index,(command,sec) in enumerate([('pos 15',2),('pos -15',2),('pos 0',2),('stop',.3),('rpm 10',1.2),('rpm 20',1.2),('rpm -20',1.2),('rpm 0',.5),('pos 0',2),('stop',.5)]):
            s.write((command+'\n').encode());a=sample(s,sec)
            np.save(out/(str(index)+'_'+command.replace(' ','_')+'.npy'),a)
            events.append({'command':command,'frames':len(a),'duration_s':sec,'file':str(index)+'_'+command.replace(' ','_')+'.npy'})
            print(command,'rpm',float(a[-500:,1].mean()),'pos',float(a[-1,3]),'Iq peak',float(abs(a[:,4]).max()),flush=True)
    finally:
        s.write(b'stop\nsend 6\n');s.flush();end=time.monotonic()+.3
        while time.monotonic()<end:s.read(max(1,min(s.in_waiting,65536)))
        s.dtr=False
        np.save(out/'all.npy',np.array(frames))
        (out/'events.json').write_text(json.dumps(events,indent=2))
