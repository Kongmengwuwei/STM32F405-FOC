"""Read or run stationary R/L identification; apply/save only when requested.

Examples: --identify --apply / --identify --apply --save. 12 V free shaft,
correct pole pairs and encoder calibration required. Close VOFA first.
"""
import argparse
import json
import struct
import threading
import time
from pathlib import Path
import numpy as np
import serial


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port',default='COM8')
    ap.add_argument('--identify',action='store_true')
    ap.add_argument('--current',type=float,default=1.5)
    ap.add_argument('--apply',action='store_true')
    ap.add_argument('--save',action='store_true')
    ap.add_argument('--out',required=True)
    args=ap.parse_args()
    if not .2<=args.current<=3 or (args.apply and not args.identify) or (args.save and args.identify and not args.apply):
        ap.error('Use current 0.2..3 A; --apply requires --identify; identify+save requires --apply')
    out=Path(args.out);out.parent.mkdir(parents=True,exist_ok=True)
    with serial.Serial(args.port,1000000,timeout=.005,write_timeout=.2) as s:
        lock=threading.Lock();rows=[];pending=bytearray();guard=None
        def send(line):
            with lock:s.write((line+'\r\n').encode('ascii'));s.flush()
        def collect(seconds):
            end=time.monotonic()+seconds
            fresh=[]
            while time.monotonic()<end:
                pending.extend(s.read(max(1,min(s.in_waiting,65536))))
                while len(pending)>=52:
                    if pending[48:52]!=b'\x00\x00\x80\x7f':del pending[:1];continue
                    frame=bytes(pending[:52]);del pending[:52]
                    timeword,seqword=struct.unpack('<II',frame[:8])
                    if seqword>>24!=7:continue
                    values=struct.unpack('<10f',frame[8:48])
                    if not all(np.isfinite(values)):raise RuntimeError('Nonfinite identification telemetry')
                    status=timeword>>24
                    if (status>>3)&15:raise RuntimeError(f'Firmware fault {(status>>3)&15}')
                    row=[timeword&0xffffff,seqword&0xffffff,status,*values]
                    rows.append(row);fresh.append(row)
            if not fresh:raise RuntimeError('No fresh group 7 telemetry')
            return fresh[-1]
        try:
            s.dtr=True;send('stop');send('clear');send('send 7');collect(.2)
            if args.identify:
                guard=threading.Timer(5.5,send,args=('stop',));guard.start()
                send(f'identify {args.current:.2f}')
                seen=False;deadline=time.monotonic()+5
                while time.monotonic()<deadline:
                    first=len(rows);row=collect(.08);phase,error=int(row[3]),int(row[4])
                    seen=seen or any(1<=int(r[3])<=6 for r in rows[first:])
                    if phase in [8,9] or (seen and phase==7):break
                else:raise RuntimeError('Identification did not finish or command was rejected')
                guard.cancel();guard.join();guard=None  # Probe has already closed PWM.
                if phase!=7 or error or not int(row[12])&1:
                    raise RuntimeError(f'Identification failed: phase={phase}, error={error}; old gains retained')
                if args.apply:
                    send('tune apply');row=collect(.15)
                    if not int(row[12])&16:raise RuntimeError('Candidate was not applied; previous tuning may still be active')
            else:row=collect(.2)
            if args.save:
                if not int(row[12])&2:raise RuntimeError('No active tuning to save')
                if int(row[12])&1 and not int(row[12])&16:raise RuntimeError('Unapplied candidate present; apply it before saving')
                send('tune save');time.sleep(.05)
                # Flash briefly stops the stream, then acquisition resumes.
                row=collect(1.8)
                if (int(row[12])>>2)&3!=2:raise RuntimeError('Flash tuning save was not verified')
            result=dict(phase=int(row[3]),error=int(row[4]),resistance_ohm=row[5],ld_h=row[6],lq_h=row[7],
                        kp_d=row[8],kp_q=row[9],ki_per_s=row[10],bandwidth_hz=row[11],
                        candidate_valid=bool(int(row[12])&1),active=bool(int(row[12])&2),save_requested=args.save,
                        candidate_applied=bool(int(row[12])&16),save_status=(int(row[12])>>2)&3)
            out.with_suffix('.json').write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps(result),flush=True)
        except Exception as exc:
            failure={'host_error':str(exc)}
            if rows:
                last=rows[-1]
                failure.update(phase=int(last[3]),error=int(last[4]),resistance_ohm=last[5],ld_h=last[6],lq_h=last[7],active=bool(int(last[12])&2))
            out.with_suffix('.json').write_text(json.dumps(failure,indent=2)+'\n')
            raise
        finally:
            send('stop')
            if guard:guard.cancel();guard.join()
            np.save(out.with_suffix('.npy'),np.asarray(rows,dtype=np.float64))
            send('send 6')


if __name__=='__main__':main()
