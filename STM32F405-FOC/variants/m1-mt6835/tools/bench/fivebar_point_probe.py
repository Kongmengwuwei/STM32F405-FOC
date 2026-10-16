"""Staged synchronous five-bar point probe. Default only reads; --drive enables motion.
Requires raised/supportable linkage and operator-established upper-stop home.
The user must authorize physical testing; legal geometry is not collision proof.
"""
import argparse,csv,json,math,struct,time,statistics
from pathlib import Path
import serial
from fivebar_tool import Solver,geometry_from_dict

class UIFrames:
    def __init__(self):self.pending=bytearray()
    def feed(self,data):
        self.pending.extend(data);out=[]
        while len(self.pending)>=100:
            if self.pending[96:100]!=b'\x00\x00\x80\x7f':
                del self.pending[0];continue
            row=struct.unpack('<24f',self.pending[:96]);del self.pending[:100]
            if row[23]==5005:out.append(row)
        return out

class Probe:
    def __init__(self,link):
        self.link=link;self.frames=UIFrames();self.last=None;self.rows=[]
        self.target=None;self.sent=0;self.progress=0;self.sequence=None
        self.driving=False;self.started=False
    def send(self,line):
        self.link.write((line+"\r\n").encode());self.link.flush()
    def pump(self):
        now=time.monotonic()
        if self.target and now-self.sent>=.05:
            self.send(self.target);self.sent=now
        for row in self.frames.feed(self.link.read(8192)):
            if row[23]!=5005:continue
            self.last=row;self.rows.append((time.monotonic(),row))
            if row[20]!=self.sequence:self.sequence=row[20];self.progress=time.monotonic()
            if self.driving:
                assert row[13]==0,f"fault {row[13]}"
                assert int(row[14])&257==257,"homing or acquisition lost"
                assert -1<=row[0]<=111 and -1<=row[1]<=111,"joint excursion"
                assert abs(row[0]-row[1])<3,"synchronous path diverged"
                assert abs(row[8])<11 and abs(row[9])<11,"current"
                assert 10<=row[10]<=14,"bus"
                if row[11]==row[12]==4:self.started=True
                if self.started:assert row[11] in (1,4) and row[12] in (1,4),"external stop"
        if self.driving:assert time.monotonic()-self.progress<.25,"stale acquisition"
    def wait(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:self.pump()
    def command(self,line):
        old_target=self.target;self.target=None
        self.wait(.04);before=self.last[16];self.send(line)
        end=time.monotonic()+.8
        while time.monotonic()<end:
            self.pump()
            if self.last[16]!=before:
                assert self.last[15]==0,f"{line}: rejected code={self.last[15]} kinematics={self.last[19]}"
                self.target=line if line.startswith(('foot pos ','leg pos ')) else old_target
                self.sent=time.monotonic()
                return
        raise AssertionError(f"{line}: no ACK")

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port',default='COM8');ap.add_argument('--drive',action='store_true')
    ap.add_argument('--home',action='store_true',help='only after operator confirms original upper stop')
    ap.add_argument('--output',type=Path,default=Path('build/dual-leg/wheel-leg-ui/point-probe.json'))
    args=ap.parse_args();root=Path(__file__).resolve().parents[4]
    geometry=json.loads((root/'docs/data/fivebar-current-geometry-20261014.json').read_text(encoding='utf-8'))
    solver=Solver('C:/mingw64/bin/gcc.exe');solver.geometry=geometry_from_dict(geometry)
    goals={}
    for q in [2.5,5,10,20,40,60,80,100,107]:
        status,pose=solver.solve([q,q],False);assert status=='ok'
        xy=[round(float(v),2) for v in pose.xy]
        status,back=solver.solve(xy,True);assert status=='ok',f'Rounded target invalid {q}: {xy}'
        goals[q]=(xy,list(back.q))
    result={'drive_requested':args.drive,'segments':[],'success':False,'stop_verified':False}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with serial.Serial(args.port,1000000,timeout=.015) as link:
        link.dtr=True;p=Probe(link);p.send('send 5');p.wait(1)
        assert p.last and p.last[13]==0 and p.last[11]==p.last[12]==0,'Not idle/fault-free'
        try:
            if args.home:
                p.command('leg home');p.wait(.1)
            assert int(p.last[14])&31==31,'Home/electrical/offset/geometry-save not ready'
            result['initial']=p.last
            if not args.drive:
                result['success']=True;return
            assert abs(p.last[0])<2 and abs(p.last[1])<2,'Start micro-validation near confirmed upper stop'
            p.command('limit 10');p.command('leg rate 10');p.command('foot speed 20')
            def segment(q,seconds,settle=True):
                xy,joints=goals[q];line=f'foot pos {xy[0]:.2f} {xy[1]:.2f}'
                start=len(p.rows);began=time.monotonic()
                p.command(line);p.target=line;p.sent=time.monotonic();p.driving=True
                p.wait(seconds)
                tail=[r for t,r in p.rows[start:] if t>time.monotonic()-.5]
                error=[max(abs(r[i]-joints[i]) for r in tail) for i in (0,1)]
                foot_error=max(math.hypot(r[4]-xy[0],r[5]-xy[1]) for r in tail)
                entry={'q_nominal':q,'xy':xy,'joint_goal':joints,'seconds':time.monotonic()-began,
                       'stable_joint_error':error,'stable_joint_std':[statistics.pstdev(r[i] for r in tail) for i in (0,1)],'estimated_foot_error_mm':foot_error,
                       'peak_iq':[max(abs(r[i]) for _,r in p.rows[start:]) for i in (8,9)],
                       'settling_expected':settle,'final':p.last}
                result['segments'].append(entry);print(json.dumps({k:v for k,v in entry.items() if k!='final'}),flush=True)
                if settle:
                    assert max(error)<1,'Did not settle within 1 joint degree'
                    assert max(entry['stable_joint_std'])<.15,'Excessive settled vibration'
            for q,d in [(2.5,3),(5,3),(10,4),(20,6)]:segment(q,d)
            # Keep both motors powered while configuring the next foot plan.
            p.command('leg pos 20 20');p.target='leg pos 20 20';p.wait(.6)
            p.command('leg rate 45');p.command('foot speed 150');p.wait(.2)
            for q,d in [(40,5),(20,5),(80,7),(40,6),(100,7),(60,6),(107,7),(20,9)]:segment(q,d)
            segment(107,2,False);segment(20,7)
            segment(107,9)
            result['success']=True
        except Exception as e:
            result['error']=str(e);print('STOP: '+str(e),flush=True)
        finally:
            if args.drive:
                p.driving=False;p.target=None;p.send('stop');p.wait(.4)
                result['stop_verified']=p.last[11]==p.last[12]==0
            result['final']=p.last;result['frames']=len(p.rows)
            args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
            with args.output.with_suffix('.csv').open('w',newline='',encoding='utf-8') as f:
                writer=csv.writer(f);writer.writerow(['host_time']+[f'ch{i}' for i in range(24)])
                writer.writerows((t,*r) for t,r in p.rows)
            print(json.dumps({k:v for k,v in result.items() if k not in ('segments','initial','final')}),flush=True)
    if not result['success']:raise SystemExit(1)
if __name__=='__main__':main()
