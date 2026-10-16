"""Synchronous, staged speed probe. --drive is required; otherwise read only.
Physical operator must confirm 12 V, raised mechanism, supported upper-stop home.
Keeps 10 A ceiling and existing 180 RPM/ADC/PWM/range protections.
"""
import argparse,csv,json,math,statistics,time
from pathlib import Path
import serial
from fivebar_point_probe import Probe
from fivebar_tool import Solver,geometry_from_dict

class SpeedProbe(Probe):
    def __init__(self,link):super().__init__(link);self.bad_tracking=None
    def pump(self):
        super().pump()
        if self.driving and self.last and self.last[11]==self.last[12]==4:
            error=max(abs(self.last[i]-self.last[i+2]) for i in (0,1))
            if error>5:
                if self.bad_tracking is None:self.bad_tracking=self.last[20]
                assert ((int(self.last[20])-int(self.bad_tracking))&0xffffff)<1000,'tracking error >5deg for 100ms'
            else:self.bad_tracking=None

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port',default='COM8');ap.add_argument('--home',action='store_true')
    ap.add_argument('--drive',action='store_true');ap.add_argument('--rates',default='45,60,90,120')
    ap.add_argument('--output',type=Path,default=Path('build/dual-leg/wheel-leg-ui/speed-probe.json'))
    args=ap.parse_args();rates=[int(x)for x in args.rates.split(',')]
    assert rates and all(x in (45,60,90,120)for x in rates) and rates==sorted(set(rates))
    root=Path(__file__).resolve().parents[4]
    solver=Solver('C:/mingw64/bin/gcc.exe');solver.geometry=geometry_from_dict(json.loads((root/'docs/data/fivebar-current-geometry-20261014.json').read_text()))
    goals={}
    for q in [2.5,5,10,20,40,60,80,100,107]:
        status,p=solver.solve([q,q],False);assert status=='ok'
        xy=[round(float(x),2)for x in p.xy];status,p=solver.solve(xy,True);assert status=='ok'
        goals[q]=(xy,list(p.q))
    result={'success':False,'stop_verified':False,'rates':rates,'segments':[]}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with serial.Serial(args.port,1000000,timeout=.015)as link:
        link.dtr=True;p=SpeedProbe(link);p.send('send 5');p.wait(1)
        assert p.last and p.last[13]==0 and p.last[11]==p.last[12]==0,'Not idle/fault free'
        try:
            if args.home:p.command('leg home');p.wait(.1)
            assert int(p.last[14])&31==31,'Home/calibration/geometry not ready'
            result['initial']=p.last
            if args.drive:
                assert abs(p.last[0])<2 and abs(p.last[1])<2,'Need operator confirmed upper stop'
                p.command('limit 10');p.command('leg rate 10');p.command('foot speed 20')
                def segment(q,seconds,rpm,settle=True):
                    xy,joints=goals[q];start=len(p.rows);began=time.monotonic()
                    line=f'foot pos {xy[0]:.2f} {xy[1]:.2f}'
                    p.command(line);p.driving=True;p.wait(seconds)
                    tail=[row for stamp,row in p.rows[start:]if stamp>time.monotonic()-.5]
                    errors=[max(abs(row[i]-joints[i])for row in tail)for i in (0,1)]
                    entry={'q_nominal':q,'motor_rate_rpm':rpm,'xy':xy,'joint_goal':joints,'seconds':time.monotonic()-began,
                      'settling_expected':settle,'stable_joint_error':errors,'stable_joint_std':[statistics.pstdev(row[i]for row in tail)for i in (0,1)],
                      'peak_iq':[max(abs(row[i])for _,row in p.rows[start:])for i in (8,9)],
                      'peak_tracking_error':max(max(abs(row[i]-row[i+2])for i in (0,1))for _,row in p.rows[start:]),'final':p.last}
                    result['segments'].append(entry);print(json.dumps({k:v for k,v in entry.items()if k!='final'}),flush=True)
                    if settle:
                        assert max(errors)<1,'Not settled within 1deg'
                        assert max(entry['stable_joint_std'])<.15,'Settled vibration'
                for q,d in [(2.5,3),(5,3),(10,4),(20,6)]:segment(q,d,10)
                for rpm in rates:
                    # Transfer ownership without releasing the supported leg.
                    p.command('leg pos 20 20');p.wait(.4)
                    p.command(f'leg rate {rpm}');p.command(f'foot speed {min(400,round(rpm*10/3))}');p.wait(.1)
                    for q,d in [(40,3),(20,3),(60,5),(20,5)]:segment(q,d,rpm)
                    segment(80,.65,rpm,False);segment(20,4,rpm)
                rpm=rates[-1]
                for q in [100,5,107,20,107]:segment(q,8 if rpm<=60 else 5,rpm)
            result['success']=True
        except Exception as e:
            result['error']=str(e);print('STOP: '+str(e),flush=True)
        finally:
            if args.drive:
                p.driving=False;p.target=None;p.send('stop');p.wait(.4)
                result['stop_verified']=p.last[11]==p.last[12]==0
            result['final']=p.last;result['frames']=len(p.rows)
            args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
            with args.output.with_suffix('.csv').open('w',newline='',encoding='utf-8')as f:
                w=csv.writer(f);w.writerow(['host_time']+[f'ch{i}'for i in range(24)]);w.writerows((t,*row)for t,row in p.rows)
            print(json.dumps({k:v for k,v in result.items()if k not in ('segments','initial','final')}),flush=True)
    if not result['success']:raise SystemExit(1)
if __name__=='__main__':main()
