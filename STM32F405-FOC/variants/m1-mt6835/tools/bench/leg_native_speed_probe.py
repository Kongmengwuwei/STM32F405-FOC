"""Native paired-joint speed probe, no reset/home. --drive required.
Only starts with valid continuous home and synchronized joints inside 2..108deg.
"""
import argparse,csv,json,statistics,time
from pathlib import Path
import serial
from fivebar_point_probe import Probe
from fivebar_tool import Solver,geometry_from_dict

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--drive',action='store_true');ap.add_argument('--home',action='store_true');ap.add_argument('--confirm-stop',action='store_true',help='Human explicitly confirmed original upper stop; requires --home');ap.add_argument('--port',default='COM8');ap.add_argument('--output',type=Path,default=Path('build/dual-leg/wheel-leg-ui/native-speed-probe.json'));a=ap.parse_args()
 r={'success':False,'stop_verified':False,'segments':[]};a.output.parent.mkdir(parents=True,exist_ok=True)
 foot_lines={}
 if a.drive:
  root=Path(__file__).resolve().parents[4];solver=Solver('C:/mingw64/bin/gcc.exe');solver.geometry=geometry_from_dict(json.loads((root/'docs/data/fivebar-current-geometry-20261014.json').read_text()))
  for q in (20,107):
   status,pose=solver.solve([q,q],False);assert status=='ok';foot_lines[q]=f'foot pos {pose.xy[0]:.2f} {pose.xy[1]:.2f}'
 with serial.Serial(a.port,1000000,timeout=.015)as link:
  link.dtr=True;p=Probe(link);p.send('send 5');p.wait(1)
  assert p.last and p.last[13]==0 and p.last[11]==p.last[12]==0,'Not idle/fault free'
  if a.confirm_stop:assert a.home,'--confirm-stop requires --home'
  if a.home:p.command('leg home confirmed' if a.confirm_stop else 'leg home');p.wait(.1)
  assert int(p.last[14])&287==287,'Home/acquisition/calibration not valid'
  r['initial']=p.last
  try:
   assert all((-.1 if a.home else 2)<=p.last[i]<=108 for i in (0,1)) and abs(p.last[0]-p.last[1])<3,'Place supported linkage inside synchronized legal workspace; do not rehome'
   if a.drive:
    p.command('limit 10');p.command('leg rate 10' if a.home else 'leg rate 45')
    def segment(q,seconds,rpm,settle=True,foot=False):
     start=len(p.rows);line=f'leg pos {q:.2f} {q:.2f}'
     if foot:line=foot_lines[q]
     p.command(line);p.driving=True;p.wait(seconds)
     tail=[row for stamp,row in p.rows[start:]if stamp>time.monotonic()-.5]
     extra=0
     if settle and (max(max(abs(row[i]-q)for row in tail)for i in (0,1))>=1 or max(statistics.pstdev(row[i]for row in tail)for i in (0,1))>=.15):
      p.wait(2);extra=2;tail=[row for stamp,row in p.rows[start:]if stamp>time.monotonic()-.5]
     e={'q_nominal':q,'mode':'foot' if foot else 'joint','extra_settle_seconds':extra,'motor_rate_rpm':rpm,'settling_expected':settle,'stable_joint_error':[max(abs(row[i]-q)for row in tail)for i in (0,1)],'stable_joint_std':[statistics.pstdev(row[i]for row in tail)for i in (0,1)],'peak_iq':[max(abs(row[i])for _,row in p.rows[start:])for i in (8,9)],'final':p.last}
     r['segments'].append(e);print(json.dumps({k:v for k,v in e.items()if k!='final'}),flush=True)
     if settle:assert max(e['stable_joint_error'])<1 and max(e['stable_joint_std'])<.15,'Not settled'
    if a.home:
     for q,d in [(2.5,3),(5,3),(10,4),(20,5)]:segment(q,d,10)
     p.command('leg rate 45')
    segment(30,3,45)
    for rpm in [45,60,90,120]:
     p.command(f'leg rate {rpm}')
     segment(50,3,rpm);segment(30,3,rpm)
    segment(80,.45,120,False);segment(30,4,120)
    for q in [100,10,107,20,107]:segment(q,4,120)
    p.command('foot speed 400')
    segment(20,4,120,True,True);segment(107,4,120,True,True)
    segment(20,.65,120,False,True);segment(107,4,120,True,True)
   r['success']=True
  except Exception as e:r['error']=str(e);print('STOP: '+str(e),flush=True)
  finally:
   if a.drive:p.driving=False;p.target=None;p.send('stop');p.wait(.4);r['stop_verified']=p.last[11]==p.last[12]==0
   r['final']=p.last;r['frames']=len(p.rows);a.output.write_text(json.dumps(r,indent=2),encoding='utf-8')
   with a.output.with_suffix('.csv').open('w',newline='',encoding='utf-8')as f:
    w=csv.writer(f);w.writerow(['host_time']+[f'ch{i}'for i in range(24)]);w.writerows((t,*row)for t,row in p.rows)
   print(json.dumps({k:v for k,v in r.items()if k not in ('segments','initial','final')}),flush=True)
 if not r['success']:raise SystemExit(1)
if __name__=='__main__':main()
