"""Response timing must follow firmware targets, regardless of host buffering."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools/bench'))
from response_probe import response

for kind in ('pos','rpm'):
    n=3000
    w=np.zeros((n,13),dtype='<u4');v=w.view('<f4')
    t=np.arange(n)*.0002
    active=(t>=.1)&(t<.5)
    v[:,10]=np.where(active,2 if kind=='pos' else 1,0)
    target=15.0 if kind=='pos' else 10.0
    v[:,5 if kind=='pos' else 8]=np.where(active,target,0)
    v[:,4 if kind=='pos' else 6]=target*(1-np.exp(-np.maximum(t-.1,0)/.04))
    w[:,0]=((0xffff00+np.arange(n,dtype='<u4')*200)&0xffffff)|np.where(active,4<<24,0).astype('<u4')
    w[:,1]=(3<<24)+np.arange(n,dtype='<u4')*2
    # Deliberately unrelated host times emulate an arbitrary USB backlog.
    events=[dict(command=f'{kind} {target}',sent_s=7.0),dict(command='stop',sent_s=7.4)]
    result=response(w.tobytes(),events)[0]
    assert abs(result['observed_start_s']-.1)<.0003
    assert .08<result['t90_s']<.12
    assert result['arrival_s'] is not None
    events[0]['command']=f'{kind} 999'
    try:response(w.tobytes(),events)
    except RuntimeError:pass
    else:raise AssertionError('Missing command should invalidate timing')
print('PASS: firmware timing, host backlog independence, timestamp wrap, missing-command rejection')
