import json,sys
from pathlib import Path
import numpy as np
p=Path(sys.argv[1] if len(sys.argv)>1 else 'build/loaded-arm-20260927');rows=[]
for e in json.loads((p/'events.json').read_text()):
    command=e['command'];a=np.load(p/e.get('file',command.replace(' ','_')+'.npy'))
    t=np.r_[0,np.cumsum(np.diff(a[:,22].astype(np.int64))&0xffffff)]/1e6
    row=dict(command=command,frames=len(a),bus_min=float(a[:,8].min()),bus_mean=float(a[:,8].mean()),
        iq_ref_peak=float(abs(a[:,4]).max()),iq_actual_peak=float(abs(a[:,5]).max()),
        saturation_fraction=float(np.mean((a[:,12]==4)&(a[:,17]<.999))),
        gaps=int(np.sum((np.diff(a[:,23].astype(np.int64))&0xffffff)!=4)),
        faults=np.unique(a[:,13]).tolist(),rejected=int(a[:,16].max()))
    if command!='stop':
        mode=2 if command.startswith('pos') else 1
        c=2 if mode==2 else 0;v=3 if mode==2 else 1;target=float(command.split()[1])
        ix=np.flatnonzero((a[:,11]==mode)&(a[:,12]==4)&np.isclose(a[:,c],target,atol=1e-5))
        f=a[ix[-min(1250,len(ix)):]]
        signal=a[:,v].copy()
        if mode==1:signal=np.convolve(signal,np.ones(50)/50,'full')[:len(signal)]
        good=(abs(signal-target)<(.5 if mode==2 else 3))&np.isin(np.arange(len(a)),ix)
        keep=np.convolve(good.astype(int),np.ones(250,dtype=int),'valid')
        passed=np.flatnonzero(keep==250)
        row.update(target=target,final_mean=float(f[:,v].mean()),final_std=float(f[:,v].std()),
            final_pp=float(np.ptp(f[:,v])),final_error=float(f[:,v].mean()-target),
            iq_final_mean=float(f[:,5].mean()),iq_final_std=float(f[:,5].std()),
            id_final_rms=float(np.sqrt(np.mean(f[:,7]**2))),
            iq_error_rms=float(np.sqrt(np.mean((f[:,4]-f[:,5])**2))),
            arrival_100ms_s=float(t[passed[0]]-t[ix[0]]) if len(passed) else None)
        initial=signal[max(0,ix[0]-1)];direction=np.sign(target-initial)
        row['overshoot']=float(max(0,(direction*(signal[ix]-target)).max()))
        if direction:
            crossing=np.flatnonzero(direction*(signal[ix]-(initial+.9*(target-initial)))>=0)
            row['t90_s']=float(t[ix[crossing[0]]]-t[ix[0]]) if len(crossing) else None
    rows.append(row)
(p/'summary.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
