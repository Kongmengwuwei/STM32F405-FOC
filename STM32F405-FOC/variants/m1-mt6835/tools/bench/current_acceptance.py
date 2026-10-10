"""Bounded, timestamped bidirectional current-loop acceptance before saving.

This verifies the small-signal loop on a free shaft, not a continuous rating.
Group 5 averages two samples on this build; reported timing resolution is 200 us.
"""
import struct
import threading
import time
import numpy as np
from benchlib import FrameReader, continuity


def analyse_current(words, amplitude=.2):
    if len(words)<400:
        raise RuntimeError('Insufficient current acceptance samples')
    x=words.view('<f4')
    stats=continuity(words,5,10000,2)
    if stats['gaps'] or stats['faults'] or not np.isfinite(x[:,2:]).all():
        raise RuntimeError('Invalid/discontinuous current acceptance telemetry')
    idle=((words[:,0]>>24)&7)==0
    if np.count_nonzero(idle)<100:raise RuntimeError('No idle noise reference')
    # The covariance trace of the d/q ADC noise is invariant to a coordinate
    # rotation and bounds the noise variance of either axis. Measure it before
    # energizing, rather than treating the board noise floor as loop ripple.
    noise_variance=float(x[idle,4].var()+x[idle,9].var())
    noise_sigma=np.sqrt(noise_variance)
    if noise_sigma>.25*amplitude:raise RuntimeError('ADC signal/noise insufficient for current acceptance')
    t=np.r_[0,np.cumsum(np.diff(words[:,0]&0xffffff).astype(np.int64)&0xffffff)]*1e-6
    # ADC noise is significant at small currents. Use a specified 1 ms
    # trailing average for response/overshoot, preserving the raw peak and
    # raw steady ripple. The online 0.8 A vector guard is never filtered.
    feedback=np.convolve(x[:,9],np.ones(5)/5,mode='full')[:len(x)]
    starts=[];after=0
    for target in (amplitude,-amplitude,amplitude):
        hits=np.flatnonzero(np.isclose(x[after:,2],target,atol=1e-4,rtol=0) &
                            (((words[after:,0]>>24)&7)==4))
        if not len(hits):raise RuntimeError('Current target missing/rejected')
        after+=int(hits[0]);starts.append(after);after+=1
    response=[]
    for i,target in enumerate((amplitude,-amplitude,amplitude)):
        start=starts[i];end=starts[i+1] if i<2 else len(x)
        idx=np.arange(start,end)
        if t[end-1]-t[start]<.028:raise RuntimeError('Current dwell too short')
        initial=float(feedback[max(0,start-1)]);step=target-initial;sign=np.sign(step)
        if abs(step)<amplitude*.5:raise RuntimeError('Current step unresolved')
        progress=sign*(feedback[idx]-initial)/abs(step)
        hits=np.flatnonzero(progress>=.9)
        t90=float(t[idx[hits[0]]]-t[start]) if len(hits) else None
        tail=idx[t[idx]>=t[end-1]-.015]
        error=float(x[tail,9].mean()-target);ripple=float(x[tail,9].std())
        excess_ripple=float(np.sqrt(max(0,ripple*ripple-noise_variance)))
        overshoot=float(max(0,np.max(sign*(feedback[idx]-target))/abs(step)))
        item=dict(target_a=target,t90_s=t90,overshoot_fraction=overshoot,
                  raw_peak_a=float(abs(x[idx,9]).max()),steady_error_a=error,steady_std_a=ripple,
                  excess_std_a=excess_ripple)
        response.append(item)
        if t90 is None or t90>.015 or overshoot>.35 or abs(error)>.1*amplitude or excess_ripple>.15*amplitude:
            raise RuntimeError(f'Current-loop acceptance failed: {item}')
    return dict(passed=True,response=response,continuity=stats,
                rpm_peak=float(abs(x[:,7]).max()),current_peak_a=float(np.hypot(x[:,4],x[:,9]).max()),
                noise_vector_sigma_a=float(noise_sigma),sample_resolution_s=.0002,response_filter_s=.001)


def validate_current(link,send,out,amplitude=.2):
    """No Flash writes; always stop, including on communication/analysis failure."""
    if not .2<=amplitude<=.5:raise ValueError('Acceptance amplitude must be 0.2..0.5 A')
    records=[];reader=FrameReader();guard=None;start_bus=None
    def capture(seconds):
        nonlocal start_bus
        deadline=time.monotonic()+seconds;last=time.monotonic()
        while time.monotonic()<deadline:
            for frame in reader.feed(link.read(max(1,min(link.in_waiting,65536)))):
                if frame.group!=5:continue
                records.append(frame.body);last=time.monotonic()
                x=struct.unpack('<12f',frame.body)
                if frame.fault or not np.isfinite(x[2:]).all() or x[10]:
                    raise RuntimeError('Fault/warning during current acceptance')
                if start_bus is None:
                    if frame.state!=0 or frame.power or not 3<=x[11]<=60:
                        raise RuntimeError('No powered idle before current acceptance')
                    start_bus=x[11]
                if abs(x[11]-start_bus)>.1*start_bus or abs(x[7])>120 or np.hypot(x[4],x[9])>.8:
                    raise RuntimeError('Current acceptance exceeded voltage/speed/current bounds')
            if time.monotonic()-last>.1:raise RuntimeError('Current acceptance telemetry stalled')
    try:
        send('stop');send('send 5')
        end=time.monotonic()+.12
        while time.monotonic()<end:link.read(max(1,min(link.in_waiting,65536)))
        link.reset_input_buffer()
        capture(.04)
        guard=threading.Timer(.6,send,args=('stop',));guard.start()
        # +I*T, -I*2T, +I*T has near-zero net torque impulse. Equal dwell
        # would leave a free shaft accelerating at the end of the test.
        for target,dwell in ((amplitude,.050),(-amplitude,.100),(amplitude,.050)):
            send(f'Iq {target:g}');capture(dwell)
    finally:
        send('stop')
        if guard:guard.cancel();guard.join()
        words=np.frombuffer(b''.join(records),dtype='<u4').reshape(-1,12).copy()
        np.save(out,words)
        send('send 7')
    return analyse_current(words,amplitude)
