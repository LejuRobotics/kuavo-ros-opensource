#!/usr/bin/env python3
"""Extract a deterministic 500-Hz solved-target replay from retained bag NPZ.

This is not raw Quest replay: poses use recorded shaped torso-relative targets;
body intent is reconstructed by the C++ runner from recorded reference FK.
"""
import argparse
import json
import numpy as np
p=argparse.ArgumentParser();p.add_argument('npz');p.add_argument('csv');p.add_argument('--start',type=float);p.add_argument('--duration',type=float,default=12.)
a=p.parse_args();z=np.load(a.npz,allow_pickle=False)
t=z['shadow/t_ns'][:,0]*1e-9
# Caller supplies an audited normal-tracking epoch; no guessed state fallback.
start=a.start if a.start is not None else 1791974052.2
ts=start+np.arange(int(round(a.duration/.002)))*.002
def interp(st,values):return np.column_stack([np.interp(ts,st,values[:,j]) for j in range(values.shape[1])])
q=interp(t,z['shadow/q']);mq=interp(t,z['shadow/measured_q'])
st=z['sensor/t_ns'][:,0]*1e-9
mv=np.column_stack([np.zeros((len(ts),3)),interp(st,z['sensor/joint_v'])[:,:18]])
pose=interp(t,z['shadow/torso_relative_tcp_pose'])
mt=z['target/t_ns'][:,0]*1e-9
rows=[(ts-start)[:,None],q,mq,mv]
for arm in range(2):
 part=pose[:,arm*7:(arm+1)*7].copy();part[:,3:7]/=np.linalg.norm(part[:,3:7],axis=1)[:,None]
 side='left' if arm==0 else 'right'
 rows.append(np.column_stack([part,interp(mt,z['target/'+side+'_elbow_hint'])]))
for side in ['left','right']:
 rows.append(np.column_stack([interp(mt,z['target/'+side+'_tcp_twist']),np.zeros((len(ts),3))]))
result=np.column_stack(rows)
assert result.shape[1]==102 and np.isfinite(result).all()
np.savetxt(a.csv,result,delimiter=',',fmt='%.12g')
print(json.dumps(dict(start=start,duration=a.duration,ticks=len(ts),columns=102,
  limitations=['100-Hz recorded shaped targets interpolated to 500 Hz','elbow feedforward unavailable: zero','body intent reconstructed from recorded FK','not raw Quest or a new physical recording']),indent=2))
