#!/usr/bin/env python3
"""Read-only closed-loop bag report. Errors use latest prior received command.

Usage: simulation_hold_report.py BAG PHASES_JSONL OUTPUT_JSON
All joint quantities are radians, rad/s and Nm. Skip 2 s of each phase for
settled metrics; this is a command tracking error, not a latency estimate.
"""
import bisect,json,sys
from collections import Counter
import numpy as np
import rosbag
phases=[json.loads(x) for x in open(sys.argv[2]) if x.strip()]
times=[x['time'] for x in phases]
labels=[x['label'] for x in phases]
stats={};counts=Counter();targets=Counter();refs=Counter();cmd=None
nonfinite=0;max_torque=None;overall_error=None;overall_velocity=None
with rosbag.Bag(sys.argv[1]) as bag:
 start=bag.get_start_time();end=bag.get_end_time()
 for topic,m,t in bag.read_messages():
  counts[topic]+=1
  now=t.to_sec();index=bisect.bisect_right(times,now)-1
  name=labels[index] if index>=0 else 'before_takeover'
  if topic=='/joint_cmd':cmd=np.asarray(m.joint_q);continue
  if topic=='/mm/teleop_tcp_target':targets[(name,int(m.control_state),bool(m.control_valid),bool(m.torso_control_enabled),bool(m.arm_hold))]+=1;continue
  if topic=='/whole_body_velocity_ik/reference/shm_mirror':
   refs[(name,int(m.authority_state),bool(m.valid),bool(m.takeover_active))]+=1
   nonfinite+=not np.isfinite(list(m.q)+list(m.dq)+list(m.ddq)).all()
  if topic!='/sensors_data_raw':continue
  q=np.asarray(m.joint_data.joint_q);v=np.asarray(m.joint_data.joint_v);tau=np.asarray(m.joint_data.joint_torque)
  nonfinite+=not np.isfinite(np.r_[q,v,tau]).all()
  max_torque=np.abs(tau) if max_torque is None else np.maximum(max_torque,np.abs(tau))
  overall_velocity=np.abs(v) if overall_velocity is None else np.maximum(overall_velocity,np.abs(v))
  if cmd is not None and len(cmd)==len(q):
   overall_error=np.abs(q-cmd) if overall_error is None else np.maximum(overall_error,np.abs(q-cmd))
  if cmd is None or len(cmd)!=len(q) or (index>=0 and now-times[index]<2):continue
  e=q-cmd
  if name not in stats:stats[name]={'n':0,'sum_e2':np.zeros(len(q)),'e_peak':np.zeros(len(q)),'v_peak':np.zeros(len(q)),'q_min':q.copy(),'q_max':q.copy(),'cmd_min':cmd.copy(),'cmd_max':cmd.copy()}
  s=stats[name];s['n']+=1;s['sum_e2']+=e*e;s['e_peak']=np.maximum(s['e_peak'],np.abs(e));s['v_peak']=np.maximum(s['v_peak'],np.abs(v));s['q_min']=np.minimum(s['q_min'],q);s['q_max']=np.maximum(s['q_max'],q);s['cmd_min']=np.minimum(s['cmd_min'],cmd);s['cmd_max']=np.maximum(s['cmd_max'],cmd)
result={'duration_s':end-start,'counts':dict(counts),'nonfinite_samples':int(nonfinite),'peak_measured_torque':max_torque.tolist(),'overall_tracking_error_peak':overall_error.tolist(),'overall_velocity_peak':overall_velocity.tolist(),'phases':{},'target_states':[[list(k),v] for k,v in targets.items()],'formal_states':[[list(k),v] for k,v in refs.items()]}
for name,s in stats.items():
 result['phases'][name]={'samples':s['n'],'error_rms':np.sqrt(s['sum_e2']/s['n']).tolist(),'error_peak':s['e_peak'].tolist(),'velocity_peak':s['v_peak'].tolist(),'measured_range':(s['q_max']-s['q_min']).tolist(),'command_range':(s['cmd_max']-s['cmd_min']).tolist()}
json.dump(result,open(sys.argv[3],'w'),indent=2)
for k,s in result['phases'].items():print(k,s['samples'],'arm error peak/rms',max(s['error_peak'][4:18]),max(s['error_rms'][4:18]),'arm range',max(s['command_range'][4:18]),'lower range',max(s['command_range'][:4]))
print('nonfinite',nonfinite,'duration',end-start)
