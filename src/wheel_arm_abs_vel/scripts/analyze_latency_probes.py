#!/usr/bin/env python3
"""Read diagnostic bags offline. No ROS master or robot commands.

Optional --start/--end are seconds from each bag's start; select steady FOLLOW
for hardware acceptance. WBC stage percentiles are conditional on >2 ms cycles.
"""
import argparse
import json
from pathlib import Path
import numpy as np
import rosbag

PREFIX='/whole_body_velocity_ik/diagnostics/'
TOPICS={'claw':'/hardware/diagnostics/leju_claw_callback_v1',
        'claw_worker':'/hardware/diagnostics/leju_claw_worker_v2',
        'wbc':'/monitor/wheel/cycle_timing_v1','mapper':PREFIX+'mapper_input_gap_v1'}

def stats(values):
    a=np.asarray(values,dtype=float);a=a[np.isfinite(a)]
    if not len(a):return {'n':0}
    return dict(n=len(a),median_ms=float(np.median(a)),p95_ms=float(np.percentile(a,95)),
                p99_ms=float(np.percentile(a,99)),max_ms=float(np.max(a)))

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bags',nargs='+')
    p.add_argument('--start',type=float,default=0);p.add_argument('--end',type=float)
    p.add_argument('--output',required=True);args=p.parse_args()
    rows={k:[] for k in TOPICS};old_wbc=[];diagnostic_drops=[]
    for path in args.bags:
        with rosbag.Bag(path) as bag:
            origin=bag.get_start_time()
            for topic,msg,t in bag.read_messages(topics=list(TOPICS.values())+['/monitor/time_cost/wbc','/monitor/wbc_diagnostics/dropped']):
                relative=t.to_sec()-origin
                if relative<args.start or (args.end is not None and relative>args.end):continue
                if topic=='/monitor/wbc_diagnostics/dropped':
                    diagnostic_drops.append(float(msg.data));continue
                if topic=='/monitor/time_cost/wbc':
                    old_wbc.append(float(msg.data));continue
                for key,value in TOPICS.items():
                    if value==topic and msg.data and msg.data[0]==(2 if key=='claw_worker' else 1):rows[key].append(msg.data)
    report=dict(bags=args.bags,start=args.start,end=args.end,missing=[k for k,v in rows.items() if not v])
    if rows['claw']:
        a=np.array(rows['claw']);assert a.shape[1]==12
        report['claw']={name:stats(a[:,i]) for name,i in [('parameter',3),('controlLejuClaw',4),('callback',5),('mode_notify',6)]}
        report['claw'].update(dropped_max=float(a[:,8].max()),writer_errors_max=float(a[:,9].max()))
    if rows['claw_worker']:
        a=np.array(rows['claw_worker']);assert a.shape[1]==14
        report['claw_worker']={name:stats(a[:,i]) for name,i in [('parameter',3),('controlLejuClaw',4),('worker',5),('mode_notify',6),('mailbox_age',13)]}
        report['claw_worker'].update(dropped_max=float(a[:,8].max()),writer_errors_max=float(a[:,9].max()))
    if rows['wbc']:
        a=np.array(rows['wbc']);assert a.shape[1]==33
        report['wbc_slow_cycles']={name:stats(a[:,i]) for name,i in [('wall',6),('cpu',7),('off_cpu',8)]}
        report['wbc_slow_cycles'].update(cpu_over_70pct=int((a[:,7]>=.7*a[:,6]).sum()),
            off_cpu_over_50pct=int((a[:,8]>=.5*a[:,6]).sum()),
            incomplete_stage_records=int((a[:,9]<6).sum()),dropped_max=float(a[:,10].max()),writer_errors_max=float(a[:,11].max()))
        report['wbc_slow_cycles']['stages']={name:{kind:stats(a[a[:,9]==6,12+3*i+j])
            for j,kind in enumerate(['wall','cpu','off_cpu'])}
            for i,name in enumerate(['sensor_obs','FK','MPC_MRT','cmd_prepare','dynamics_RNEA_or_WBC','publish','tail'])}
    if rows['mapper']:
        a=np.array(rows['mapper']);assert a.shape[1]==14
        report['mapper']={name:stats(a[:,i]) for name,i in [('entry_gap',7),('source_stamp_interval',10),('entry_gap_thread_cpu',12)]}
        report['mapper'].update(source_kind_counts={str(int(k)):int((a[:,8]==k).sum()) for k in np.unique(a[:,8])},
            gaps_over_38ms=int((a[:,7]>38).sum()),gaps_over_100ms=int((a[:,7]>100).sum()),dropped_max=float(a[:,13].max()))
    if diagnostic_drops:
        report['wbc_diagnostics_dropped_max']=max(diagnostic_drops)
    if old_wbc:
        report['legacy_wbc_body']=stats(old_wbc)
        report['legacy_wbc_body']['over_2ms_pct']=float(100*np.mean(np.array(old_wbc)>2.0))
    report['interpretation']='off_cpu includes blocking and preemption, not a unique lock diagnosis; use sched trace. No slow WBC rows can mean no slow cycles or missing capture; verify topic publishers. Legacy WBC body excludes the final chassis/tail work.'
    Path(args.output).write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))

if __name__=='__main__':main()
