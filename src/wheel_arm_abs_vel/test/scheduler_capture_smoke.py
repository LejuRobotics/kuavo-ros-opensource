#!/usr/bin/env python3
"""Root-only trace smoke test; moves only its own test thread, no robot/ROS."""
import argparse
import json
import os
from pathlib import Path
import sys
import threading
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from tools.scheduler_capture import FlightTrace, TRACE_ROOT, read

p=argparse.ArgumentParser();p.add_argument('--output',required=True);args=p.parse_args()
assert os.geteuid()==0, 'sudo required for private tracefs instance'
cpus=sorted(os.sched_getaffinity(0));assert len(cpus)>=2
out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
before={k:read(TRACE_ROOT/k) for k in ('tracing_on','current_tracer','trace_clock','events/enable')}
ready=threading.Event();go=threading.Event();identity=[]
def migrate():
    identity.append(threading.get_native_id());ready.set();go.wait()
    for i in range(80):
        os.sched_setaffinity(0,{cpus[i%2]})
        time.sleep(.002)
t=threading.Thread(target=migrate);t.start();ready.wait()
trace=None
try:
    trace=FlightTrace(out,[identity[0],os.getpid()],os.getpid())
    started=time.monotonic_ns();go.set();t.join()
    event=dict(begin=started,end=time.monotonic_ns(),category='self_test',duration_ns=time.monotonic_ns()-started)
    metadata=trace.snapshot('own_thread',event)
    text=(out/'own_thread.trace').read_text()
    counts={name:text.count(name+':') for name in ['sched_switch','sched_migrate_task','sched_wakeup']}
    assert counts['sched_switch']>0 and counts['sched_migrate_task']>0,counts
    assert read(trace.path/'tracing_on')=='1','capture did not resume'
    metadata.update(counts=counts,own_test_tid=identity[0],own_test_pid=os.getpid())
finally:
    go.set();t.join()
    if trace:trace.close()
after={k:read(TRACE_ROOT/k) for k in before};assert before==after,(before,after)
metadata.update(global_before=before,global_after=after,passed=True)
(out/'result.json').write_text(json.dumps(metadata,indent=2))
print(json.dumps({k:metadata[k] for k in ['passed','counts','snapshot_mode','capture_blind_interval_ns']},indent=2))
