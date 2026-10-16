#!/usr/bin/env python3
import legacy_dependencies
"""Persistent launch-time thread partition. Never discovers or changes old PIDs."""
import json, os, sys
from pathlib import Path

def cpu_list(text):
    result=set()
    for field in text.strip().split(','):
        if not field: continue
        span=field.split('-');lo=int(span[0]);hi=int(span[-1])
        if lo<0 or hi<lo or hi>=1024: raise ValueError('invalid CPU range')
        result.update(range(lo,hi+1))
    return result

def validate(profile, sysroot=Path('/sys/devices/system/cpu')):
    online=cpu_list((sysroot/'online').read_text())
    isolated=cpu_list((sysroot/'isolated').read_text())
    roles=['wbc_cpus','driver_reserved_cpus','ik_cpus','mapper_cpus','background_cpus']
    occupied={}
    for role in roles:
        requested=set(profile[role])
        if not requested or not requested<=online: raise ValueError('unavailable CPUs: '+role)
        for cpu in requested:
            siblings=cpu_list((sysroot/('cpu%d/topology/thread_siblings_list'%cpu)).read_text())
            for sibling in siblings:
                if sibling in occupied and occupied[sibling]!=role:
                    raise ValueError('physical core shared by '+role+' and '+occupied[sibling])
                occupied[sibling]=role
    if not (set(profile['wbc_cpus'])|set(profile['ik_cpus']))<=isolated:
        raise ValueError('WBC/IK CPUs must already be kernel-isolated')
    if set(profile['background_cpus'])&isolated: raise ValueError('background uses isolated CPU')
    if not 1<=profile['ik_fifo_priority']<90: raise ValueError('IK priority must remain below WBC')
    return profile

def main():
    if len(sys.argv)<3: raise SystemExit('usage: runtime_partition.py wrapper role [args...]')
    path=Path(os.environ.get('KUAVO_RUNTIME_PROFILE',str(Path(__file__).resolve().parent.parent/'config/runtime_partition.json')))
    profile=validate(json.loads(path.read_text()))
    # Apply only to the calling thread before exec; ROS/background threads inherit
    # the ordinary policy. IK/Mapper opt in from their actual critical thread.
    os.sched_setscheduler(0,os.SCHED_OTHER,os.sched_param(0))
    os.sched_setaffinity(0,set(profile['background_cpus']))
    os.environ['KUAVO_RUNTIME_PARTITION_JSON']=json.dumps(profile,separators=(',',':'))
    os.environ['KUAVO_IK_CPUS']=','.join(map(str,profile['ik_cpus']))
    os.environ['KUAVO_IK_PRIORITY']=str(profile['ik_fifo_priority'])
    os.environ['KUAVO_SCHED_APPLIED']='1'
    for name in ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS']:
        os.environ[name]='1'
    sys.stderr.write('[runtime-partition] '+sys.argv[2]+' background='+str(profile['background_cpus'])+' IK='+str(profile['ik_cpus'])+' Mapper='+str(profile['mapper_cpus'])+'\n')
    os.execv(sys.argv[1],sys.argv[1:])

if __name__=='__main__': main()
