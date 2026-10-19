#!/usr/bin/env python3
"""Persistent launch-time thread partition. Never discovers or changes old PIDs."""
import json, os, platform, re, sys
from pathlib import Path

def cpu_list(text):
    result=set()
    for field in text.strip().split(','):
        if not field: continue
        span=field.split('-');lo=int(span[0]);hi=int(span[-1])
        if lo<0 or hi<lo or hi>=1024: raise ValueError('invalid CPU range')
        result.update(range(lo,hi+1))
    return result

def validate(profile, sysroot=Path('/sys/devices/system/cpu'), require_ik_isolated=True):
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
    protected = set(profile['wbc_cpus']) | (set(profile['ik_cpus']) if require_ik_isolated else set())
    if not protected <= isolated:
        raise ValueError('WBC/IK CPUs must already be kernel-isolated')
    if set(profile['background_cpus'])&isolated: raise ValueError('background uses isolated CPU')
    if 'sidecar_cpus' in profile:
        sidecar = set(profile['sidecar_cpus'])
        if not sidecar or not sidecar <= online:
            raise ValueError('unavailable CPUs: sidecar_cpus')
        for cpu in sidecar:
            siblings = cpu_list((sysroot/('cpu%d/topology/thread_siblings_list' % cpu)).read_text())
            if any(occupied.get(sibling) not in (None, 'background_cpus') for sibling in siblings):
                raise ValueError('sidecar shares a physical core with a protected role')
    if not 1<=profile['ik_fifo_priority']<90: raise ValueError('IK priority must remain below WBC')
    return profile

# These match the unchanged ARM controller and shared hardware drivers.
# Do not silently remap them: doing so would require a separate driver change.
ARM_WBC_CPU = 2
ARM_DRIVER_CPUS = {7}
CPU_ROOT = Path('/sys/devices/system/cpu')
DEFAULT_PROFILE = Path(__file__).resolve().parent.parent/'config/runtime_partition.json'
AAEON_PROFILE = DEFAULT_PROFILE.with_name('runtime_partition_aaeon_255h.json')


def cpu_model_name(cpuinfo=Path('/proc/cpuinfo')):
    for line in cpuinfo.read_text().splitlines():
        if line.startswith('model name'):
            return line.split(':', 1)[1].strip()
    return ''


def aaeon_profile(profile, sysroot, allowed, strict):
    """Use the audited 255H CPU map, also selectable by an explicit JSON file."""
    online = cpu_list((sysroot/'online').read_text())
    isolated = cpu_list((sysroot/'isolated').read_text())
    available = online if allowed is None else online & set(allowed)
    if profile['wbc_cpus'] != [2, 3] or profile['driver_reserved_cpus'] != [6, 7]:
        raise ValueError('255H profile must keep WBC 2/3 and driver reservation 6/7')
    if profile['ik_cpus'] != [5] or profile['mapper_cpus'] != [4]:
        raise ValueError('255H profile requires P-core IK 5 and Mapper 4')
    if profile['sidecar_cpus'] != [14, 15]:
        raise ValueError('255H profile must prefer LP-E sidecar CPUs 14/15')
    if not {2, 3, 7} <= isolated:
        raise ValueError('255H WBC 2/3 and driver 7 must already be kernel-isolated')
    if not {2, 3, 4, 5, 6, 7} <= available:
        raise ValueError('255H critical CPUs must be online and inside the cgroup cpuset')
    profile['background_cpus'] = sorted(set(profile['background_cpus']) & available - isolated)
    low_power = set(profile['sidecar_cpus']) & available
    # An offline/cpuset-excluded LP-E core must never make I/O fall onto a P core.
    profile['sidecar_cpus'] = sorted(low_power or
        (set(profile['background_cpus']) & set(range(8, 14))))
    profile['ik_isolation_mode'] = 'hard' if 5 in isolated else 'soft'
    profile['launch_fence'] = True
    return validate(profile, sysroot, require_ik_isolated=(strict or 5 in isolated))


def topology(sysroot=CPU_ROOT):
    online = cpu_list((sysroot/'online').read_text())
    isolated = cpu_list((sysroot/'isolated').read_text())
    siblings = {}
    capacities = {}
    frequencies = {}
    for cpu in sorted(online):
        root = sysroot/('cpu%d' % cpu)
        siblings[cpu] = cpu_list((root/'topology/thread_siblings_list').read_text())
        if cpu not in siblings[cpu]:
            raise ValueError('invalid topology for CPU %d' % cpu)
        for field, target in [('cpu_capacity', capacities),
                              ('cpufreq/cpuinfo_max_freq', frequencies)]:
            path = root/field
            target[cpu] = int(path.read_text()) if path.exists() else 0
    return online, isolated, siblings, capacities, frequencies


def cgroup_cpu_limit(procroot=Path('/proc/self'), fsroot=Path('/')):
    """Read the cpuset ceiling, not the inherited per-thread affinity mask.

    isolcpus can exclude RT cores from a normal launcher's inherited mask while
    still allowing explicit binding. Only a cgroup cpuset is a hard ceiling.
    """
    memberships = []
    for line in (procroot/'cgroup').read_text().splitlines():
        hierarchy, controllers, path = line.split(':', 2)
        if 'cpuset' in controllers.split(','):
            memberships.append(('cgroup', path))
        elif hierarchy == '0' and not controllers:
            memberships.append(('cgroup2', path))
    if not memberships:
        return None
    mounts = []
    for line in (procroot/'mountinfo').read_text().splitlines():
        before, after = line.split(' - ', 1)
        fields, options = before.split(), after.split()
        if options[0] == 'cgroup2' or (options[0] == 'cgroup' and 'cpuset' in options[2].split(',')):
            # Kernel mountinfo escapes whitespace and backslashes as octal.
            def unescape(value):
                for encoded, decoded in [('\\040', ' '), ('\\011', '\t'),
                                         ('\\012', '\n'), ('\\134', '\\')]:
                    value = value.replace(encoded, decoded)
                return value
            mounts.append((options[0], Path(unescape(fields[3])),
                           fsroot/unescape(fields[4]).lstrip('/')))
    limits = []
    for kind, member in memberships:
        resolved = False
        for mount_kind, root, mountpoint in mounts:
            if mount_kind != kind:
                continue
            try:
                relative = Path(member).relative_to(root)
            except ValueError:
                continue
            current = mountpoint/relative
            if not current.is_dir():
                continue
            resolved = True
            while True:
                names = ['cpuset.cpus.effective'] if kind == 'cgroup2' else ['cpuset.effective_cpus', 'cpuset.cpus']
                found = False
                for name in names:
                    file = current/name
                    if file.exists():
                        value = cpu_list(file.read_text())
                        if value:
                            limits.append(value)
                            found = True
                            break
                if found or current == mountpoint:
                    break
                current = current.parent
            break
        if not resolved:
            raise ValueError('cannot resolve the current cgroup cpuset; inspect /proc/self/cgroup and mountinfo')
    return set.intersection(*limits) if limits else None


def boolean(value):
    if str(value).lower() in ('true', '1', 'yes'): return True
    if str(value).lower() in ('false', '0', 'no'): return False
    raise ValueError('expected true/false, got ' + str(value))


def arm_profile(sysroot=CPU_ROOT, allowed=None, require_ik_hard_isolation=False,
                driver_reserved_cpus=None):
    """Hard IK isolation first, otherwise CPU 3 with a launch-local soft fence."""
    online, isolated, siblings, capacities, frequencies = topology(sysroot)
    allowed = online if allowed is None else set(allowed)
    drivers = ARM_DRIVER_CPUS if driver_reserved_cpus is None else set(driver_reserved_cpus)
    if not ARM_DRIVER_CPUS <= drivers or ARM_WBC_CPU in drivers:
        raise ValueError('ARM driver reservation must include CPU 7 and exclude WBC 2')
    fixed = drivers | {ARM_WBC_CPU}
    if not fixed <= online & allowed:
        raise ValueError('ARM controller/driver CPUs must be online and allowed: ' + str(sorted(fixed)))
    if ARM_WBC_CPU not in isolated:
        raise ValueError('ARM WBC CPU 2 must already be kernel-isolated')

    def family(cpus):
        return set().union(*(siblings[cpu] for cpu in cpus)) if cpus else set()

    def rank(cpu):
        return (-capacities[cpu], -frequencies[cpu], cpu)

    reserved = family(fixed)
    candidates = (online & isolated & allowed) - reserved
    candidates = {cpu for cpu in candidates
                  if (siblings[cpu] & online) <= isolated & allowed}
    mode = 'hard'
    # On the audited 12-core AGX keep IK on CPU3 even if another isolated core exists.
    if online == set(range(12)) and 3 not in candidates:
        candidates = set()
    if candidates:
        ik = 3 if 3 in candidates else min(candidates, key=rank)
    else:
        if require_ik_hard_isolation:
            raise ValueError('strict mode: ARM IK needs a separate allowed kernel-isolated core')
        ik = 3
        if ik not in online & allowed or siblings[ik] & reserved:
            raise ValueError('soft isolation requires online, cpuset-allowed CPU 3 outside WBC/drivers')
        mode = 'soft'
    reserved |= family({ik})
    candidates = (online & allowed) - reserved - isolated
    candidates = {cpu for cpu in candidates if not siblings[cpu] & (reserved | isolated)}
    mapper_choices = candidates - family({0, 1} & online)
    if not mapper_choices:
        raise ValueError('ARM Mapper needs a non-isolated core outside WBC/IK/drivers/0/1')
    mapper = 4 if 4 in mapper_choices else min(mapper_choices, key=rank)
    background = candidates - family({mapper})
    if not background:
        raise ValueError('ARM profile leaves no background CPU')
    profile = dict(version=1, wbc_cpus=[ARM_WBC_CPU],
                   driver_reserved_cpus=sorted(drivers), ik_cpus=[ik],
                   mapper_cpus=[mapper], background_cpus=sorted(background),
                   ik_fifo_priority=40, mapper_nice=-5,
                   python_switch_interval_sec=0.001, ik_isolation_mode=mode,
                   platform='agx_arm', launch_fence=True,
                   sidecar_cpus=sorted(background & {10, 11}) or sorted(background)[-2:])
    return validate(profile, sysroot, require_ik_isolated=(mode == 'hard'))




def select_profile(environ=None, machine=None, sysroot=CPU_ROOT, allowed=None, cpu_model=None):
    """Select the audited 255H map or ARM topology; other x86 retains its profile."""
    environ = os.environ if environ is None else environ
    machine = platform.machine().lower() if machine is None else machine.lower()
    explicit = environ.get('KUAVO_RUNTIME_PROFILE')
    arm = machine in ('aarch64', 'arm64')
    if not arm:
        if explicit is not None:
            path = Path(explicit)
        else:
            model = cpu_model_name() if cpu_model is None else cpu_model
            path = AAEON_PROFILE if re.search(r'\bCore\b.*\bUltra\s+7\s+255H\b', model) else DEFAULT_PROFILE
        profile = json.loads(path.read_text())
        if profile.get('platform') == 'aaeon_255h':
            strict = boolean(environ.get('KUAVO_REQUIRE_IK_HARD_ISOLATION', 'false'))
            return aaeon_profile(profile, sysroot, allowed, strict), str(path)
        # Unrelated x86 hosts and explicitly selected legacy JSON retain their policy.
        return validate(profile, sysroot), str(path)
    strict = boolean(environ.get('KUAVO_REQUIRE_IK_HARD_ISOLATION', 'false'))
    drivers = cpu_list(environ.get('KUAVO_ARM_DRIVER_RESERVED_CPUS', '7'))
    if explicit is None:
        profile = arm_profile(sysroot, allowed, strict, drivers)
        source = 'auto-arm-topology'
    else:
        path = Path(explicit)
        profile = json.loads(path.read_text())
        isolated = cpu_list((sysroot/'isolated').read_text())
        hard = set(profile['ik_cpus']) <= isolated
        profile['ik_isolation_mode'] = 'hard' if hard else 'soft'
        profile['platform'] = 'agx_arm'
        profile['launch_fence'] = True
        profile.setdefault('sidecar_cpus', sorted(set(profile['background_cpus']) & {10, 11})
                           or profile['background_cpus'][-2:])
        profile = validate(profile, sysroot, require_ik_isolated=(strict or hard))
        source = str(path)
    if profile['wbc_cpus'] != [ARM_WBC_CPU]:
        raise ValueError('ARM profile must match the controller WBC CPU 2')
    if not (ARM_DRIVER_CPUS | drivers) <= set(profile['driver_reserved_cpus']):
        raise ValueError('ARM profile must reserve configured driver CPUs including 7')
    used = set().union(*(set(profile[role]) for role in
                        ['wbc_cpus', 'driver_reserved_cpus', 'ik_cpus',
                         'mapper_cpus', 'background_cpus', 'sidecar_cpus']))
    if allowed is not None and not used <= set(allowed):
        raise ValueError('ARM profile requests CPUs outside the cgroup cpuset')
    return profile, source


def profile_warnings(profile):
    if profile.get('ik_isolation_mode') != 'soft': return []
    return ['Soft-Isolation Fallback: IK core is not kernel-isolated; '
            + ('stock AGX isolcpus=2,7; ' if profile.get('platform') == 'agx_arm' else 'AAEON 255H; ')
            + 'IK=' + str(profile['ik_cpus']) +
            ' requests SCHED_FIFO; launch-local background fence=' +
            str(profile['background_cpus']) +
            '. External services/IRQs are not isolated. '
            'Use require_ik_hard_isolation:=true for strict rejection.']


def report_warnings(profile):
    for warning in profile_warnings(profile):
        sys.stderr.write('[runtime-partition][WARN] ' + warning + '\n')


def inspect_profile():
    """Read-only preflight: no ROS import, affinity, scheduler, or file mutation."""
    online, isolated, siblings, capacities, frequencies = topology()
    result = dict(machine=platform.machine(), online=sorted(online),
                  isolated=sorted(isolated), inherited_affinity=sorted(os.sched_getaffinity(0)),
                  siblings={str(k): sorted(v) for k, v in siblings.items()},
                  capacity=capacities, max_frequency_khz=frequencies)
    try:
        limit = cgroup_cpu_limit()
        result['cgroup_cpu_limit'] = sorted(limit) if limit is not None else None
        result['profile'], result['source'] = select_profile(allowed=limit)
        result['warnings'] = profile_warnings(result['profile'])
        report_warnings(result['profile'])
        result['valid'] = True
    except (ValueError, OSError, KeyError, TypeError) as error:
        result.update(valid=False, error=str(error))
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result['valid'] else 2


def main():
    if sys.argv[1:] == ['--check']:
        raise SystemExit(inspect_profile())
    command = sys.argv[1:]
    parent_launch = command[:1] == ['--launch']
    if parent_launch:
        command = command[1:]
    if len(command)<2: raise SystemExit('usage: runtime_partition.py --check | [--launch] wrapper role [args...]')
    arm = platform.machine().lower() in ('aarch64', 'arm64')
    limit = cgroup_cpu_limit() if arm else None
    profile, source = select_profile(allowed=limit)
    if not arm and profile.get('launch_fence'):
        limit = cgroup_cpu_limit()
        if limit is not None:
            profile, source = select_profile(allowed=limit)
    if parent_launch and not profile.get('launch_fence'):
        os.execv(command[0], command)
        return
    if profile.get('launch_fence'):
        sys.stderr.write('[runtime-partition][INFO] profile=' + source +
                         ' IK isolation=' + profile['ik_isolation_mode'] +
                         ' sidecar=' + str(profile['sidecar_cpus']) + '\n')
        report_warnings(profile)
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
    sys.stderr.write('[runtime-partition] '+command[1]+' background='+str(profile['background_cpus'])+' IK='+str(profile['ik_cpus'])+' Mapper='+str(profile['mapper_cpus'])+'\n')
    # Python 3.8 buffers redirected stderr; exec does not flush Python streams.
    sys.stderr.flush()
    os.execv(command[0],command)

if __name__=='__main__': main()
