#!/usr/bin/env python3
"""Guided personal glove calibration. Isolated acquisition; no robot commands."""
import argparse
import contextlib
import datetime
import errno
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import uuid

# catkin devel relays set __file__ to this source script; installed siblings
# reside in the package's lib directory.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from gesture_calibration import (DEFAULT_PROFILE_PATH, STAGES, MAX_GAP_SEC,
                                 MIN_RATE_HZ, MAX_SAMPLES_PER_SIDE, atomic_save_profile,
                                 fit_profile, input_contract, load_profile, validate_partial_recording)
from glove_gesture_node import positions

TOPICS = {'left': '/qianshi/glove_left/fingertips',
          'right': '/qianshi/glove_right/fingertips'}
DISCARD = {'/glove_gesture_calibration/discard_left',
           '/glove_gesture_calibration/discard_right'}
ALLOWED_NODES = {'/rosout', '/glove_gesture_calibrator',
                 '/glove_calibration_receiver', '/sg100_glove_both'}
ALLOWED_PUBLISHERS = set(TOPICS.values()) | DISCARD | {'/rosout', '/rosout_agg'}
BLOCKED_PROGRAMS = {'sg100_windows_udp_node.py', 'sg100_glove_node.py',
                    'sg100_live_native.py', 'glove_gesture_node.py',
                    'monitor_quest3_velocity.py', 'monitor_quest3.py',
                    'monitor_quest3_cpp', 'humanoid_quest_absolute_control'}
PROMPTS = {
    'open': '双手舒适张开，所有手指分开，不捏合',
    'relaxed': '双手自然放松，不捏合、不握拳',
    'flat': '双手四指伸直并拢，拇指分开；不要握拳',
    'index': '双手分别拇指捏食指（X+A），其余手指松开',
    'middle': '双手分别拇指捏中指，食指和小指松开',
    'little': '双手分别拇指捏小指（交互手势），食指和中指松开',
    'fist': '双手握拳',
    'left_fist': '仅左手握拳，右手张开（检查单拳是否误触停机）',
    'right_fist': '仅右手握拳，左手张开',
    'xb': '左拇指捏食指、右拇指捏中指（X+B），其余手指松开',
    'xy': '左拇指同时捏食指和中指（X+Y），左无名指/小指伸开，右手张开',
}


def profile_name(value):
    if not isinstance(value, str) or not re.fullmatch(r'[\w-]{1,48}', value):
        raise ValueError('名字限1–48个中文/字母/数字/下划线/短横线，不能含路径分隔符')
    return value


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_symlink():
        raise ValueError('Refusing symbolic-link output: ' + str(path))
    encoded = json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    fd, temporary = tempfile.mkstemp(prefix=path.name + '.', dir=str(path.parent))
    try:
        with os.fdopen(fd, 'w') as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def process_table():
    table = {}
    for directory in Path('/proc').iterdir():
        if not directory.name.isdigit():
            continue
        try:
            argv = (directory/'cmdline').read_bytes().split(b'\0')
            # The executable name inside (...) can contain spaces.
            stat = (directory/'stat').read_text().rsplit(')', 1)[1].split()
            table[int(directory.name)] = (int(stat[1]),
                [part.decode(errors='replace') for part in argv if part])
        except (OSError, ValueError, IndexError):
            continue
    return table


def assert_no_other_receivers(owned_root=None):
    table = process_table()
    owned = {os.getpid()}
    if owned_root is not None:
        owned.add(owned_root)
        changed = True
        while changed:
            before = len(owned)
            owned.update(pid for pid, (parent, _) in table.items() if parent in owned)
            changed = len(owned) != before
    conflicts = []
    for pid, (_, argv) in table.items():
        if pid not in owned and any(Path(arg).name in BLOCKED_PROGRAMS for arg in argv):
            conflicts.append('%s: %s' % (pid, ' '.join(argv[:3])))
    if conflicts:
        raise RuntimeError('先正常退出已有手套/Quest遥操接收任务，再标定：\n' + '\n'.join(conflicts))


def check_graph(state, caller='/glove_gesture_calibrator'):
    publishers, subscribers, services = state
    nodes = {node for group in state for _, clients in group for node in clients}
    unexpected = nodes - ALLOWED_NODES
    if unexpected:
        raise RuntimeError('标定ROS master出现非采集节点：' + ', '.join(sorted(unexpected)))
    for topic, clients in publishers:
        if clients and topic not in ALLOWED_PUBLISHERS:
            raise RuntimeError('标定不允许输出话题：' + topic)
    for topic, clients in subscribers:
        if topic in DISCARD and clients:
            raise RuntimeError('丢弃关节话题不能有订阅者：' + topic)
        if topic in TOPICS.values() and any(node != caller for node in clients):
            raise RuntimeError('指尖数据出现标定器以外的订阅者：' + topic)


def live_process_identities():
    """Snapshot parent PID and kernel start time; ignore exited zombies."""
    members = {}
    for directory in Path('/proc').iterdir():
        if not directory.name.isdigit():
            continue
        try:
            fields = (directory/'stat').read_text().rsplit(')', 1)[1].split()
            if fields[0] != 'Z':
                members[int(directory.name)] = (int(fields[1]), int(fields[19]))
        except (OSError, ValueError, IndexError):
            continue
    return members


def refresh_owned_processes(known):
    # roslaunch creates a separate session for each node. Follow ancestry,
    # retaining observed identities even when a parent exits/reparents a child.
    table = live_process_identities()
    owned = {pid for pid, birth in known.items()
             if pid in table and table[pid][1] == birth}
    while True:
        more = {pid for pid, (parent, _) in table.items() if parent in owned}
        if more.issubset(owned):
            break
        owned.update(more)
    known.update({pid: table[pid][1] for pid in owned})
    return {pid: known[pid] for pid in owned}


def stop_owned_processes(process, known):
    for sig, timeout in ((signal.SIGINT, 15.), (signal.SIGTERM, 5.), (signal.SIGKILL, 5.)):
        owned = refresh_owned_processes(known)
        if not owned:
            process.wait(timeout=1.)
            return
        for pid, birth in owned.items():
            # Recheck identity immediately before signaling; never signal a
            # reused PID or unrelated receivers sharing this workstation.
            current = live_process_identities().get(pid)
            if current is not None and current[1] == birth:
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            process.poll()
            if not refresh_owned_processes(known):
                process.wait(timeout=1.)
                return
            time.sleep(.05)
    raise RuntimeError('本次采集进程未完全退出，请检查采集日志')


def check_master_port(port):
    if not 1024 <= port <= 65535 or port == 11311:
        raise ValueError('标定master端口必须为非11311的1024–65535端口')
    probe = socket.socket()
    try:
        # Match the ROS XML-RPC server's TCP reuse behavior. TIME_WAIT from a
        # clean previous exit is not an active master. Do not use SO_REUSEPORT.
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind(('127.0.0.1', port))
        probe.listen(1)
    except OSError as exc:
        if exc.errno == errno.EADDRINUSE:
            raise RuntimeError('标定master端口 %d 正被占用，拒绝连接已有master；'
                               '请检查已有采集任务，或用 --master-port 指定其他空闲端口' % port) from exc
        raise RuntimeError('无法检查标定master端口 %d：%s' % (port, exc)) from exc
    finally:
        probe.close()


@contextlib.contextmanager
def isolated_receiver(args):
    import yaml
    assert_no_other_receivers()
    check_master_port(args.master_port)
    token = uuid.uuid4().hex
    uri = 'http://127.0.0.1:%d' % args.master_port
    os.environ['ROS_MASTER_URI'] = uri
    os.environ['ROS_IP'] = '127.0.0.1'
    os.environ.pop('ROS_HOSTNAME', None)
    command = ['roslaunch', '--port', str(args.master_port), 'sg100_glove_bridge',
               'glove_gesture_calibration_input.launch', 'session_token:=' + token,
               'backend:=' + args.backend,
               'left_port:=' + str(args.left_port), 'right_port:=' + str(args.right_port),
               'bind_address:=' + args.bind_address,
               'left_module_map:=' + yaml.safe_dump(args.left_module_map, default_flow_style=True).strip(),
               'right_module_map:=' + yaml.safe_dump(args.right_module_map, default_flow_style=True).strip(),
               'left_id:=' + str(args.left_id), 'right_id:=' + str(args.right_id),
               'serial_port:=' + args.serial_port, 'sdk_python:=' + args.sdk_python]
    process = subprocess.Popen(command, start_new_session=True)
    identity = live_process_identities().get(process.pid)
    known_processes = {process.pid: identity[1]} if identity is not None else {}
    try:
        import rosgraph
        master = rosgraph.Master('/glove_gesture_calibrator', master_uri=uri)
        deadline = time.monotonic() + 30.
        while True:
            refresh_owned_processes(known_processes)
            if process.poll() is not None:
                raise RuntimeError('采集后端启动失败，请检查端口占用或SDK环境')
            try:
                found = master.getParam('/glove_gesture_calibration/session_token')
                if found != token:
                    raise RuntimeError('标定master身份不匹配')
                break
            except (OSError, rosgraph.MasterError):
                if time.monotonic() >= deadline:
                    raise RuntimeError('等待独立标定master超时')
                time.sleep(.1)
        def guard():
            refresh_owned_processes(known_processes)
            if process.poll() is not None:
                raise RuntimeError('采集后端已退出')
            if os.environ.get('ROS_MASTER_URI') != uri:
                raise RuntimeError('标定ROS master地址发生变化')
            if master.getParam('/glove_gesture_calibration/session_token') != token:
                raise RuntimeError('标定session已变化')
            assert_no_other_receivers(process.pid)
            check_graph(master.getSystemState())
        guard()
        yield guard
    finally:
        stop_owned_processes(process, known_processes)


class Collector:
    def __init__(self, guard):
        import rospy
        from std_msgs.msg import Float32MultiArray
        self.rospy = rospy
        self.guard = guard
        self.lock = threading.Lock()
        self.epoch = time.monotonic()
        self.latest = {side: None for side in TOPICS}
        self.buffers = None
        self.errors = []
        self.raw_last = {}
        self.window_resets = 0
        self.reset_reason = ''
        self.subscribers = [rospy.Subscriber(topic, Float32MultiArray,
            lambda msg, side=side: self.receive(side, msg.data), queue_size=1000)
            for side, topic in TOPICS.items()]

    def _restart_window_locked(self, reason):
        # A window is a simultaneous two-hand trace. Never retain the other
        # hand's old samples when either side becomes invalid or discontinuous.
        self.buffers = {side: [] for side in TOPICS}
        self.raw_last = {}
        self.window_resets += 1
        self.reset_reason = reason

    def receive(self, side, data):
        now = time.monotonic()
        try:
            values = [float(v) for v in data]
            positions(values)
        except (ValueError, TypeError, OverflowError):
            with self.lock:
                if self.buffers is not None:
                    self._restart_window_locked(side + ': invalid raw DoF frame')
            return
        with self.lock:
            self.latest[side] = now
            if self.buffers is None:
                return
            previous = self.raw_last.get(side)
            if previous is not None and now - previous > MAX_GAP_SEC:
                self._restart_window_locked('%s: ROS到达间隔 %.3fs 超过 %.3fs' %
                                            (side, now - previous, MAX_GAP_SEC))
            self.raw_last[side] = now
            samples = self.buffers[side]
            if len(samples) >= MAX_SAMPLES_PER_SIDE:
                if not self.errors:
                    self.errors.append('Too many samples in one hold')
                return
            # Every accepted callback is a real frame. Do not discard the
            # tightly spaced frames within a UDP delivery burst.
            samples.append({'t': now - self.epoch, 'data': values})

    def wait_ready(self, timeout=30.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.guard()
            now = time.monotonic()
            with self.lock:
                ready = all(t is not None and now-t <= MAX_GAP_SEC for t in self.latest.values())
            if ready:
                return
            if self.rospy.is_shutdown():
                raise RuntimeError('标定接收已关闭')
            time.sleep(.1)
        raise RuntimeError('左右手新鲜DoF数据不足；检查SendDof、端口及五指模块映射')

    @staticmethod
    def sample_statistics(samples, elapsed):
        count = len(samples)
        span = samples[-1]['t'] - samples[0]['t'] if count >= 2 else 0.
        return dict(count=count, span=span,
                    hz=(count-1)/span if span > 0 else 0.,
                    gap=max((b['t']-a['t'] for a, b in zip(samples, samples[1:])), default=0.),
                    age=elapsed-samples[-1]['t'] if count else float('inf'))

    @staticmethod
    def statistics_text(statistics):
        return '; '.join('%s: %d帧, 有效%.2fs, %.1fHz, 最大到达间隔%.3fs, 末帧距今%.3fs' %
                         (side, value['count'], value['span'], value['hz'], value['gap'], value['age'])
                         for side, value in statistics.items())

    def capture(self, seconds):
        start = time.monotonic()
        deadline = start + seconds + 10.
        self.wait_ready(timeout=max(.01, deadline - time.monotonic()))
        required_span = max(1., seconds - .25)
        with self.lock:
            self.buffers = {side: [] for side in TOPICS}
            self.errors = []
            self.raw_last = {}
            self.window_resets = 0
            self.reset_reason = ''
        complete = False
        printed_resets = 0
        try:
            next_check = time.monotonic()
            next_progress = next_check + 1.
            while time.monotonic() < deadline:
                if self.rospy.is_shutdown():
                    raise RuntimeError('采集提前退出')
                if time.monotonic() >= next_check:
                    self.guard()
                    next_check = time.monotonic() + .3
                with self.lock:
                    if all(self.buffers.values()) and abs(
                            self.buffers['left'][0]['t'] - self.buffers['right'][0]['t']) > MAX_GAP_SEC:
                        self._restart_window_locked('左右首帧到达偏差过大')
                    elapsed = time.monotonic() - self.epoch
                    stats = {side: self.sample_statistics(values, elapsed)
                             for side, values in self.buffers.items()}
                    complete = all(value['span'] >= required_span and
                                   value['hz'] >= MIN_RATE_HZ and
                                   value['gap'] <= MAX_GAP_SEC and
                                   value['age'] <= MAX_GAP_SEC
                                   for value in stats.values())
                    if complete:
                        complete = all(abs(self.buffers['left'][index]['t'] -
                                           self.buffers['right'][index]['t']) <= MAX_GAP_SEC
                                       for index in (0, -1))
                    fatal = bool(self.errors)
                    resets, reason = self.window_resets, self.reset_reason
                    if fatal or complete:
                        break
                now = time.monotonic()
                if resets != printed_resets:
                    print('继续保持当前动作：%s；双手窗口已重置，自动重新计时（%d次，剩余%.1fs）' %
                          (reason, resets, max(0., deadline-now)), flush=True)
                    printed_resets = resets
                if now >= next_progress:
                    print('采集中，目标有效%.1fs，剩余%.1fs；%s' %
                          (required_span, max(0., deadline-now), self.statistics_text(stats)), flush=True)
                    next_progress = now + 1.
                time.sleep(.01)
        finally:
            with self.lock:
                buffers, self.buffers = self.buffers, None
                errors = list(self.errors)
                resets, reason = self.window_resets, self.reset_reason
                elapsed = time.monotonic() - self.epoch
        stats = {side: self.sample_statistics(values, elapsed) for side, values in buffers.items()}
        detail = self.statistics_text(stats)
        # Recheck the detached snapshot: a callback may have reset the window
        # between leaving the success check and acquiring the final lock.
        complete = complete and all(
            v['span'] >= required_span and v['hz'] >= MIN_RATE_HZ and
            v['gap'] <= MAX_GAP_SEC and v['age'] <= MAX_GAP_SEC for v in stats.values())
        if complete:
            complete = all(abs(buffers['left'][i]['t']-buffers['right'][i]['t']) <= MAX_GAP_SEC
                           for i in (0, -1))
        if errors:
            raise RuntimeError('采集不能继续：' + '; '.join(sorted(set(errors))) + '\n' + detail)
        if not complete:
            raise ValueError('已达采集时限，未得到双手连续%.2fs、至少%.0fHz的合格窗口；'
                             '到达间隔/末帧年龄/左右时间偏差要求不超过%.2fs。'
                             '本次自动重置%d次，最近原因：%s。\n%s' %
                             (required_span, MIN_RATE_HZ, MAX_GAP_SEC, resets, reason or '数据密度不足', detail))
        print('采集质量：' + detail, flush=True)
        return buffers

    def close(self):
        for subscriber in self.subscribers:
            subscriber.unregister()


def prompt(message):
    reply = input(message + '；准备好按回车开始，输入 q 退出：').strip().lower()
    if reply == 'q':
        raise KeyboardInterrupt()


def collect(args, contract, recording):
    completed = len(recording['segments'])
    total = len(STAGES)*3
    if completed == total:
        return
    with isolated_receiver(args) as guard:
        import rospy
        rospy.init_node('glove_gesture_calibrator', disable_signals=True)
        collector = Collector(guard)
        if completed:
            last_time = max(sample['t'] for segment in recording['segments']
                            for side in TOPICS for sample in segment[side])
            collector.epoch -= last_time + 1.
        try:
            print('仅采集手套数据，不需要Quest或运控。检测左右SendDof…', flush=True)
            collector.wait_ready()
            for round_index in (1, 2, 3):
                for stage_index, stage in enumerate(STAGES):
                    ordinal = (round_index-1)*len(STAGES)+stage_index
                    if ordinal < completed:
                        continue
                    print('\n第 %d/3 轮，动作 %d/%d，已保存 %d/%d：%s' %
                          (round_index, stage_index+1, len(STAGES), len(recording['segments']),
                           total, PROMPTS[stage]), flush=True)
                    while True:
                        prompt('先双手张开并松开全部捏合，保持到完成提示（有效至少1秒）')
                        try:
                            release = collector.capture(1.25)
                            break
                        except ValueError as exc:
                            print(str(exc) + '\n本步未保存；可改善网络后重试，或输入 q 退出后断点继续。', flush=True)
                    while True:
                        prompt(PROMPTS[stage])
                        print('保持当前动作直到完成提示；有效至少 %.1f 秒，网络抖动时自动重新计时，'
                              '本次总等待最多 %.1f 秒。' % (args.hold_seconds, args.hold_seconds+10.25), flush=True)
                        try:
                            samples = collector.capture(args.hold_seconds + .25)
                            break
                        except ValueError as exc:
                            print(str(exc) + '\n保持/重新摆好当前目标后按回车重试，无需重复刚才的释放动作；'
                                  '输入 q 可保存进度退出。', flush=True)
                    candidate = dict(recording, segments=recording['segments'] + [
                        dict(stage=stage, round=round_index, release=release, **samples)])
                    validated = validate_partial_recording(candidate, contract, recording['profile_name'])
                    recording['segments'] = validated['segments']
                    checkpoint = getattr(args, '_checkpoint', None)
                    if checkpoint is not None:
                        checkpoint(recording)
                    print('采集完成，进度 %d/%d 已保存，请松开。' %
                          (len(recording['segments']), total), flush=True)
        finally:
            collector.close()
            rospy.signal_shutdown('glove calibration collection complete')


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    modes = p.add_mutually_exclusive_group()
    modes.add_argument('--from-recording', type=Path, help='离线重算本工具保存的原始标定JSON')
    modes.add_argument('--resume-recording', type=Path, help='校验并继续未完成的原始采集；另存新文件保留原记录')
    modes.add_argument('--activate', metavar='NAME', help='选择已通过验证的个人配置；下次遥操加载')
    modes.add_argument('--list', action='store_true', help='列出已保存的个人配置')
    p.add_argument('--profile', help='操作者名字（可中文）')
    p.add_argument('--directory', type=Path, default=Path(DEFAULT_PROFILE_PATH).parent)
    p.add_argument('--no-activate', action='store_true', help='只保存具名配置，不更新启动默认active')
    p.add_argument('--backend', choices=('windows_udp', 'linux_sdk'), default='windows_udp')
    p.add_argument('--left-port', type=int, default=18003)
    p.add_argument('--right-port', type=int, default=18002)
    p.add_argument('--bind-address', default='0.0.0.0')
    p.add_argument('--left-module-map', nargs=5, default=list('ABCDE'))
    p.add_argument('--right-module-map', nargs=5, default=list('ABCDE'))
    p.add_argument('--left-id', type=int, default=12)
    p.add_argument('--right-id', type=int, default=2)
    p.add_argument('--serial-port', default='auto')
    p.add_argument('--sdk-python', default='')
    p.add_argument('--master-port', type=int, default=11339)
    p.add_argument('--hold-seconds', type=float, default=4.)
    return p


def run(args):
    directory = args.directory.expanduser().absolute()
    active = directory/'active.yaml'
    if args.list:
        for path in sorted((directory/'profiles').glob('*.yaml')):
            try:
                profile = load_profile(path)
                print('%s\t%s\t%s' % (profile['profile_name'], profile['input']['backend'], path))
            except ValueError as exc:
                print('%s\t无效：%s' % (path, exc))
        return 0
    if args.activate:
        name = profile_name(args.activate)
        profile = load_profile(directory/'profiles'/(name+'.yaml'))
        atomic_save_profile(active, profile)
        print('已选择 %s；下次遥操自动加载：%s' % (name, active))
        return 0
    if not math.isfinite(args.hold_seconds) or not 3. <= args.hold_seconds <= 15.:
        raise ValueError('标定固定三轮，每个目标保持3–15秒')
    stamp = datetime.datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f')
    if args.from_recording:
        source = args.from_recording.expanduser().resolve()
        if source.stat().st_size > 100*1024*1024:
            raise ValueError('标定原始记录超过100MB')
        recording = json.loads(source.read_text())
        if not isinstance(recording, dict):
            raise ValueError('原始记录必须为JSON对象')
        name = profile_name(args.profile or recording.get('profile_name'))
        recording['profile_name'] = name
    else:
        if not sys.stdin.isatty():
            raise ValueError('引导采集需要交互终端；离线数据使用 --from-recording')
        contract = input_contract(args.backend, args.left_module_map, args.right_module_map,
                                  args.left_id, args.right_id)
        if args.resume_recording:
            source = args.resume_recording.expanduser()
            if source.is_symlink() or not source.is_file() or source.stat().st_size > 100*1024*1024:
                raise ValueError('断点记录必须为不超过100MB的普通文件，不能是符号链接')
            recording = validate_partial_recording(json.loads(source.read_text()), contract, args.profile)
            name = profile_name(recording['profile_name'])
            print('已验证断点：%s，完成%d/%d段，将从下一动作继续；原记录保留。' %
                  (source, len(recording['segments']), len(STAGES)*3), flush=True)
        else:
            if args.profile is None:
                raise ValueError('首次标定请指定 --profile 操作者名字')
            name = profile_name(args.profile)
            recording = {'schema_version': 1, 'profile_name': name, 'input': contract, 'segments': []}
        raw_path = directory/'recordings'/(name+'_'+stamp+'.json')
        args._checkpoint = lambda document: write_json(raw_path, document)
        print('逐步保存到：' + str(raw_path), flush=True)
        try:
            collect(args, contract, recording)
        finally:
            if recording['segments']:
                write_json(raw_path, recording)
                print('原始记录已保存：' + str(raw_path), flush=True)
    report_path = directory/'reports'/(name+'_'+stamp+'.json')
    try:
        result = fit_profile(
            recording, suppress_fist_pinches=recording["input"]["backend"] == "windows_udp")
    except ValueError as exc:
        write_json(report_path, {'passed': False, 'profile_name': name, 'error': str(exc)})
        raise ValueError(str(exc) + '\n标定未生效，原active配置未改。报告：' + str(report_path)) from exc
    profile_path = directory/'profiles'/(name+'.yaml')
    # Save a validated named result before replacing the startup selection.
    atomic_save_profile(profile_path, result)
    write_json(report_path, {'passed': True, 'profile_name': name, 'quality': result['quality'],
                             'thresholds': result['thresholds'], 'activated': False})
    if not args.no_activate:
        atomic_save_profile(active, result)
        try:
            write_json(report_path, {'passed': True, 'profile_name': name, 'quality': result['quality'],
                                     'thresholds': result['thresholds'], 'activated': True})
        except (OSError, ValueError) as exc:
            # Activation already committed. A nonessential report update must
            # not turn it into a claimed calibration failure.
            print('配置已生效，但报告状态更新失败：' + str(exc), file=sys.stderr)
    print('标定通过：' + str(profile_path))
    print('报告：' + str(report_path))
    print('下次遥操将自动加载：' + str(active) if not args.no_activate else '未修改启动配置（--no-activate）')
    print('本次只写个人手势阈值，不修改手柄安装距离/姿态标定。')
    return 0


def main(argv=None):
    try:
        return run(parser().parse_args(argv))
    except KeyboardInterrupt:
        print('\n已取消；未完成的标定不会替换active配置。', file=sys.stderr)
        return 130
    except (ValueError, RuntimeError, OSError, KeyError) as exc:
        print('标定失败：' + str(exc), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
