#!/usr/bin/env python3
"""SG100 自动 udev 配置：读取掌板产品 SN（0x28），按 HM-SG-L/R 识别左右。"""
import argparse
import datetime
import glob
import os
import re
import select
import shutil
import struct
import subprocess
import sys
import tempfile
import time
from contextlib import ExitStack, contextmanager
from pathlib import Path

RULE = Path('/etc/udev/rules.d/99-sg100-hands.rules')
ALIASES = ('lefthand', 'righthand')


def crc32_bzip2(data):
    crc = 0xffffffff
    for byte in data:
        crc ^= byte << 24
        for _ in range(8):
            crc = ((crc << 1) ^ (0x04c11db7 if crc & 0x80000000 else 0)) & 0xffffffff
    return crc ^ 0xffffffff


def packet(command):
    body = bytes((0x48, 0x4d, 0, 0xfd, 0xfe, command))
    return body + struct.pack('<I', crc32_bzip2(body))


def frames(buffer):
    result = []
    while len(buffer) >= 2:
        start = buffer.find(b'HM')
        if start < 0:
            del buffer[:-1]
            break
        del buffer[:start]
        if len(buffer) < 6:
            break
        size = 10 + buffer[2]
        if len(buffer) < size:
            break
        body = bytes(buffer[:size-4])
        if crc32_bzip2(body) != struct.unpack('<I', buffer[size-4:size])[0]:
            del buffer[0]
            continue
        del buffer[:size]
        result.append((body[3], body[4], body[5], body[6:]))
    return result


@contextmanager
def serial_port(port):
    import termios
    import fcntl
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    previous = None
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        previous = termios.tcgetattr(fd)
        settings = termios.tcgetattr(fd)
        settings[0] = settings[1] = settings[3] = 0
        settings[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        settings[4] = settings[5] = termios.B1000000
        settings[6][termios.VMIN] = 0
        settings[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, settings)
        yield fd
    finally:
        try:
            if previous is not None:
                termios.tcsetattr(fd, termios.TCSANOW, previous)
            fcntl.ioctl(fd, termios.TIOCNXCL)
        finally:
            os.close(fd)


def request(fd, command, expected_size=None, timeout=1.0):
    import termios
    for _ in range(2):
        termios.tcflush(fd, termios.TCIFLUSH)
        outgoing = packet(command)
        if os.write(fd, outgoing) != len(outgoing):
            raise RuntimeError('串口请求写入不完整。')
        buffer = bytearray()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            ready, _, _ = select.select([fd], [], [], max(0, deadline - time.monotonic()))
            if not ready:
                break
            try:
                data = os.read(fd, 4096)
            except BlockingIOError:
                continue
            if not data:
                raise RuntimeError('串口已断开。')
            buffer.extend(data)
            for dst, src, cmd, payload in frames(buffer):
                if dst != 0xfe or src != 0xfd or cmd != (command | 0x80):
                    continue
                if expected_size is not None and len(payload) != expected_size:
                    raise RuntimeError('掌板响应 0x{:02x} 长度 {}，预期 {}。'.format(cmd, len(payload), expected_size))
                return payload
    raise RuntimeError('掌板查询 0x{:02x} 超时；未识别左右。'.format(command))


def decode_identity(payload):
    # CMD_SN_READ (0x28): status(1), SN length(1), ASCII bytes (no NUL terminator).
    if not payload or payload[0] != 0:
        raise RuntimeError('产品 SN 读取失败，状态：' + (payload[:1].hex() or '缺失'))
    if len(payload) < 2 or len(payload) != payload[1] + 2:
        raise RuntimeError('产品 SN 响应长度不一致。')
    if payload[1] == 0:
        raise RuntimeError('掌板尚未写入产品 SN。')
    try:
        sn = payload[2:].decode('ascii')
    except UnicodeDecodeError:
        raise RuntimeError('产品 SN 不是 ASCII 编号，原始值：' + payload[2:].hex())
    match = re.fullmatch(r'HM-SG-([LR])-(?:[A-Z0-9]-){3}[0-9]{2}(?:0[1-9]|1[0-2])-[0-9]{5}', sn)
    if not match:
        raise RuntimeError('产品 SN 格式无效：' + repr(sn))
    return {'side': 'left' if match[1] == 'L' else 'right', 'sn': sn}


def probe(port):
    import termios
    with ExitStack() as stack:
        try:
            fd = stack.enter_context(serial_port(port))
            original_mode = request(fd, 0x27, 6)
        except (RuntimeError, OSError, termios.error) as error:
            print(f'跳过无法探测的串口：{port}（{error}）')
            return None  # 未确认是 SG100，不改变该设备模式。
        if original_mode[0] not in (0, 1) or original_mode[2] or original_mode[3]:
            raise RuntimeError('掌板处于串口互联/Fake PC 等特殊模式，拒绝干扰：' + port)
        needs_resume = original_mode[0] == 0
        try:
            if needs_resume:
                try:
                    request(fd, 0x24, 1)
                except RuntimeError:
                    pass  # 文档允许暂停应答丢失时通过 0x27 确认模式。
                paused = request(fd, 0x27, 6)
                if paused[0] != 1 or paused[1]:
                    raise RuntimeError('未确认进入暂停模式。')
            first = decode_identity(request(fd, 0x28, timeout=2.0))
            second = decode_identity(request(fd, 0x28, timeout=2.0))
            if first != second:
                raise RuntimeError('两次读取的产品 SN 不一致，拒绝配置。')
            return first
        finally:
            if needs_resume:
                ack = request(fd, 0x25, 1)
                restored = request(fd, 0x27, 6)
                if ack != b'\0' or restored != original_mode:
                    raise RuntimeError('原掌板模式恢复失败，请先检查硬件状态，不要启动整机。')


def port_is_idle(port):
    if not shutil.which('fuser'):
        raise RuntimeError('需要 fuser 检查串口占用（Ubuntu: sudo apt install psmisc）。')
    result = subprocess.run(['fuser', port], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if result.returncode == 1 and not result.stdout.strip():
        return True
    if result.returncode == 0 and result.stdout.strip():
        return False
    raise RuntimeError('串口占用检查失败：' + port + ' ' + result.stderr)


def ensure_idle(ports):
    for port in ports:
        if not port_is_idle(port):
            raise RuntimeError('手串口已被占用，请先停止整机和手驱动：' + port)


def run(*args):
    return subprocess.run(args, check=True, text=True, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE).stdout


def properties(port):
    result = {}
    for line in run('udevadm', 'info', '-q', 'property', '-n', port).splitlines():
        if '=' in line:
            key, value = line.split('=', 1)
            result[key] = value
    return result


def snapshot():
    result = {}
    for port in sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')):
        try:
            result[port] = properties(port)
        except subprocess.CalledProcessError:
            pass
    return result


def stable_identity(port, devices):
    info = devices[port]
    for key in ('ID_SERIAL', 'ID_PATH'):
        if key == 'ID_SERIAL' and not info.get('ID_SERIAL_SHORT'):
            # ID_SERIAL without a hardware serial can be only a product label.
            continue
        value = info.get(key)
        if (value and sum(p.get(key) == value for p in devices.values()) == 1
                and not re.search(r'["\\\n\r*?\[\]]', value)):
            return key, value
    raise RuntimeError(port + ' 没有可唯一匹配的 ID_SERIAL 或 ID_PATH。')


def candidate_ports(devices):
    ports = sorted(p for p in devices if re.fullmatch(r'/dev/tty(?:USB|ACM)\d+', p))
    if len(ports) < 2:
        raise RuntimeError('未找到至少两个 USB 串口（ttyUSB/ttyACM）。')
    return ports


def conflicting_rules():
    conflicts = []
    for directory in ('/etc/udev/rules.d', '/run/udev/rules.d',
                      '/usr/lib/udev/rules.d', '/lib/udev/rules.d'):
        for path in Path(directory).glob('*.rules'):
            if path == RULE:
                continue
            for number, line in enumerate(path.read_text(errors='replace').splitlines(), 1):
                line = line.split('#', 1)[0]
                if 'SYMLINK' in line and any(a in line for a in ALIASES):
                    entry = '{}:{}: {}'.format(path, number, line.strip())
                    if entry not in conflicts:
                        conflicts.append(entry)
    return conflicts


def install_rules(content, ports):
    if RULE.is_symlink():
        raise RuntimeError('规则文件不能是符号链接。')
    RULE.parent.mkdir(parents=True, exist_ok=True)
    if RULE.exists():
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        shutil.copy2(RULE, RULE.with_name(RULE.name + '.bak.' + stamp))
    fd, temp = tempfile.mkstemp(prefix='.sg100-', dir=str(RULE.parent))
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            stream.write(content)
        os.chmod(temp, 0o644)
        os.replace(temp, RULE)
    finally:
        if os.path.exists(temp):
            os.unlink(temp)
    run('udevadm', 'control', '--reload-rules')
    for port in ports:
        run('udevadm', 'trigger', '--action=add', '--subsystem-match=tty',
            '--sysname-match=' + Path(port).name)
    run('udevadm', 'settle', '--timeout=10')
    for alias, port in zip(ALIASES, ports):
        link = '/dev/' + alias
        if not os.path.islink(link) or os.path.realpath(link) != port:
            raise RuntimeError('映射验证失败：' + link)
    print('配置完成。')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe-only', action='store_true', help='只读取左右手身份')
    parser.add_argument('--dry-run', action='store_true', help='预览规则，不写入')
    args = parser.parse_args()
    if not sys.platform.startswith('linux'):
        raise RuntimeError('请在机器人 Ubuntu 上运行。')
    if os.geteuid() != 0:
        raise RuntimeError('请使用 sudo 运行。')
    print('请先停止整机和手驱动；读取掌板产品 SN（0x28）区分 L/R，结束后恢复原模式。')
    devices = snapshot()
    candidates = candidate_ports(devices)
    readings = []
    for port in candidates:
        if not port_is_idle(port):
            print('跳过已占用串口：' + port)
            continue
        info = probe(port)
        if info is not None:
            readings.append((port, info))
            print(f"{port}: product_sn={info['sn']} ({info['side']})")
    if len(readings) != 2 or {info['side'] for _, info in readings} != {'left', 'right'}:
        raise RuntimeError('必须恰好识别到 L/R 各一只手，未修改规则。')
    if args.probe_only:
        return
    readings.sort(key=lambda item: item[1]['side'])
    ports = [port for port, _ in readings]
    identities = [stable_identity(port, devices) for port in ports]
    lines = ['# SG100 product SN (CMD 0x28): HM-SG-L = left, HM-SG-R = right']
    for alias, port, (key, value) in zip(ALIASES, ports, identities):
        lines.append(f'SUBSYSTEM=="tty", ENV{{{key}}}=="{value}", '
                     f'GROUP="dialout", MODE="0660", SYMLINK+="{alias}"')
        print(f'/dev/{alias} -> {port}')
    content = '\n'.join(lines) + '\n'
    if args.dry_run:
        print(content, end='')
        return
    conflicts = conflicting_rules()
    if conflicts:
        raise RuntimeError('其他文件存在同名映射，请先处理：\n' + '\n'.join(conflicts))
    current = snapshot()
    for port, identity in zip(ports, identities):
        key, value = identity
        if [p for p, info in current.items() if info.get(key) == value] != [port]:
            raise RuntimeError('识别后设备发生变化，请重新运行。')
    ensure_idle(ports)
    install_rules(content, ports)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, subprocess.CalledProcessError) as error:
        print('错误：' + str(error), file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        sys.exit(130)
