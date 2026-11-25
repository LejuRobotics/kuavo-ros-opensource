#!/usr/bin/env python3
"""按 USB product=LJ485A/LJ485B 安装同名 udev 别名，不打开串口。"""
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

NAMES = ('LJ485A', 'LJ485B')
RULE = Path('/etc/udev/rules.d/99-lj485-ports.rules')
TTY_ROOT = Path('/sys/class/tty')
RULE_DIRS = ('/etc/udev/rules.d', '/run/udev/rules.d',
             '/usr/lib/udev/rules.d', '/lib/udev/rules.d')


def find_ports():
    ports = {}
    for tty in sorted(TTY_ROOT.iterdir()):
        if not re.fullmatch(r'tty(?:USB|ACM)\d+', tty.name):
            continue
        device = (tty / 'device').resolve()
        for parent in (device, *device.parents):
            product = parent / 'product'
            try:
                name = product.read_text().strip()
            except FileNotFoundError:
                continue
            if name in NAMES:
                if name in ports:
                    raise RuntimeError(f'{name} 同时匹配 {ports[name]} 和 {tty.name}，拒绝分配。')
                ports[name] = tty.name
            break
    return ports


def check_conflicts():
    for directory in RULE_DIRS:
        for path in Path(directory).glob('*.rules'):
            if path.name == RULE.name:
                continue
            for line in path.read_text(errors='replace').splitlines():
                line = line.split('#', 1)[0]
                for value in re.findall(r'SYMLINK\s*(?:\+=|:=|=)\s*"([^"]*)"', line):
                    if set(value.split()) & set(NAMES):
                        raise RuntimeError(f'已有规则分配 LJ485 别名，请先核对：{path}')


def run(*args):
    subprocess.run(args, check=True)


def main():
    if not sys.platform.startswith('linux') or os.geteuid() != 0:
        raise RuntimeError('请在目标 Linux 机器上使用 sudo python3 运行。')
    if not shutil.which('udevadm'):
        raise RuntimeError('未找到 udevadm。')
    check_conflicts()
    ports = find_ports()
    RULE.parent.mkdir(parents=True, exist_ok=True)
    if RULE.is_symlink():
        raise RuntimeError('目标规则文件是软链接，拒绝覆盖。')
    content = ''.join(
        f'SUBSYSTEM=="tty", KERNEL=="ttyUSB*|ttyACM*", ATTRS{{product}}=="{name}", '
        f'GROUP="dialout", MODE="0660", SYMLINK+="{name}"\n' for name in NAMES)
    fd, temp = tempfile.mkstemp(prefix='.lj485-', dir=str(RULE.parent))
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            stream.write(content)
        os.chmod(temp, 0o644)
        os.replace(temp, RULE)
    finally:
        if os.path.exists(temp):
            os.unlink(temp)
    run('udevadm', 'control', '--reload-rules')
    for tty in sorted(ports.values()):
        run('udevadm', 'trigger', '--action=add', '--subsystem-match=tty',
            '--sysname-match=' + tty)
    run('udevadm', 'settle', '--timeout=10')
    for name in NAMES:
        if name not in ports:
            print(f'{name}：当前未检测到此产品名，规则已安装，之后插入会自动匹配。')
            continue
        alias = Path('/dev') / name
        target = Path('/dev') / ports[name]
        if not alias.is_symlink() or alias.resolve() != target.resolve():
            raise RuntimeError(f'{alias} 映射验证失败；规则已写入，请检查 udev 日志。')
        print(f'{alias} -> {target}')
    print(f'规则已安装：{RULE}')


if __name__ == '__main__':
    try:
        main()
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f'错误：{error}', file=sys.stderr)
        sys.exit(1)
