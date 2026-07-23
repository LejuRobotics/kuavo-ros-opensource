#!/usr/bin/env python3
"""Python 侧只读 ArmTrajShmData（与 kuavo_common::ArmTrajShmManager 布局一致）。

绕开 /kuavo_arm_traj 的 TCPROS 订阅，直接监控 IK→WBC 共享内存。

监控策略（与 C++ Reader 并存）:
  - 只 shmat，不抢 Writer 的 memset
  - 不用 POSIX 信号量
  - 用 seq 双读一致性检查做无锁快照
"""

from __future__ import annotations

import ctypes
import threading
import time
from ctypes import c_bool, c_double, c_int, c_uint32, c_uint64, c_void_p

SHM_KEY = 343434
MAX_ARM_JOINTS = 14
IPC_CREAT = 0o1000


class ArmTrajShmData(ctypes.Structure):
    _fields_ = [
        ("seq", c_uint64),
        ("stamp_nsec", c_uint64),
        ("num_joints", c_uint32),
        ("valid", c_bool),
        ("_pad", ctypes.c_uint8 * 3),
        ("position", c_double * MAX_ARM_JOINTS),
        ("velocity", c_double * MAX_ARM_JOINTS),
        ("effort", c_double * MAX_ARM_JOINTS),
    ]


assert ctypes.sizeof(ArmTrajShmData) == 360
assert ArmTrajShmData.position.offset == 24


class ArmTrajShmReader:
    def __init__(self):
        self._ok = False
        self._shm_id = -1
        self._ptr = None
        self._last_seq = 0
        self._libc = ctypes.CDLL("libc.so.6", use_errno=True)
        self._libc.shmget.argtypes = [c_int, ctypes.c_size_t, c_int]
        self._libc.shmget.restype = c_int
        self._libc.shmat.argtypes = [c_int, c_void_p, c_int]
        self._libc.shmat.restype = c_void_p
        self._libc.shmdt.argtypes = [c_void_p]
        self._libc.shmdt.restype = c_int

    @property
    def ok(self):
        return self._ok

    def initialize(self):
        if self._ok:
            return True
        ctypes.set_errno(0)
        shmid = self._libc.shmget(SHM_KEY, ctypes.sizeof(ArmTrajShmData), IPC_CREAT | 0o666)
        if shmid < 0:
            return False
        self._shm_id = shmid

        ctypes.set_errno(0)
        ptr = self._libc.shmat(shmid, None, 0)
        if not ptr or ptr == c_void_p(-1).value:
            return False
        self._ptr = int(ptr)
        self._ok = True
        try:
            ArmTrajShmData.from_address(self._ptr)
        except Exception:
            self.cleanup()
            return False
        return True

    def cleanup(self):
        self._ok = False
        if self._ptr:
            self._libc.shmdt(c_void_p(self._ptr))
            self._ptr = None
        self._shm_id = -1
        self._last_seq = 0

    def read_if_updated(self):
        """返回 dict 或 None。position/velocity/effort 为 rad。"""
        if not self._ok or not self._ptr:
            return None
        data = ArmTrajShmData.from_address(self._ptr)
        seq1 = int(data.seq)
        if (not bool(data.valid)) or seq1 == 0 or seq1 == self._last_seq:
            return None
        nj = int(data.num_joints)
        if nj <= 0 or nj > MAX_ARM_JOINTS:
            return None
        stamp = int(data.stamp_nsec)
        pos = [float(data.position[i]) for i in range(nj)]
        vel = [float(data.velocity[i]) for i in range(nj)]
        eff = [float(data.effort[i]) for i in range(nj)]
        seq2 = int(data.seq)
        if seq2 != seq1:
            return None
        self._last_seq = seq1
        return {
            "seq": seq1,
            "stamp_nsec": stamp,
            "num_joints": nj,
            "position": pos,
            "velocity": vel,
            "effort": eff,
        }


class ArmTrajShmPoller:
    """后台线程轮询 SHM，回调 on_sample(dict)。"""

    def __init__(self, on_sample, poll_sleep_s=0.0002):
        self._reader = ArmTrajShmReader()
        self._on_sample = on_sample
        self._poll_sleep_s = float(poll_sleep_s)
        self._stop = threading.Event()
        self._thread = None
        self.stats = {
            "init_ok": False,
            "updates": 0,
            "first_seq": None,
            "last_seq": None,
            "last_stamp_nsec": None,
            "max_inter_update_ms": 0.0,
        }
        self._last_wall = None

    def start(self):
        ok = self._reader.initialize()
        self.stats["init_ok"] = bool(ok)
        if not ok:
            return False
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="arm_traj_shm_poll", daemon=True)
        self._thread.start()
        return True

    def stop(self):
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._thread = None
        self._reader.cleanup()

    def _loop(self):
        while not self._stop.is_set():
            sample = self._reader.read_if_updated()
            if sample is None:
                time.sleep(self._poll_sleep_s)
                continue
            now = time.time()
            if self._last_wall is not None:
                dt_ms = (now - self._last_wall) * 1000.0
                if dt_ms > self.stats["max_inter_update_ms"]:
                    self.stats["max_inter_update_ms"] = dt_ms
            self._last_wall = now
            self.stats["updates"] += 1
            if self.stats["first_seq"] is None:
                self.stats["first_seq"] = sample["seq"]
            self.stats["last_seq"] = sample["seq"]
            self.stats["last_stamp_nsec"] = sample["stamp_nsec"]
            try:
                self._on_sample(sample)
            except Exception:
                pass
