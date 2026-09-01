#!/usr/bin/env python3
"""PICO UDP scheme-B diagnostics runtime helpers."""

from __future__ import annotations

import json
import gzip
import math
import os
import queue
import re
import shutil
import socket
import statistics
import struct
import subprocess
import threading
import time
from collections import Counter, defaultdict, deque
from dataclasses import dataclass
from typing import Callable, Deque, Dict, Iterable, Optional, Tuple

from . import pico_diagnostics_pb2 as diag_pb


PROTOCOL_VERSION = 1
HEADER_MAGIC = b"PDT1"
HEADER_SIZE = 80
HEADER_STRUCT = struct.Struct("!4sHHHHI16sQQQQIIQ")

MSG_UNSPECIFIED = 0
MSG_VRDATA = 1
MSG_CLOCK_SYNC_REQUEST = 2
MSG_CLOCK_SYNC_RESPONSE = 3
MSG_DIAGNOSTIC_RESULT = 4
MSG_PICO_SENDER_SUMMARY = 5

STREAM_UNSPECIFIED = 0
STREAM_SKELETON = 1
STREAM_CONTROLLER = 2
STREAM_OTHER = 3

STREAM_NAMES = {
    STREAM_UNSPECIFIED: "UNSPECIFIED",
    STREAM_SKELETON: "SKELETON",
    STREAM_CONTROLLER: "CONTROLLER",
    STREAM_OTHER: "OTHER",
}

MSG_NAMES = {
    MSG_UNSPECIFIED: "UNSPECIFIED",
    MSG_VRDATA: "VRDATA",
    MSG_CLOCK_SYNC_REQUEST: "CLOCK_SYNC_REQUEST",
    MSG_CLOCK_SYNC_RESPONSE: "CLOCK_SYNC_RESPONSE",
    MSG_DIAGNOSTIC_RESULT: "DIAGNOSTIC_RESULT",
    MSG_PICO_SENDER_SUMMARY: "PICO_SENDER_SUMMARY",
}


@dataclass
class FixedDiagnosticTransportHeader:
    protocol_version: int = PROTOCOL_VERSION
    message_type: int = MSG_UNSPECIFIED
    stream: int = STREAM_UNSPECIFIED
    flags: int = 0
    session_uuid: bytes = b"\x00" * 16
    packet_sequence: int = 0
    stream_sequence: int = 0
    capture_monotonic_ns: int = 0
    send_monotonic_ns: int = 0
    clock_model_version: int = 0
    payload_size: int = 0
    reserved: int = 0

    def pack(self) -> bytes:
        session_uuid = self.session_uuid
        if len(session_uuid) != 16:
            raise ValueError("session_uuid must be exactly 16 bytes")
        return HEADER_STRUCT.pack(
            HEADER_MAGIC,
            HEADER_SIZE,
            int(self.protocol_version),
            int(self.message_type),
            int(self.stream),
            int(self.flags),
            session_uuid,
            int(self.packet_sequence),
            int(self.stream_sequence),
            int(self.capture_monotonic_ns),
            int(self.send_monotonic_ns),
            int(self.clock_model_version),
            int(self.payload_size),
            int(self.reserved),
        )

    @classmethod
    def parse(cls, data: bytes) -> "FixedDiagnosticTransportHeader":
        if len(data) < HEADER_SIZE:
            raise ValueError("datagram shorter than fixed header")
        unpacked = HEADER_STRUCT.unpack(data[:HEADER_SIZE])
        magic = unpacked[0]
        if magic != HEADER_MAGIC:
            raise ValueError("unknown protocol magic")
        header_size = unpacked[1]
        if header_size != HEADER_SIZE:
            raise ValueError("unsupported header_size %s" % header_size)
        return cls(
            protocol_version=unpacked[2],
            message_type=unpacked[3],
            stream=unpacked[4],
            flags=unpacked[5],
            session_uuid=unpacked[6],
            packet_sequence=unpacked[7],
            stream_sequence=unpacked[8],
            capture_monotonic_ns=unpacked[9],
            send_monotonic_ns=unpacked[10],
            clock_model_version=unpacked[11],
            payload_size=unpacked[12],
            reserved=unpacked[13],
        )


@dataclass
class ReceivedDatagram:
    raw_data: bytes
    payload: bytes
    addr: Tuple[str, int]
    recv_monotonic_ns: int
    header: Optional[FixedDiagnosticTransportHeader] = None
    legacy: bool = False
    enqueue_monotonic_ns: int = 0
    process_start_monotonic_ns: int = 0
    protobuf_parse_done_monotonic_ns: int = 0
    process_done_monotonic_ns: int = 0

    @property
    def payload_len(self) -> int:
        return len(self.payload)


def monotonic_ns() -> int:
    return time.monotonic_ns()


class JsonlDiagnosticLogger:
    def __init__(
        self,
        enabled: bool,
        log_dir: str,
        flush_interval_sec: float = 1.0,
        max_queue_size: int = 4096,
        max_file_size_mb: float = 300.0,
        compress_closed_files: bool = True,
    ):
        self.enabled = enabled
        self.log_dir = os.path.expanduser(log_dir or "~/.ros/pico_diagnostics")
        self.flush_interval_sec = flush_interval_sec
        self.max_file_size_bytes = int(max_file_size_mb * 1024 * 1024) if max_file_size_mb > 0 else 0
        self.compress_closed_files = compress_closed_files
        self.queue: "queue.Queue[dict]" = queue.Queue(maxsize=max_queue_size)
        self.dropped = 0
        self._stop = threading.Event()
        self._thread = None
        self._file = None
        self._base_name = ""
        self._part_index = 0
        self._compression_threads = []

        if self.enabled:
            os.makedirs(self.log_dir, exist_ok=True)
            self._base_name = time.strftime("pico_diag_%Y%m%d_%H%M%S") + "_pid%s" % os.getpid()
            self._open_next_file()
            self._thread = threading.Thread(target=self._run, daemon=True, name="pico-diag-log")
            self._thread.start()
        else:
            self.path = ""

    def _path_for_part(self, part_index: int) -> str:
        if part_index <= 0:
            name = "%s.jsonl" % self._base_name
        else:
            name = "%s_part%03d.jsonl" % (self._base_name, part_index)
        return os.path.join(self.log_dir, name)

    def _open_next_file(self):
        self.path = self._path_for_part(self._part_index)
        self._file = open(self.path, "a", buffering=1)
        self._part_index += 1

    def _compress_file(self, path: str):
        if not self.compress_closed_files or not path or not os.path.isfile(path):
            return
        if path.endswith(".gz") or os.path.getsize(path) == 0:
            return
        gz_path = path + ".gz"
        tmp_path = gz_path + ".tmp"
        try:
            with open(path, "rb") as src, gzip.open(tmp_path, "wb", compresslevel=3) as dst:
                shutil.copyfileobj(src, dst, length=1024 * 1024)
            os.replace(tmp_path, gz_path)
            os.remove(path)
        except Exception:
            try:
                if os.path.exists(tmp_path):
                    os.remove(tmp_path)
            except Exception:
                pass

    def _compress_file_async(self, path: str):
        if not self.compress_closed_files:
            return
        thread = threading.Thread(
            target=self._compress_file,
            args=(path,),
            daemon=True,
            name="pico-diag-gzip",
        )
        self._compression_threads.append(thread)
        thread.start()

    def _rotate_if_needed(self):
        if not self._file or self.max_file_size_bytes <= 0:
            return
        if self._file.tell() < self.max_file_size_bytes:
            return
        closed_path = self.path
        self._file.flush()
        self._file.close()
        self._open_next_file()
        self._compress_file_async(closed_path)

    def log(self, event: str, **fields):
        if not self.enabled:
            return
        record = {"event": event, "wall_time": time.time()}
        record.update(fields)
        try:
            self.queue.put_nowait(record)
        except queue.Full:
            self.dropped += 1

    def _run(self):
        last_flush = time.monotonic()
        while not self._stop.is_set() or not self.queue.empty():
            try:
                record = self.queue.get(timeout=0.2)
            except queue.Empty:
                record = None
            if record is not None and self._file:
                self._file.write(json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n")
                self._rotate_if_needed()
            if self._file and time.monotonic() - last_flush >= self.flush_interval_sec:
                self._file.flush()
                last_flush = time.monotonic()

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join()
        if self._file:
            final_path = self.path
            self._file.flush()
            self._file.close()
            self._file = None
            if os.path.isfile(final_path) and os.path.getsize(final_path) == 0 and self._part_index > 1:
                os.remove(final_path)
            else:
                self._compress_file(final_path)
        for thread in self._compression_threads:
            thread.join()


class PingMonitor:
    def __init__(
        self,
        logger: JsonlDiagnosticLogger,
        enabled: bool,
        get_vr_ip: Callable[[], str],
        lower_ip: str = "",
        interval_sec: float = 5.0,
        timeout_sec: float = 1.0,
    ):
        self.logger = logger
        self.enabled = enabled
        self.get_vr_ip = get_vr_ip
        self.lower_ip = lower_ip.strip()
        self.interval_sec = max(0.5, float(interval_sec))
        self.timeout_sec = max(0.2, float(timeout_sec))
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        if not self.enabled:
            return
        self._thread = threading.Thread(target=self._run, daemon=True, name="pico-diag-ping")
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    def _run(self):
        while not self._stop.wait(self.interval_sec):
            targets = []
            vr_ip = self.get_vr_ip()
            if vr_ip:
                targets.append(("vr", vr_ip))
            if self.lower_ip:
                targets.append(("lower", self.lower_ip))
            seen = set()
            for target_name, target_ip in targets:
                if target_ip in seen:
                    continue
                seen.add(target_ip)
                self._ping_once(target_name, target_ip)

    def _ping_once(self, target_name: str, target_ip: str):
        started_ns = monotonic_ns()
        cmd = [
            "ping",
            "-n",
            "-c",
            "1",
            "-W",
            str(max(1, int(math.ceil(self.timeout_sec)))),
            target_ip,
        ]
        try:
            completed = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=self.timeout_sec + 1.0,
                check=False,
            )
            output = (completed.stdout or "") + "\n" + (completed.stderr or "")
            match = re.search(r"time[=<]([0-9.]+)\s*ms", output)
            reachable = completed.returncode == 0 and match is not None
            rtt_ms = float(match.group(1)) if match else None
            self.logger.log(
                "ping_sample",
                target_name=target_name,
                target_ip=target_ip,
                reachable=reachable,
                rtt_ms=rtt_ms,
                command_returncode=completed.returncode,
                duration_ns=monotonic_ns() - started_ns,
            )
        except Exception as exc:
            self.logger.log(
                "ping_sample",
                target_name=target_name,
                target_ip=target_ip,
                reachable=False,
                rtt_ms=None,
                error=str(exc),
                duration_ns=monotonic_ns() - started_ns,
            )


class StreamSequenceStats:
    def __init__(self):
        self.received_unique = 0
        self.lost = 0
        self.out_of_order = 0
        self.duplicates = 0
        self.last_sequence = 0
        self.seen: Deque[int] = deque(maxlen=512)
        self.seen_set = set()

    def observe(self, sequence: int):
        if sequence <= 0:
            self.received_unique += 1
            return

        if sequence in self.seen_set:
            self.duplicates += 1
            return

        self.received_unique += 1
        self.seen.append(sequence)
        self.seen_set.add(sequence)
        if len(self.seen) == self.seen.maxlen:
            self.seen_set = set(self.seen)

        if self.last_sequence == 0:
            self.last_sequence = sequence
            return

        if sequence == self.last_sequence + 1:
            self.last_sequence = sequence
        elif sequence > self.last_sequence + 1:
            self.lost += sequence - self.last_sequence - 1
            self.last_sequence = sequence
        else:
            self.out_of_order += 1


class ClockSyncModel:
    def __init__(
        self,
        min_valid_samples: int = 20,
        max_valid_rtt_ns: int = 20_000_000,
        max_valid_residual_ns: int = 5_000_000,
    ):
        self.min_valid_samples = min_valid_samples
        self.max_valid_rtt_ns = max_valid_rtt_ns
        self.max_valid_residual_ns = max_valid_residual_ns
        self.samples: Deque[dict] = deque(maxlen=240)
        self.version = 0
        self.valid = False
        self.slope = 1.0
        self.offset_ns = 0
        self.last_sync_rtt_ns = 0
        self.last_residual_ns = 0
        self.residual_p95_ns = 0
        self.invalid_reason = "no_samples"

    def add_sample(self, t1: int, t2: int, t3: int, t4: int) -> dict:
        rtt_ns = max(0, (t4 - t1) - (t3 - t2))
        self.last_sync_rtt_ns = int(rtt_ns)
        midpoint_ubuntu = (t1 + t4) / 2.0
        midpoint_pico = (t2 + t3) / 2.0
        offset_ns = int(round(midpoint_ubuntu - midpoint_pico))
        sample = {
            "t1": t1,
            "t2": t2,
            "t3": t3,
            "t4": t4,
            "rtt_ns": rtt_ns,
            "offset_ns": offset_ns,
            "valid_input": rtt_ns <= self.max_valid_rtt_ns,
            "min_valid_samples": self.min_valid_samples,
            "max_valid_rtt_ns": self.max_valid_rtt_ns,
            "max_valid_residual_ns": self.max_valid_residual_ns,
        }
        self.samples.append(sample)
        self._refit()
        sample["model_version"] = self.version
        sample["model_valid"] = self.valid
        sample["residual_ns"] = self.last_residual_ns
        sample["invalid_reason"] = self.invalid_reason
        return sample

    def _refit(self):
        candidates = [s for s in self.samples if s["valid_input"]]
        if len(candidates) < self.min_valid_samples:
            self.valid = False
            self.invalid_reason = "not_enough_valid_samples"
            if candidates:
                self.offset_ns = int(statistics.median(s["offset_ns"] for s in candidates))
            return

        candidates = sorted(candidates, key=lambda s: s["rtt_ns"])
        keep_count = max(self.min_valid_samples, int(math.ceil(len(candidates) * 0.6)))
        candidates = candidates[:keep_count]

        xs = [float((s["t2"] + s["t3"]) / 2.0) for s in candidates]
        ys = [float((s["t1"] + s["t4"]) / 2.0) for s in candidates]
        x_mean = sum(xs) / len(xs)
        y_mean = sum(ys) / len(ys)
        denom = sum((x - x_mean) ** 2 for x in xs)
        if denom <= 0.0:
            slope = 1.0
        else:
            slope = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)) / denom
        intercept = y_mean - slope * x_mean
        residuals = [int(round(y - (slope * x + intercept))) for x, y in zip(xs, ys)]
        abs_residuals = sorted(abs(r) for r in residuals)
        p95_index = min(len(abs_residuals) - 1, int(math.ceil(len(abs_residuals) * 0.95)) - 1)
        residual_p95_ns = abs_residuals[p95_index]

        self.slope = slope
        self.offset_ns = int(round(intercept))
        self.last_sync_rtt_ns = int(candidates[-1]["rtt_ns"])
        self.last_residual_ns = int(residuals[-1])
        self.residual_p95_ns = int(residual_p95_ns)

        valid = residual_p95_ns <= self.max_valid_residual_ns
        if valid and not self.valid:
            self.version += 1
        elif valid and self.valid:
            self.version += 1
        self.valid = valid
        self.invalid_reason = "" if valid else "residual_too_large"

    def map_pico_to_ubuntu(self, pico_monotonic_ns: int) -> Optional[int]:
        if not self.valid or pico_monotonic_ns <= 0:
            return None
        return int(round(self.slope * float(pico_monotonic_ns) + float(self.offset_ns)))

    def status_pb(self) -> diag_pb.ClockModelStatus:
        status = diag_pb.ClockModelStatus()
        status.clock_model_version = int(self.version)
        status.valid = bool(self.valid)
        status.slope = float(self.slope)
        status.offset_ns = int(self.offset_ns)
        status.sample_count = len(self.samples)
        status.last_sync_rtt_ns = int(self.last_sync_rtt_ns)
        status.last_residual_ns = int(self.last_residual_ns)
        status.residual_p95_ns = int(self.residual_p95_ns)
        return status


class PicoDiagnosticsRuntime:
    def __init__(
        self,
        sock: socket.socket,
        logger: JsonlDiagnosticLogger,
        enabled: bool = True,
        udp_reply_enabled: bool = True,
        sync_enabled: bool = True,
        diagnostic_publish_hz: float = 1.0,
        sync_fast_interval_sec: float = 0.05,
        sync_valid_interval_sec: float = 1.0,
        allow_legacy_vrdata: bool = False,
        ping_enabled: bool = True,
        ping_lower_ip: str = "",
        ping_interval_sec: float = 5.0,
        ping_timeout_sec: float = 1.0,
        sync_min_valid_samples: int = 20,
        sync_max_valid_rtt_ms: float = 20.0,
        sync_max_valid_residual_ms: float = 5.0,
    ):
        self.sock = sock
        self.logger = logger
        self.enabled = enabled
        self.udp_reply_enabled = udp_reply_enabled
        self.sync_enabled = sync_enabled
        self.diagnostic_publish_hz = diagnostic_publish_hz
        self.sync_fast_interval_sec = sync_fast_interval_sec
        self.sync_valid_interval_sec = sync_valid_interval_sec
        self.allow_legacy_vrdata = allow_legacy_vrdata

        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread = None
        self.peer_addr: Optional[Tuple[str, int]] = None
        self.session_uuid = b"\x00" * 16
        self.ubuntu_packet_sequence = 0
        self.sync_sequence = 0
        self.last_sync_sent_at = 0.0
        self.last_diag_sent_at = 0.0
        self.sync_min_valid_samples = max(1, int(sync_min_valid_samples))
        self.sync_max_valid_rtt_ns = max(1, int(float(sync_max_valid_rtt_ms) * 1_000_000.0))
        self.sync_max_valid_residual_ns = max(1, int(float(sync_max_valid_residual_ms) * 1_000_000.0))
        self.clock = self._new_clock_model()
        self.global_stats = StreamSequenceStats()
        self.stream_stats: Dict[int, StreamSequenceStats] = defaultdict(StreamSequenceStats)
        self.latency_windows: Dict[int, Deque[Tuple[float, int]]] = defaultdict(lambda: deque(maxlen=2000))
        self.field_combinations = Counter()
        self.queue_dropped_packets = 0
        self.parse_failed_packets = 0
        self.unauthorized_dropped_packets = 0
        self.unknown_protocol_packets = 0
        self.pico_summary = None
        self.last_business_header: Optional[FixedDiagnosticTransportHeader] = None
        self.last_business_headers: Dict[int, FixedDiagnosticTransportHeader] = {}
        self.ping_monitor = PingMonitor(
            logger=self.logger,
            enabled=self.enabled and ping_enabled,
            get_vr_ip=self._current_peer_ip,
            lower_ip=ping_lower_ip,
            interval_sec=ping_interval_sec,
            timeout_sec=ping_timeout_sec,
        )

    def _new_clock_model(self) -> ClockSyncModel:
        return ClockSyncModel(
            min_valid_samples=self.sync_min_valid_samples,
            max_valid_rtt_ns=self.sync_max_valid_rtt_ns,
            max_valid_residual_ns=self.sync_max_valid_residual_ns,
        )

    def start(self):
        if not self.enabled:
            return
        self._thread = threading.Thread(target=self._run, daemon=True, name="pico-diag-runtime")
        self._thread.start()
        self.ping_monitor.start()

    def stop(self):
        self._stop.set()
        self.ping_monitor.stop()
        if self._thread:
            self._thread.join(timeout=1.0)

    def _current_peer_ip(self) -> str:
        with self._lock:
            return self.peer_addr[0] if self.peer_addr else ""

    def set_peer(self, addr: Tuple[str, int], session_uuid: Optional[bytes] = None):
        with self._lock:
            self.peer_addr = addr
            if session_uuid and len(session_uuid) == 16 and session_uuid != self.session_uuid:
                self.session_uuid = session_uuid
                self.clock = self._new_clock_model()
                self.logger.log("session_changed", session_uuid=session_uuid.hex(), peer=list(addr))

    def record_unauthorized_drop(self, addr: Tuple[str, int], payload_len: int):
        with self._lock:
            self.unauthorized_dropped_packets += 1
        self.logger.log("unauthorized_drop", source_ip=addr[0], source_port=addr[1], payload_len=payload_len)

    def record_queue_drop(self, datagram: Optional[ReceivedDatagram]):
        with self._lock:
            self.queue_dropped_packets += 1
        self.logger.log(
            "queue_drop",
            packet_sequence=getattr(datagram.header, "packet_sequence", 0) if datagram and datagram.header else 0,
            stream=getattr(datagram.header, "stream", 0) if datagram and datagram.header else 0,
        )

    def decode_datagram(
        self,
        raw_data: bytes,
        addr: Tuple[str, int],
        recv_monotonic_ns: int,
    ) -> Optional[ReceivedDatagram]:
        if not self.enabled:
            return ReceivedDatagram(raw_data=raw_data, payload=raw_data, addr=addr, recv_monotonic_ns=recv_monotonic_ns, legacy=True)

        try:
            header = FixedDiagnosticTransportHeader.parse(raw_data)
            if len(raw_data) - HEADER_SIZE != header.payload_size:
                raise ValueError("payload_size mismatch")
            payload = raw_data[HEADER_SIZE:]
        except Exception as exc:
            if self.allow_legacy_vrdata:
                self.logger.log("legacy_vrdata", source_ip=addr[0], payload_len=len(raw_data), reason=str(exc))
                return ReceivedDatagram(raw_data=raw_data, payload=raw_data, addr=addr, recv_monotonic_ns=recv_monotonic_ns, legacy=True)
            with self._lock:
                self.unknown_protocol_packets += 1
            self.logger.log("unknown_protocol", source_ip=addr[0], source_port=addr[1], payload_len=len(raw_data), error=str(exc))
            return None

        self.set_peer(addr, header.session_uuid)
        with self._lock:
            self.global_stats.observe(header.packet_sequence)

        if header.message_type == MSG_CLOCK_SYNC_RESPONSE:
            self._handle_clock_sync_response(header, payload, addr, recv_monotonic_ns)
            return None
        if header.message_type == MSG_PICO_SENDER_SUMMARY:
            self._handle_pico_sender_summary(header, payload, addr, recv_monotonic_ns)
            return None
        if header.message_type != MSG_VRDATA:
            self.logger.log(
                "non_business_packet",
                message_type=header.message_type,
                message_type_name=MSG_NAMES.get(header.message_type, "UNKNOWN"),
                source_ip=addr[0],
                payload_len=len(payload),
            )
            return None

        with self._lock:
            self.stream_stats[header.stream].observe(header.stream_sequence)
            self.last_business_header = header
            self.last_business_headers[header.stream] = header
            mapped_send = self.clock.map_pico_to_ubuntu(header.send_monotonic_ns)
            if mapped_send is not None:
                latency_ns = max(0, recv_monotonic_ns - mapped_send)
                self.latency_windows[header.stream].append((time.monotonic(), latency_ns))

        return ReceivedDatagram(
            raw_data=raw_data,
            payload=payload,
            addr=addr,
            recv_monotonic_ns=recv_monotonic_ns,
            header=header,
            legacy=False,
        )

    def record_vrdata_fields(self, datagram: ReceivedDatagram, fields: Iterable[str], parse_ok: bool):
        fields_tuple = tuple(sorted(fields))
        with self._lock:
            self.field_combinations[fields_tuple] += 1
            if not parse_ok:
                self.parse_failed_packets += 1
                return
            header = datagram.header
            if header:
                mapped_send = self.clock.map_pico_to_ubuntu(header.send_monotonic_ns)
                latency_ns = None
                latency_valid = False
                if mapped_send is not None:
                    latency_ns = max(0, datagram.recv_monotonic_ns - mapped_send)
                    latency_valid = True
            else:
                latency_ns = None
                latency_valid = False

        self.logger.log(
            "vrdata_packet",
            recv_monotonic_ns=datagram.recv_monotonic_ns,
            source_ip=datagram.addr[0],
            source_port=datagram.addr[1],
            payload_len=datagram.payload_len,
            legacy=datagram.legacy,
            fields=list(fields_tuple),
            parse_ok=parse_ok,
            session_uuid=datagram.header.session_uuid.hex() if datagram.header else "",
            stream=datagram.header.stream if datagram.header else 0,
            stream_name=STREAM_NAMES.get(datagram.header.stream, "UNKNOWN") if datagram.header else "LEGACY",
            packet_sequence=datagram.header.packet_sequence if datagram.header else 0,
            stream_sequence=datagram.header.stream_sequence if datagram.header else 0,
            send_monotonic_ns=datagram.header.send_monotonic_ns if datagram.header else 0,
            clock_model_version=self.clock.version,
            latency_valid=latency_valid,
            one_way_latency_ns=latency_ns,
            protocol_violation_multi_payload=len(fields_tuple) > 1,
        )

    def record_parse_failure(self, datagram: ReceivedDatagram, error: str):
        with self._lock:
            self.parse_failed_packets += 1
        self.logger.log(
            "parse_failure",
            source_ip=datagram.addr[0],
            payload_len=datagram.payload_len,
            error=error,
            packet_sequence=datagram.header.packet_sequence if datagram.header else 0,
        )

    @staticmethod
    def _duration_ns(end_ns: int, start_ns: int) -> Optional[int]:
        if end_ns > 0 and start_ns > 0 and end_ns >= start_ns:
            return end_ns - start_ns
        return None

    def record_processing_timing(
        self,
        datagram: ReceivedDatagram,
        fields: Iterable[str],
        process_done_monotonic_ns: int,
        handled_controller: bool,
        handled_full_body: bool,
        process_ok: bool,
        error: str = "",
    ):
        if not self.logger.enabled:
            return
        datagram.process_done_monotonic_ns = process_done_monotonic_ns
        fields_tuple = tuple(sorted(fields))
        self.logger.log(
            "vrdata_processing_timing",
            source_ip=datagram.addr[0],
            source_port=datagram.addr[1],
            legacy=datagram.legacy,
            payload_len=datagram.payload_len,
            fields=list(fields_tuple),
            handled_controller=handled_controller,
            handled_full_body=handled_full_body,
            process_ok=process_ok,
            error=error,
            recv_monotonic_ns=datagram.recv_monotonic_ns,
            enqueue_monotonic_ns=datagram.enqueue_monotonic_ns,
            process_start_monotonic_ns=datagram.process_start_monotonic_ns,
            protobuf_parse_done_monotonic_ns=datagram.protobuf_parse_done_monotonic_ns,
            process_done_monotonic_ns=process_done_monotonic_ns,
            recv_to_enqueue_ns=self._duration_ns(datagram.enqueue_monotonic_ns, datagram.recv_monotonic_ns),
            queue_wait_ns=self._duration_ns(datagram.process_start_monotonic_ns, datagram.enqueue_monotonic_ns),
            recv_to_process_start_ns=self._duration_ns(datagram.process_start_monotonic_ns, datagram.recv_monotonic_ns),
            protobuf_parse_ns=self._duration_ns(datagram.protobuf_parse_done_monotonic_ns, datagram.process_start_monotonic_ns),
            business_process_ns=self._duration_ns(process_done_monotonic_ns, datagram.protobuf_parse_done_monotonic_ns),
            recv_to_process_done_ns=self._duration_ns(process_done_monotonic_ns, datagram.recv_monotonic_ns),
            queue_to_process_done_ns=self._duration_ns(process_done_monotonic_ns, datagram.enqueue_monotonic_ns),
            session_uuid=datagram.header.session_uuid.hex() if datagram.header else "",
            stream=datagram.header.stream if datagram.header else 0,
            stream_name=STREAM_NAMES.get(datagram.header.stream, "UNKNOWN") if datagram.header else "LEGACY",
            packet_sequence=datagram.header.packet_sequence if datagram.header else 0,
            stream_sequence=datagram.header.stream_sequence if datagram.header else 0,
            send_monotonic_ns=datagram.header.send_monotonic_ns if datagram.header else 0,
            clock_model_version=self.clock.version,
        )

    def _handle_clock_sync_response(
        self,
        header: FixedDiagnosticTransportHeader,
        payload: bytes,
        addr: Tuple[str, int],
        recv_monotonic_ns: int,
    ):
        response = diag_pb.ClockSyncResponse()
        try:
            response.ParseFromString(payload)
        except Exception as exc:
            self.logger.log("clock_sync_response_parse_failure", source_ip=addr[0], error=str(exc))
            return
        with self._lock:
            sample = self.clock.add_sample(
                int(response.ubuntu_send_monotonic_ns),
                int(response.pico_receive_monotonic_ns),
                int(header.send_monotonic_ns),
                int(recv_monotonic_ns),
            )
        self.logger.log("clock_sync_sample", **sample, session_uuid=header.session_uuid.hex())

    def _handle_pico_sender_summary(
        self,
        header: FixedDiagnosticTransportHeader,
        payload: bytes,
        addr: Tuple[str, int],
        recv_monotonic_ns: int,
    ):
        summary = diag_pb.PicoSenderSummary()
        try:
            summary.ParseFromString(payload)
        except Exception as exc:
            self.logger.log("pico_sender_summary_parse_failure", source_ip=addr[0], error=str(exc))
            return
        with self._lock:
            self.pico_summary = summary
        self.logger.log(
            "pico_sender_summary",
            recv_monotonic_ns=recv_monotonic_ns,
            source_ip=addr[0],
            session_uuid=summary.session_uuid.hex(),
            summary_sequence=summary.summary_sequence,
            final=summary.final,
            last_packet_sequence=summary.last_packet_sequence,
            socket_send_success_packets=summary.socket_send_success_packets,
            socket_send_failed_packets=summary.socket_send_failed_packets,
            streams=[
                {
                    "stream": item.stream,
                    "last_stream_sequence": item.last_stream_sequence,
                    "socket_send_success_packets": item.socket_send_success_packets,
                    "socket_send_failed_packets": item.socket_send_failed_packets,
                }
                for item in summary.streams
            ],
        )

    def _run(self):
        while not self._stop.is_set():
            now = time.monotonic()
            with self._lock:
                peer = self.peer_addr
                sync_interval = self.sync_valid_interval_sec if self.clock.valid else self.sync_fast_interval_sec
                should_sync = (
                    self.sync_enabled
                    and self.udp_reply_enabled
                    and peer is not None
                    and now - self.last_sync_sent_at >= sync_interval
                )
                diag_interval = 1.0 / self.diagnostic_publish_hz if self.diagnostic_publish_hz > 0 else 0.0
                should_diag = (
                    self.udp_reply_enabled
                    and peer is not None
                    and diag_interval > 0.0
                    and now - self.last_diag_sent_at >= diag_interval
                )
            if should_sync:
                self.send_clock_sync_request()
            if should_diag:
                self.send_diagnostic_result()
            time.sleep(0.01)

    def _next_ubuntu_packet_sequence(self) -> int:
        with self._lock:
            self.ubuntu_packet_sequence += 1
            return self.ubuntu_packet_sequence

    def send_clock_sync_request(self):
        with self._lock:
            peer = self.peer_addr
            session_uuid = self.session_uuid
            self.sync_sequence += 1
            sync_sequence = self.sync_sequence
            self.last_sync_sent_at = time.monotonic()
        if not peer:
            return
        request = diag_pb.ClockSyncRequest()
        request.protocol_version = PROTOCOL_VERSION
        request.session_uuid = session_uuid
        request.sync_sequence = sync_sequence
        payload = request.SerializeToString()
        header = FixedDiagnosticTransportHeader(
            message_type=MSG_CLOCK_SYNC_REQUEST,
            stream=STREAM_UNSPECIFIED,
            session_uuid=session_uuid,
            packet_sequence=self._next_ubuntu_packet_sequence(),
            stream_sequence=0,
            payload_size=len(payload),
            clock_model_version=self.clock.version,
        )
        header.send_monotonic_ns = monotonic_ns()
        try:
            self.sock.sendto(header.pack() + payload, peer)
            self.logger.log("clock_sync_request_sent", peer=list(peer), sync_sequence=sync_sequence, t1=header.send_monotonic_ns)
        except Exception as exc:
            self.logger.log("clock_sync_request_send_failure", peer=list(peer), error=str(exc))

    def send_diagnostic_result(self):
        outbound_results = []
        with self._lock:
            peer = self.peer_addr
            session_uuid = self.session_uuid
            self.last_diag_sent_at = time.monotonic()
        if not peer:
            return

        with self._lock:
            stream_ids = sorted(self.last_business_headers.keys()) or [STREAM_UNSPECIFIED]
            clock_status = self.clock.status_pb()
            clock_valid = self.clock.valid
            for stream_id in stream_ids:
                header_ref = self.last_business_headers.get(stream_id)
                stream_stats = self.stream_stats[stream_id]
                latency_ns = self._latest_latency_locked(stream_id)

                result = diag_pb.DiagnosticResult()
                result.protocol_version = PROTOCOL_VERSION
                result.session_uuid = session_uuid
                result.stream = stream_id
                if header_ref:
                    result.packet_sequence = header_ref.packet_sequence
                    result.stream_sequence = header_ref.stream_sequence
                result.clock_model.CopyFrom(clock_status)
                result.latency_valid = latency_ns is not None and clock_valid
                if result.latency_valid:
                    result.one_way_latency_ns = int(latency_ns)
                result.window_received_packets = stream_stats.received_unique
                result.window_lost_packets = stream_stats.lost
                denom = stream_stats.received_unique + stream_stats.lost
                result.window_loss_rate = float(stream_stats.lost) / float(denom) if denom > 0 else 0.0
                result.total_received_packets = self.global_stats.received_unique
                result.total_lost_packets = self.global_stats.lost
                total_denom = self.global_stats.received_unique + self.global_stats.lost
                result.total_loss_rate = float(self.global_stats.lost) / float(total_denom) if total_denom > 0 else 0.0
                result.out_of_order_packets = self.global_stats.out_of_order + stream_stats.out_of_order
                result.duplicate_packets = self.global_stats.duplicates + stream_stats.duplicates
                result.queue_dropped_packets = self.queue_dropped_packets
                result.parse_failed_packets = self.parse_failed_packets
                result.unauthorized_dropped_packets = self.unauthorized_dropped_packets

                payload = result.SerializeToString()
                out_header = FixedDiagnosticTransportHeader(
                    message_type=MSG_DIAGNOSTIC_RESULT,
                    stream=stream_id,
                    session_uuid=session_uuid,
                    packet_sequence=self._next_ubuntu_packet_sequence(),
                    stream_sequence=0,
                    payload_size=len(payload),
                    clock_model_version=self.clock.version,
                )
                outbound_results.append((stream_id, out_header, payload, result))

        for stream_id, out_header, payload, result in outbound_results:
            out_header.send_monotonic_ns = monotonic_ns()
            try:
                self.sock.sendto(out_header.pack() + payload, peer)
                self.logger.log(
                    "diagnostic_result_sent",
                    peer=list(peer),
                    stream=stream_id,
                    packet_sequence=result.packet_sequence,
                    stream_sequence=result.stream_sequence,
                    latency_valid=result.latency_valid,
                    one_way_latency_ns=result.one_way_latency_ns if result.latency_valid else None,
                    clock_model_version=result.clock_model.clock_model_version,
                    clock_model_valid=result.clock_model.valid,
                    clock_last_sync_rtt_ns=result.clock_model.last_sync_rtt_ns,
                    clock_last_residual_ns=result.clock_model.last_residual_ns,
                    clock_residual_p95_ns=result.clock_model.residual_p95_ns,
                    window_received_packets=result.window_received_packets,
                    window_lost_packets=result.window_lost_packets,
                    window_loss_rate=result.window_loss_rate,
                    total_received_packets=result.total_received_packets,
                    total_lost_packets=result.total_lost_packets,
                    total_loss_rate=result.total_loss_rate,
                    out_of_order_packets=result.out_of_order_packets,
                    duplicate_packets=result.duplicate_packets,
                    queue_dropped_packets=result.queue_dropped_packets,
                    parse_failed_packets=result.parse_failed_packets,
                    unauthorized_dropped_packets=result.unauthorized_dropped_packets,
                )
            except Exception as exc:
                self.logger.log("diagnostic_result_send_failure", peer=list(peer), stream=stream_id, error=str(exc))

    def _latest_latency_locked(self, stream_id: int) -> Optional[int]:
        window = self.latency_windows.get(stream_id)
        if not window:
            return None
        cutoff = time.monotonic() - 5.0
        while window and window[0][0] < cutoff:
            window.popleft()
        if not window:
            return None
        return window[-1][1]
