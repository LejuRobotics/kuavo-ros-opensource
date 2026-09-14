#!/usr/bin/env python3
"""Convert PICO diagnostic JSONL logs to CSV."""

import argparse
import csv
import glob
import gzip
import json
import os
import re
import sys
from collections import defaultdict
from datetime import datetime, timedelta, timezone


BEIJING_TZ = timezone(timedelta(hours=8))


PREFERRED_FIELDS = [
    "event",
    "wall_time",
    "session_uuid",
    "source_ip",
    "source_port",
    "peer",
    "stream",
    "stream_name",
    "packet_sequence",
    "stream_sequence",
    "fields",
    "payload_len",
    "legacy",
    "parse_ok",
    "latency_valid",
    "one_way_latency_ns",
    "clock_model_version",
    "recv_monotonic_ns",
    "send_monotonic_ns",
    "enqueue_monotonic_ns",
    "process_start_monotonic_ns",
    "protobuf_parse_done_monotonic_ns",
    "process_done_monotonic_ns",
    "recv_to_enqueue_ns",
    "queue_wait_ns",
    "protobuf_parse_ns",
    "business_process_ns",
    "recv_to_process_done_ns",
    "target_name",
    "target_ip",
    "reachable",
    "rtt_ms",
    "model_valid",
    "rtt_ns",
    "residual_ns",
    "valid_input",
    "clock_last_sync_rtt_ns",
    "clock_last_residual_ns",
    "clock_residual_p95_ns",
    "window_loss_rate",
    "total_loss_rate",
    "error",
    "_file",
    "_lineno",
]

COMPACT_FIELDS = [
    "id",
    "数据流",
    "总包序号",
    "分流序号",
    "VR发送时间(估算,北京时间)",
    "机器人收到时间(北京时间)",
    "PICO到机器人单向时延ms",
    "时延有效",
    "处理耗时ms(可选)",
    "Ping目标",
    "Ping RTT ms",
    "当前流累计丢包率%",
    "当前流累计丢包数",
    "时钟有效",
    "同步RTT ms",
    "同步残差ms",
    "备注",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert pico_diag_*.jsonl diagnostic logs to CSV.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--log-dir",
        default="~/.ros/pico_diagnostics",
        help="directory containing pico_diag_*.jsonl",
    )
    parser.add_argument(
        "--files",
        nargs="*",
        default=[],
        help="explicit JSONL files; overrides --log-dir selection when provided",
    )
    parser.add_argument(
        "--output",
        default="",
        help="single CSV output path; defaults to <log-dir>/pico_diag_export.csv",
    )
    parser.add_argument(
        "--per-event-dir",
        default="",
        help="write one CSV per event into this directory instead of one sparse CSV",
    )
    parser.add_argument(
        "--compact",
        action="store_true",
        help="write a compact human-readable CSV for delivery/acceptance instead of all raw fields",
    )
    parser.add_argument(
        "--events",
        nargs="*",
        default=[],
        help="only export these event names; empty exports all events",
    )
    parser.add_argument(
        "--since-wall-time",
        type=float,
        default=0.0,
        help="only include records with wall_time >= this Unix timestamp; 0 disables",
    )
    parser.add_argument(
        "--latest-window-sec",
        type=float,
        default=7200.0,
        help="use files modified within this many seconds of the newest log file; <=0 uses all files",
    )
    return parser.parse_args()


def safe_float(value):
    try:
        if value is None:
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def safe_int(value):
    try:
        if value is None:
            return None
        return int(value)
    except (TypeError, ValueError):
        return None


def ns_to_ms(value):
    value = safe_float(value)
    if value is None:
        return None
    return value / 1_000_000.0


def fmt_ms(value):
    value = safe_float(value)
    if value is None:
        return ""
    return f"{value:.3f}"


def fmt_pct(value):
    value = safe_float(value)
    if value is None:
        return ""
    return f"{value * 100.0:.2f}"


def fmt_wall_time(value):
    value = safe_float(value)
    if value is None or value <= 0:
        return ""
    return datetime.fromtimestamp(value, tz=BEIJING_TZ).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def select_files(args):
    if args.files:
        files = [os.path.expanduser(path) for path in args.files]
    else:
        pattern = os.path.join(os.path.expanduser(args.log_dir), "pico_diag_*.jsonl")
        gz_pattern = pattern + ".gz"
        files = sorted(set(glob.glob(pattern) + glob.glob(gz_pattern)))

    files = [path for path in files if os.path.isfile(path)]
    if not files:
        return []

    if not args.files and args.latest_window_sec > 0:
        newest_mtime = max(os.path.getmtime(path) for path in files)
        cutoff = newest_mtime - args.latest_window_sec
        files = [path for path in files if os.path.getmtime(path) >= cutoff]

    return sorted(files)


def open_log_text(path):
    if path.endswith(".gz"):
        return gzip.open(path, "rt", encoding="utf-8")
    return open(path, "r", encoding="utf-8")


def csv_value(value):
    if value is None:
        return ""
    if isinstance(value, (dict, list, tuple)):
        return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    if isinstance(value, bool):
        return "true" if value else "false"
    return value


def load_records(files, events, since_wall_time):
    wanted_events = set(events)
    records = []
    bad_lines = 0
    for path in files:
        with open_log_text(path) as handle:
            for lineno, line in enumerate(handle, 1):
                line = line.strip()
                if not line:
                    continue
                try:
                    record = json.loads(line)
                except Exception:
                    bad_lines += 1
                    continue
                if wanted_events and record.get("event") not in wanted_events:
                    continue
                if since_wall_time and float(record.get("wall_time", 0.0) or 0.0) < since_wall_time:
                    continue
                record["_file"] = path
                record["_lineno"] = lineno
                records.append(record)
    records.sort(key=lambda item: (float(item.get("wall_time", 0.0) or 0.0), item.get("_file", ""), item.get("_lineno", 0)))
    return records, bad_lines


def ordered_fields(records):
    seen = set()
    all_fields = []
    for field in PREFERRED_FIELDS:
        for record in records:
            if field in record:
                seen.add(field)
                all_fields.append(field)
                break
    extra = set()
    for record in records:
        extra.update(record.keys())
    all_fields.extend(sorted(extra - seen))
    return all_fields


def write_csv(records, path):
    fields = ordered_fields(records)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for record in records:
            writer.writerow({field: csv_value(record.get(field)) for field in fields})
    return path


def safe_filename(name):
    name = name or "unknown"
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", name).strip("_") or "unknown"


def write_per_event(records, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    by_event = defaultdict(list)
    for record in records:
        by_event[record.get("event", "")].append(record)

    paths = []
    for event, event_records in sorted(by_event.items()):
        path = os.path.join(out_dir, f"{safe_filename(event)}.csv")
        write_csv(event_records, path)
        paths.append(path)
    return paths


def processing_key(record):
    return (
        record.get("session_uuid", ""),
        safe_int(record.get("stream")),
        safe_int(record.get("packet_sequence")),
    )


def stream_name(record):
    name = record.get("stream_name")
    if name:
        return name
    stream = safe_int(record.get("stream"))
    return {
        1: "SKELETON",
        2: "CONTROLLER",
        3: "OTHER",
    }.get(stream, str(stream or ""))


def compact_note(vr_record, latest_clock, loss_rate):
    notes = []
    if not bool(vr_record.get("latency_valid", False)):
        notes.append("单向时延无效")
    if latest_clock and not bool(latest_clock.get("model_valid", False)):
        reason = latest_clock.get("invalid_reason") or "clock_model_invalid"
        notes.append(reason)
    if loss_rate and loss_rate > 0.0:
        notes.append("该流存在丢包")
    return "; ".join(notes)


def write_compact_csv(records, path):
    processing_by_packet = {}
    for record in records:
        if record.get("event") != "vrdata_processing_timing":
            continue
        processing_by_packet[processing_key(record)] = record

    latest_ping = None
    latest_vr_ping = None
    latest_clock = None
    stream_state = defaultdict(lambda: {
        "last_sequence": None,
        "received_unique": 0,
        "lost": 0,
        "seen": set(),
    })
    rows = []

    for record in records:
        event = record.get("event", "")
        if event == "ping_sample":
            if bool(record.get("reachable", False)):
                latest_ping = record
                if record.get("target_name") == "vr":
                    latest_vr_ping = record
            continue
        if event == "clock_sync_sample":
            latest_clock = record
            continue
        if event != "vrdata_packet":
            continue

        stream = safe_int(record.get("stream"))
        stream_sequence = safe_int(record.get("stream_sequence"))
        state = stream_state[stream]
        if stream_sequence is not None and stream_sequence not in state["seen"]:
            state["seen"].add(stream_sequence)
            state["received_unique"] += 1
            last_sequence = state["last_sequence"]
            if last_sequence is None:
                state["last_sequence"] = stream_sequence
            elif stream_sequence > last_sequence:
                if stream_sequence > last_sequence + 1:
                    state["lost"] += stream_sequence - last_sequence - 1
                state["last_sequence"] = stream_sequence

        denom = state["received_unique"] + state["lost"]
        loss_rate = (float(state["lost"]) / float(denom)) if denom > 0 else 0.0

        latency_ns = safe_int(record.get("one_way_latency_ns"))
        latency_valid = bool(record.get("latency_valid", False))
        wall_time = safe_float(record.get("wall_time"))
        send_wall_time = None
        if latency_valid and latency_ns is not None and wall_time is not None:
            send_wall_time = wall_time - float(latency_ns) / 1_000_000_000.0

        processing = processing_by_packet.get(processing_key(record), {})
        process_ms = ns_to_ms(processing.get("recv_to_process_done_ns"))
        ping = latest_vr_ping or latest_ping or {}

        rows.append({
            "id": len(rows) + 1,
            "数据流": stream_name(record),
            "总包序号": record.get("packet_sequence", ""),
            "分流序号": record.get("stream_sequence", ""),
            "VR发送时间(估算,北京时间)": fmt_wall_time(send_wall_time),
            "机器人收到时间(北京时间)": fmt_wall_time(wall_time),
            "PICO到机器人单向时延ms": fmt_ms(ns_to_ms(latency_ns) if latency_valid else None),
            "时延有效": "是" if latency_valid else "否",
            "处理耗时ms(可选)": fmt_ms(process_ms),
            "Ping目标": ping.get("target_ip", ""),
            "Ping RTT ms": fmt_ms(ping.get("rtt_ms")),
            "当前流累计丢包率%": fmt_pct(loss_rate),
            "当前流累计丢包数": state["lost"],
            "时钟有效": "是" if latest_clock and bool(latest_clock.get("model_valid", False)) else "否",
            "同步RTT ms": fmt_ms(ns_to_ms(latest_clock.get("rtt_ns")) if latest_clock else None),
            "同步残差ms": fmt_ms(ns_to_ms(latest_clock.get("residual_ns")) if latest_clock else None),
            "备注": compact_note(record, latest_clock, loss_rate),
        })

    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=COMPACT_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    return path, len(rows)


def main():
    args = parse_args()
    files = select_files(args)
    if not files:
        print("No pico_diag_*.jsonl files found.", file=sys.stderr)
        return 2

    records, bad_lines = load_records(files, args.events, args.since_wall_time)
    if not records:
        print("No records matched the filters.", file=sys.stderr)
        return 3

    if args.compact:
        if args.output:
            out_path = os.path.expanduser(args.output)
        else:
            out_path = os.path.join(os.path.expanduser(args.log_dir), "pico_diag_compact.csv")
        _, row_count = write_compact_csv(records, out_path)
        print(f"Input files: {len(files)}")
        print(f"Records loaded: {len(records)}")
        print(f"Rows exported: {row_count}")
        print(f"Bad JSONL lines skipped: {bad_lines}")
        print(f"Compact CSV: {out_path}")
    elif args.per_event_dir:
        out_dir = os.path.expanduser(args.per_event_dir)
        paths = write_per_event(records, out_dir)
        print(f"Input files: {len(files)}")
        print(f"Records exported: {len(records)}")
        print(f"Bad JSONL lines skipped: {bad_lines}")
        print(f"Per-event CSV directory: {out_dir}")
        for path in paths:
            print(f"CSV: {path}")
    else:
        if args.output:
            out_path = os.path.expanduser(args.output)
        else:
            out_path = os.path.join(os.path.expanduser(args.log_dir), "pico_diag_export.csv")
        write_csv(records, out_path)
        print(f"Input files: {len(files)}")
        print(f"Records exported: {len(records)}")
        print(f"Bad JSONL lines skipped: {bad_lines}")
        print(f"CSV: {out_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
