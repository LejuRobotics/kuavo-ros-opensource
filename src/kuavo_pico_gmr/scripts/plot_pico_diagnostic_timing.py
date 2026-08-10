#!/usr/bin/env python3
"""Plot and summarize PICO diagnostic timing JSONL logs."""

import argparse
import csv
import glob
import gzip
import html
import json
import math
import os
import statistics
import sys
import time
import zipfile
from collections import Counter, defaultdict
from datetime import datetime

from pico_diagnostic_jsonl_to_csv import write_compact_csv


os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

NS_TO_MS = 1.0 / 1_000_000.0

RECEIVER_METRICS = [
    ("recv_to_enqueue_ns", "recv->enqueue"),
    ("queue_wait_ns", "queue wait"),
    ("protobuf_parse_ns", "protobuf parse"),
    ("business_process_ns", "receiver business"),
    ("recv_to_process_done_ns", "recv->receiver done"),
]

NETWORK_METRICS = [
    ("one_way_latency_ns", "PICO send->Ubuntu recv"),
]

NETWORK_REPORT_EVENTS = {
    "vrdata_packet",
    "vrdata_processing_timing",
    "clock_sync_sample",
    "pico_sender_summary",
    "queue_drop",
    "unknown_protocol",
    "parse_failure",
    "ping_sample",
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate timing plots and a summary from PICO diagnostic JSONL logs.",
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
        "--latest-window-sec",
        type=float,
        default=7200.0,
        help="use files modified within this many seconds of the newest log file; <=0 uses all files",
    )
    parser.add_argument(
        "--since-wall-time",
        type=float,
        default=0.0,
        help="only include records with wall_time >= this Unix timestamp; 0 disables",
    )
    parser.add_argument(
        "--out-dir",
        default="",
        help="report directory; defaults to <log-dir>/report_<timestamp>",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=5000,
        help="maximum points per metric in time-series plots",
    )
    parser.add_argument(
        "--no-zip",
        action="store_true",
        help="do not create a zip archive for the generated report",
    )
    return parser.parse_args()


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


def load_records(files, since_wall_time=0.0):
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
                if since_wall_time and float(record.get("wall_time", 0.0) or 0.0) < since_wall_time:
                    continue
                record["_file"] = path
                record["_lineno"] = lineno
                records.append(record)
    records.sort(key=lambda item: (float(item.get("wall_time", 0.0) or 0.0), item.get("_file", ""), item.get("_lineno", 0)))
    return records, bad_lines


def ns_to_ms(value):
    if value is None:
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value):
        return None
    return value * NS_TO_MS


def finite_values(records, field, valid_field=None):
    values = []
    for record in records:
        if valid_field and not bool(record.get(valid_field, False)):
            continue
        value = ns_to_ms(record.get(field))
        if value is not None:
            values.append(value)
    return values


def percentile(sorted_values, q):
    if not sorted_values:
        return None
    if len(sorted_values) == 1:
        return sorted_values[0]
    pos = (len(sorted_values) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return sorted_values[lo]
    frac = pos - lo
    return sorted_values[lo] * (1.0 - frac) + sorted_values[hi] * frac


def summarize_values(values):
    if not values:
        return None
    values = sorted(values)
    return {
        "count": len(values),
        "min_ms": values[0],
        "mean_ms": statistics.mean(values),
        "p50_ms": percentile(values, 0.50),
        "p90_ms": percentile(values, 0.90),
        "p95_ms": percentile(values, 0.95),
        "p99_ms": percentile(values, 0.99),
        "max_ms": values[-1],
    }


def fmt_ms(value):
    if value is None:
        return "--"
    return f"{value:.3f}"


def fmt_pct(value):
    if value is None:
        return "--"
    return f"{value * 100.0:.2f}%"


def fmt_num(value):
    if value is None:
        return "--"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def stream_name(stream):
    return {
        0: "UNSPECIFIED",
        1: "SKELETON",
        2: "CONTROLLER",
        3: "OTHER",
    }.get(int(stream or 0), f"STREAM_{stream}")


def safe_float(value, default=0.0):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def safe_int(value, default=0):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def make_out_dir(args):
    if args.out_dir:
        out_dir = os.path.expanduser(args.out_dir)
    else:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_dir = os.path.join(os.path.expanduser(args.log_dir), f"report_{stamp}")
    os.makedirs(out_dir, exist_ok=True)
    return out_dir


def downsample(points, max_points):
    if max_points <= 0 or len(points) <= max_points:
        return points
    step = int(math.ceil(len(points) / float(max_points)))
    return points[::step]


def records_for_event(records, event):
    return [record for record in records if record.get("event") == event]


def metric_points(records, field, valid_field=None):
    points = []
    for index, record in enumerate(records):
        if valid_field and not bool(record.get(valid_field, False)):
            continue
        value = ns_to_ms(record.get(field))
        if value is None:
            continue
        x = float(record.get("wall_time", 0.0) or 0.0)
        points.append((x, value, index))
    if not points:
        return []
    first = points[0][0]
    normalized = []
    for x, value, index in points:
        normalized.append(((x - first) if x > 0 and first > 0 else index, value))
    return normalized


def plot_time_series(records, metrics, title, out_path, max_points, valid_fields=None):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        return f"matplotlib unavailable: {exc}"

    plt.figure(figsize=(13, 6))
    plotted = False
    for field, label in metrics:
        valid_field = valid_fields.get(field) if valid_fields else None
        points = downsample(metric_points(records, field, valid_field=valid_field), max_points)
        if not points:
            continue
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        plt.plot(xs, ys, linewidth=1.2, label=label)
        plotted = True
    if not plotted:
        plt.close()
        return "no data"
    plt.title(title)
    plt.xlabel("relative wall time (s)")
    plt.ylabel("duration (ms)")
    plt.grid(True, alpha=0.3)
    plt.legend(loc="best")
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()
    return ""


def plot_box(records_by_event, groups, out_path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        return f"matplotlib unavailable: {exc}"

    labels = []
    data = []
    for event, metrics in groups:
        records = records_by_event.get(event, [])
        for field, label in metrics:
            values = finite_values(records, field, valid_field="latency_valid" if field == "one_way_latency_ns" else None)
            if values:
                labels.append(label)
                data.append(values)
    if not data:
        return "no data"

    plt.figure(figsize=(14, max(5, 0.35 * len(labels))))
    plt.boxplot(data, vert=False, showfliers=False)
    plt.yticks(range(1, len(labels) + 1), labels)
    plt.xlabel("duration (ms)")
    plt.title("Timing distribution")
    plt.grid(True, axis="x", alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()
    return ""


def plot_ping(records, out_path, max_points):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        return f"matplotlib unavailable: {exc}"

    by_target = defaultdict(list)
    for record in records:
        if record.get("event") != "ping_sample":
            continue
        if not bool(record.get("reachable", False)):
            continue
        rtt = record.get("rtt_ms")
        try:
            rtt = float(rtt)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(rtt):
            continue
        target = f"{record.get('target_name', '')}:{record.get('target_ip', '')}"
        by_target[target].append((float(record.get("wall_time", 0.0) or 0.0), rtt))

    if not by_target:
        return "no data"

    plt.figure(figsize=(13, 5))
    for target, points in sorted(by_target.items()):
        points = downsample(points, max_points)
        first = points[0][0]
        xs = [(x - first) if x > 0 and first > 0 else i for i, (x, _) in enumerate(points)]
        ys = [value for _, value in points]
        plt.plot(xs, ys, marker="o", markersize=2, linewidth=1.2, label=target)
    plt.title("Ping RTT")
    plt.xlabel("relative wall time per target (s)")
    plt.ylabel("RTT (ms)")
    plt.grid(True, alpha=0.3)
    plt.legend(loc="best")
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()
    return ""


def plot_sync_quality(records, out_path, max_points):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        return f"matplotlib unavailable: {exc}"

    points_rtt = []
    points_residual = []
    for record in records:
        if record.get("event") != "clock_sync_sample":
            continue
        x = safe_float(record.get("wall_time"))
        rtt = ns_to_ms(record.get("rtt_ns"))
        residual = ns_to_ms(record.get("residual_ns"))
        if rtt is not None:
            points_rtt.append((x, rtt))
        if residual is not None:
            points_residual.append((x, abs(residual)))

    if not points_rtt and not points_residual:
        return "no data"

    plt.figure(figsize=(13, 5))
    first_candidates = [p[0] for p in points_rtt + points_residual if p[0] > 0]
    first = min(first_candidates) if first_candidates else 0.0
    for label, points in [("sync RTT", points_rtt), ("abs residual", points_residual)]:
        points = downsample(points, max_points)
        if not points:
            continue
        xs = [(x - first) if x > 0 and first > 0 else i for i, (x, _) in enumerate(points)]
        ys = [value for _, value in points]
        plt.plot(xs, ys, linewidth=1.2, label=label)
    plt.title("Clock sync quality")
    plt.xlabel("relative wall time (s)")
    plt.ylabel("duration/error (ms)")
    plt.grid(True, alpha=0.3)
    plt.legend(loc="best")
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()
    return ""


def plot_loss_summary(network_report, out_path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        return f"matplotlib unavailable: {exc}"

    labels = []
    rates = []
    for row in network_report.get("stream_rows", []):
        rate = row.get("missing_rate")
        if rate is None:
            continue
        labels.append("%s\n%s" % (row["stream_name"], row["session_uuid"][:8]))
        rates.append(rate * 100.0)
    for row in network_report.get("datagram_rows", []):
        rate = row.get("missing_rate")
        if rate is None:
            continue
        labels.append("DATAGRAM\n%s" % row["session_uuid"][:8])
        rates.append(rate * 100.0)

    if not rates:
        return "no data"

    plt.figure(figsize=(12, max(5, 0.45 * len(labels))))
    ypos = list(range(len(labels)))
    plt.barh(ypos, rates)
    plt.yticks(ypos, labels)
    plt.xlabel("loss / missing rate (%)")
    plt.title("Packet loss by stream and session")
    plt.grid(True, axis="x", alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()
    return ""


def collect_summaries(records_by_event):
    rows = []
    specs = [
        ("vrdata_packet", NETWORK_METRICS, {"one_way_latency_ns": "latency_valid"}),
        ("vrdata_processing_timing", RECEIVER_METRICS, {}),
    ]
    for event, metrics, valid_fields in specs:
        records = records_by_event.get(event, [])
        for field, label in metrics:
            summary = summarize_values(finite_values(records, field, valid_field=valid_fields.get(field)))
            if not summary:
                continue
            row = {
                "event": event,
                "field": field,
                "label": label,
            }
            row.update(summary)
            rows.append(row)
    return rows


def summarize_ping(records):
    by_target = defaultdict(list)
    unreachable = defaultdict(int)
    for record in records:
        if record.get("event") != "ping_sample":
            continue
        target = f"{record.get('target_name', '')}:{record.get('target_ip', '')}"
        if bool(record.get("reachable", False)) and record.get("rtt_ms") is not None:
            try:
                by_target[target].append(float(record["rtt_ms"]))
            except (TypeError, ValueError):
                pass
        else:
            unreachable[target] += 1
    rows = []
    for target in sorted(set(by_target.keys()) | set(unreachable.keys())):
        summary = summarize_values(by_target[target])
        rows.append((target, summary, unreachable[target]))
    return rows


def summarize_sequence(values):
    values = [safe_int(value) for value in values if safe_int(value) > 0]
    if not values:
        return {
            "count": 0,
            "unique": 0,
            "first": None,
            "last": None,
            "missing": 0,
            "missing_rate": None,
        }
    unique = set(values)
    first = min(unique)
    last = max(unique)
    expected = last - first + 1
    missing = max(0, expected - len(unique))
    return {
        "count": len(values),
        "unique": len(unique),
        "first": first,
        "last": last,
        "missing": missing,
        "missing_rate": (float(missing) / float(expected)) if expected > 0 else None,
    }


def summarize_arrival_order(values):
    backsteps = 0
    gap_before_late = 0
    max_gap = 0
    examples = []
    last = 0
    for value in values:
        seq = safe_int(value)
        if seq <= 0:
            continue
        if last and seq < last:
            backsteps += 1
            if len(examples) < 5:
                examples.append((last, seq))
        elif last and seq > last + 1:
            gap = seq - last - 1
            gap_before_late += gap
            max_gap = max(max_gap, gap)
        if seq > last:
            last = seq
    return {
        "arrival_backsteps": backsteps,
        "online_gap_before_late": gap_before_late,
        "max_online_gap": max_gap,
        "examples": examples,
    }


def collect_network_report(records):
    sessions = defaultdict(
        lambda: {
            "vrdata": [],
            "sync": [],
            "summary": [],
            "streams": defaultdict(list),
            "latencies_ms": [],
            "fields": defaultdict(int),
        }
    )
    queue_drop_by_stream = defaultdict(int)
    queue_drop_total = 0
    event_counts = defaultdict(int)
    unknown_protocol = 0
    parse_failed = 0

    for record in records:
        event = record.get("event", "")
        if event not in NETWORK_REPORT_EVENTS:
            continue
        event_counts[event] += 1
        session_uuid = record.get("session_uuid", "")
        if event == "vrdata_packet":
            session = sessions[session_uuid]
            session["vrdata"].append(record)
            stream = safe_int(record.get("stream"))
            session["streams"][stream].append(safe_int(record.get("stream_sequence")))
            fields = tuple(record.get("fields", []))
            session["fields"][fields] += 1
            if bool(record.get("latency_valid", False)):
                latency = ns_to_ms(record.get("one_way_latency_ns"))
                if latency is not None:
                    session["latencies_ms"].append(latency)
        elif event == "clock_sync_sample":
            sessions[session_uuid]["sync"].append(record)
        elif event == "pico_sender_summary":
            sessions[session_uuid]["summary"].append(record)
        elif event == "queue_drop":
            queue_drop_total += 1
            queue_drop_by_stream[safe_int(record.get("stream"))] += 1
        elif event == "unknown_protocol":
            unknown_protocol += 1
        elif event == "parse_failure":
            parse_failed += 1

    session_rows = []
    stream_rows = []
    datagram_rows = []
    sync_rows = []
    latency_values = []
    latency_valid_packets = 0
    vrdata_packets = 0

    for session_uuid, session in sessions.items():
        if not session_uuid:
            continue
        vrdata = session["vrdata"]
        if not vrdata and not session["sync"] and not session["summary"]:
            continue
        wall_times = [safe_float(r.get("wall_time")) for r in vrdata if r.get("wall_time") is not None]
        duration = (max(wall_times) - min(wall_times)) if wall_times else 0.0
        vrdata_count = len(vrdata)
        valid_count = len(session["latencies_ms"])
        vrdata_packets += vrdata_count
        latency_valid_packets += valid_count
        latency_values.extend(session["latencies_ms"])

        packet_values = [safe_int(r.get("packet_sequence")) for r in vrdata]
        packet_summary = summarize_sequence(packet_values)
        order_summary = summarize_arrival_order(packet_values)
        session_rows.append(
            {
                "session_uuid": session_uuid,
                "duration_sec": duration,
                "vrdata_packets": vrdata_count,
                "vrdata_hz": (float(vrdata_count) / duration) if duration > 0 else None,
                "latency_valid_packets": valid_count,
                "latency_valid_rate": (float(valid_count) / float(vrdata_count)) if vrdata_count else None,
                "packet_missing_if_vr_only": packet_summary["missing"],
                "arrival_backsteps": order_summary["arrival_backsteps"],
                "fields": dict(session["fields"]),
            }
        )

        for stream, values in sorted(session["streams"].items()):
            summary = summarize_sequence(values)
            stream_rows.append(
                {
                    "session_uuid": session_uuid,
                    "stream": stream,
                    "stream_name": stream_name(stream),
                    "first": summary["first"],
                    "last": summary["last"],
                    "received_unique": summary["unique"],
                    "missing": summary["missing"],
                    "missing_rate": summary["missing_rate"],
                }
            )

        summary_records = session["summary"]
        if summary_records:
            latest_summary = summary_records[-1]
            pico_success = safe_int(latest_summary.get("socket_send_success_packets"))
            observed_known = (
                packet_summary["unique"]
                + len(summary_records)
                + len(session["sync"])
            )
            missing = max(0, pico_success - observed_known) if pico_success else None
            datagram_rows.append(
                {
                    "session_uuid": session_uuid,
                    "pico_socket_send_success": pico_success,
                    "pico_socket_send_failed": safe_int(latest_summary.get("socket_send_failed_packets")),
                    "ubuntu_observed_known": observed_known,
                    "missing": missing,
                    "missing_rate": (float(missing) / float(pico_success)) if pico_success and missing is not None else None,
                    "summary_count": len(summary_records),
                }
            )

        sync_records = session["sync"]
        if sync_records:
            rtt_ms = [ns_to_ms(r.get("rtt_ns")) for r in sync_records]
            rtt_ms = [v for v in rtt_ms if v is not None]
            residual_ms = [ns_to_ms(r.get("residual_ns")) for r in sync_records]
            residual_ms = [abs(v) for v in residual_ms if v is not None]
            valid_input = sum(1 for r in sync_records if bool(r.get("valid_input", False)))
            model_valid = sum(1 for r in sync_records if bool(r.get("model_valid", False)))
            invalid_reasons = Counter(
                r.get("invalid_reason", "")
                for r in sync_records
                if r.get("invalid_reason")
            )
            first_vr_time = min(wall_times) if wall_times else None
            first_valid_times = [safe_float(r.get("wall_time")) for r in sync_records if bool(r.get("model_valid", False))]
            time_to_valid = None
            if first_vr_time is not None and first_valid_times:
                time_to_valid = min(first_valid_times) - first_vr_time
            latest = sync_records[-1]
            sync_rows.append(
                {
                    "session_uuid": session_uuid,
                    "sync_samples": len(sync_records),
                    "valid_input_samples": valid_input,
                    "model_valid_samples": model_valid,
                    "time_to_first_valid_sec": time_to_valid,
                    "rtt_summary": summarize_values(rtt_ms),
                    "residual_summary": summarize_values(residual_ms),
                    "last_model_valid": bool(latest.get("model_valid", False)),
                    "last_rtt_ms": ns_to_ms(latest.get("rtt_ns")),
                    "last_residual_ms": ns_to_ms(latest.get("residual_ns")),
                    "last_model_version": latest.get("model_version"),
                    "max_valid_rtt_ms": ns_to_ms(latest.get("max_valid_rtt_ns")),
                    "max_valid_residual_ms": ns_to_ms(latest.get("max_valid_residual_ns")),
                    "latest_invalid_reason": latest.get("invalid_reason", ""),
                    "invalid_reasons": dict(invalid_reasons),
                }
            )

    return {
        "event_counts": dict(event_counts),
        "session_rows": sorted(session_rows, key=lambda row: row["vrdata_packets"], reverse=True),
        "stream_rows": stream_rows,
        "datagram_rows": datagram_rows,
        "sync_rows": sync_rows,
        "latency_summary": summarize_values(latency_values),
        "latency_valid_packets": latency_valid_packets,
        "vrdata_packets": vrdata_packets,
        "queue_drop_total": queue_drop_total,
        "queue_drop_by_stream": dict(queue_drop_by_stream),
        "unknown_protocol": unknown_protocol,
        "parse_failed": parse_failed,
    }


def write_metrics_csv(rows, out_path):
    fields = ["event", "field", "label", "count", "min_ms", "mean_ms", "p50_ms", "p90_ms", "p95_ms", "p99_ms", "max_ms"]
    with open(out_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def metric_table(rows):
    lines = [
        "| Stage | Metric | Count | Mean ms | P50 ms | P90 ms | P95 ms | P99 ms | Max ms |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            "| {event} | {label} | {count} | {mean} | {p50} | {p90} | {p95} | {p99} | {maxv} |".format(
                event=row["event"],
                label=row["label"],
                count=row["count"],
                mean=fmt_ms(row["mean_ms"]),
                p50=fmt_ms(row["p50_ms"]),
                p90=fmt_ms(row["p90_ms"]),
                p95=fmt_ms(row["p95_ms"]),
                p99=fmt_ms(row["p99_ms"]),
                maxv=fmt_ms(row["max_ms"]),
            )
        )
    return "\n".join(lines)


def sync_table(sync_rows):
    lines = [
        "| Session | Samples | Valid input | Valid model | First valid s | RTT P50/P95 ms | Residual P95 ms | Limits ms | Last reason |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for row in sync_rows:
        rtt = row.get("rtt_summary") or {}
        residual = row.get("residual_summary") or {}
        limits = "{rtt}/{residual}".format(
            rtt=fmt_ms(row.get("max_valid_rtt_ms")),
            residual=fmt_ms(row.get("max_valid_residual_ms")),
        )
        lines.append(
            "| `{sid}` | {samples} | {valid_input} | {valid} | {first_valid} | {rtt_p50}/{rtt_p95} | {res_p95} | {limits} | {reason} |".format(
                sid=row["session_uuid"][:12],
                samples=row["sync_samples"],
                valid_input=row.get("valid_input_samples", 0),
                valid=row["model_valid_samples"],
                first_valid=fmt_num(row.get("time_to_first_valid_sec")),
                rtt_p50=fmt_ms(rtt.get("p50_ms")),
                rtt_p95=fmt_ms(rtt.get("p95_ms")),
                res_p95=fmt_ms(residual.get("p95_ms")),
                limits=limits,
                reason=row.get("latest_invalid_reason") or ("valid" if row.get("last_model_valid") else "unknown"),
            )
        )
    return "\n".join(lines)


def loss_table(stream_rows, datagram_rows):
    lines = [
        "| Type | Session | Stream | Received | Missing | Missing rate |",
        "|---|---|---|---:|---:|---:|",
    ]
    for row in stream_rows:
        lines.append(
            "| stream | `{sid}` | {stream} | {received} | {missing} | {rate} |".format(
                sid=row["session_uuid"][:12],
                stream=row["stream_name"],
                received=row["received_unique"],
                missing=row["missing"],
                rate=fmt_pct(row["missing_rate"]),
            )
        )
    for row in datagram_rows:
        lines.append(
            "| datagram | `{sid}` | ALL | {received} | {missing} | {rate} |".format(
                sid=row["session_uuid"][:12],
                received=row["ubuntu_observed_known"],
                missing=row["missing"],
                rate=fmt_pct(row["missing_rate"]),
            )
        )
    return "\n".join(lines)


def queue_drop_text(network_report):
    total = network_report.get("queue_drop_total", 0)
    by_stream = network_report.get("queue_drop_by_stream", {})
    if not total:
        return "No receiver queue drops were recorded."
    parts = []
    for stream, count in sorted(by_stream.items()):
        parts.append(f"{stream_name(stream)}={count}")
    return f"Receiver queue drops: {total} ({', '.join(parts)})."


def conclusion_lines(network_report, ping_rows):
    lines = []
    latency = network_report.get("latency_summary") or {}
    if latency:
        lines.append(
            "PICO->Ubuntu one-way latency typical value is P50 {p50} ms; tail jitter is P95 {p95} ms and max {maxv} ms.".format(
                p50=fmt_ms(latency.get("p50_ms")),
                p95=fmt_ms(latency.get("p95_ms")),
                maxv=fmt_ms(latency.get("max_ms")),
            )
        )
    valid_packets = network_report.get("latency_valid_packets", 0)
    total_packets = network_report.get("vrdata_packets", 0)
    if total_packets:
        lines.append(
            "Clock-sync-backed latency coverage is {rate} ({valid}/{total} VRDATA packets).".format(
                rate=fmt_pct(float(valid_packets) / float(total_packets)),
                valid=valid_packets,
                total=total_packets,
            )
        )
    sync_rows = network_report.get("sync_rows", [])
    if sync_rows:
        worst_rtt_p95 = max(
            ((row.get("rtt_summary") or {}).get("p95_ms") or 0.0)
            for row in sync_rows
        )
        worst_residual_p95 = max(
            ((row.get("residual_summary") or {}).get("p95_ms") or 0.0)
            for row in sync_rows
        )
        valid_model_samples = sum(row.get("model_valid_samples", 0) for row in sync_rows)
        valid_input_samples = sum(row.get("valid_input_samples", 0) for row in sync_rows)
        if valid_model_samples:
            lines.append(
                "Clock sync model became valid; worst session RTT P95 is {rtt} ms and residual P95 is {residual} ms.".format(
                    rtt=fmt_ms(worst_rtt_p95),
                    residual=fmt_ms(worst_residual_p95),
                )
            )
        else:
            lines.append(
                "Clock sync packets were received but the model never became valid; valid input samples {valid_input}, worst session RTT P95 {rtt} ms, residual P95 {residual} ms.".format(
                    valid_input=valid_input_samples,
                    rtt=fmt_ms(worst_rtt_p95),
                    residual=fmt_ms(worst_residual_p95),
                )
            )
    stream_rows = network_report.get("stream_rows", [])
    if stream_rows:
        worst_stream = max(stream_rows, key=lambda row: row.get("missing_rate") or 0.0)
        lines.append(
            "Worst stream missing rate is {rate} on {stream} session {sid}.".format(
                rate=fmt_pct(worst_stream.get("missing_rate")),
                stream=worst_stream["stream_name"],
                sid=worst_stream["session_uuid"][:8],
            )
        )
    if network_report.get("queue_drop_total", 0):
        lines.append(queue_drop_text(network_report))
    if ping_rows:
        reachable = [row for row in ping_rows if row[1]]
        if reachable:
            worst_ping = max(reachable, key=lambda row: (row[1] or {}).get("p95_ms") or 0.0)
            lines.append(
                "Worst ping target is {target}: P50 {p50} ms, P95 {p95} ms, max {maxv} ms.".format(
                    target=worst_ping[0],
                    p50=fmt_ms(worst_ping[1].get("p50_ms")),
                    p95=fmt_ms(worst_ping[1].get("p95_ms")),
                    maxv=fmt_ms(worst_ping[1].get("max_ms")),
                )
            )
    if not lines:
        lines.append("No network diagnostic records were found.")
    return lines


def write_summary(out_path, files, records, bad_lines, rows, ping_rows, plot_results, network_report):
    event_counts = defaultdict(int)
    for record in records:
        event = record.get("event", "")
        if event not in NETWORK_REPORT_EVENTS:
            continue
        event_counts[event] += 1

    lines = []
    lines.append("# PICO Network Diagnostic Summary")
    lines.append("")
    lines.append(f"- Generated at: {datetime.now().isoformat(timespec='seconds')}")
    lines.append(f"- Input files: {len(files)}")
    lines.append(f"- Records: {len(records)}")
    lines.append(f"- Bad JSONL lines skipped: {bad_lines}")
    if files:
        lines.append("")
        lines.append("## Files")
        for path in files:
            lines.append(f"- `{path}`")
    lines.append("")
    lines.append("## Event Counts")
    for event, count in sorted(event_counts.items()):
        lines.append(f"- `{event}`: {count}")

    lines.append("")
    lines.append("## Executive Summary")
    for line in conclusion_lines(network_report, ping_rows):
        lines.append(f"- {line}")

    lines.append("")
    lines.append("## Clock Sync Quality")
    if network_report.get("sync_rows"):
        lines.append(sync_table(network_report["sync_rows"]))
        lines.append("")
        lines.append("- RTT is the round-trip time of Ubuntu->PICO->Ubuntu sync packets; high RTT means the sync channel is jittery.")
        lines.append("- Residual is how far a sync sample deviates from the fitted PICO->Ubuntu clock model; lower residual means a cleaner model.")
    else:
        lines.append("No clock sync samples were found.")

    lines.append("")
    lines.append("## Packet Loss")
    if network_report.get("stream_rows") or network_report.get("datagram_rows"):
        lines.append(loss_table(network_report.get("stream_rows", []), network_report.get("datagram_rows", [])))
        lines.append("")
        lines.append(f"- {queue_drop_text(network_report)}")
        lines.append("- Stream rows are based on `stream_sequence` and are the best numbers for front-end SKELETON/CONTROLLER display.")
        lines.append("- Datagram rows are approximate end-to-end reconciliation using `PICO_SENDER_SUMMARY` plus observed business/sync/summary records.")
    else:
        lines.append("No sequence data was found.")

    lines.append("")
    lines.append("## Timing Metrics")
    if rows:
        lines.append(metric_table(rows))
    else:
        lines.append("No timing metrics were found.")

    lines.append("")
    lines.append("## Ping Metrics")
    if ping_rows:
        lines.append("| Target | Reachable Count | Unreachable Count | Mean ms | P50 ms | P95 ms | Max ms |")
        lines.append("|---|---:|---:|---:|---:|---:|---:|")
        for target, summary, unreachable_count in ping_rows:
            if summary:
                lines.append(
                    f"| `{target}` | {summary['count']} | {unreachable_count} | "
                    f"{fmt_ms(summary['mean_ms'])} | {fmt_ms(summary['p50_ms'])} | "
                    f"{fmt_ms(summary['p95_ms'])} | {fmt_ms(summary['max_ms'])} |"
                )
            else:
                lines.append(f"| `{target}` | 0 | {unreachable_count} | -- | -- | -- | -- |")
    else:
        lines.append("No ping samples were found.")

    lines.append("")
    lines.append("## Observations")
    p95_rows = [row for row in rows if row.get("p95_ms") is not None]
    if p95_rows:
        top = sorted(p95_rows, key=lambda row: row["p95_ms"], reverse=True)[:5]
        lines.append("Largest P95 timing components in this run:")
        for row in top:
            lines.append(f"- `{row['label']}` from `{row['event']}`: P95 {fmt_ms(row['p95_ms'])} ms")
    else:
        lines.append("No timing components were available for observation.")
    lines.append("- No pass/fail conclusion is made because no baseline or acceptance threshold was provided.")

    lines.append("")
    lines.append("## Plots")
    for name, result in plot_results:
        if result:
            lines.append(f"- `{name}`: skipped ({result})")
        else:
            lines.append(f"- `{name}`")

    with open(out_path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")


def html_table(headers, rows):
    out = ["<table>", "<thead><tr>"]
    for header in headers:
        out.append(f"<th>{html.escape(str(header))}</th>")
    out.append("</tr></thead><tbody>")
    for row in rows:
        out.append("<tr>")
        for cell in row:
            out.append(f"<td>{html.escape(str(cell))}</td>")
        out.append("</tr>")
    out.append("</tbody></table>")
    return "\n".join(out)


def write_html(out_path, files, records, bad_lines, rows, ping_rows, plot_results, network_report):
    event_counts = network_report.get("event_counts", {})
    conclusion = conclusion_lines(network_report, ping_rows)

    sync_rows = []
    for row in network_report.get("sync_rows", []):
        rtt = row.get("rtt_summary") or {}
        residual = row.get("residual_summary") or {}
        sync_rows.append([
            row["session_uuid"][:12],
            row["sync_samples"],
            row["model_valid_samples"],
            fmt_num(row.get("time_to_first_valid_sec")),
            fmt_ms(rtt.get("p50_ms")),
            fmt_ms(rtt.get("p95_ms")),
            fmt_ms(residual.get("p95_ms")),
            row.get("last_model_valid"),
        ])

    loss_rows = []
    for row in network_report.get("stream_rows", []):
        loss_rows.append([
            "stream",
            row["session_uuid"][:12],
            row["stream_name"],
            row["received_unique"],
            row["missing"],
            fmt_pct(row["missing_rate"]),
        ])
    for row in network_report.get("datagram_rows", []):
        loss_rows.append([
            "datagram",
            row["session_uuid"][:12],
            "ALL",
            row["ubuntu_observed_known"],
            row["missing"],
            fmt_pct(row["missing_rate"]),
        ])

    timing_rows = []
    for row in rows:
        timing_rows.append([
            row["event"],
            row["label"],
            row["count"],
            fmt_ms(row["mean_ms"]),
            fmt_ms(row["p50_ms"]),
            fmt_ms(row["p95_ms"]),
            fmt_ms(row["p99_ms"]),
            fmt_ms(row["max_ms"]),
        ])

    ping_table_rows = []
    for target, summary, unreachable in ping_rows:
        if summary:
            ping_table_rows.append([
                target,
                summary["count"],
                unreachable,
                fmt_ms(summary.get("p50_ms")),
                fmt_ms(summary.get("p95_ms")),
                fmt_ms(summary.get("max_ms")),
            ])
        else:
            ping_table_rows.append([target, 0, unreachable, "--", "--", "--"])

    plot_html = []
    for name, result in plot_results:
        if result:
            plot_html.append(f"<p><code>{html.escape(name)}</code> skipped: {html.escape(result)}</p>")
        else:
            plot_html.append(f"<section><h3>{html.escape(name)}</h3><img src=\"{html.escape(name)}\" alt=\"{html.escape(name)}\"></section>")

    file_items = "\n".join(f"<li><code>{html.escape(path)}</code></li>" for path in files)
    event_items = "\n".join(
        f"<li><code>{html.escape(event)}</code>: {count}</li>"
        for event, count in sorted(event_counts.items())
    )
    conclusion_items = "\n".join(f"<li>{html.escape(line)}</li>" for line in conclusion)

    content = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <title>PICO 网络诊断报告</title>
  <style>
    body {{ font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; margin: 24px; color: #202124; line-height: 1.45; }}
    h1 {{ font-size: 26px; margin-bottom: 4px; }}
    h2 {{ font-size: 19px; margin-top: 28px; border-bottom: 1px solid #dadce0; padding-bottom: 6px; }}
    h3 {{ font-size: 15px; margin: 18px 0 8px; }}
    .meta, .hint {{ color: #5f6368; }}
    .summary {{ background: #f8fafd; border: 1px solid #d2e3fc; padding: 14px 18px; border-radius: 6px; }}
    table {{ border-collapse: collapse; width: 100%; margin: 12px 0 18px; font-size: 13px; }}
    th, td {{ border: 1px solid #dadce0; padding: 6px 8px; text-align: right; }}
    th:first-child, td:first-child, th:nth-child(2), td:nth-child(2), th:nth-child(3), td:nth-child(3) {{ text-align: left; }}
    code {{ background: #f1f3f4; padding: 1px 4px; border-radius: 3px; }}
    img {{ width: 100%; max-width: 1180px; border: 1px solid #dadce0; margin: 4px 0 18px; }}
  </style>
</head>
<body>
  <h1>PICO 网络诊断报告</h1>
  <p class="meta">Generated at: {html.escape(datetime.now().isoformat(timespec='seconds'))} | Records: {len(records)} | Bad JSON lines: {bad_lines}</p>

  <h2>开头总结</h2>
  <div class="summary"><ul>{conclusion_items}</ul></div>

  <h2>输入文件</h2>
  <ul>{file_items}</ul>
  <p class="hint">验收和对外沟通优先看 <code>compact.csv</code>；需要排查细节时再看 <code>metrics.csv</code> 和原始 JSONL。</p>

  <h2>时间同步效果</h2>
  <p class="hint">RTT 是 Ubuntu->PICO->Ubuntu 同步包往返耗时；残差是同步样本和时钟模型的偏差。RTT 看网络抖动，残差看模型拟合质量。</p>
  {html_table(["Session", "Samples", "Valid model samples", "First valid s", "RTT P50 ms", "RTT P95 ms", "Residual P95 ms", "Last valid"], sync_rows) if sync_rows else "<p>No clock sync samples.</p>"}

  <h2>网络时延与丢包</h2>
  <p class="hint">前端展示丢包率优先使用 stream 行，即 SKELETON/CONTROLLER 的分流丢包率。datagram 行用于整体网络归因。</p>
  {html_table(["Type", "Session", "Stream", "Received", "Missing", "Missing rate"], loss_rows) if loss_rows else "<p>No packet loss data.</p>"}
  <p>{html.escape(queue_drop_text(network_report))}</p>

  <h2>网络接收侧时间数据</h2>
  {html_table(["Event", "Metric", "Count", "Mean ms", "P50 ms", "P95 ms", "P99 ms", "Max ms"], timing_rows) if timing_rows else "<p>No timing metrics.</p>"}

  <h2>Ping</h2>
  {html_table(["Target", "Reachable", "Unreachable", "P50 ms", "P95 ms", "Max ms"], ping_table_rows) if ping_table_rows else "<p>No ping samples.</p>"}

  <h2>事件数量</h2>
  <ul>{event_items}</ul>

  <h2>图表</h2>
  {''.join(plot_html)}
</body>
</html>
"""
    with open(out_path, "w", encoding="utf-8") as handle:
        handle.write(content)


def create_zip(out_dir):
    zip_path = out_dir.rstrip(os.sep) + ".zip"
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for root, _, files in os.walk(out_dir):
            for name in files:
                path = os.path.join(root, name)
                archive.write(path, os.path.relpath(path, out_dir))
    return zip_path


def main():
    args = parse_args()
    files = select_files(args)
    if not files:
        print("No pico_diag_*.jsonl files found.", file=sys.stderr)
        return 2

    out_dir = make_out_dir(args)
    records, bad_lines = load_records(files, since_wall_time=args.since_wall_time)
    records_by_event = defaultdict(list)
    for record in records:
        event = record.get("event", "")
        if event not in NETWORK_REPORT_EVENTS:
            continue
        records_by_event[event].append(record)

    rows = collect_summaries(records_by_event)
    ping_rows = summarize_ping(records)
    network_report = collect_network_report(records)
    write_metrics_csv(rows, os.path.join(out_dir, "metrics.csv"))
    compact_csv_path = os.path.join(out_dir, "compact.csv")
    write_compact_csv(records, compact_csv_path)

    plot_results = []
    plot_results.append((
        "01_network_latency.png",
        plot_time_series(
            records_by_event.get("vrdata_packet", []),
            NETWORK_METRICS,
            "PICO send to Ubuntu receive latency",
            os.path.join(out_dir, "01_network_latency.png"),
            args.max_points,
            valid_fields={"one_way_latency_ns": "latency_valid"},
        ),
    ))
    plot_results.append((
        "02_receiver_timing.png",
        plot_time_series(
            records_by_event.get("vrdata_processing_timing", []),
            RECEIVER_METRICS,
            "Receiver node timing",
            os.path.join(out_dir, "02_receiver_timing.png"),
            args.max_points,
        ),
    ))
    plot_results.append((
        "03_timing_distribution.png",
        plot_box(
            records_by_event,
            [
                ("vrdata_packet", NETWORK_METRICS),
                ("vrdata_processing_timing", RECEIVER_METRICS),
            ],
            os.path.join(out_dir, "03_timing_distribution.png"),
        ),
    ))
    plot_results.append((
        "04_ping_rtt.png",
        plot_ping(records, os.path.join(out_dir, "04_ping_rtt.png"), args.max_points),
    ))
    plot_results.append((
        "05_clock_sync_quality.png",
        plot_sync_quality(records, os.path.join(out_dir, "05_clock_sync_quality.png"), args.max_points),
    ))
    plot_results.append((
        "06_packet_loss.png",
        plot_loss_summary(network_report, os.path.join(out_dir, "06_packet_loss.png")),
    ))

    write_summary(
        os.path.join(out_dir, "summary.md"),
        files=files,
        records=records,
        bad_lines=bad_lines,
        rows=rows,
        ping_rows=ping_rows,
        plot_results=plot_results,
        network_report=network_report,
    )
    write_html(
        os.path.join(out_dir, "report.html"),
        files=files,
        records=records,
        bad_lines=bad_lines,
        rows=rows,
        ping_rows=ping_rows,
        plot_results=plot_results,
        network_report=network_report,
    )

    zip_path = ""
    if not args.no_zip:
        zip_path = create_zip(out_dir)

    print(f"Report written to: {out_dir}")
    print(f"HTML: {os.path.join(out_dir, 'report.html')}")
    print(f"Summary: {os.path.join(out_dir, 'summary.md')}")
    print(f"Metrics CSV: {os.path.join(out_dir, 'metrics.csv')}")
    print(f"Compact CSV: {compact_csv_path}")
    if zip_path:
        print(f"Zip: {zip_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
