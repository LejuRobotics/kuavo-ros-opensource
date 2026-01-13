#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import csv
import sys
import math

def load_times(path):
    times = []
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            times.append(float(row["recv_time_sec"]))
    return sorted(times)

def stats_from_times(times):
    if len(times) < 2:
        return None
    periods = [t2 - t1 for t1, t2 in zip(times, times[1:])]
    duration = times[-1] - times[0]
    mean_p = sum(periods) / len(periods)
    rate = 1.0 / mean_p if mean_p > 0 else float("inf")
    var = sum((p - mean_p)**2 for p in periods) / len(periods)
    std = math.sqrt(var)
    return {
        "count": len(times),
        "duration": duration,
        "rate": rate,
        "min": min(periods),
        "max": max(periods),
        "mean": mean_p,
        "std": std,
    }

def print_stats(name, s):
    print("=== %s ===" % name)
    print("Messages      :", s["count"])
    print("Time span (s) :", "%.3f" % s["duration"])
    print("Avg rate (Hz) :", "%.3f" % s["rate"])
    print("Min period (s):", "%.4f" % s["min"])
    print("Max period (s):", "%.4f" % s["max"])
    print("Mean period(s):", "%.4f" % s["mean"])
    print("Std period (s):", "%.4f" % s["std"])
    print("")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python compare_quest3_rates.py /tmp/quest3_python.csv /tmp/quest3_cpp.csv")
        sys.exit(1)

    py_csv, cpp_csv = sys.argv[1], sys.argv[2]

    py_times = load_times(py_csv)
    cpp_times = load_times(cpp_csv)

    py_stats = stats_from_times(py_times)
    cpp_stats = stats_from_times(cpp_times)

    print_stats("Python", py_stats)
    print_stats("C++   ", cpp_stats)

    rate_diff = cpp_stats["rate"] - py_stats["rate"]
    rate_ratio = cpp_stats["rate"] / py_stats["rate"] if py_stats["rate"] > 0 else float("inf")
    print("Rate difference (cpp - python): %.3f Hz" % rate_diff)
    print("Rate ratio (cpp / python)      : %.3f x" % rate_ratio)