"""Latest-sample transport primitives. No ROS, kinematics or motion shaping."""
import legacy_dependencies

import math
import threading


class SourceClockGate:
    """Reject old/backlogged samples using excess delay, not absolute latency.

    The minimum receive-minus-source offset includes the unknown clock offset
    and minimum path delay. Its residual is only a relative age estimate.
    Reset explicitly after reconnect; an out-of-order packet is not a recenter.
    """

    def __init__(self, max_age=0.12):
        self.max_age = max_age
        self.reset()

    def reset(self):
        self.latest_source = None
        self.minimum_offset = None
        self.relative_age = 0.0
        self.rejected = 0
        self.duplicates = 0

    def accept(self, timestamp_ms, received):
        source = float(timestamp_ms) * 0.001
        if not math.isfinite(source) or source <= 0.0 or not math.isfinite(received):
            self.rejected += 1
            return False
        if self.latest_source is not None and source < self.latest_source:
            self.rejected += 1
            return False
        offset = received - source
        self.minimum_offset = offset if self.minimum_offset is None else min(
            self.minimum_offset, offset)
        self.relative_age = max(0.0, offset - self.minimum_offset)
        if self.relative_age > self.max_age:
            self.rejected += 1
            return False
        if source == self.latest_source:
            self.duplicates += 1
        self.latest_source = source
        return True


class LatestSample:
    """One complete immutable packet; never queue a Cartesian trajectory."""

    def __init__(self):
        self.condition = threading.Condition()
        self.sample = None
        self.overwritten = 0

    def put(self, sample):
        with self.condition:
            if self.sample is not None:
                self.overwritten += 1
            self.sample = sample
            self.condition.notify()

    def take(self, timeout=0.05):
        with self.condition:
            if self.sample is None:
                self.condition.wait(timeout)
            result, self.sample = self.sample, None
            return result

    def clear(self):
        with self.condition:
            self.sample = None
