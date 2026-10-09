"""Durable, rosbag-independent Task 1 grasp diagnostics."""

import json
import os
import time
from pathlib import Path


SCHEMA_VERSION = 1
DEFAULT_FILENAME = "task1_grasp_diagnostics.jsonl"


class Task1GraspDiagnosticRecorder:
    """Append one self-contained JSON record and flush it immediately."""

    def __init__(self, seed, path=None):
        configured = path or os.environ.get("TASK1_DIAGNOSTICS_PATH")
        self.path = Path(configured or DEFAULT_FILENAME).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.seed = int(seed)
        self.started_monotonic = time.monotonic()

    def write(self, event, **fields):
        payload = {
            "schema_version": SCHEMA_VERSION,
            "event": str(event),
            "seed": self.seed,
            "wall_time_s": time.time(),
            "task_elapsed_s": time.monotonic() - self.started_monotonic,
        }
        payload.update(fields)
        encoded = (json.dumps(
            payload, sort_keys=True, separators=(",", ":")) + "\n").encode(
                "utf-8")
        descriptor = os.open(
            str(self.path), os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
        try:
            os.write(descriptor, encoded)
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
        return payload
