"""Atomic score snapshots shared by task, collect and model entry points."""
import json
import os
import tempfile
import time


def atomic_write(path, text):
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=".score-", dir=directory)
    try:
        # The scorer runs as container root; host evaluators must retain the
        # same read access as the previous ordinary 0644 score files.
        os.fchmod(descriptor, 0o644)
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(text)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def write_score(score_file, payload, latest_file=None):
    """JSON is authoritative; each file replacement is individually atomic."""
    payload = dict(payload, updated_at=time.time())
    for path in dict.fromkeys(p for p in (score_file, latest_file) if p):
        atomic_write(path, "{}\n".format(payload["total"]))
        atomic_write(os.path.splitext(path)[0] + ".json",
                     json.dumps(payload, ensure_ascii=False, indent=2) + "\n")


def reset_score(score_file, metadata, latest_file=None):
    write_score(score_file, dict(metadata, total=0, components={},
                                status="preparing"), latest_file)


def invalidate_score(score_file, reason, latest_file=None):
    with open(os.path.splitext(score_file)[0] + ".json", encoding="utf-8") as stream:
        payload = json.load(stream)
    payload.update(status="invalid", finish_reason=str(reason))
    write_score(score_file, payload, latest_file)
